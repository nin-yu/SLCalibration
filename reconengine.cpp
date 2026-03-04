#include "reconengine.h"
#include <QDebug>
#include <QFileInfo>
#include <cmath>

ReconEngine::ReconEngine(QObject *parent)
    : QObject(parent)
    , m_calibLoaded(false)
    , m_decoder(nullptr)
    , m_nGrayCode(5)
    , m_nPhaseShift(4)
    , m_modulationThreshold(0.1)
    , m_minZ(200.0f)
    , m_maxZ(1500.0f)
    , m_tx(0.0f)
    , m_tz(0.0f)
{
}

ReconEngine::~ReconEngine()
{
    if (m_decoder) {
        delete m_decoder;
        m_decoder = nullptr;
    }
}

bool ReconEngine::loadCalibration(const QString& calibFilePath)
{
    QFileInfo fileInfo(calibFilePath);
    if (!fileInfo.exists()) {
        emit errorOccurred(QString("Calibration file not found: %1").arg(calibFilePath));
        return false;
    }

    // Load calibration data using cv::FileStorage
    cv::FileStorage fs(calibFilePath.toStdString(), cv::FileStorage::READ);
    if (!fs.isOpened()) {
        emit errorOccurred(QString("Failed to open calibration file: %1").arg(calibFilePath));
        return false;
    }

    // Read camera parameters
    fs["camMatrix"] >> m_calibData.camMatrix;
    fs["camDist"] >> m_calibData.camDist;
    
    // Read projector parameters
    fs["projMatrix"] >> m_calibData.projMatrix;
    fs["projDist"] >> m_calibData.projDist;
    
    // Read extrinsic parameters
    fs["R_CamToProj"] >> m_calibData.R_CamToProj;
    fs["T_CamToProj"] >> m_calibData.T_CamToProj;
    
    // Read resolution and frequency
    int camWidth, camHeight, projWidth, projHeight;
    fs["camRes_width"] >> camWidth;
    fs["camRes_height"] >> camHeight;
    fs["projRes_width"] >> projWidth;
    fs["projRes_height"] >> projHeight;
    fs["proj_frequency"] >> m_calibData.projFrequency;
    
    m_calibData.camRes = cv::Size(camWidth, camHeight);
    m_calibData.projRes = cv::Size(projWidth, projHeight);
    
    // Read optional board-to-camera transform
    fs["R_BoardToCam"] >> m_calibData.R_BoardToCam;
    fs["T_BoardToCam"] >> m_calibData.T_BoardToCam;
    
    fs.release();

    // Validate loaded data
    if (m_calibData.camMatrix.empty() || m_calibData.projMatrix.empty() ||
        m_calibData.R_CamToProj.empty() || m_calibData.T_CamToProj.empty()) {
        emit errorOccurred("Calibration data is incomplete");
        return false;
    }

    // Initialize undistortion maps
    initUndistortMaps();

    // Create decoder
    if (m_decoder) {
        delete m_decoder;
    }
    m_decoder = new StructuredLightDecoder(
        m_nGrayCode, 
        m_nPhaseShift, 
        m_calibData.projFrequency, 
        m_calibData.projRes, 
        "vertical"
    );

    // Precompute triangulation coefficients
    // Get camera intrinsic parameters
    double fx_cam = m_calibData.camMatrix.at<double>(0, 0);
    double fy_cam = m_calibData.camMatrix.at<double>(1, 1);
    double cx_cam = m_calibData.camMatrix.at<double>(0, 2);
    double cy_cam = m_calibData.camMatrix.at<double>(1, 2);

    // Get projector intrinsic parameters
    double fx_proj = m_calibData.projMatrix.at<double>(0, 0);
    double cx_proj = m_calibData.projMatrix.at<double>(0, 2);

    // Get extrinsic parameters (rotation and translation)
    cv::Mat R = m_calibData.R_CamToProj;
    cv::Mat T = m_calibData.T_CamToProj;
    
    m_tx = static_cast<float>(T.at<double>(0, 0));
    m_tz = static_cast<float>(T.at<double>(2, 0));

    // Compute Jacobian components for triangulation
    // J_x = R[0,0] - R[2,0] * x_c_norm
    // J_z = R[0,2] - R[2,2] * x_c_norm
    // For each pixel (u, v): x_c_norm = (u - cx_cam) / fx_cam

    int rows = m_calibData.camRes.height;
    int cols = m_calibData.camRes.width;
    
    m_xCamNorm.create(rows, cols, CV_32F);
    m_yCamNorm.create(rows, cols, CV_32F);
    m_Jx.create(rows, cols, CV_32F);
    m_Jz.create(rows, cols, CV_32F);

    double r00 = R.at<double>(0, 0);
    double r02 = R.at<double>(0, 2);
    double r20 = R.at<double>(2, 0);
    double r22 = R.at<double>(2, 2);

    for (int v = 0; v < rows; ++v) {
        for (int u = 0; u < cols; ++u) {
            float x_c = static_cast<float>((u - cx_cam) / fx_cam);
            float y_c = static_cast<float>((v - cy_cam) / fy_cam);
            
            m_xCamNorm.at<float>(v, u) = x_c;
            m_yCamNorm.at<float>(v, u) = y_c;
            m_Jx.at<float>(v, u) = static_cast<float>(r00 - r20 * x_c);
            m_Jz.at<float>(v, u) = static_cast<float>(r02 - r22 * x_c);
        }
    }

    m_calibLoaded = true;
    qDebug() << "Calibration loaded successfully from" << calibFilePath;
    qDebug() << "Camera resolution:" << m_calibData.camRes.width << "x" << m_calibData.camRes.height;
    qDebug() << "Projector resolution:" << m_calibData.projRes.width << "x" << m_calibData.projRes.height;
    qDebug() << "Projector frequency:" << m_calibData.projFrequency;

    return true;
}

void ReconEngine::initUndistortMaps()
{
    if (m_calibData.camMatrix.empty() || m_calibData.camDist.empty()) {
        return;
    }

    // Compute undistortion maps
    cv::initUndistortRectifyMap(
        m_calibData.camMatrix,
        m_calibData.camDist,
        cv::Mat(),  // No rectification
        m_calibData.camMatrix,  // Use same camera matrix
        m_calibData.camRes,
        CV_32FC1,
        m_undistortMapX,
        m_undistortMapY
    );

    qDebug() << "Undistortion maps initialized:" 
             << m_undistortMapX.cols << "x" << m_undistortMapX.rows;
}

void ReconEngine::undistortImages(const QVector<cv::Mat>& input, QVector<cv::Mat>& output)
{
    output.clear();
    output.reserve(input.size());

    for (const cv::Mat& img : input) {
        cv::Mat undistorted;
        if (!m_undistortMapX.empty() && !m_undistortMapY.empty()) {
            cv::remap(img, undistorted, m_undistortMapX, m_undistortMapY, cv::INTER_LINEAR);
        } else {
            undistorted = img.clone();
        }
        output.append(undistorted);
    }
}

cv::Mat ReconEngine::createModulationMask(const QVector<cv::Mat>& psImages, const cv::Mat& darkImage)
{
    if (psImages.empty()) {
        return cv::Mat();
    }

    // Convert to std::vector for compatibility
    std::vector<cv::Mat> psVec;
    for (const cv::Mat& img : psImages) {
        psVec.push_back(img);
    }

    // Use StructuredLightDecoder's static method
    return StructuredLightDecoder::createModulationMask(psVec, darkImage, m_modulationThreshold);
}

PointCloudT::Ptr ReconEngine::reconstruct(const QVector<cv::Mat>& imageBatch)
{
    if (!m_calibLoaded) {
        emit errorOccurred("Calibration not loaded");
        return nullptr;
    }

    // Validate batch size (Pattern 8: 1 white + 1 dark + 5 GC + 4 PS = 11)
    const int EXPECTED_BATCH_SIZE = 11;
    if (imageBatch.size() != EXPECTED_BATCH_SIZE) {
        emit errorOccurred(QString("Invalid batch size: expected %1, got %2")
            .arg(EXPECTED_BATCH_SIZE).arg(imageBatch.size()));
        return nullptr;
    }

    // 1. Undistort all images
    QVector<cv::Mat> undistorted;
    undistortImages(imageBatch, undistorted);

    // 2. Separate images
    // [0] = White, [1] = Dark, [2-6] = GC, [7-10] = PS
    cv::Mat whiteImg = undistorted[0];
    cv::Mat darkImg = undistorted[1];
    
    std::vector<cv::Mat> gcImages;
    for (int i = 2; i <= 6; ++i) {
        gcImages.push_back(undistorted[i]);
    }
    
    std::vector<cv::Mat> psImages;
    for (int i = 7; i <= 10; ++i) {
        psImages.push_back(undistorted[i]);
    }

    // 3. Create modulation mask
    QVector<cv::Mat> psVec;
    for (const cv::Mat& img : psImages) {
        psVec.append(img);
    }
    cv::Mat validMask = createModulationMask(psVec, darkImg);
    
    if (validMask.empty()) {
        emit errorOccurred("Failed to create modulation mask");
        return nullptr;
    }

    // 4. Decode phase using StructuredLightDecoder
    cv::Mat absPhase = m_decoder->decode(gcImages, psImages, darkImg);
    
    if (absPhase.empty()) {
        emit errorOccurred("Phase decoding failed");
        return nullptr;
    }

    // 5. Convert phase to projector pixel coordinate
    cv::Mat phaseMapU = m_decoder->phaseToPixels(absPhase);

    // 6. Triangulate to get 3D points
    cv::Mat points3D = triangulate3D(phaseMapU, validMask);

    // 7. Convert to PCL point cloud
    PointCloudT::Ptr cloud = convertToPCL(points3D, validMask);

    return cloud;
}

cv::Mat ReconEngine::triangulate3D(const cv::Mat& phaseMapU, const cv::Mat& validMask)
{
    int rows = phaseMapU.rows;
    int cols = phaseMapU.cols;
    
    cv::Mat points3D(rows, cols, CV_32FC3, cv::Scalar(0, 0, 0));

    // Get projector intrinsic parameters
    double fx_proj = m_calibData.projMatrix.at<double>(0, 0);
    double cx_proj = m_calibData.projMatrix.at<double>(0, 2);

    for (int v = 0; v < rows; ++v) {
        for (int u = 0; u < cols; ++u) {
            // Check if pixel is valid
            if (validMask.at<uchar>(v, u) == 0) {
                continue;
            }

            // Get projector x coordinate (normalized)
            float u_proj = phaseMapU.at<float>(v, u);
            float x_p_norm = static_cast<float>((u_proj - cx_proj) / fx_proj);

            // Get precomputed values
            float x_c = m_xCamNorm.at<float>(v, u);
            float y_c = m_yCamNorm.at<float>(v, u);
            float Jx = m_Jx.at<float>(v, u);
            float Jz = m_Jz.at<float>(v, u);

            // Triangulation formula:
            // Z_c = (x_p_norm * tz - tx) / (Jx - Jz * x_p_norm)
            float denom = Jx - Jz * x_p_norm;
            if (std::abs(denom) < 1e-6f) {
                continue;  // Skip degenerate case
            }

            float Z_c = (x_p_norm * m_tz - m_tx) / denom;

            // Depth range check
            if (Z_c < m_minZ || Z_c > m_maxZ) {
                continue;
            }

            // Compute X and Y from normalized coordinates
            float X_c = x_c * Z_c;
            float Y_c = y_c * Z_c;

            // Store 3D point
            cv::Vec3f& point = points3D.at<cv::Vec3f>(v, u);
            point[0] = X_c;
            point[1] = Y_c;
            point[2] = Z_c;
        }
    }

    return points3D;
}

PointCloudT::Ptr ReconEngine::convertToPCL(const cv::Mat& points3D, const cv::Mat& validMask)
{
    PointCloudT::Ptr cloud(new PointCloudT);
    
    int rows = points3D.rows;
    int cols = points3D.cols;

    // Count valid points first for pre-allocation
    int validCount = 0;
    for (int v = 0; v < rows; ++v) {
        for (int u = 0; u < cols; ++u) {
            const cv::Vec3f& pt = points3D.at<cv::Vec3f>(v, u);
            if (pt[2] > 0 && validMask.at<uchar>(v, u) > 0) {
                ++validCount;
            }
        }
    }

    cloud->points.reserve(validCount);
    cloud->width = validCount;
    cloud->height = 1;
    cloud->is_dense = false;

    // Add valid points
    for (int v = 0; v < rows; ++v) {
        for (int u = 0; u < cols; ++u) {
            const cv::Vec3f& pt = points3D.at<cv::Vec3f>(v, u);
            if (pt[2] > 0 && validMask.at<uchar>(v, u) > 0) {
                PointT pclPoint;
                pclPoint.x = pt[0];
                pclPoint.y = pt[1];
                pclPoint.z = pt[2];
                cloud->points.push_back(pclPoint);
            }
        }
    }

    qDebug() << "Point cloud generated:" << cloud->points.size() << "points";

    return cloud;
}
