#include "reconengine.h"
#include <QDebug>
#include <QFile>
#include <opencv2/imgproc.hpp>

ReconEngine::ReconEngine(QObject* parent)
    : QObject(parent)
    , m_decoder(nullptr)
    , m_calibrationLoaded(false)
{
}

ReconEngine::~ReconEngine()
{
    if (m_decoder)
    {
        delete m_decoder;
        m_decoder = nullptr;
    }
}

bool ReconEngine::loadCalibration(const QString& calibFilePath, double scale)
{
    // 检查文件是否存在
    if (!QFile::exists(calibFilePath))
    {
        emit errorOccurred(QString("标定文件不存在: %1").arg(calibFilePath));
        return false;
    }

    // 读取XML标定文件
    cv::FileStorage fs(calibFilePath.toStdString(), cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        emit errorOccurred(QString("无法打开标定文件: %1").arg(calibFilePath));
        return false;
    }

    // 读取标定数据
    fs["camMatrix"] >> m_calibData.camMatrix;
    fs["camDist"] >> m_calibData.camDist;
    fs["projMatrix"] >> m_calibData.projMatrix;
    fs["projDist"] >> m_calibData.projDist;
    fs["R_CamToProj"] >> m_calibData.R_CamToProj;
    fs["T_CamToProj"] >> m_calibData.T_CamToProj;
    m_calibData.camRes.width = (int)fs["camRes_width"];
    m_calibData.camRes.height = (int)fs["camRes_height"];
    m_calibData.projRes.width = (int)fs["projRes_width"];
    m_calibData.projRes.height = (int)fs["projRes_height"];
    m_calibData.proj_frequency = (int)fs["proj_frequency"];
    
    // 读取标定板坐标系变换（可选）
    fs["R_BoardToCam"] >> m_calibData.R_BoardToCam;
    fs["T_BoardToCam"] >> m_calibData.T_BoardToCam;
    fs.release();

    // 验证必要数据
    if (m_calibData.camMatrix.empty() || m_calibData.R_CamToProj.empty())
    {
        emit errorOccurred("标定文件缺少必要数据");
        return false;
    }

    // 如果没有标定板坐标系变换，使用默认值（单位矩阵）
    if (m_calibData.R_BoardToCam.empty() || m_calibData.T_BoardToCam.empty())
    {
        m_calibData.R_BoardToCam = cv::Mat::eye(3, 3, CV_64F);
        m_calibData.T_BoardToCam = cv::Mat::zeros(3, 1, CV_64F);
    }
    m_calibData.R_BoardToCam.convertTo(m_calibData.R_BoardToCam_32f, CV_32F);
    m_calibData.T_BoardToCam.convertTo(m_calibData.T_BoardToCam_32f, CV_32F);

    // 根据缩放因子调整分辨率
    if (scale != 1.0 && scale > 0)
    {
        cv::Size scaledCamRes = cv::Size(
            (int)(m_calibData.camRes.width * scale),
            (int)(m_calibData.camRes.height * scale)
        );

        // 缩放内参矩阵
        m_calibData.camMatrix.at<double>(0, 0) *= scale; // fx
        m_calibData.camMatrix.at<double>(1, 1) *= scale; // fy
        m_calibData.camMatrix.at<double>(0, 2) *= scale; // cx
        m_calibData.camMatrix.at<double>(1, 2) *= scale; // cy

        m_calibData.camRes = scaledCamRes;
    }

    // 计算去畸变映射
    m_calibData.camMatrixNew = cv::getOptimalNewCameraMatrix(
        m_calibData.camMatrix, m_calibData.camDist,
        m_calibData.camRes, 1, m_calibData.camRes, &m_calibData.roi
    );

    cv::initUndistortRectifyMap(
        m_calibData.camMatrix, m_calibData.camDist, cv::Mat(),
        m_calibData.camMatrixNew, m_calibData.camRes,
        CV_32FC1, m_calibData.mapx, m_calibData.mapy
    );

    // 预计算重建参数
    precomputeReconstructionParams();

    // 创建结构光解码器
    int freq = m_calibData.proj_frequency;
    if (freq == 0) freq = m_params.projFrequency;

    m_decoder = new StructuredLightDecoderRecon(
        m_params.nGrayCode,
        m_params.nPhaseShift,
        freq,
        m_calibData.projRes,
        "vertical",
        this
    );

    m_calibrationLoaded = true;
    qDebug() << "[ReconEngine] 标定数据加载成功:" << calibFilePath;
    qDebug() << "[ReconEngine] 相机分辨率:" << m_calibData.camRes.width << "x" << m_calibData.camRes.height;
    qDebug() << "[ReconEngine] 投影仪分辨率:" << m_calibData.projRes.width << "x" << m_calibData.projRes.height;

    return true;
}

void ReconEngine::precomputeReconstructionParams()
{
    // 提取相机内参
    m_calibData.fx_cam = m_calibData.camMatrixNew.at<double>(0, 0);
    m_calibData.fy_cam = m_calibData.camMatrixNew.at<double>(1, 1);
    m_calibData.cx_cam = m_calibData.camMatrixNew.at<double>(0, 2);
    m_calibData.cy_cam = m_calibData.camMatrixNew.at<double>(1, 2);

    // 提取投影仪内参
    m_calibData.fx_proj = m_calibData.projMatrix.at<double>(0, 0);
    m_calibData.cx_proj = m_calibData.projMatrix.at<double>(0, 2);

    // 提取平移向量
    m_calibData.tx = m_calibData.T_CamToProj.at<double>(0, 0);
    m_calibData.tz = m_calibData.T_CamToProj.at<double>(2, 0);

    // 创建归一化坐标网格
    cv::Mat u_cam_grid(m_calibData.camRes, CV_32F);
    cv::Mat v_cam_grid(m_calibData.camRes, CV_32F);

    #pragma omp parallel for
    for (int r = 0; r < m_calibData.camRes.height; ++r)
    {
        for (int c = 0; c < m_calibData.camRes.width; ++c)
        {
            u_cam_grid.at<float>(r, c) = (float)c;
            v_cam_grid.at<float>(r, c) = (float)r;
        }
    }

    // 计算归一化坐标
    m_calibData.x_c_norm = (u_cam_grid - m_calibData.cx_cam) / m_calibData.fx_cam;
    m_calibData.y_c_norm = (v_cam_grid - m_calibData.cy_cam) / m_calibData.fy_cam;

    // 计算雅可比矩阵基
    cv::Mat R;
    m_calibData.R_CamToProj.convertTo(R, CV_32F);
    m_calibData.J_x_base = R.at<float>(0, 0) * m_calibData.x_c_norm +
                          R.at<float>(0, 1) * m_calibData.y_c_norm +
                          R.at<float>(0, 2);
    m_calibData.J_z_base = R.at<float>(2, 0) * m_calibData.x_c_norm +
                          R.at<float>(2, 1) * m_calibData.y_c_norm +
                          R.at<float>(2, 2);
}

PointCloudT::Ptr ReconEngine::reconstruct(const ReconImageBatch& imageBatch)
{
    if (!m_calibrationLoaded)
    {
        emit errorOccurred("未加载标定数据");
        return PointCloudT::Ptr();
    }

    // 验证图像数量
    const int expectedCount = m_params.nGrayCode + m_params.nPhaseShift;
    if (imageBatch.size() != expectedCount)
    {
        emit errorOccurred(QString("图像数量不正确: 期望 %1 张, 实际 %2 张")
            .arg(expectedCount).arg(imageBatch.size()));
        return PointCloudT::Ptr();
    }

    // 分离格雷码和相移图像
    std::vector<cv::Mat> gc_imgs;
    std::vector<cv::Mat> ps_imgs;

    for (int i = 0; i < m_params.nGrayCode; ++i)
    {
        gc_imgs.push_back(imageBatch[i]);
    }
    for (int i = m_params.nGrayCode; i < expectedCount; ++i)
    {
        ps_imgs.push_back(imageBatch[i]);
    }

    // 去畸变
    std::vector<cv::Mat> gc_undist, ps_undist;
    for (const auto& img : gc_imgs)
    {
        cv::Mat undist;
        cv::remap(img, undist, m_calibData.mapx, m_calibData.mapy, cv::INTER_LINEAR);
        gc_undist.push_back(undist);
    }
    for (const auto& img : ps_imgs)
    {
        cv::Mat undist;
        cv::remap(img, undist, m_calibData.mapx, m_calibData.mapy, cv::INTER_LINEAR);
        ps_undist.push_back(undist);
    }

    // 创建调制掩码
    cv::Mat valid_mask = createModulationMask(ps_undist, 
        static_cast<int>(m_params.modulationThreshold));

    if (valid_mask.empty())
    {
        emit errorOccurred("创建调制掩码失败");
        return PointCloudT::Ptr();
    }

    // 解码结构光
    cv::Mat unwrapped_phase = m_decoder->decode_without_dark_img(gc_undist, ps_undist);

    if (unwrapped_phase.empty())
    {
        emit errorOccurred("相位解码失败");
        return PointCloudT::Ptr();
    }

    // 相位转像素坐标
    cv::Mat map_up = m_decoder->phase_to_pixels(unwrapped_phase);

    // 重建三维点
    cv::Mat points_3d;
    if (!reconstruct3DPoints(map_up, valid_mask, points_3d))
    {
        emit errorOccurred("三维重建失败");
        return PointCloudT::Ptr();
    }

    // 转换为PCL点云
    PointCloudT::Ptr cloud(new PointCloudT);
    cloud->reserve(points_3d.rows);

    for (int i = 0; i < points_3d.rows; ++i)
    {
        cv::Vec3f pt = points_3d.at<cv::Vec3f>(i, 0);
        if (std::isfinite(pt[0]) && std::isfinite(pt[1]) && std::isfinite(pt[2]))
        {
            PointT pcl_pt;
            pcl_pt.x = pt[0];
            pcl_pt.y = pt[1];
            pcl_pt.z = pt[2];
            cloud->push_back(pcl_pt);
        }
    }

    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;

    qDebug() << "[ReconEngine] 重建完成，点数:" << cloud->size();

    return cloud;
}

cv::Mat ReconEngine::createModulationMask(const std::vector<cv::Mat>& ps_imgs, int threshold)
{
    if (ps_imgs.size() != 4)
    {
        return cv::Mat();
    }

    std::vector<cv::Mat> i(4);
    for (size_t k = 0; k < 4; ++k)
    {
        ps_imgs[k].convertTo(i[k], CV_32F);
    }

    // 计算调制
    cv::Mat numerator = i[3] - i[1];
    cv::Mat denominator = i[0] - i[2];
    cv::Mat modulation;
    cv::magnitude(numerator, denominator, modulation);

    // 创建掩码
    cv::Mat mask;
    cv::threshold(modulation, mask, (double)threshold, 255, cv::THRESH_BINARY);
    mask.convertTo(mask, CV_8U);

    // 形态学处理
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::erode(mask, mask, kernel);
    cv::dilate(mask, mask, kernel);

    return mask;
}

bool ReconEngine::reconstruct3DPoints(const cv::Mat& map_up,
    const cv::Mat& valid_mask,
    cv::Mat& out_points_3d)
{
    // 计算归一化投影仪坐标
    cv::Mat x_p_norm = (map_up - m_calibData.cx_proj) / m_calibData.fx_proj;

    // 计算分母
    cv::Mat denominator = m_calibData.J_x_base - m_calibData.J_z_base.mul(x_p_norm);

    // 处理接近零的分母
    cv::Mat denom_mask = cv::abs(denominator) < 1e-6;
    denominator.setTo(1.0, denom_mask);

    // 计算深度 Zc
    cv::Mat Z_c = (x_p_norm * m_calibData.tz - m_calibData.tx) / denominator;

    // 计算 Xc, Yc
    cv::Mat X_c = m_calibData.x_c_norm.mul(Z_c);
    cv::Mat Y_c = m_calibData.y_c_norm.mul(Z_c);

    // 深度范围过滤
    cv::Mat z_mask_low, z_mask_high, z_mask;
    cv::compare(Z_c, m_params.minDepth, z_mask_low, cv::CMP_GT);
    cv::compare(Z_c, m_params.maxDepth, z_mask_high, cv::CMP_LT);
    cv::bitwise_and(z_mask_low, z_mask_high, z_mask);
    cv::bitwise_and(z_mask, valid_mask, z_mask);

    // 统计有效点数
    int n_valid_points = cv::countNonZero(z_mask);
    if (n_valid_points == 0)
    {
        return false;
    }

    // 创建输出数组
    out_points_3d.create(n_valid_points, 1, CV_32FC3);

    int pt_idx = 0;
    for (int r = 0; r < m_calibData.camRes.height; ++r)
    {
        for (int c = 0; c < m_calibData.camRes.width; ++c)
        {
            if (z_mask.at<uchar>(r, c) > 0)
            {
                // 相机坐标系下的点
                cv::Mat p_cam = (cv::Mat_<float>(3, 1) <<
                    X_c.at<float>(r, c),
                    Y_c.at<float>(r, c),
                    Z_c.at<float>(r, c)
                );

                // 转换到标定板坐标系
                cv::Mat p_board = m_calibData.R_BoardToCam_32f * p_cam + 
                                  m_calibData.T_BoardToCam_32f;

                out_points_3d.at<cv::Vec3f>(pt_idx, 0) = cv::Vec3f(
                    p_board.at<float>(0, 0),
                    p_board.at<float>(1, 0),
                    p_board.at<float>(2, 0)
                );
                pt_idx++;
            }
        }
    }

    return true;
}

void ReconEngine::setModulationThreshold(double threshold)
{
    m_params.modulationThreshold = threshold;
}

void ReconEngine::setDepthRange(float minDepth, float maxDepth)
{
    m_params.minDepth = minDepth;
    m_params.maxDepth = maxDepth;
}
