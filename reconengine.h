#ifndef RECONENGINE_H
#define RECONENGINE_H

#include <QObject>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "gcpscalib.h"
#include "structuredlightdecoder.h"

// Point cloud types (same as pointcloudwidget.h)
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

/**
 * @class ReconEngine
 * @brief 3D reconstruction engine for structured light scanning
 * 
 * This class encapsulates the complete reconstruction pipeline:
 * - Image undistortion
 * - Phase decoding (Gray code + Phase shift)
 * - Triangulation to 3D points
 * - Conversion to PCL point cloud
 */
class ReconEngine : public QObject
{
    Q_OBJECT

public:
    explicit ReconEngine(QObject *parent = nullptr);
    ~ReconEngine();

    /**
     * @brief Load calibration data from XML file
     * @param calibFilePath Path to calibration_data.xml
     * @return true if loaded successfully
     */
    bool loadCalibration(const QString& calibFilePath);

    /**
     * @brief Check if calibration data is loaded
     */
    bool isCalibrationLoaded() const { return m_calibLoaded; }

    /**
     * @brief Reconstruct 3D point cloud from a batch of images
     * @param imageBatch Vector of 11 images (Pattern 8: white, dark, 5 GC, 4 PS)
     * @return Point cloud pointer, or nullptr on failure
     * 
     * Image order in batch (Pattern 8):
     *   [0] = White image
     *   [1] = Dark image
     *   [2-6] = Gray code images (5 images)
     *   [7-10] = Phase shift images (4 images)
     */
    PointCloudT::Ptr reconstruct(const QVector<cv::Mat>& imageBatch);

    /**
     * @brief Get calibration data
     */
    const CalibrationData& getCalibrationData() const { return m_calibData; }

    /**
     * @brief Set modulation threshold for mask generation
     * @param threshold Value between 0.0 and 1.0 (default: 0.1)
     */
    void setModulationThreshold(double threshold) { m_modulationThreshold = threshold; }

    /**
     * @brief Set valid depth range for filtering
     * @param minZ Minimum Z value in mm (default: 200)
     * @param maxZ Maximum Z value in mm (default: 1500)
     */
    void setDepthRange(float minZ, float maxZ) { m_minZ = minZ; m_maxZ = maxZ; }

signals:
    /**
     * @brief Signal emitted when an error occurs
     */
    void errorOccurred(const QString& message);

private:
    /**
     * @brief Undistort a batch of images using precomputed maps
     */
    void undistortImages(const QVector<cv::Mat>& input, QVector<cv::Mat>& output);

    /**
     * @brief Create modulation mask from phase shift images
     */
    cv::Mat createModulationMask(const QVector<cv::Mat>& psImages, const cv::Mat& darkImage);

    /**
     * @brief Perform triangulation to get 3D points
     * @param phaseMapU Vertical phase map (converted to projector U coordinate)
     * @param validMask Binary mask of valid pixels
     * @return CV_32FC3 matrix containing 3D points (X, Y, Z)
     */
    cv::Mat triangulate3D(const cv::Mat& phaseMapU, const cv::Mat& validMask);

    /**
     * @brief Convert OpenCV 3D points matrix to PCL point cloud
     */
    PointCloudT::Ptr convertToPCL(const cv::Mat& points3D, const cv::Mat& validMask);

    /**
     * @brief Initialize undistortion maps from calibration data
     */
    void initUndistortMaps();

    // Calibration data
    CalibrationData m_calibData;
    bool m_calibLoaded;

    // Structured light decoder
    StructuredLightDecoder* m_decoder;

    // Precomputed undistortion maps
    cv::Mat m_undistortMapX;
    cv::Mat m_undistortMapY;

    // Reconstruction parameters
    int m_nGrayCode;
    int m_nPhaseShift;
    double m_modulationThreshold;
    float m_minZ;
    float m_maxZ;

    // Precomputed triangulation coefficients
    cv::Mat m_xCamNorm;  // Normalized camera x coordinates
    cv::Mat m_yCamNorm;  // Normalized camera y coordinates
    cv::Mat m_Jx;        // Jacobian x component
    cv::Mat m_Jz;        // Jacobian z component
    float m_tx, m_tz;    // Translation components
};

#endif // RECONENGINE_H
