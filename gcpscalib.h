#ifndef GCPSCALIB_H
#define GCPSCALIB_H

#include <QObject>
#include <opencv2/opencv.hpp>
#include <vector>
#include <QDebug>
#include <fstream>
#include <sstream>
#include <ctime>
#include <iomanip>
// 结构体定义
struct CalibrationData {
    cv::Mat camMatrix;      // 相机内参矩阵
    cv::Mat camDist;        // 相机畸变系数
    cv::Mat projMatrix;     // 投影仪内参矩阵
    cv::Mat projDist;       // 投影仪畸变系数
    cv::Mat R_CamToProj;    // 相机到投影仪的旋转矩阵
    cv::Mat T_CamToProj;    // 相机到投影仪的平移向量
    cv::Size camRes;        // 相机分辨率
    cv::Size projRes;       // 投影仪分辨率
    int projFrequency;      // 投影仪频率
    double rmsProj;         // 投影仪标定RMS误差
    double rmsStereo;       // 立体标定RMS误差
    
    // 世界坐标系转换
    cv::Mat R_BoardToCam;   // 标定板到相机的旋转矩阵
    cv::Mat T_BoardToCam;   // 标定板到相机的平移向量
};

class GcPsCalib : public QObject
{
    Q_OBJECT

public:
    explicit GcPsCalib(QObject *parent = nullptr);
    
    // 主要功能函数
    bool calibrate(const std::vector<std::string>& imagePaths, 
                  const cv::Size& chessboardSize, 
                  float squareSize, 
                  const cv::Size& projSize, 
                  int projFreq, 
                  int nGrayCode, 
                  int nPhaseShift,
                  CalibrationData& calibData);
    
    // 图像分组和处理函数
    std::vector<std::vector<std::string>> groupCalibrationImages(const std::vector<std::string>& allImagePaths);
    bool processPoseGroup(const std::vector<std::string>& poseImages, 
                         const cv::Size& chessboardSize, 
                         float squareSize, 
                         const cv::Size& projSize, 
                         int projFreq, 
                         int nGrayCode, 
                         int nPhaseShift,
                         std::vector<cv::Point3f>& worldPoints,
                         std::vector<cv::Point2f>& cameraPoints,
                         std::vector<cv::Point2f>& projectorPoints);
    
    
    // 标定结果保存和加载
    bool saveCalibrationData(const CalibrationData& calibData, const std::string& filePath);
    bool loadCalibrationData(CalibrationData& calibData, const std::string& filePath);
    
    
    // 相位解码相关函数（已弃用，现在使用独立的StructuredLightDecoder类）
    // [[deprecated("Use StructuredLightDecoder::decode instead")]]
    cv::Mat decodeGrayCode(const std::vector<cv::Mat>& grayCodeImages, int nBits);
    // [[deprecated("Use StructuredLightDecoder::decode instead")]]
    cv::Mat decodePhaseShift(const std::vector<cv::Mat>& phaseImages, int nSteps, double initialPhase = 0.0);
    // [[deprecated("Use StructuredLightDecoder::decode instead")]]
    cv::Mat combineGrayCodeAndPhase(const cv::Mat& grayCodePhase, const cv::Mat& phase);
    cv::Mat phaseToPixels(const cv::Mat& phaseMap, int projResolution, double projFreq);// 相位到像素转换
    std::vector<cv::Point2f> getSubpixelValues(const cv::Mat& mapU, const cv::Mat& mapV, const std::vector<cv::Point2f>& points);    // 获取亚像素值



    
signals:
    void calibrationProgress(int progress, int total);
    void calibrationFinished(bool success);
    
private:
    
    // 内部辅助函数
    bool findChessboardCornersEnhanced(const cv::Mat& image, 
                                      const cv::Size& patternSize, 
                                      std::vector<cv::Point2f>& corners);
    
    cv::Mat createModulationMask(const std::vector<cv::Mat>& phaseImages, 
                                const cv::Mat& darkImage, 
                                double threshold);
    
    // 调试报告输出函数
    void writeDebugReport(const std::string& filePath,
                          const cv::Size& chessboardSize,
                          float squareSize,
                          const cv::Size& projSize,
                          int projFreq,
                          int nGrayCode,
                          int nPhaseShift,
                          const cv::Size& camSize,
                          int successCount,
                          int totalPoses,
                          const cv::Mat& projMatrix,
                          const cv::Mat& projDist,
                          double rmsProj,
                          const std::vector<cv::Mat>& rvecsProj,
                          const std::vector<cv::Mat>& tvecsProj,
                          const cv::Mat& R,
                          const cv::Mat& T,
                          const cv::Mat& E,
                          const cv::Mat& F,
                          double rmsStereo,
                          const cv::Mat& camMatrix,
                          const cv::Mat& camDist,
                          const std::vector<std::vector<cv::Point2f>>& allCameraPoints,
                          const std::vector<std::vector<cv::Point2f>>& allProjectorPoints);
};

#endif // GCPSCALIB_H
