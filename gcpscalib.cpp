#include "gcpscalib.h"
#include "structuredlightdecoder.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <regex>
#include <functional>
#include <iomanip>
#include <sstream>
#include <qdir.h>
#include <filesystem>

// 实现 GcPsCalib 类

GcPsCalib::GcPsCalib(QObject *parent)  // 构造函数，初始化GcPsCalib对象
    : QObject{parent}
{

}

bool GcPsCalib::calibrate(const std::vector<std::string>& imagePaths,  // 标定图像路径列表
                          const cv::Size& chessboardSize,  // 棋盘格尺寸
                          float squareSize,  // 棋盘格方块大小
                          const cv::Size& projSize,  // 投影仪分辨率
                          int projFreq,  // 投影仪频率
                          int nGrayCode,  // 格雷码位数
                          int nPhaseShift,  // 相移步数
                          CalibrationData& calibData)  // 标定数据输出
{
    // 1. 图像分组
    std::vector<std::vector<std::string>> poseGroups = groupCalibrationImages(imagePaths);
    
    if (poseGroups.empty()) {
        std::cerr << "Error: No pose groups found." << std::endl;
        return false;
    }
    
    // 2. 初始化数据容器
    std::vector<std::vector<cv::Point3f>> allWorldPoints;
    std::vector<std::vector<cv::Point2f>> allCameraPoints;
    std::vector<std::vector<cv::Point2f>> allProjectorPoints;
    
    // 创建世界坐标模板
    std::vector<cv::Point3f> worldCornersTemplate;
    for (int i = 0; i < chessboardSize.height; ++i) {
        for (int j = 0; j < chessboardSize.width; ++j) {
            worldCornersTemplate.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        }
    }
    
    // 3. 处理每个姿态组
    int totalPoses = poseGroups.size();
    int successCount = 0;
    
    for (int i = 0; i < totalPoses; ++i) {

        std::vector<cv::Point3f> worldPoints;
        std::vector<cv::Point2f> cameraPoints;
        std::vector<cv::Point2f> projectorPoints;
        
        if (processPoseGroup(poseGroups[i], chessboardSize, squareSize, 
                            projSize, projFreq, nGrayCode, nPhaseShift,
                            worldPoints, cameraPoints, projectorPoints)) {
            allWorldPoints.push_back(worldCornersTemplate);
            allCameraPoints.push_back(cameraPoints);
            allProjectorPoints.push_back(projectorPoints);
            successCount++;
        }
    }
    
    if (allWorldPoints.empty()) {
        std::cerr << "Error: No valid poses were processed." << std::endl;
        return false;
    }
    
    // 4. 获取相机分辨率（从第一组中的白场图像）
    cv::Size camSize;
    if (!poseGroups.empty() && !poseGroups[0].empty()) {
        // 在第一组中查找白场图像
        for (const auto& imagePath : poseGroups[0]) {
            std::string basename = imagePath.substr(imagePath.find_last_of("/\\") + 1);
            if (basename.find("_White") != std::string::npos || basename.find("_white_") != std::string::npos) {
                cv::Mat firstImg = cv::imread(imagePath);
                camSize = firstImg.size();
                break;
            }
        }
        // 如果没有找到白场图像，使用第一张图像
        if (camSize.width == 0 || camSize.height == 0) {
            cv::Mat firstImg = cv::imread(poseGroups[0][0]);
            camSize = firstImg.size();
        }
    } else {
        std::cerr << "Error: No images available for camera resolution detection." << std::endl;
        return false;
    }
    
    // 调试输出：打印allWorldPoints和allProjectorPoints的内容
    std::cout << "======= 调试信息：标定点数据 =======" << std::endl;
    std::cout << "总共有 " << allWorldPoints.size() << " 组姿态数据" << std::endl;

    for(size_t poseIdx = 0; poseIdx < allWorldPoints.size(); poseIdx++) {
        std::cout << "\n--- 姿态 " << poseIdx << " ---" << std::endl;
        std::cout << "世界坐标点数量: " << allWorldPoints[poseIdx].size() << std::endl;
        std::cout << "投影仪坐标点数量: " << allProjectorPoints[poseIdx].size() << std::endl;

        // 检查两组点的数量是否匹配
        if(allWorldPoints[poseIdx].size() != allProjectorPoints[poseIdx].size()) {
            std::cout << "警告: 姿态 " << poseIdx << " 的世界坐标点与投影仪坐标点数量不匹配!" << std::endl;
        }

        // 打印前几个点的坐标（限制输出数量以避免过多信息）
        size_t printCount = std::min(static_cast<size_t>(10), allWorldPoints[poseIdx].size());
        for(size_t ptIdx = 0; ptIdx < printCount; ptIdx++) {
            std::cout << "  点 " << ptIdx 
                      << ": 世界坐标(" << allWorldPoints[poseIdx][ptIdx].x 
                      << ", " << allWorldPoints[poseIdx][ptIdx].y 
                      << ", " << allWorldPoints[poseIdx][ptIdx].z 
                      << ") <-> 投影仪坐标(" << allProjectorPoints[poseIdx][ptIdx].x 
                      << ", " << allProjectorPoints[poseIdx][ptIdx].y << ")" << std::endl;
        }

        if(allWorldPoints[poseIdx].size() > 10) {
            std::cout << "  ... 还有 " << (allWorldPoints[poseIdx].size() - 10) << " 个点未显示" << std::endl;
        }
    }

    std::cout << "===================================" << std::endl;
    
    // 5. 执行投影仪标定
    cv::Mat projMatrix, projDist;
    std::vector<cv::Mat> rvecsProj, tvecsProj;
    double rmsProj = cv::calibrateCamera(allWorldPoints, allProjectorPoints,
                                         projSize, projMatrix, projDist, 
                                         rvecsProj, tvecsProj);
    
    // 6. 执行立体标定
    cv::Mat R, T, E, F;
    double rmsStereo = cv::stereoCalibrate(allWorldPoints, allCameraPoints, allProjectorPoints,
                                           calibData.camMatrix, calibData.camDist,
                                           projMatrix, projDist,
                                           camSize, R, T, E, F, cv::CALIB_FIX_INTRINSIC);
    
    // 7. 保存标定结果
    calibData.projMatrix = projMatrix;
    calibData.projDist = projDist;
    calibData.R_CamToProj = R;
    calibData.T_CamToProj = T;
    calibData.camRes = camSize;
    calibData.projRes = projSize;
    calibData.projFrequency = projFreq;
    calibData.rmsProj = rmsProj;
    calibData.rmsStereo = rmsStereo;
    
    // 8. 输出调试报告
    writeDebugReport("debug/calibration_debug.txt",
                     chessboardSize, squareSize, projSize, projFreq,
                     nGrayCode, nPhaseShift, camSize, successCount, totalPoses,
                     projMatrix, projDist, rmsProj, rvecsProj, tvecsProj,
                     R, T, E, F, rmsStereo,
                     calibData.camMatrix, calibData.camDist,
                     allCameraPoints, allProjectorPoints);
    
    emit calibrationFinished(true);
    return true;
}

std::vector<std::vector<std::string>> GcPsCalib::groupCalibrationImages(const std::vector<std::string>& allImagePaths)  // 按姿态分组标定图像
{
    std::vector<std::vector<std::string>> poseGroups;
    std::map<std::string, std::vector<std::string>> groupedFiles;
    std::regex poseRegex("Pose_\\d+");
        
    // 按姿态分组图像
    for (const auto& path : allImagePaths) {
        std::string basename = path.substr(path.find_last_of("/\\") + 1);
        std::smatch match;
        if (std::regex_search(basename, match, poseRegex)) {
            std::string posePrefix = match[0].str();
            groupedFiles[posePrefix].push_back(path);
        }
    }
    
    // 转换为向量格式，并对每个组内的文件路径进行排序
    for (auto& pair : groupedFiles) {
        // 关键修改：对每个 Pose 组内的文件进行字母序排序
        std::sort(pair.second.begin(), pair.second.end());
        poseGroups.push_back(pair.second);
    }
    
    return poseGroups;
}

bool GcPsCalib::processPoseGroup(const std::vector<std::string>& poseImages,  // 单个姿态的图像列表
                                 const cv::Size& chessboardSize,  // 棋盘格尺寸
                                 float squareSize,  // 棋盘格方块大小
                                 const cv::Size& projSize,  // 投影仪分辨率
                                 int projFreq,  // 投影仪频率
                                 int nGrayCode,  // 格雷码位数
                                 int nPhaseShift,  // 相移步数
                                 std::vector<cv::Point3f>& worldPoints,  // 世界坐标点输出
                                 std::vector<cv::Point2f>& cameraPoints,  // 相机坐标点输出
                                 std::vector<cv::Point2f>& projectorPoints)  // 投影仪坐标点输出
{
    // 期望文件数量：1 Dark + 1 White + 5 GC_H + 5 GC_V + 4 PS_V + 4 PS_H = 20 张
    const int EXPECTED_IMAGE_COUNT = 20;
    
    if (poseImages.size() < EXPECTED_IMAGE_COUNT) {
        std::cerr << "Error: Expected at least " << EXPECTED_IMAGE_COUNT << " images, got " << poseImages.size() << std::endl;
        return false;
    }
    
    // 1. 按位置索引加载图像（与 BasicDemo 参考实现一致）
    // 文件顺序（已排序）：[0]=Dark, [1]=White, [2-6]=GC_H, [7-11]=GC_V, [12-15]=PS_V, [16-19]=PS_H
    cv::Mat darkImg = cv::imread(poseImages[0], cv::IMREAD_GRAYSCALE);
    cv::Mat whiteImg = cv::imread(poseImages[1], cv::IMREAD_GRAYSCALE);
    
    std::vector<cv::Mat> gcHImgs, gcVImgs, psVImgs, psHImgs;
    for (int i = 2; i <= 6; i++) {
        gcHImgs.push_back(cv::imread(poseImages[i], cv::IMREAD_GRAYSCALE));
    }
    for (int i = 7; i <= 11; i++) {
        gcVImgs.push_back(cv::imread(poseImages[i], cv::IMREAD_GRAYSCALE));
    }
    for (int i = 12; i <= 15; i++) {
        psVImgs.push_back(cv::imread(poseImages[i], cv::IMREAD_GRAYSCALE));
    }
    for (int i = 16; i <= 19; i++) {
        psHImgs.push_back(cv::imread(poseImages[i], cv::IMREAD_GRAYSCALE));
    }
    
    // 2. 验证图像加载成功
    if (darkImg.empty()) {
        std::cerr << "Error: Failed to load dark image: " << poseImages[0] << std::endl;
        return false;
    }
    if (whiteImg.empty()) {
        std::cerr << "Error: Failed to load white image: " << poseImages[1] << std::endl;
        return false;
    }
    
    // 验证 GC/PS 图像数量
    if (gcHImgs.size() != nGrayCode || gcVImgs.size() != nGrayCode ||
        psHImgs.size() != nPhaseShift || psVImgs.size() != nPhaseShift) {
        std::cerr << "Error: Incorrect number of GC/PS images loaded." << std::endl;
        return false;
    }
    
    // 验证所有图像加载成功
    for (size_t i = 0; i < gcHImgs.size(); i++) {
        if (gcHImgs[i].empty()) {
            std::cerr << "Error: Failed to load GC_H image " << i << std::endl;
            return false;
        }
    }
    for (size_t i = 0; i < gcVImgs.size(); i++) {
        if (gcVImgs[i].empty()) {
            std::cerr << "Error: Failed to load GC_V image " << i << std::endl;
            return false;
        }
    }
    for (size_t i = 0; i < psVImgs.size(); i++) {
        if (psVImgs[i].empty()) {
            std::cerr << "Error: Failed to load PS_V image " << i << std::endl;
            return false;
        }
    }
    for (size_t i = 0; i < psHImgs.size(); i++) {
        if (psHImgs[i].empty()) {
            std::cerr << "Error: Failed to load PS_H image " << i << std::endl;
            return false;
        }
    }
    
    // 2. 检测相机图像中的角点
    bool found = findChessboardCornersEnhanced(whiteImg, chessboardSize, cameraPoints);
    if (!found) {
        std::cerr << "Error: Failed to find chessboard corners in white image." << std::endl;
        return false;
    }
    
    // 调试输出：打印cameraPoints的内容
    std::cout << "======= 调试信息：相机角点数据 =======" << std::endl;
    std::cout << "检测到的相机角点数量: " << cameraPoints.size() << std::endl;
    
    // 打印前几个角点坐标（限制输出数量以避免过多信息）
    size_t printCount = std::min(static_cast<size_t>(5), cameraPoints.size());
    for(size_t ptIdx = 0; ptIdx < printCount; ptIdx++) {
        std::cout << "  角点 " << ptIdx << ": (" 
                  << std::fixed << std::setprecision(2) << cameraPoints[ptIdx].x 
                  << ", " << cameraPoints[ptIdx].y << ")" << std::endl;
    }
    
    if(cameraPoints.size() > 5) {
        std::cout << "  ... 还有 " << (cameraPoints.size() - 5) << " 个角点未显示" << std::endl;
    }
    
    std::cout << "=====================================" << std::endl;
    
    // 3. 创建世界坐标点
    worldPoints.clear();
    for (int i = 0; i < chessboardSize.height; ++i) {
        for (int j = 0; j < chessboardSize.width; ++j) {
            worldPoints.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        }
    }
    
    // 4. 解码垂直和水平方向的相位
    StructuredLightDecoder decoderV(nGrayCode, nPhaseShift, projFreq, projSize, "vertical");
    StructuredLightDecoder decoderH(nGrayCode, nPhaseShift, projFreq, projSize, "horizontal");
    
    cv::Mat absPhaseV = decoderV.decode(gcVImgs, psVImgs, darkImg);
    cv::Mat absPhaseH = decoderH.decode(gcHImgs, psHImgs, darkImg);
    
    if (absPhaseV.empty() || absPhaseH.empty()) {
        std::cerr << "Error: Phase decoding failed." << std::endl;
        return false;
    }
    
    // 调试输出：打印absPhaseV和absPhaseH的内容
    std::cout << "======= 调试信息：相位数据 =======" << std::endl;
    std::cout << "绝对相位V维度: " << absPhaseV.rows << "x" << absPhaseV.cols << std::endl;
    std::cout << "绝对相位H维度: " << absPhaseH.rows << "x" << absPhaseH.cols << std::endl;
    
    // 打印相位矩阵的一些统计信息
    if(!absPhaseV.empty()) {
        double minValV, maxValV;
        cv::minMaxLoc(absPhaseV, &minValV, &maxValV);
        std::cout << "absPhaseV - 最小值: " << std::fixed << std::setprecision(2) << minValV 
                  << ", 最大值: " << maxValV << std::endl;
        
        // 打印左上角区域的一些相位值
        int sampleRows = std::min(3, absPhaseV.rows);
        int sampleCols = std::min(3, absPhaseV.cols);
        std::cout << "absPhaseV样本值:" << std::endl;
        for(int r = 0; r < sampleRows; r++) {
            std::cout << "  行" << r << ": ";
            for(int c = 0; c < sampleCols; c++) {
                std::cout << std::fixed << std::setprecision(2) << absPhaseV.at<float>(r, c) << " ";
            }
            std::cout << std::endl;
        }
    }
    
    if(!absPhaseH.empty()) {
        double minValH, maxValH;
        cv::minMaxLoc(absPhaseH, &minValH, &maxValH);
        std::cout << "absPhaseH - 最小值: " << std::fixed << std::setprecision(2) << minValH 
                  << ", 最大值: " << maxValH << std::endl;
        
        // 打印左上角区域的一些相位值
        int sampleRows = std::min(3, absPhaseH.rows);
        int sampleCols = std::min(3, absPhaseH.cols);
        std::cout << "absPhaseH样本值:" << std::endl;
        for(int r = 0; r < sampleRows; r++) {
            std::cout << "  行" << r << ": ";
            for(int c = 0; c < sampleCols; c++) {
                std::cout << std::fixed << std::setprecision(2) << absPhaseH.at<float>(r, c) << " ";
            }
            std::cout << std::endl;
        }
    }
    
    std::cout << "=====================================" << std::endl;
    
    // 5. 转换相位为投影仪像素坐标
    cv::Mat mapU = decoderV.phaseToPixels(absPhaseV);
    cv::Mat mapV = decoderH.phaseToPixels(absPhaseH);
    
    // 6. 获取投影仪对应点
    projectorPoints = getSubpixelValues(mapU, mapV, cameraPoints);
    
    return true;
}

bool GcPsCalib::findChessboardCornersEnhanced(const cv::Mat& image,  // 输入图像
                                              const cv::Size& patternSize,  // 棋盘格模式尺寸
                                              std::vector<cv::Point2f>& corners)  // 检测到的角点输出
{
    // 与 BasicDemo 参考实现保持一致，增加 CALIB_CB_FAST_CHECK 标志以提高检测效率
    bool found = cv::findChessboardCorners(image, patternSize, corners,
                                           cv::CALIB_CB_ADAPTIVE_THRESH | 
                                           cv::CALIB_CB_FAST_CHECK | 
                                           cv::CALIB_CB_NORMALIZE_IMAGE);
    
    if (found) {
        // 进行亚像素精化
        cv::cornerSubPix(image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
    }
    
    return found;
}

cv::Mat GcPsCalib::phaseToPixels(const cv::Mat& phaseMap,  // 相位图
                                 int projResolution,  // 投影仪分辨率
                                 double projFreq)  // 投影仪频率
{
    // 使用与原实现相同的方式
    cv::Mat pixelMap = cv::Mat::zeros(phaseMap.size(), CV_32F);

    // 将相位转换为像素坐标
    for (int y = 0; y < phaseMap.rows; ++y) {
        for (int x = 0; x < phaseMap.cols; ++x) {
            float phase = phaseMap.at<float>(y, x);
            // 转换为像素坐标
            pixelMap.at<float>(y, x) = (phase * projResolution) / (2 * CV_PI * projFreq);
        }
    }

    return pixelMap;
}

std::vector<cv::Point2f> GcPsCalib::getSubpixelValues(const cv::Mat& mapU,  // U方向映射图
                                 const cv::Mat& mapV,  // V方向映射图
                                 const std::vector<cv::Point2f>& points)  // 输入点
{
    // 委托给 StructuredLightDecoder 类进行处理
    std::cerr << "Warning: getSubpixelValues is deprecated. Use StructuredLightDecoder::getSubpixelValues instead." << std::endl;
    return StructuredLightDecoder::getSubpixelValues(mapU, mapV, points);
}

// 辅助函数：将矩阵转换为固定小数格式的字符串（不使用科学计数法）
static std::string matToFixedString(const cv::Mat& mat, int precision = 15)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    
    if (mat.rows == 1 || mat.cols == 1) {
        // 向量格式：确保列向量输出
        int rows = mat.rows * mat.cols;
        oss << "[";
        for (int i = 0; i < rows; i++) {
            double val = mat.at<double>(i);
            oss << val;
            if (i < rows - 1) oss << "; ";
        }
        oss << "]";
    } else {
        // 矩阵格式
        oss << "[";
        for (int i = 0; i < mat.rows; i++) {
            if (i > 0) oss << "; ";
            for (int j = 0; j < mat.cols; j++) {
                if (j > 0) oss << ", ";
                oss << mat.at<double>(i, j);
            }
        }
        oss << "]";
    }
    return oss.str();
}

bool GcPsCalib::saveCalibrationData(const CalibrationData& calibData,  // 标定数据
                                 const std::string& filePath)  // 文件路径
{
    cv::FileStorage fs(filePath, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "Error: Could not open file for writing: " << filePath << std::endl;
        return false;
    }
    
    // 写入相机内参矩阵（使用固定小数格式）
    fs << "camMatrix" << calibData.camMatrix;
    
    // 确保camDist是5x1列向量格式，使用固定小数格式
    cv::Mat camDistCol;
    if (calibData.camDist.rows == 1 && calibData.camDist.cols == 5) {
        // 如果是1x5行向量，转换为5x1列向量
        calibData.camDist.copyTo(camDistCol);
        camDistCol = camDistCol.t();
    } else if (calibData.camDist.rows == 5 && calibData.camDist.cols == 1) {
        calibData.camDist.copyTo(camDistCol);
    } else {
        // 其他情况，确保至少有5个元素
        calibData.camDist.copyTo(camDistCol);
    }
    
    // 使用固定小数格式写入camDist（5行1列）
    fs << "camDist" << camDistCol;
    
    // 写入投影仪内参和畸变（使用固定小数格式）
    fs << "projMatrix" << calibData.projMatrix;
    fs << "projDist" << calibData.projDist;
    fs << "R_CamToProj" << calibData.R_CamToProj;
    fs << "T_CamToProj" << calibData.T_CamToProj;
    
    // 写入整型参数
    fs << "camRes_width" << calibData.camRes.width;
    fs << "camRes_height" << calibData.camRes.height;
    fs << "projRes_width" << calibData.projRes.width;
    fs << "projRes_height" << calibData.projRes.height;
    fs << "proj_frequency" << calibData.projFrequency;
    
    // 写入浮点参数（确保固定小数格式）
    fs << "rmsProj" << calibData.rmsProj;
    fs << "rmsStereo" << calibData.rmsStereo;
    
    if (!calibData.R_BoardToCam.empty()) {
        fs << "R_BoardToCam" << calibData.R_BoardToCam;
        fs << "T_BoardToCam" << calibData.T_BoardToCam;
    }
    
    fs.release();
    
    return true;
}

bool GcPsCalib::loadCalibrationData(CalibrationData& calibData,  // 标定数据
                                 const std::string& filePath)  // 文件路径
{
    cv::FileStorage fs(filePath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Error: Could not open file for reading: " << filePath << std::endl;
        return false;
    }
    
    fs["cameraMatrix"] >> calibData.camMatrix;
    fs["distCoeffs"] >> calibData.camDist;
    fs["projMatrix"] >> calibData.projMatrix;
    fs["projDist"] >> calibData.projDist;
    fs["R_CamToProj"] >> calibData.R_CamToProj;
    fs["T_CamToProj"] >> calibData.T_CamToProj;
    
    int camWidth, camHeight, projWidth, projHeight;
    fs["imageSize_width"] >> camWidth;
    fs["imageSize_height"] >> camHeight;
    fs["projRes_width"] >> projWidth;
    fs["projRes_height"] >> projHeight;
    
    calibData.camRes = cv::Size(camWidth, camHeight);
    calibData.projRes = cv::Size(projWidth, projHeight);
    
    fs["proj_frequency"] >> calibData.projFrequency;
    fs["rmsProj"] >> calibData.rmsProj;
    fs["rmsStereo"] >> calibData.rmsStereo;
    
    // 检查是否存在坐标系转换数据
    fs["R_BoardToCam"] >> calibData.R_BoardToCam;
    fs["T_BoardToCam"] >> calibData.T_BoardToCam;
    
    fs.release();
    
    return true;
}

cv::Mat GcPsCalib::createModulationMask(const std::vector<cv::Mat>& phaseImages,  // 相移图像序列
                                        const cv::Mat& darkImage,  // 暗场图像
                                        double threshold)  // 阈值
{
    // 委托给 StructuredLightDecoder 类进行处理
    return StructuredLightDecoder::createModulationMask(phaseImages, darkImage, threshold);
}

void GcPsCalib::writeDebugReport(const std::string& filePath,
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
                                  const std::vector<std::vector<cv::Point2f>>& allProjectorPoints)
{
    // 创建 debug 目录
    std::string debugDir = filePath.substr(0, filePath.find_last_of("/\\"));
    std::filesystem::create_directories(debugDir);


    std::ofstream ofs(filePath);
    if (!ofs.is_open()) {
        std::cerr << "Error: Failed to open debug file for writing: " << filePath << std::endl;
        return;
    }

    // 获取当前时间
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
#ifdef _WIN32
    localtime_s(&tstruct, &now);
#else
    localtime_r(&now, &tstruct);
#endif
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tstruct);

    ofs << std::fixed << std::setprecision(6);
    ofs << "============================================================\n";
    ofs << "投影仪标定调试报告\n";
    ofs << "生成时间: " << buf << "\n";
    ofs << "============================================================\n\n";

    // 标定输入参数
    ofs << "【标定输入参数】\n";
    ofs << "- 棋盘格尺寸: " << chessboardSize.width << " x " << chessboardSize.height << "\n";
    ofs << "- 方格大小: " << squareSize << " mm\n";
    ofs << "- 投影仪分辨率: " << projSize.width << " x " << projSize.height << "\n";
    ofs << "- 投影仪频率: " << projFreq << " (周期数)\n";
    ofs << "- 格雷码位数: " << nGrayCode << "\n";
    ofs << "- 相移步数: " << nPhaseShift << "\n";
    ofs << "- 相机分辨率: " << camSize.width << " x " << camSize.height << "\n\n";

    // 流程4：Pose 处理统计
    ofs << "【流程4：Pose 处理统计】\n";
    ofs << "- 总 Pose 数量: " << totalPoses << "\n";
    ofs << "- 有效 Pose 数量: " << successCount << "\n";
    for (size_t i = 0; i < allCameraPoints.size() && i < 5; i++) {
        ofs << "- Pose " << i << " 角点数量: " << allCameraPoints[i].size() << "\n";
        if (!allCameraPoints[i].empty()) {
            ofs << "  相机角点样本 (前3个):\n";
            for (size_t j = 0; j < 3 && j < allCameraPoints[i].size(); j++) {
                ofs << "    (" << allCameraPoints[i][j].x << ", " << allCameraPoints[i][j].y << ")\n";
            }
        }
        if (!allProjectorPoints[i].empty()) {
            ofs << "  投影仪角点样本 (前3个):\n";
            for (size_t j = 0; j < 3 && j < allProjectorPoints[i].size(); j++) {
                ofs << "    (" << allProjectorPoints[i][j].x << ", " << allProjectorPoints[i][j].y << ")\n";
            }
        }
    }
    if (allCameraPoints.size() > 5) {
        ofs << "  ... 还有 " << (allCameraPoints.size() - 5) << " 个 Pose 未显示\n";
    }
    ofs << "\n";

    // 流程5：投影仪标定结果
    ofs << "【流程5：投影仪标定结果】\n";
    ofs << "- RMS 重投影误差: " << rmsProj << "\n";
    ofs << "- 投影仪内参矩阵:\n";
    for (int i = 0; i < projMatrix.rows; i++) {
        ofs << "  [";
        for (int j = 0; j < projMatrix.cols; j++) {
            ofs << projMatrix.at<double>(i, j);
            if (j < projMatrix.cols - 1) ofs << ", ";
        }
        ofs << "]\n";
    }
    ofs << "  (fx=" << projMatrix.at<double>(0,0) << ", fy=" << projMatrix.at<double>(1,1);
    ofs << ", cx=" << projMatrix.at<double>(0,2) << ", cy=" << projMatrix.at<double>(1,2) << ")\n";
    
    ofs << "- 投影仪畸变系数: [";
    for (int i = 0; i < projDist.rows; i++) {
        ofs << projDist.at<double>(i, 0);
        if (i < projDist.rows - 1) ofs << ", ";
    }
    ofs << "]\n";
    ofs << "  (k1=" << projDist.at<double>(0,0) << ", k2=" << projDist.at<double>(1,0);
    ofs << ", p1=" << projDist.at<double>(2,0) << ", p2=" << projDist.at<double>(3,0);
    ofs << ", k3=" << projDist.at<double>(4,0) << ")\n";

    ofs << "- 每个 Pose 的位姿 (旋转向量和平移向量):\n";
    for (size_t i = 0; i < rvecsProj.size() && i < 5; i++) {
        ofs << "  Pose " << i << ":\n";
        ofs << "    rvec: [" << rvecsProj[i].at<double>(0) << ", " 
            << rvecsProj[i].at<double>(1) << ", " << rvecsProj[i].at<double>(2) << "]\n";
        ofs << "    tvec: [" << tvecsProj[i].at<double>(0) << ", " 
            << tvecsProj[i].at<double>(1) << ", " << tvecsProj[i].at<double>(2) << "]\n";
    }
    if (rvecsProj.size() > 5) {
        ofs << "  ... 还有 " << (rvecsProj.size() - 5) << " 个 Pose 未显示\n";
    }
    ofs << "\n";

    // 流程6：立体标定结果
    ofs << "【流程6：立体标定结果】\n";
    ofs << "- RMS 误差: " << rmsStereo << "\n";
    ofs << "- 旋转矩阵 R (相机到投影仪):\n";
    for (int i = 0; i < R.rows; i++) {
        ofs << "  [";
        for (int j = 0; j < R.cols; j++) {
            ofs << R.at<double>(i, j);
            if (j < R.cols - 1) ofs << ", ";
        }
        ofs << "]\n";
    }
    ofs << "- 平移向量 T (相机到投影仪): [";
    for (int i = 0; i < T.rows; i++) {
        ofs << T.at<double>(i, 0);
        if (i < T.rows - 1) ofs << ", ";
    }
    ofs << "]\n";
    ofs << "- 本质矩阵 E:\n";
    for (int i = 0; i < E.rows; i++) {
        ofs << "  [";
        for (int j = 0; j < E.cols; j++) {
            ofs << E.at<double>(i, j);
            if (j < E.cols - 1) ofs << ", ";
        }
        ofs << "]\n";
    }
    ofs << "- 基础矩阵 F:\n";
    for (int i = 0; i < F.rows; i++) {
        ofs << "  [";
        for (int j = 0; j < F.cols; j++) {
            ofs << F.at<double>(i, j);
            if (j < F.cols - 1) ofs << ", ";
        }
        ofs << "]\n";
    }
    ofs << "\n";

    // 相机参数（输入）
    ofs << "【相机参数（输入）】\n";
    ofs << "- 相机内参矩阵:\n";
    for (int i = 0; i < camMatrix.rows; i++) {
        ofs << "  [";
        for (int j = 0; j < camMatrix.cols; j++) {
            ofs << camMatrix.at<double>(i, j);
            if (j < camMatrix.cols - 1) ofs << ", ";
        }
        ofs << "]\n";
    }
    ofs << "- 相机畸变系数: [";
    for (int i = 0; i < camDist.rows; i++) {
        ofs << camDist.at<double>(i, 0);
        if (i < camDist.rows - 1) ofs << ", ";
    }
    ofs << "]\n";

    ofs << "\n============================================================\n";
    ofs.close();
    
    std::cout << "调试报告已保存到: " << filePath << std::endl;
}
