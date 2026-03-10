#include "calibrationutils.h"
#include <QCoreApplication>
#include <QDateTime>
#include <QFile>

// ==================== 路径管理 ====================

QString CalibrationUtils::getBaseCalibrationPath()
{
    QString appPath = QCoreApplication::applicationDirPath();
    QString basePath = appPath + "/CalibrateData";
    createDirectoryIfNotExists(basePath);
    return basePath;
}

QString CalibrationUtils::getDevicePath(bool isLeftDevice)
{
    QString basePath = getBaseCalibrationPath();
    QString deviceFolder = isLeftDevice ? "LeftSL" : "RightSL";
    QString devicePath = basePath + "/" + deviceFolder;
    createDirectoryIfNotExists(devicePath);
    return devicePath;
}

QString CalibrationUtils::getCalibrationTypePath(bool isLeftDevice, const QString& calibType)
{
    QString devicePath = getDevicePath(isLeftDevice);
    QString typePath = devicePath + "/" + calibType;
    createDirectoryIfNotExists(typePath);
    return typePath;
}

bool CalibrationUtils::createDirectoryIfNotExists(const QString& path)
{
    QDir dir(path);
    if (!dir.exists()) {
        return dir.mkpath(".");
    }
    return true;
}

// ==================== 投影仪标定辅助 ====================

int CalibrationUtils::getNextPoseNumber(const QString& projectorCalibPath)
{
    QDir dir(projectorCalibPath);
    if (!dir.exists()) {
        return 1;
    }

    QStringList filters;
    filters << "Pose_*.bmp" << "Pose_*.jpg" << "Pose_*.png";
    QFileInfoList fileList = dir.entryInfoList(filters, QDir::Files);

    if (fileList.isEmpty()) {
        return 1;
    }

    QRegularExpression poseRegex("^Pose_(\\d+)_");
    int maxPoseNumber = 0;

    for (const QFileInfo& fi : fileList) {
        QRegularExpressionMatch match = poseRegex.match(fi.fileName());
        if (match.hasMatch()) {
            int poseNum = match.captured(1).toInt();
            if (poseNum > maxPoseNumber) {
                maxPoseNumber = poseNum;
            }
        }
    }

    return maxPoseNumber + 1;
}

QString CalibrationUtils::formatPoseNumber(int poseNumber)
{
    return QString("%1").arg(poseNumber, 2, 10, QLatin1Char('0'));
}

// ==================== 图像保存 ====================

bool CalibrationUtils::saveCalibrationImage(const cv::Mat& image, 
                                             const QString& calibType, 
                                             int imageIndex,
                                             bool isLeftDevice)
{
    if (image.empty()) {
        return false;
    }

    QString typePath = getCalibrationTypePath(isLeftDevice, calibType);
    
    QString fileName = QString("image_%1_%2.jpg")
                      .arg(imageIndex, 3, 10, QLatin1Char('0'))
                      .arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
    QString fullPath = typePath + "/" + fileName;

    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(95);

    try {
        return cv::imwrite(fullPath.toStdString(), image, compression_params);
    } catch (const cv::Exception&) {
        return false;
    }
}

bool CalibrationUtils::saveCornerDetectionResult(const QString& imagePath,
                                                  const cv::Mat& image,
                                                  const std::vector<cv::Point2f>& corners,
                                                  bool detectionSuccess,
                                                  int patternCols,
                                                  int patternRows)
{
    QFileInfo imageFileInfo(imagePath);
    QString imageDirPath = imageFileInfo.absolutePath();
    QString resultDirPath = imageDirPath + "/result";
    
    QDir resultDir(resultDirPath);
    if (!resultDir.exists()) {
        if (!resultDir.mkpath(".")) {
            return false;
        }
    }
    
    cv::Mat resultImage;
    if (image.channels() == 1) {
        cv::cvtColor(image, resultImage, cv::COLOR_GRAY2BGR);
    } else {
        resultImage = image.clone();
    }
    
    QString baseName = imageFileInfo.completeBaseName();
    QString resultFileName = QString("%1_corners.jpg").arg(baseName);
    QString resultFilePath = resultDirPath + "/" + resultFileName;
    
    if (detectionSuccess && !corners.empty()) {
        cv::drawChessboardCorners(resultImage, 
                                   cv::Size(patternCols, patternRows),
                                   corners, 
                                   true);
        
        std::string statusText = "Corners: " + std::to_string(corners.size()) + " [OK]";
        cv::putText(resultImage, statusText, 
                    cv::Point(10, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, 
                    cv::Scalar(0, 255, 0), 2);
    } else {
        cv::putText(resultImage, "Corner Detection FAILED", 
                    cv::Point(10, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, 
                    cv::Scalar(0, 0, 255), 2);
    }
    
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(95);
    
    try {
        return cv::imwrite(resultFilePath.toStdString(), resultImage, compression_params);
    } catch (const cv::Exception&) {
        return false;
    }
}

bool CalibrationUtils::saveCoordinateImage(const cv::Mat& image, 
                                           bool isLeftDevice, 
                                           QString& savedImagePath)
{
    if (image.empty()) {
        savedImagePath.clear();
        return false;
    }

    QString coordPath = getCalibrationTypePath(isLeftDevice, "Coordinate");
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    QString fileName = QString("coordinate_image_%1.bmp").arg(timestamp);
    savedImagePath = coordPath + "/" + fileName;

    try {
        bool success = cv::imwrite(savedImagePath.toStdString(), image);
        if (!success) {
            savedImagePath.clear();
        }
        return success;
    } catch (const cv::Exception&) {
        savedImagePath.clear();
        return false;
    }
}

// ==================== 目录清理 ====================

bool CalibrationUtils::clearCoordinateDirectory(bool isLeftDevice)
{
    QString coordPath = getCalibrationTypePath(isLeftDevice, "Coordinate");
    QStringList imageFilters = getImageFilters();

    // 清空 Coordinate 主目录中的图像
    QDir dir(coordPath);
    QFileInfoList imageFiles = dir.entryInfoList(imageFilters, QDir::Files);
    for (const QFileInfo& fi : imageFiles) {
        QFile::remove(fi.absoluteFilePath());
    }

    // 清空 Coordinate/result 子目录中的图像
    QString resultPath = coordPath + "/result";
    QDir resultDir(resultPath);
    if (resultDir.exists()) {
        QFileInfoList resultFiles = resultDir.entryInfoList(imageFilters, QDir::Files);
        for (const QFileInfo& fi : resultFiles) {
            QFile::remove(fi.absoluteFilePath());
        }
    } else {
        resultDir.mkpath(".");
    }

    return true;
}

int CalibrationUtils::clearCalibrationDirectory(bool isLeftDevice, const QString& calibType)
{
    QString calibPath = getCalibrationTypePath(isLeftDevice, calibType);
    QStringList imageFilters = getImageFilters();

    QDir dir(calibPath);
    QFileInfoList imageFiles = dir.entryInfoList(imageFilters, QDir::Files);
    
    int deletedCount = 0;
    for (const QFileInfo& fi : imageFiles) {
        if (QFile::remove(fi.absoluteFilePath())) {
            deletedCount++;
        }
    }

    // 同时清理 result 子目录
    QString resultPath = calibPath + "/result";
    QDir resultDir(resultPath);
    if (resultDir.exists()) {
        QFileInfoList resultFiles = resultDir.entryInfoList(imageFilters, QDir::Files);
        for (const QFileInfo& fi : resultFiles) {
            if (QFile::remove(fi.absoluteFilePath())) {
                deletedCount++;
            }
        }
    }

    return deletedCount;
}

// ==================== 文件查找 ====================

QFileInfoList CalibrationUtils::getCalibrationImages(bool isLeftDevice, const QString& calibType)
{
    QString calibPath = getCalibrationTypePath(isLeftDevice, calibType);
    QDir dir(calibPath);
    
    if (!dir.exists()) {
        return QFileInfoList();
    }
    
    return dir.entryInfoList(getImageFilters(), QDir::Files, QDir::Name);
}

// ==================== 私有辅助 ====================

QStringList CalibrationUtils::getImageFilters()
{
    return QStringList() << "*.jpg" << "*.bmp" << "*.png" << "*.tiff" << "*.tif";
}
