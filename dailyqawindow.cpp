#include "dailyqawindow.h"
#include "ui_dailyqawindow.h"
#include "ProjectorController.h"
#include "CameraController.h"
#include "calibrationutils.h"
#include "gcpscalib.h"
#include "qadbmanager.h"

#include <QMessageBox>
#include <QDateTime>
#include <QDate>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QFileInfoList>
#include <QMap>
#include <QRegularExpression>
#include <QThread>
#include <QApplication>
#include <QTextCursor>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

namespace {
constexpr float kQACameraExposureUs = 45000.0f;
constexpr float kQACameraGain = 0.0f;
constexpr int kQAProjectorExposureUs = 500000;
constexpr int kQAProjectorPattern = 6;
constexpr int kQAProjectorTrigger = 0;  // 0 = 内触发
constexpr int kQACameraTriggerMode = 1;  // 1 = 外触发
constexpr const char* kQACameraTriggerSource = "Line0";
constexpr unsigned int kQACameraTriggerActivation = 1;  // 1 = 下降沿
constexpr const char* kQACameraTriggerSourceReadback = "0";  // Line0 enum value
constexpr int kImageTimeoutMs = 3000;
constexpr unsigned int kImageBufferSize = 5 * 1024 * 1024;
constexpr int kExpectedPoseImageCount = 20;
constexpr int kDefaultProjFrequency = 16;
constexpr int kDefaultProjWidth = 912;
constexpr int kDefaultProjHeight = 1140;
constexpr int kBoardCols = 20;
constexpr int kBoardRows = 20;
constexpr float kBoardSquareSizeMm = 10.0f;
constexpr int kGrayCodeCount = 5;
constexpr int kPhaseShiftCount = 4;
constexpr int kMinValidPointCount = 320;
constexpr const char* kFixedPoseTransformName = "center+inv";

struct ErrorStats {
    double mean = -1.0;
    double rms = -1.0;
    double p95 = -1.0;
    double max = -1.0;
    int count = 0;
};

double calculatePercentile(std::vector<double> values, double percentile)
{
    if (values.empty()) {
        return -1.0;
    }

    std::sort(values.begin(), values.end());
    const double position = percentile * static_cast<double>(values.size() - 1);
    const size_t lowIndex = static_cast<size_t>(std::floor(position));
    const size_t highIndex = static_cast<size_t>(std::ceil(position));
    if (lowIndex == highIndex) {
        return values[lowIndex];
    }

    const double ratio = position - static_cast<double>(lowIndex);
    return values[lowIndex] * (1.0 - ratio) + values[highIndex] * ratio;
}

ErrorStats calculateErrorStats(const std::vector<cv::Point2f>& measured,
                               const std::vector<cv::Point2f>& projected)
{
    ErrorStats stats;
    const size_t pointCount = std::min(measured.size(), projected.size());
    if (pointCount == 0) {
        return stats;
    }

    double sum = 0.0;
    double sumSquare = 0.0;
    double maxValue = 0.0;
    std::vector<double> errors;
    errors.reserve(pointCount);
    for (size_t i = 0; i < pointCount; ++i) {
        const double err = cv::norm(measured[i] - projected[i]);
        errors.push_back(err);
        sum += err;
        sumSquare += err * err;
        maxValue = std::max(maxValue, err);
    }

    stats.count = static_cast<int>(pointCount);
    stats.mean = sum / static_cast<double>(pointCount);
    stats.rms = std::sqrt(sumSquare / static_cast<double>(pointCount));
    stats.p95 = calculatePercentile(errors, 0.95);
    stats.max = maxValue;
    return stats;
}

void centerBoardPointsInPlace(std::vector<cv::Point3f>& boardPoints,
                              int boardCols,
                              int boardRows,
                              float squareSizeMm)
{
    if (boardPoints.empty()) {
        return;
    }

    const float offsetX = (static_cast<float>(boardCols) - 1.0f) * squareSizeMm * 0.5f;
    const float offsetY = (static_cast<float>(boardRows) - 1.0f) * squareSizeMm * 0.5f;
    for (cv::Point3f& point : boardPoints) {
        point.x -= offsetX;
        point.y -= offsetY;
    }
}

bool toRotation3x3(const cv::Mat& input, cv::Mat& output)
{
    if (input.empty()) {
        return false;
    }
    input.convertTo(output, CV_64F);
    return output.rows == 3 && output.cols == 3;
}

bool toVector3x1(const cv::Mat& input, cv::Mat& output)
{
    if (input.empty()) {
        return false;
    }

    cv::Mat temp;
    input.convertTo(temp, CV_64F);
    if (temp.rows == 1 && temp.cols == 3) {
        temp = temp.t();
    } else if (temp.total() == 3 && (temp.rows != 3 || temp.cols != 1)) {
        temp = temp.reshape(1, 3);
    }

    if (temp.rows != 3 || temp.cols != 1) {
        return false;
    }
    output = temp;
    return true;
}

QString findPoseWhiteImage(const QString& imageDirPath,
                           const QString& poseNumber)
{
    QDir imageDir(imageDirPath);
    if (!imageDir.exists()) {
        return QString();
    }

    const QStringList filters{
        QString("Pose_%1_Img_01_*.bmp").arg(poseNumber),
        QString("Pose_%1_Img_01_*.png").arg(poseNumber),
        QString("Pose_%1_Img_01_*.jpg").arg(poseNumber),
        QString("Pose_%1_Img_01_*.jpeg").arg(poseNumber),
        QString("Pose_%1_Img_01_*.tif").arg(poseNumber),
        QString("Pose_%1_Img_01_*.tiff").arg(poseNumber)
    };
    const QFileInfoList files = imageDir.entryInfoList(filters, QDir::Files, QDir::Name);
    if (files.isEmpty()) {
        return QString();
    }

    const QRegularExpression whiteRegex("_white", QRegularExpression::CaseInsensitiveOption);
    for (const QFileInfo& fileInfo : files) {
        if (whiteRegex.match(fileInfo.fileName()).hasMatch()) {
            return fileInfo.absoluteFilePath();
        }
    }
    return files.first().absoluteFilePath();
}

QString extractPoseIdFromImagePath(const QString& imagePath)
{
    const QString fileName = QFileInfo(imagePath).fileName();
    const QRegularExpression fileRegex(
        "^Pose_(\\d+)_Img_\\d{2}_.*\\.(bmp|png|jpg|jpeg|tif|tiff)$",
        QRegularExpression::CaseInsensitiveOption);
    const QRegularExpressionMatch match = fileRegex.match(fileName);
    if (!match.hasMatch()) {
        return QString();
    }
    return match.captured(1);
}

QString patternNameForIndex(int index)
{
    if (index < 1) {
        return "White";
    }
    if (index < 6) {
        return QString("GC_H_%1").arg(index - 1);
    }
    if (index < 11) {
        return QString("GC_V_%1").arg(index - 6);
    }
    if (index < 15) {
        return QString("PS_V_%1").arg(index - 11);
    }
    return QString("PS_H_%1").arg(index - 15);
}
}  // namespace

DailyQAWindow::DailyQAWindow(const DeviceConfig& config, QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::DailyQAWindow)
    , m_deviceConfig(config)
    , m_projectorController(new ProjectorController())
    , m_cameraController(new CCameraController())
    , m_isRunning(false)
{
    ui->setupUi(this);

    connect(ui->pushButton_StartQA, &QPushButton::clicked,
            this, &DailyQAWindow::onStartQAClicked);
    connect(ui->pushButton_StartCompute, &QPushButton::clicked,
            this, &DailyQAWindow::onStartComputeClicked);

    if (!m_projectorController->loadDll()) {
        QMessageBox::warning(this, tr("警告"), tr("投影仪控制初始化失败，日检可能无法执行。"));
    }

    updateDeviceInfo();

    logMessage("每日 QA 检测窗口已启动");
    logMessage(QString("日检固定参数: 相机曝光=%1us, 相机增益=%2, 投影曝光=%3us, 图案=%4, 投影触发=内触发, 相机触发=外触发(Line0,下降沿)")
               .arg(kQACameraExposureUs, 0, 'f', 0)
               .arg(kQACameraGain, 0, 'f', 0)
               .arg(kQAProjectorExposureUs)
               .arg(kQAProjectorPattern));
}

DailyQAWindow::~DailyQAWindow()
{
    delete m_cameraController;
    m_cameraController = nullptr;

    delete m_projectorController;
    m_projectorController = nullptr;

    delete ui;
}

void DailyQAWindow::updateDeviceInfo()
{
    QString info;
    info += QString("左侧相机: %1\n").arg(m_deviceConfig.leftCameraSN.isEmpty() ? "未设置" : m_deviceConfig.leftCameraSN);
    info += QString("右侧相机: %1\n").arg(m_deviceConfig.rightCameraSN.isEmpty() ? "未设置" : m_deviceConfig.rightCameraSN);
    info += QString("左侧投影仪: %1\n").arg(m_deviceConfig.leftProjectorTag);
    info += QString("右侧投影仪: %1\n").arg(m_deviceConfig.rightProjectorTag);
    info += "日检数据路径: QAData/{LeftSL|RightSL}/{Projector|Camera}";
    
    ui->label_DeviceInfo->setText(info);
}

void DailyQAWindow::logMessage(const QString& message)
{
    const QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    ui->textEdit_QALog->append(QString("[%1] %2").arg(timestamp).arg(message));

    QTextCursor cursor = ui->textEdit_QALog->textCursor();
    cursor.movePosition(QTextCursor::End);
    while (ui->textEdit_QALog->document()->blockCount() > 200) {
        cursor.movePosition(QTextCursor::Start);
        cursor.select(QTextCursor::BlockUnderCursor);
        cursor.removeSelectedText();
        cursor.deleteChar();
    }
}

QString DailyQAWindow::getQABasePath() const
{
    const QString basePath = QCoreApplication::applicationDirPath() + "/QAData";
    QDir().mkpath(basePath);
    return basePath;
}

QString DailyQAWindow::getQATypedPath(const QString& sideFolder, const QString& typeFolder) const
{
    const QString path = QString("%1/%2/%3")
                             .arg(getQABasePath())
                             .arg(sideFolder)
                             .arg(typeFolder);
    QDir().mkpath(path);
    return path;
}

int DailyQAWindow::getNextPoseNumber(const QString& projectorPath) const
{
    return CalibrationUtils::getNextPoseNumber(projectorPath);
}

QString DailyQAWindow::formatPoseNumber(int poseNumber) const
{
    return CalibrationUtils::formatPoseNumber(poseNumber);
}

bool DailyQAWindow::saveFrame(void* cameraHandle,
                              unsigned char* imageBuffer,
                              unsigned int imageBufferSize,
                              const QString& filePath,
                              int timeoutMs)
{
    MV_FRAME_OUT_INFO_EX frameInfo = {0};
    if (!m_cameraController->GetImage(cameraHandle, imageBuffer, imageBufferSize, &frameInfo, timeoutMs) ||
        frameInfo.nHeight == 0 || frameInfo.nWidth == 0) {
        logMessage(QString("取图失败: %1 (超时%2ms)").arg(filePath).arg(timeoutMs));
        return false;
    }

    cv::Mat image(frameInfo.nHeight, frameInfo.nWidth, CV_8UC1, imageBuffer);
    if (!cv::imwrite(filePath.toStdString(), image)) {
        logMessage(QString("保存图像失败: %1").arg(filePath));
        return false;
    }

    return true;
}

bool DailyQAWindow::copyWhiteFrameToCameraPath(const QString& projectorFilePath, const QString& cameraPath)
{
    const QFileInfo fi(projectorFilePath);
    const QString targetPath = cameraPath + "/" + fi.fileName();
    if (QFile::exists(targetPath)) {
        QFile::remove(targetPath);
    }
    if (!QFile::copy(projectorFilePath, targetPath)) {
        logMessage(QString("复制 White 图失败: %1 -> %2")
                       .arg(projectorFilePath)
                       .arg(targetPath));
        return false;
    }

    logMessage(QString("已复制 White 图到相机目录: %1").arg(targetPath));
    return true;
}

bool DailyQAWindow::captureDarkImage(const QString& projectorTag,
                                     void* cameraHandle,
                                     unsigned char* imageBuffer,
                                     unsigned int imageBufferSize,
                                     const QString& projectorPath,
                                     int poseNumber)
{
    m_projectorController->pauseProjector(projectorTag.toStdString());
    QThread::msleep(100);

    if (!m_cameraController->SetTriggerMode(cameraHandle, 1) ||
        !m_cameraController->SetTriggerSource(cameraHandle, "Software")) {
        logMessage("设置软件触发失败");
        return false;
    }

    if (!m_cameraController->StartImageCapture(cameraHandle)) {
        logMessage("启动相机失败(软件触发)");
        return false;
    }

    QThread::msleep(80);
    if (!m_cameraController->TriggerSoftware(cameraHandle)) {
        logMessage("发送软件触发失败");
        m_cameraController->StopImageCapture(cameraHandle);
        return false;
    }

    const QString darkPath = QString("%1/Pose_%2_Img_00_Dark.bmp")
                                 .arg(projectorPath)
                                 .arg(formatPoseNumber(poseNumber));
    const bool ok = saveFrame(cameraHandle, imageBuffer, imageBufferSize, darkPath, kImageTimeoutMs);
    m_cameraController->StopImageCapture(cameraHandle);
    if (ok) {
        logMessage(QString("[Pose %1] 暗场图已保存: %2")
                       .arg(formatPoseNumber(poseNumber))
                       .arg(darkPath));
    }
    return ok;
}

bool DailyQAWindow::capturePatternSequence(const QString& projectorTag,
                                           void* cameraHandle,
                                           unsigned char* imageBuffer,
                                           unsigned int imageBufferSize,
                                           const QString& projectorPath,
                                           const QString& cameraPath,
                                           int poseNumber)
{
    if (!m_cameraController->SetTriggerMode(cameraHandle, kQACameraTriggerMode)) {
        logMessage("设置相机外触发模式失败");
        return false;
    }
    if (!m_cameraController->SetTriggerSource(cameraHandle, kQACameraTriggerSource)) {
        logMessage("设置相机触发源Line0失败");
        return false;
    }
    if (!m_cameraController->SetTriggerActivation(cameraHandle, kQACameraTriggerActivation)) {
        logMessage("设置相机触发边沿为下降沿失败");
        return false;
    }

    int triggerMode = -1;
    unsigned int triggerActivation = 999;
    char triggerSource[64] = {0};
    if (!m_cameraController->GetTriggerMode(cameraHandle, triggerMode) ||
        triggerMode != kQACameraTriggerMode) {
        logMessage(QString("相机触发模式校验失败: current=%1, expected=%2")
                       .arg(triggerMode)
                       .arg(kQACameraTriggerMode));
        return false;
    }
    if (!m_cameraController->GetTriggerSource(cameraHandle, triggerSource, sizeof(triggerSource)) ||
        QString::fromLatin1(triggerSource) != QString::fromLatin1(kQACameraTriggerSourceReadback)) {
        logMessage(QString("相机触发源校验失败: current=%1, expected=%2(Line0)")
                       .arg(QString::fromLatin1(triggerSource))
                       .arg(kQACameraTriggerSourceReadback));
        return false;
    }
    if (!m_cameraController->GetTriggerActivation(cameraHandle, triggerActivation) ||
        triggerActivation != kQACameraTriggerActivation) {
        logMessage(QString("相机触发边沿校验失败: current=%1, expected=%2(下降沿)")
                       .arg(triggerActivation)
                       .arg(kQACameraTriggerActivation));
        return false;
    }

    logMessage("相机触发参数已确认: 外触发 + Line0 + 下降沿");

    if (!m_cameraController->StartImageCapture(cameraHandle)) {
        logMessage("启动相机失败(硬触发)");
        return false;
    }

    if (!m_projectorController->sendAndPlayProjector(projectorTag.toStdString(),
                                                     kQAProjectorTrigger,
                                                     kQAProjectorPattern,
                                                     kQAProjectorExposureUs)) {
        logMessage("发送投影序列失败");
        m_cameraController->StopImageCapture(cameraHandle);
        return false;
    }

    for (int i = 0; i < 19; ++i) {
        const QString patternName = patternNameForIndex(i);
        const QString imagePath = QString("%1/Pose_%2_Img_%3_%4.bmp")
                                      .arg(projectorPath)
                                      .arg(formatPoseNumber(poseNumber))
                                      .arg(i + 1, 2, 10, QLatin1Char('0'))
                                      .arg(patternName);

        if (!saveFrame(cameraHandle, imageBuffer, imageBufferSize, imagePath, kImageTimeoutMs)) {
            m_projectorController->pauseProjector(projectorTag.toStdString());
            m_cameraController->StopImageCapture(cameraHandle);
            return false;
        }

        logMessage(QString("[Pose %1] 图像 %2/19 已保存: %3")
                       .arg(formatPoseNumber(poseNumber))
                       .arg(i + 1)
                       .arg(imagePath));

        if (patternName == "White" && !copyWhiteFrameToCameraPath(imagePath, cameraPath)) {
            m_projectorController->pauseProjector(projectorTag.toStdString());
            m_cameraController->StopImageCapture(cameraHandle);
            return false;
        }

        QApplication::processEvents();
    }

    m_projectorController->pauseProjector(projectorTag.toStdString());
    m_cameraController->StopImageCapture(cameraHandle);
    return true;
}

DailyQAWindow::SideRunResult DailyQAWindow::runSideDailyQA(const SideConfig& side)
{
    logMessage(QString("开始检查 %1 结构光设备连接").arg(side.displayName));

    if (side.cameraSN.isEmpty() || side.projectorTag.isEmpty()) {
        logMessage(QString("%1 配置不完整，已跳过").arg(side.displayName));
        return SideRunResult::Skipped;
    }

    if (!m_projectorController->checkConnection(side.projectorTag.toStdString())) {
        logMessage(QString("%1 投影仪未连接，已跳过").arg(side.displayName));
        return SideRunResult::Skipped;
    }

    void* cameraHandle = m_cameraController->OpenCameraBySN(side.cameraSN.toStdString().c_str());
    if (!cameraHandle) {
        logMessage(QString("%1 相机未连接或打开失败，已跳过").arg(side.displayName));
        return SideRunResult::Skipped;
    }

    if (!m_projectorController->initProjector(side.projectorTag.toStdString(), 7)) {
        logMessage(QString("%1 投影仪初始化失败").arg(side.displayName));
        m_cameraController->CloseCamera(cameraHandle);
        return SideRunResult::Failed;
    }

    if (!m_cameraController->SetExposureTime(cameraHandle, kQACameraExposureUs) ||
        !m_cameraController->SetGain(cameraHandle, kQACameraGain)) {
        logMessage(QString("%1 相机参数设置失败(曝光=%2us, 增益=%3)")
                       .arg(side.displayName)
                       .arg(kQACameraExposureUs, 0, 'f', 0)
                       .arg(kQACameraGain, 0, 'f', 0));
        m_projectorController->pauseProjector(side.projectorTag.toStdString());
        m_cameraController->CloseCamera(cameraHandle);
        return SideRunResult::Failed;
    }

    const QString projectorPath = getQATypedPath(side.folderName, "Projector");
    const QString cameraPath = getQATypedPath(side.folderName, "Camera");
    const int poseNumber = getNextPoseNumber(projectorPath);
    std::vector<unsigned char> imageBuffer(kImageBufferSize);

    logMessage(QString("%1 参数已固定: CamExp=%2us CamGain=%3 ProjExp=%4us Pattern=%5 ProjTrigger=内触发 CamTrigger=外触发(Line0,下降沿)")
                   .arg(side.displayName)
                   .arg(kQACameraExposureUs, 0, 'f', 0)
                   .arg(kQACameraGain, 0, 'f', 0)
                   .arg(kQAProjectorExposureUs)
                   .arg(kQAProjectorPattern));
    logMessage(QString("%1 开始采集 Pose %2")
                   .arg(side.displayName)
                   .arg(formatPoseNumber(poseNumber)));

    if (!captureDarkImage(side.projectorTag,
                          cameraHandle,
                          imageBuffer.data(),
                          static_cast<unsigned int>(imageBuffer.size()),
                          projectorPath,
                          poseNumber)) {
        m_projectorController->pauseProjector(side.projectorTag.toStdString());
        m_cameraController->StopImageCapture(cameraHandle);
        m_cameraController->CloseCamera(cameraHandle);
        return SideRunResult::Failed;
    }

    if (!capturePatternSequence(side.projectorTag,
                                cameraHandle,
                                imageBuffer.data(),
                                static_cast<unsigned int>(imageBuffer.size()),
                                projectorPath,
                                cameraPath,
                                poseNumber)) {
        m_projectorController->pauseProjector(side.projectorTag.toStdString());
        m_cameraController->StopImageCapture(cameraHandle);
        m_cameraController->CloseCamera(cameraHandle);
        return SideRunResult::Failed;
    }

    m_cameraController->CloseCamera(cameraHandle);
    logMessage(QString("%1 日检采图完成: Pose %2 (共20张, White已复制到Camera目录)")
                   .arg(side.displayName)
                   .arg(formatPoseNumber(poseNumber)));
    return SideRunResult::Success;
}

bool DailyQAWindow::collectSinglePoseImages(const QString& projectorPath,
                                            QString& poseNumber,
                                            QStringList& orderedPoseFiles,
                                            QString& reason) const
{
    poseNumber.clear();
    orderedPoseFiles.clear();
    reason.clear();

    QDir dir(projectorPath);
    if (!dir.exists()) {
        reason = QString("目录不存在: %1").arg(projectorPath);
        return false;
    }

    const QStringList filters{
        "Pose_*.bmp", "Pose_*.png", "Pose_*.jpg",
        "Pose_*.jpeg", "Pose_*.tif", "Pose_*.tiff"
    };
    const QFileInfoList fileList = dir.entryInfoList(filters, QDir::Files, QDir::Name);
    if (fileList.isEmpty()) {
        reason = "未找到任何 Pose 图像";
        return false;
    }

    const QRegularExpression fileRegex(
        "^Pose_(\\d+)_Img_(\\d{2})_.*\\.(bmp|png|jpg|jpeg|tif|tiff)$",
        QRegularExpression::CaseInsensitiveOption);

    QMap<QString, QMap<int, QString>> poseImageMap;
    for (const QFileInfo& fi : fileList) {
        const QRegularExpressionMatch match = fileRegex.match(fi.fileName());
        if (!match.hasMatch()) {
            continue;
        }

        const QString poseId = match.captured(1);
        const int imageIndex = match.captured(2).toInt();
        if (imageIndex < 0 || imageIndex >= kExpectedPoseImageCount) {
            continue;
        }

        QMap<int, QString>& indexMap = poseImageMap[poseId];
        if (indexMap.contains(imageIndex)) {
            reason = QString("Pose_%1 存在重复图像序号 Img_%2")
                         .arg(poseId)
                         .arg(imageIndex, 2, 10, QLatin1Char('0'));
            return false;
        }
        indexMap.insert(imageIndex, fi.absoluteFilePath());
    }

    if (poseImageMap.isEmpty()) {
        reason = "未找到符合命名规则的 Pose 图像";
        return false;
    }

    QString latestPoseId;
    int latestPoseNumber = -1;
    for (auto it = poseImageMap.constBegin(); it != poseImageMap.constEnd(); ++it) {
        bool ok = false;
        const int currentPoseNumber = it.key().toInt(&ok);
        if (!ok) {
            continue;
        }
        if (currentPoseNumber > latestPoseNumber) {
            latestPoseNumber = currentPoseNumber;
            latestPoseId = it.key();
        }
    }
    if (latestPoseNumber < 0 || latestPoseId.isEmpty()) {
        reason = "未找到可解析的 Pose 编号";
        return false;
    }

    poseNumber = latestPoseId;
    const QMap<int, QString> imageMap = poseImageMap.value(latestPoseId);

    QStringList missingIndices;
    for (int i = 0; i < kExpectedPoseImageCount; ++i) {
        if (!imageMap.contains(i)) {
            missingIndices << QString("%1").arg(i, 2, 10, QLatin1Char('0'));
        }
    }
    if (!missingIndices.isEmpty()) {
        reason = QString("Pose_%1 图像不完整，缺少 Img_%2")
                     .arg(poseNumber)
                     .arg(missingIndices.join(", "));
        return false;
    }

    for (int i = 0; i < kExpectedPoseImageCount; ++i) {
        orderedPoseFiles << imageMap.value(i);
    }
    return true;
}

QString DailyQAWindow::getQACalibFilePath(const SideConfig& side) const
{
    const QString calibFileName =
        side.folderName.compare("LeftSL", Qt::CaseInsensitive) == 0
            ? "SLCalibParams_Left.xml"
            : "SLCalibParams_Right.xml";
    return QString("%1/%2/%3").arg(getQABasePath()).arg(side.folderName).arg(calibFileName);
}

DailyQAWindow::SideComputeResult DailyQAWindow::computeSideProjectionError(const SideConfig& side,
                                                                            QString& summaryMessage,
                                                                            SideComputeData& computeData)
{
    summaryMessage.clear();
    computeData = SideComputeData{};

    const QString projectorPath = getQATypedPath(side.folderName, "Projector");
    const QString cameraPath = getQATypedPath(side.folderName, "Camera");

    QString poseNumber;
    QStringList orderedPoseFiles;
    QString reason;
    if (!collectSinglePoseImages(projectorPath, poseNumber, orderedPoseFiles, reason)) {
        summaryMessage = QString("%1: 跳过计算 - %2").arg(side.displayName, reason);
        logMessage(summaryMessage);
        return SideComputeResult::Skipped;
    }
    for (const QString& imagePath : orderedPoseFiles) {
        const QString imagePoseId = extractPoseIdFromImagePath(imagePath);
        if (imagePoseId != poseNumber) {
            summaryMessage = QString("%1: 计算失败 - 计算图像Pose不一致(期望 Pose_%2, 实际文件 %3)")
                                 .arg(side.displayName)
                                 .arg(poseNumber)
                                 .arg(QFileInfo(imagePath).fileName());
            logMessage(summaryMessage);
            return SideComputeResult::Failed;
        }
    }
    logMessage(QString("%1: 本次计算使用 Pose_%2").arg(side.displayName, poseNumber));
    computeData.poseNumber = poseNumber;

    const QString calibFilePath = getQACalibFilePath(side);
    if (!QFileInfo::exists(calibFilePath)) {
        summaryMessage = QString("%1: 计算失败 - 未找到标定文件 %2").arg(side.displayName, calibFilePath);
        logMessage(summaryMessage);
        return SideComputeResult::Failed;
    }

    GcPsCalib calibrator;
    CalibrationData calibData;
    if (!calibrator.loadCalibrationData(calibData, calibFilePath.toStdString())) {
        summaryMessage = QString("%1: 计算失败 - 读取标定文件失败").arg(side.displayName);
        logMessage(summaryMessage);
        return SideComputeResult::Failed;
    }

    if (calibData.camMatrix.empty() || calibData.camDist.empty() ||
        calibData.projMatrix.empty() || calibData.projDist.empty() ||
        calibData.R_CamToProj.empty() || calibData.T_CamToProj.empty() ||
        calibData.R_BoardToCam.empty() || calibData.T_BoardToCam.empty()) {
        summaryMessage = QString("%1: 计算失败 - 标定参数不完整(缺少固定板金标准位姿或外参)").arg(side.displayName);
        logMessage(summaryMessage);
        return SideComputeResult::Failed;
    }

    cv::Size projSize = calibData.projRes;
    if (projSize.width <= 0 || projSize.height <= 0) {
        projSize = cv::Size(kDefaultProjWidth, kDefaultProjHeight);
        logMessage(QString("%1: 标定文件缺少投影分辨率，使用默认值 %2x%3")
                       .arg(side.displayName)
                       .arg(projSize.width)
                       .arg(projSize.height));
    }

    int projFrequency = calibData.projFrequency;
    if (projFrequency <= 0) {
        projFrequency = kDefaultProjFrequency;
        logMessage(QString("%1: 标定文件缺少投影频率，使用默认值 %2")
                       .arg(side.displayName)
                       .arg(projFrequency));
    }

    try {
        std::vector<std::string> poseImages;
        poseImages.reserve(static_cast<size_t>(orderedPoseFiles.size()));
        for (const QString& filePath : orderedPoseFiles) {
            poseImages.push_back(filePath.toStdString());
        }

        std::vector<cv::Point3f> worldPoints;
        std::vector<cv::Point2f> cameraPoints;
        std::vector<cv::Point2f> projectorPoints;

        if (!calibrator.processPoseGroup(poseImages,
                                         cv::Size(kBoardCols, kBoardRows),
                                         kBoardSquareSizeMm,
                                         projSize,
                                         projFrequency,
                                         kGrayCodeCount,
                                         kPhaseShiftCount,
                                         worldPoints,
                                         cameraPoints,
                                         projectorPoints)) {
            summaryMessage = QString("%1: 计算失败 - Pose_%2 解码或角点提取失败")
                                 .arg(side.displayName, poseNumber);
            logMessage(summaryMessage);
            return SideComputeResult::Failed;
        }

        const size_t pairCount = std::min({worldPoints.size(), cameraPoints.size(), projectorPoints.size()});
        if (pairCount == 0) {
            summaryMessage = QString("%1: 计算失败 - 有效对应点数量为 0").arg(side.displayName);
            logMessage(summaryMessage);
            return SideComputeResult::Failed;
        }

        std::vector<cv::Point3f> filteredWorldPoints;
        std::vector<cv::Point2f> filteredCameraPoints;
        std::vector<cv::Point2f> filteredProjectorPoints;
        filteredWorldPoints.reserve(pairCount);
        filteredCameraPoints.reserve(pairCount);
        filteredProjectorPoints.reserve(pairCount);

        int rejectedInvalidProjector = 0;
        int rejectedInvalidNumeric = 0;
        for (size_t i = 0; i < pairCount; ++i) {
            const cv::Point3f& wp = worldPoints[i];
            const cv::Point2f& cp = cameraPoints[i];
            const cv::Point2f& pp = projectorPoints[i];

            const bool finiteValues =
                std::isfinite(wp.x) && std::isfinite(wp.y) && std::isfinite(wp.z) &&
                std::isfinite(cp.x) && std::isfinite(cp.y) &&
                std::isfinite(pp.x) && std::isfinite(pp.y);
            if (!finiteValues) {
                ++rejectedInvalidNumeric;
                continue;
            }

            const bool projectorInRange =
                pp.x >= 0.0f && pp.x < static_cast<float>(projSize.width) &&
                pp.y >= 0.0f && pp.y < static_cast<float>(projSize.height);
            const bool projectorNotZero = !(std::abs(pp.x) < 1e-3f && std::abs(pp.y) < 1e-3f);
            if (!projectorInRange || !projectorNotZero) {
                ++rejectedInvalidProjector;
                continue;
            }

            filteredWorldPoints.push_back(wp);
            filteredCameraPoints.push_back(cp);
            filteredProjectorPoints.push_back(pp);
        }

        worldPoints.swap(filteredWorldPoints);
        cameraPoints.swap(filteredCameraPoints);
        projectorPoints.swap(filteredProjectorPoints);

        if (rejectedInvalidProjector > 0 || rejectedInvalidNumeric > 0) {
            logMessage(QString("%1: 点过滤完成，保留%2/%3，剔除无效投影点=%4，剔除非法数值点=%5")
                           .arg(side.displayName)
                           .arg(worldPoints.size())
                           .arg(pairCount)
                           .arg(rejectedInvalidProjector)
                           .arg(rejectedInvalidNumeric));
        }

        const size_t matchedCount = std::min({worldPoints.size(), cameraPoints.size(), projectorPoints.size()});
        if (matchedCount < static_cast<size_t>(kMinValidPointCount)) {
            summaryMessage = QString("%1: 计算失败 - 有效对应点数量不足(%2 < %3)")
                                 .arg(side.displayName)
                                 .arg(static_cast<int>(matchedCount))
                                 .arg(kMinValidPointCount);
            logMessage(summaryMessage);
            return SideComputeResult::Failed;
        }
        worldPoints.resize(matchedCount);
        cameraPoints.resize(matchedCount);
        projectorPoints.resize(matchedCount);

        std::vector<cv::Point3f> worldPointsCentered = worldPoints;
        centerBoardPointsInPlace(worldPointsCentered, kBoardCols, kBoardRows, kBoardSquareSizeMm);

        cv::Mat camMatrix;
        cv::Mat camDist;
        cv::Mat projMatrix;
        cv::Mat projDist;
        calibData.camMatrix.convertTo(camMatrix, CV_64F);
        calibData.camDist.convertTo(camDist, CV_64F);
        calibData.projMatrix.convertTo(projMatrix, CV_64F);
        calibData.projDist.convertTo(projDist, CV_64F);

        cv::Mat R_boardToCam;
        cv::Mat T_boardToCam;
        cv::Mat R_camToProj;
        cv::Mat T_camToProj;
        if (!toRotation3x3(calibData.R_BoardToCam, R_boardToCam) ||
            !toVector3x1(calibData.T_BoardToCam, T_boardToCam) ||
            !toRotation3x3(calibData.R_CamToProj, R_camToProj) ||
            !toVector3x1(calibData.T_CamToProj, T_camToProj)) {
            summaryMessage = QString("%1: 计算失败 - 位姿或外参矩阵维度不正确").arg(side.displayName);
            logMessage(summaryMessage);
            return SideComputeResult::Failed;
        }

        // 固定使用 center+inv 组合（中心坐标系 + R/T 逆变换）。
        cv::Mat R_boardToCamInverted = R_boardToCam.t();
        cv::Mat T_boardToCamInverted = -R_boardToCamInverted * T_boardToCam;
        cv::Mat R_boardToProj = R_camToProj * R_boardToCamInverted;
        cv::Mat T_boardToProj = R_camToProj * T_boardToCamInverted + T_camToProj;
        cv::Mat Rvec_boardToCam;
        cv::Mat Rvec_boardToProj;
        cv::Rodrigues(R_boardToCamInverted, Rvec_boardToCam);
        cv::Rodrigues(R_boardToProj, Rvec_boardToProj);

        std::vector<cv::Point2f> cameraProjected;
        std::vector<cv::Point2f> projectorProjected;
        cv::projectPoints(worldPointsCentered,
                          Rvec_boardToCam,
                          T_boardToCamInverted,
                          camMatrix,
                          camDist,
                          cameraProjected);
        cv::projectPoints(worldPointsCentered,
                          Rvec_boardToProj,
                          T_boardToProj,
                          projMatrix,
                          projDist,
                          projectorProjected);

        const ErrorStats camStats = calculateErrorStats(cameraPoints, cameraProjected);
        const ErrorStats projStats = calculateErrorStats(projectorPoints, projectorProjected);
        if (camStats.count <= 0 || projStats.count <= 0) {
            summaryMessage = QString("%1: 计算失败 - 误差统计点数量不足").arg(side.displayName);
            logMessage(summaryMessage);
            return SideComputeResult::Failed;
        }
        logMessage(QString("%1: 已固定坐标转换策略=%2")
                       .arg(side.displayName)
                       .arg(kFixedPoseTransformName));
        computeData.pointCount = static_cast<int>(matchedCount);
        computeData.cameraMeanPx = camStats.mean;
        computeData.cameraRmsPx = camStats.rms;
        computeData.cameraP95Px = camStats.p95;
        computeData.cameraMaxPx = camStats.max;
        computeData.projectorMeanPx = projStats.mean;
        computeData.projectorRmsPx = projStats.rms;
        computeData.projectorP95Px = projStats.p95;
        computeData.projectorMaxPx = projStats.max;

        auto save2DCompareImage = [&](const cv::Mat& baseGrayImage,
                                      const std::vector<cv::Point2f>& measuredPoints,
                                      const std::vector<cv::Point2f>& projectedPoints,
                                      const QString& titleText,
                                      const QString& savePath) -> bool {
            if (baseGrayImage.empty()) {
                return false;
            }

            cv::Mat overlayImage;
            cv::cvtColor(baseGrayImage, overlayImage, cv::COLOR_GRAY2BGR);
            const size_t drawCount = std::min(measuredPoints.size(), projectedPoints.size());
            for (size_t i = 0; i < drawCount; ++i) {
                const cv::Point2f& measured = measuredPoints[i];
                const cv::Point2f& projected = projectedPoints[i];
                if (!std::isfinite(measured.x) || !std::isfinite(measured.y) ||
                    !std::isfinite(projected.x) || !std::isfinite(projected.y)) {
                    continue;
                }

                const cv::Point measuredPt(cvRound(measured.x), cvRound(measured.y));
                const cv::Point projectedPt(cvRound(projected.x), cvRound(projected.y));
                const bool measuredInImage =
                    measuredPt.x >= 0 && measuredPt.x < overlayImage.cols &&
                    measuredPt.y >= 0 && measuredPt.y < overlayImage.rows;
                const bool projectedInImage =
                    projectedPt.x >= 0 && projectedPt.x < overlayImage.cols &&
                    projectedPt.y >= 0 && projectedPt.y < overlayImage.rows;
                if (!measuredInImage && !projectedInImage) {
                    continue;
                }

                if (measuredInImage) {
                    cv::circle(overlayImage, measuredPt, 2, cv::Scalar(0, 255, 0), -1);
                }
                if (projectedInImage) {
                    cv::circle(overlayImage, projectedPt, 2, cv::Scalar(0, 0, 255), -1);
                }
                if (measuredInImage && projectedInImage) {
                    cv::line(overlayImage, measuredPt, projectedPt, cv::Scalar(0, 255, 255), 1);
                }
            }

            cv::putText(overlayImage, titleText.toStdString(), cv::Point(24, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
            cv::putText(overlayImage, "measured(green) projected(red)", cv::Point(24, 72),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
            return cv::imwrite(savePath.toStdString(), overlayImage);
        };

        const QString cameraWhiteImagePath = findPoseWhiteImage(cameraPath, poseNumber);
        const QString projectorWhiteImagePath = findPoseWhiteImage(projectorPath, poseNumber);

        const QString cameraResultDirPath = cameraPath + "/result";
        const QString projectorResultDirPath = projectorPath + "/result";
        QDir().mkpath(cameraResultDirPath);
        QDir().mkpath(projectorResultDirPath);

        const QString cameraWhitePoseId = extractPoseIdFromImagePath(cameraWhiteImagePath);
        cv::Mat cameraWhiteImage = cv::imread(cameraWhiteImagePath.toStdString(), cv::IMREAD_GRAYSCALE);
        if (cameraWhitePoseId != poseNumber) {
            logMessage(QString("%1: 警告 - 相机结果图Pose不一致(计算Pose_%2, 底图=%3)，跳过相机2D对比图输出")
                           .arg(side.displayName)
                           .arg(poseNumber)
                           .arg(QFileInfo(cameraWhiteImagePath).fileName()));
        } else if (cameraWhiteImage.empty()) {
            logMessage(QString("%1: 警告 - 无法读取相机 _White 图(Pose_%2/Img_01)，跳过相机2D对比图输出")
                           .arg(side.displayName)
                           .arg(poseNumber));
        } else {
            const QString cameraResultPath = QString("%1/Pose_%2_Camera2DCompare.png").arg(cameraResultDirPath, poseNumber);
            if (save2DCompareImage(cameraWhiteImage, cameraPoints, cameraProjected, "camera 2D compare", cameraResultPath)) {
                logMessage(QString("%1: 相机2D对比图已保存: %2").arg(side.displayName, cameraResultPath));
            } else {
                logMessage(QString("%1: 警告 - 相机2D对比图保存失败: %2").arg(side.displayName, cameraResultPath));
            }
        }

        const QString projectorWhitePoseId = extractPoseIdFromImagePath(projectorWhiteImagePath);
        cv::Mat projectorWhiteImage = cv::imread(projectorWhiteImagePath.toStdString(), cv::IMREAD_GRAYSCALE);
        if (projectorWhitePoseId != poseNumber) {
            logMessage(QString("%1: 警告 - 投影仪结果图Pose不一致(计算Pose_%2, 底图=%3)，跳过投影仪2D对比图输出")
                           .arg(side.displayName)
                           .arg(poseNumber)
                           .arg(QFileInfo(projectorWhiteImagePath).fileName()));
        } else if (projectorWhiteImage.empty()) {
            logMessage(QString("%1: 警告 - 无法读取投影仪 _White 图(Pose_%2/Img_01)，跳过投影仪2D对比图输出")
                           .arg(side.displayName)
                           .arg(poseNumber));
        } else {
            const QString projectorResultPath = QString("%1/Pose_%2_Projector2DCompare.png").arg(projectorResultDirPath, poseNumber);
            if (save2DCompareImage(projectorWhiteImage, projectorPoints, projectorProjected, "projector 2D compare", projectorResultPath)) {
                logMessage(QString("%1: 投影仪2D对比图已保存: %2").arg(side.displayName, projectorResultPath));
            } else {
                logMessage(QString("%1: 警告 - 投影仪2D对比图保存失败: %2").arg(side.displayName, projectorResultPath));
            }
        }

        summaryMessage = QString("%1: Pose_%2 计算完成 | 坐标转换=%3 | 相机误差 mean=%4 px, rms=%5 px, p95=%6 px, max=%7 px, N=%8 | "
                                 "投影误差 mean=%9 px, rms=%10 px, p95=%11 px, max=%12 px, N=%13")
                             .arg(side.displayName)
                             .arg(poseNumber)
                             .arg(kFixedPoseTransformName)
                             .arg(camStats.mean, 0, 'f', 4)
                             .arg(camStats.rms, 0, 'f', 4)
                             .arg(camStats.p95, 0, 'f', 4)
                             .arg(camStats.max, 0, 'f', 4)
                             .arg(camStats.count)
                             .arg(projStats.mean, 0, 'f', 4)
                             .arg(projStats.rms, 0, 'f', 4)
                             .arg(projStats.p95, 0, 'f', 4)
                             .arg(projStats.max, 0, 'f', 4)
                             .arg(projStats.count);
        logMessage(summaryMessage);
        return SideComputeResult::Success;
    } catch (const cv::Exception& e) {
        summaryMessage = QString("%1: 计算失败 - OpenCV异常: %2").arg(side.displayName, e.what());
        logMessage(summaryMessage);
        return SideComputeResult::Failed;
    }
}

void DailyQAWindow::onStartQAClicked()
{
    if (m_isRunning) {
        logMessage("当前已有 QA 任务在执行，请稍候");
        return;
    }

    m_isRunning = true;
    ui->pushButton_StartQA->setEnabled(false);
    ui->pushButton_StartCompute->setEnabled(false);

    // 重要：先刷新 USB 投影仪连接表。
    // checkConnection() 依赖 detectProjectors() 填充内部序列号映射。
    const int projectorCount = m_projectorController->detectProjectors();
    if (projectorCount <= 0) {
        logMessage("未检测到任何投影仪设备");
    } else {
        QStringList projectorList;
        for (int i = 0; i < projectorCount; ++i) {
            const std::string serial = m_projectorController->getProjectorSerial(i);
            if (!serial.empty()) {
                projectorList << QString::fromStdString(serial);
            }
        }
        logMessage(QString("已检测到投影仪 %1 台: %2")
                       .arg(projectorCount)
                       .arg(projectorList.join(", ")));
    }

    const SideConfig leftSide{
        "左侧",
        "LeftSL",
        m_deviceConfig.leftCameraSN,
        m_deviceConfig.leftProjectorTag
    };
    const SideConfig rightSide{
        "右侧",
        "RightSL",
        m_deviceConfig.rightCameraSN,
        m_deviceConfig.rightProjectorTag
    };

    logMessage("开始每日 QA 日检采图流程(左右两组顺序执行)");

    const SideRunResult leftResult = runSideDailyQA(leftSide);
    const SideRunResult rightResult = runSideDailyQA(rightSide);

    int successCount = 0;
    int skippedCount = 0;
    int failedCount = 0;
    const SideRunResult results[2] = {leftResult, rightResult};
    for (SideRunResult result : results) {
        if (result == SideRunResult::Success) {
            ++successCount;
        } else if (result == SideRunResult::Skipped) {
            ++skippedCount;
        } else {
            ++failedCount;
        }
    }

    const QString summary = QString("每日 QA 完成: 成功=%1, 跳过=%2, 失败=%3")
                                .arg(successCount)
                                .arg(skippedCount)
                                .arg(failedCount);
    logMessage(summary);

    if (failedCount > 0) {
        QMessageBox::warning(this, tr("每日QA"), summary);
    } else {
        QMessageBox::information(this, tr("每日QA"), summary);
    }

    m_isRunning = false;
    ui->pushButton_StartQA->setEnabled(true);
    ui->pushButton_StartCompute->setEnabled(true);
}

void DailyQAWindow::onStartComputeClicked()
{
    if (m_isRunning) {
        logMessage("当前已有 QA 任务在执行，请稍候");
        return;
    }

    if (!QADbManager::instance().isInitialized()) {
        logMessage("QA数据库未初始化，无法写入每日QA计算结果");
        QMessageBox::warning(this, tr("每日QA计算"), tr("QA数据库未初始化，无法执行计算写库流程"));
        return;
    }

    m_isRunning = true;
    ui->pushButton_StartQA->setEnabled(false);
    ui->pushButton_StartCompute->setEnabled(false);

    const SideConfig leftSide{
        "左侧",
        "LeftSL",
        m_deviceConfig.leftCameraSN,
        m_deviceConfig.leftProjectorTag
    };
    const SideConfig rightSide{
        "右侧",
        "RightSL",
        m_deviceConfig.rightCameraSN,
        m_deviceConfig.rightProjectorTag
    };
    std::vector<SideConfig> sides;
    sides.push_back(leftSide);
    sides.push_back(rightSide);

    const auto toDbSide = [](const SideConfig& side) -> QString {
        return side.folderName.compare("LeftSL", Qt::CaseInsensitive) == 0 ? "left" : "right";
    };

    int successCount = 0;
    int skippedCount = 0;
    int failedCount = 0;

    logMessage("开始每日QA计算流程(固定标定板金标准位姿, 左右两组顺序执行)");
    for (const SideConfig& side : sides) {
        SideComputeData computeData;
        QString summaryMessage;
        const SideComputeResult computeResult = computeSideProjectionError(side, summaryMessage, computeData);

        QString resultStatus = "failed";
        if (computeResult == SideComputeResult::Success) {
            resultStatus = "success";
        } else if (computeResult == SideComputeResult::Skipped) {
            resultStatus = "skipped";
        }

        const bool insertOk = QADbManager::instance().insertDailyQAReport(
            QDate::currentDate(),
            toDbSide(side),
            "daily_qa",
            side.cameraSN,
            side.projectorTag,
            resultStatus,
            computeData.poseNumber,
            computeData.pointCount,
            computeData.cameraMeanPx,
            computeData.cameraRmsPx,
            computeData.cameraP95Px,
            computeData.cameraMaxPx,
            computeData.projectorMeanPx,
            computeData.projectorRmsPx,
            computeData.projectorP95Px,
            computeData.projectorMaxPx,
            summaryMessage);

        SideComputeResult finalResult = computeResult;
        if (!insertOk) {
            logMessage(QString("%1: QA结果写入数据库失败").arg(side.displayName));
            finalResult = SideComputeResult::Failed;
        }

        if (finalResult == SideComputeResult::Success) {
            ++successCount;
        } else if (finalResult == SideComputeResult::Skipped) {
            ++skippedCount;
        } else {
            ++failedCount;
        }
    }

    const QString summary = QString("每日 QA 计算完成: 成功=%1, 跳过=%2, 失败=%3")
                                .arg(successCount)
                                .arg(skippedCount)
                                .arg(failedCount);
    logMessage(summary);

    if (failedCount > 0) {
        QMessageBox::warning(this, tr("每日QA计算"), summary);
    } else {
        QMessageBox::information(this, tr("每日QA计算"), summary);
    }

    m_isRunning = false;
    ui->pushButton_StartQA->setEnabled(true);
    ui->pushButton_StartCompute->setEnabled(true);
}
