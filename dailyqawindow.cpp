#include "dailyqawindow.h"
#include "ui_dailyqawindow.h"
#include "ProjectorController.h"
#include "CameraController.h"
#include "calibrationutils.h"
#include "gcpscalib.h"

#include <QMessageBox>
#include <QDateTime>
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

struct ErrorStats {
    double mean = -1.0;
    double rms = -1.0;
    double max = -1.0;
    int count = 0;
};

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
    for (size_t i = 0; i < pointCount; ++i) {
        const double err = cv::norm(measured[i] - projected[i]);
        sum += err;
        sumSquare += err * err;
        maxValue = std::max(maxValue, err);
    }

    stats.count = static_cast<int>(pointCount);
    stats.mean = sum / static_cast<double>(pointCount);
    stats.rms = std::sqrt(sumSquare / static_cast<double>(pointCount));
    stats.max = maxValue;
    return stats;
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

std::vector<cv::Point3f> buildBoardPoints(const cv::Size& boardSize, float squareSizeMm)
{
    std::vector<cv::Point3f> objectPoints;
    objectPoints.reserve(static_cast<size_t>(boardSize.width * boardSize.height));
    for (int row = 0; row < boardSize.height; ++row) {
        for (int col = 0; col < boardSize.width; ++col) {
            objectPoints.emplace_back(col * squareSizeMm, row * squareSizeMm, 0.0f);
        }
    }
    return objectPoints;
}

bool solveBoardPoseFromWhiteImage(const QString& whiteImagePath,
                                  const cv::Mat& camMatrixInput,
                                  const cv::Mat& camDistInput,
                                  const cv::Size& boardSize,
                                  float squareSizeMm,
                                  cv::Mat& R_boardToCam,
                                  cv::Mat& T_boardToCam,
                                  QString& reason)
{
    reason.clear();

    cv::Mat whiteImage = cv::imread(whiteImagePath.toStdString(), cv::IMREAD_GRAYSCALE);
    if (whiteImage.empty()) {
        reason = QString("无法读取相机White图: %1").arg(whiteImagePath);
        return false;
    }

    std::vector<cv::Point2f> corners;
    const bool found = cv::findChessboardCorners(
        whiteImage,
        boardSize,
        corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
    if (!found) {
        reason = QString("White图未检测到 %1x%2 棋盘角点")
                     .arg(boardSize.width)
                     .arg(boardSize.height);
        return false;
    }

    cv::cornerSubPix(whiteImage,
                     corners,
                     cv::Size(11, 11),
                     cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));

    const std::vector<cv::Point3f> objectPoints = buildBoardPoints(boardSize, squareSizeMm);
    cv::Mat camMatrix;
    cv::Mat camDist;
    camMatrixInput.convertTo(camMatrix, CV_64F);
    camDistInput.convertTo(camDist, CV_64F);

    cv::Mat rvec;
    cv::Mat tvec;
    if (!cv::solvePnP(objectPoints, corners, camMatrix, camDist, rvec, tvec)) {
        reason = "坐标系标定失败: solvePnP 未收敛";
        return false;
    }

    cv::Rodrigues(rvec, R_boardToCam);
    if (!toVector3x1(tvec, T_boardToCam)) {
        reason = "坐标系标定失败: 平移向量格式无效";
        return false;
    }

    return true;
}

bool upsertBoardPoseToCalibrationFile(const QString& calibFilePath,
                                      const cv::Mat& R_boardToCam,
                                      const cv::Mat& T_boardToCam,
                                      QString& reason)
{
    reason.clear();

    cv::Mat R;
    cv::Mat T;
    if (!toRotation3x3(R_boardToCam, R) || !toVector3x1(T_boardToCam, T)) {
        reason = "写入失败: 位姿矩阵维度无效";
        return false;
    }

    cv::FileStorage fsTmp(".tmp_calib.xml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
    fsTmp << "R_BoardToCam" << R;
    fsTmp << "T_BoardToCam" << T;
    const QString xmlContent = QString::fromStdString(fsTmp.releaseAndGetString());

    const QRegularExpression rRegex("<R_BoardToCam[\\s\\S]*?</R_BoardToCam>");
    const QRegularExpression tRegex("<T_BoardToCam[\\s\\S]*?</T_BoardToCam>");
    const QRegularExpressionMatch rMatch = rRegex.match(xmlContent);
    const QRegularExpressionMatch tMatch = tRegex.match(xmlContent);
    if (!rMatch.hasMatch() || !tMatch.hasMatch()) {
        reason = "写入失败: 无法生成位姿XML片段";
        return false;
    }

    QFile inputFile(calibFilePath);
    if (!inputFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
        reason = QString("写入失败: 无法读取标定文件 %1").arg(calibFilePath);
        return false;
    }
    QString fileContent = QString::fromUtf8(inputFile.readAll());
    inputFile.close();

    const QString rNode = rMatch.captured(0);
    const QString tNode = tMatch.captured(0);

    const bool hasR = rRegex.match(fileContent).hasMatch();
    const bool hasT = tRegex.match(fileContent).hasMatch();

    if (hasR) {
        fileContent.replace(rRegex, rNode);
    } else {
        fileContent.replace("</opencv_storage>", rNode + "\n</opencv_storage>");
    }

    if (hasT) {
        fileContent.replace(tRegex, tNode);
    } else {
        fileContent.replace("</opencv_storage>", tNode + "\n</opencv_storage>");
    }

    QFile outputFile(calibFilePath);
    if (!outputFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        reason = QString("写入失败: 无法写入标定文件 %1").arg(calibFilePath);
        return false;
    }
    outputFile.write(fileContent.toUtf8());
    outputFile.close();
    return true;
}

QString getCameraWhiteImagePath(const QString& cameraPath, const QString& projectorWhiteImagePath)
{
    const QString cameraWhitePath = cameraPath + "/" + QFileInfo(projectorWhiteImagePath).fileName();
    if (QFileInfo::exists(cameraWhitePath)) {
        return cameraWhitePath;
    }
    return projectorWhiteImagePath;
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
    logMessage(QString("日检固定参数: 相机曝光=%1us, 相机增益=%2, 投影曝光=%3us, 图案=%4, 触发模式=内触发")
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
    if (!m_cameraController->SetTriggerMode(cameraHandle, 1) ||
        !m_cameraController->SetTriggerSource(cameraHandle, "Line0")) {
        logMessage("设置硬触发失败");
        return false;
    }
    m_cameraController->SetTriggerActivation(cameraHandle, 0);

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

    logMessage(QString("%1 参数已固定: CamExp=%2us CamGain=%3 ProjExp=%4us Pattern=%5 Trigger=内触发")
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

    if (poseImageMap.size() != 1) {
        reason = QString("检测到 %1 个 Pose，要求目录中恰好 1 个 Pose").arg(poseImageMap.size());
        return false;
    }

    const auto poseIt = poseImageMap.constBegin();
    poseNumber = poseIt.key();
    const QMap<int, QString> imageMap = poseIt.value();

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
                                                                            QString& summaryMessage)
{
    summaryMessage.clear();

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
        calibData.R_CamToProj.empty() || calibData.T_CamToProj.empty()) {
        summaryMessage = QString("%1: 计算失败 - 标定参数不完整").arg(side.displayName);
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

        const size_t matchedCount = std::min({worldPoints.size(), cameraPoints.size(), projectorPoints.size()});
        if (matchedCount == 0) {
            summaryMessage = QString("%1: 计算失败 - 有效对应点数量为 0").arg(side.displayName);
            logMessage(summaryMessage);
            return SideComputeResult::Failed;
        }
        worldPoints.resize(matchedCount);
        cameraPoints.resize(matchedCount);
        projectorPoints.resize(matchedCount);

        cv::Mat camMatrix;
        cv::Mat camDist;
        cv::Mat projMatrix;
        cv::Mat projDist;
        calibData.camMatrix.convertTo(camMatrix, CV_64F);
        calibData.camDist.convertTo(camDist, CV_64F);
        calibData.projMatrix.convertTo(projMatrix, CV_64F);
        calibData.projDist.convertTo(projDist, CV_64F);

        bool autoCoordinateCalibrated = false;
        if (calibData.R_BoardToCam.empty() || calibData.T_BoardToCam.empty()) {
            const QString projectorWhiteImagePath = orderedPoseFiles.value(1);
            const QString cameraWhiteImagePath = getCameraWhiteImagePath(cameraPath, projectorWhiteImagePath);

            logMessage(QString("%1: 标定文件缺少 R_BoardToCam/T_BoardToCam，开始使用当前相机White图自动执行坐标系标定")
                           .arg(side.displayName));
            logMessage(QString("%1: 坐标系标定输入图像: %2").arg(side.displayName, cameraWhiteImagePath));

            cv::Mat R_boardToCam;
            cv::Mat T_boardToCam;
            QString coordinateReason;
            if (!solveBoardPoseFromWhiteImage(cameraWhiteImagePath,
                                              camMatrix,
                                              camDist,
                                              cv::Size(kBoardCols, kBoardRows),
                                              kBoardSquareSizeMm,
                                              R_boardToCam,
                                              T_boardToCam,
                                              coordinateReason)) {
                summaryMessage = QString("%1: 计算失败 - 缺少固定板位姿且自动坐标系标定失败: %2。请确认White图中棋盘完整清晰后重试。")
                                     .arg(side.displayName, coordinateReason);
                logMessage(summaryMessage);
                return SideComputeResult::Failed;
            }

            QString writeReason;
            if (!upsertBoardPoseToCalibrationFile(calibFilePath, R_boardToCam, T_boardToCam, writeReason)) {
                summaryMessage = QString("%1: 计算失败 - 自动坐标系标定成功，但写回标定文件失败: %2")
                                     .arg(side.displayName, writeReason);
                logMessage(summaryMessage);
                return SideComputeResult::Failed;
            }

            calibData.R_BoardToCam = R_boardToCam;
            calibData.T_BoardToCam = T_boardToCam;
            autoCoordinateCalibrated = true;

            const QString autoInfo = QString("%1: 已自动补齐 R_BoardToCam/T_BoardToCam，并写回 %2")
                                         .arg(side.displayName, calibFilePath);
            logMessage(autoInfo);
            QMessageBox::information(
                this,
                tr("每日QA计算"),
                QString("%1\n\n缺少固定板位姿数据，已使用当前相机White图自动完成坐标系标定并继续计算。")
                    .arg(autoInfo));
        }

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

        cv::Mat R_boardToProj = R_camToProj * R_boardToCam;
        cv::Mat T_boardToProj = R_camToProj * T_boardToCam + T_camToProj;
        cv::Mat rvecBoardToProj;
        cv::Rodrigues(R_boardToProj, rvecBoardToProj);

        // 相机预测点使用完整外参链路：
        // Board -> Projector (R_boardToProj, T_boardToProj) 再 Projector -> Camera
        const cv::Mat R_projToCam = R_camToProj.t();
        const cv::Mat T_projToCam = -R_projToCam * T_camToProj;
        const cv::Mat R_boardToCamFromChain = R_projToCam * R_boardToProj;
        const cv::Mat T_boardToCamFromChain = R_projToCam * T_boardToProj + T_projToCam;
        cv::Mat rvecBoardToCamFromChain;
        cv::Rodrigues(R_boardToCamFromChain, rvecBoardToCamFromChain);

        std::vector<cv::Point2f> cameraProjected;
        std::vector<cv::Point2f> projectorProjected;
        cv::projectPoints(worldPoints,
                          rvecBoardToCamFromChain,
                          T_boardToCamFromChain,
                          camMatrix,
                          camDist,
                          cameraProjected);
        cv::projectPoints(worldPoints, rvecBoardToProj, T_boardToProj, projMatrix, projDist, projectorProjected);

        const ErrorStats camStats = calculateErrorStats(cameraPoints, cameraProjected);
        const ErrorStats projStats = calculateErrorStats(projectorPoints, projectorProjected);
        if (camStats.count <= 0 || projStats.count <= 0) {
            summaryMessage = QString("%1: 计算失败 - 误差统计点数量不足").arg(side.displayName);
            logMessage(summaryMessage);
            return SideComputeResult::Failed;
        }

        const QString whiteImagePath = getCameraWhiteImagePath(cameraPath, orderedPoseFiles.value(1));
        cv::Mat whiteImage = cv::imread(whiteImagePath.toStdString(), cv::IMREAD_GRAYSCALE);
        if (whiteImage.empty()) {
            logMessage(QString("%1: 警告 - 无法读取 White 图，跳过叠加图输出").arg(side.displayName));
        } else {
            cv::Mat overlayImage;
            cv::cvtColor(whiteImage, overlayImage, cv::COLOR_GRAY2BGR);

            const size_t drawCount = std::min(cameraPoints.size(), cameraProjected.size());
            for (size_t i = 0; i < drawCount; ++i) {
                const cv::Point measuredPt(cvRound(cameraPoints[i].x), cvRound(cameraPoints[i].y));
                const cv::Point projectedPt(cvRound(cameraProjected[i].x), cvRound(cameraProjected[i].y));

                cv::circle(overlayImage, measuredPt, 2, cv::Scalar(0, 255, 0), -1);
                cv::circle(overlayImage, projectedPt, 2, cv::Scalar(0, 0, 255), -1);
                cv::line(overlayImage, measuredPt, projectedPt, cv::Scalar(0, 255, 255), 1);
            }

            const QString resultDirPath = cameraPath + "/result";
            QDir().mkpath(resultDirPath);
            const QString resultPath = QString("%1/Pose_%2_2DCompare.png").arg(resultDirPath, poseNumber);
            if (cv::imwrite(resultPath.toStdString(), overlayImage)) {
                logMessage(QString("%1: 相机2D对比图已保存: %2").arg(side.displayName, resultPath));
            } else {
                logMessage(QString("%1: 警告 - 相机2D对比图保存失败: %2").arg(side.displayName, resultPath));
            }
        }

        const QString poseNote = autoCoordinateCalibrated ? " (已自动补齐坐标系位姿)" : "";
        summaryMessage = QString("%1: Pose_%2 计算完成%3 | 相机误差 mean=%4 px, rms=%5 px, max=%6 px, N=%7 | "
                                 "投影误差 mean=%8 px, rms=%9 px, max=%10 px, N=%11")
                             .arg(side.displayName)
                             .arg(poseNumber)
                             .arg(poseNote)
                             .arg(camStats.mean, 0, 'f', 4)
                             .arg(camStats.rms, 0, 'f', 4)
                             .arg(camStats.max, 0, 'f', 4)
                             .arg(camStats.count)
                             .arg(projStats.mean, 0, 'f', 4)
                             .arg(projStats.rms, 0, 'f', 4)
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
        logMessage("当前已有任务在执行，请稍候");
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

    logMessage("开始每日 QA 误差计算流程(按左右两组分别输出)");

    QString leftSummary;
    QString rightSummary;
    const SideComputeResult leftResult = computeSideProjectionError(leftSide, leftSummary);
    const SideComputeResult rightResult = computeSideProjectionError(rightSide, rightSummary);

    int successCount = 0;
    int skippedCount = 0;
    int failedCount = 0;
    const SideComputeResult results[2] = {leftResult, rightResult};
    for (SideComputeResult result : results) {
        if (result == SideComputeResult::Success) {
            ++successCount;
        } else if (result == SideComputeResult::Skipped) {
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

    const QString detail = QString("%1\n%2").arg(leftSummary, rightSummary);
    if (failedCount > 0) {
        QMessageBox::warning(this, tr("每日QA计算"), summary + "\n\n" + detail);
    } else {
        QMessageBox::information(this, tr("每日QA计算"), summary + "\n\n" + detail);
    }

    m_isRunning = false;
    ui->pushButton_StartQA->setEnabled(true);
    ui->pushButton_StartCompute->setEnabled(true);
}
