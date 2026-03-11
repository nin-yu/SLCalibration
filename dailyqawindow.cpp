#include "dailyqawindow.h"
#include "ui_dailyqawindow.h"

#include "calibrationwindow.h"
#include "qadbmanager.h"
#include "gcpscalib.h"
#include "calibrationutils.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QMessageBox>
#include <QPushButton>
#include <QSettings>

#include <opencv2/opencv.hpp>

#include <cmath>
#include <numeric>
#include <vector>

namespace
{
QString sideToFolder(const QString& deviceSide)
{
    return deviceSide == "left" ? "LeftSL" : "RightSL";
}

QString sideToCode(const QString& deviceSide)
{
    return deviceSide == "left" ? "Left" : "Right";
}

QString sanitizeDetailsValue(QString value)
{
    value.replace(";", ",");
    value.replace("\n", " ");
    value.replace("\r", " ");
    return value;
}

QString resolvePathFromAppDir(const QString& pathValue)
{
    if (pathValue.isEmpty()) {
        return QDir(QCoreApplication::applicationDirPath()).absoluteFilePath("QAData");
    }
    if (QDir::isAbsolutePath(pathValue)) {
        return QDir::cleanPath(pathValue);
    }
    return QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(pathValue);
}

QString findLatestBatchDirectory(const QString& projectorRootPath, QString& errorMessage)
{
    QDir rootDir(projectorRootPath);
    if (!rootDir.exists()) {
        errorMessage = QString("QA目录不存在: %1").arg(projectorRootPath);
        return QString();
    }

    QFileInfoList subDirs = rootDir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Time);
    if (!subDirs.isEmpty()) {
        return subDirs.first().absoluteFilePath();
    }

    // 兼容历史数据：若无批次子目录但根目录直接放 Pose 图像，则直接使用根目录。
    QStringList poseFilters;
    poseFilters << "Pose_*.bmp" << "Pose_*.jpg" << "Pose_*.png" << "Pose_*.tif" << "Pose_*.tiff";
    QFileInfoList poseFiles = rootDir.entryInfoList(poseFilters, QDir::Files, QDir::Name);
    if (!poseFiles.isEmpty()) {
        return rootDir.absolutePath();
    }

    errorMessage = QString("未找到批次目录或Pose图像: %1").arg(projectorRootPath);
    return QString();
}

std::vector<std::string> collectBatchImagePaths(const QString& batchPath)
{
    QDir dir(batchPath);
    QStringList filters;
    filters << "*.jpg" << "*.bmp" << "*.png" << "*.tiff" << "*.tif";
    QFileInfoList fileList = dir.entryInfoList(filters, QDir::Files, QDir::Name);

    std::vector<std::string> imagePaths;
    imagePaths.reserve(static_cast<size_t>(fileList.size()));
    for (const QFileInfo& fileInfo : fileList) {
        imagePaths.push_back(fileInfo.absoluteFilePath().toStdString());
    }
    return imagePaths;
}

bool loadCalibrationFile(const QString& filePath, CalibrationData& outData, QString& errorMessage)
{
    if (!QFileInfo::exists(filePath)) {
        errorMessage = QString("标定文件不存在: %1").arg(filePath);
        return false;
    }

    GcPsCalib calibrator;
    if (!calibrator.loadCalibrationData(outData, filePath.toStdString())) {
        errorMessage = QString("读取标定文件失败: %1").arg(filePath);
        return false;
    }
    return true;
}

double computePointSetRmse(const std::vector<cv::Point2f>& referencePoints,
                           const std::vector<cv::Point2f>& predictedPoints)
{
    if (referencePoints.empty() || referencePoints.size() != predictedPoints.size()) {
        return -1.0;
    }

    double sumSquaredError = 0.0;
    for (size_t i = 0; i < referencePoints.size(); ++i) {
        const double dx = referencePoints[i].x - predictedPoints[i].x;
        const double dy = referencePoints[i].y - predictedPoints[i].y;
        sumSquaredError += (dx * dx + dy * dy);
    }

    return std::sqrt(sumSquaredError / static_cast<double>(referencePoints.size()));
}

QString findWhiteImageInPoseGroup(const std::vector<std::string>& poseGroup)
{
    for (const std::string& imagePath : poseGroup) {
        QString fileName = QFileInfo(QString::fromStdString(imagePath)).fileName();
        if (fileName.contains("_White", Qt::CaseInsensitive) ||
            fileName.contains("_white_", Qt::CaseInsensitive)) {
            return QString::fromStdString(imagePath);
        }
    }

    if (poseGroup.size() > 1) {
        return QString::fromStdString(poseGroup[1]);  // 标定命名下 Img_01 为白场图
    }
    return QString();
}

bool computeCameraPoseBatchMetric(const std::vector<std::vector<std::string>>& poseGroups,
                                  const CalibrationData& cameraBaseline,
                                  int patternRows,
                                  int patternCols,
                                  double squareSizeMm,
                                  int minValidPoseCount,
                                  double& outMeanRmse,
                                  int& outValidPoseCount,
                                  QString& errorMessage)
{
    outMeanRmse = -1.0;
    outValidPoseCount = 0;

    if (cameraBaseline.camMatrix.empty() || cameraBaseline.camDist.empty()) {
        errorMessage = "相机标定数据不完整(camMatrix/camDist缺失)";
        return false;
    }

    if (patternRows < 2 || patternCols < 2 || squareSizeMm <= 0.0) {
        errorMessage = "棋盘参数非法";
        return false;
    }

    std::vector<cv::Point3f> objectPoints;
    objectPoints.reserve(static_cast<size_t>(patternRows * patternCols));
    for (int row = 0; row < patternRows; ++row) {
        for (int col = 0; col < patternCols; ++col) {
            objectPoints.emplace_back(
                static_cast<float>(col * squareSizeMm),
                static_cast<float>(row * squareSizeMm),
                0.0f
            );
        }
    }

    const cv::Size boardSize(patternCols, patternRows);
    std::vector<double> poseErrors;

    for (const auto& poseGroup : poseGroups) {
        const QString whiteImagePath = findWhiteImageInPoseGroup(poseGroup);
        if (whiteImagePath.isEmpty()) {
            continue;
        }

        cv::Mat grayImage = cv::imread(whiteImagePath.toStdString(), cv::IMREAD_GRAYSCALE);
        if (grayImage.empty()) {
            continue;
        }

        std::vector<cv::Point2f> imageCorners;
        const bool found = cv::findChessboardCorners(
            grayImage,
            boardSize,
            imageCorners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE
        );
        if (!found) {
            continue;
        }

        cv::cornerSubPix(
            grayImage,
            imageCorners,
            cv::Size(11, 11),
            cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001)
        );

        cv::Mat rvec, tvec;
        if (!cv::solvePnP(objectPoints, imageCorners, cameraBaseline.camMatrix, cameraBaseline.camDist, rvec, tvec)) {
            continue;
        }

        std::vector<cv::Point2f> reprojCorners;
        cv::projectPoints(objectPoints, rvec, tvec, cameraBaseline.camMatrix, cameraBaseline.camDist, reprojCorners);
        const double poseRmse = computePointSetRmse(imageCorners, reprojCorners);
        if (std::isfinite(poseRmse) && poseRmse >= 0.0) {
            poseErrors.push_back(poseRmse);
        }
    }

    outValidPoseCount = static_cast<int>(poseErrors.size());
    if (outValidPoseCount < minValidPoseCount) {
        errorMessage = QString("相机有效Pose不足: valid=%1, required=%2")
                           .arg(outValidPoseCount)
                           .arg(minValidPoseCount);
        return false;
    }

    outMeanRmse = std::accumulate(poseErrors.begin(), poseErrors.end(), 0.0) /
                  static_cast<double>(poseErrors.size());
    return true;
}

bool computeProjectorPoseBatchMetric(const std::vector<std::vector<std::string>>& poseGroups,
                                     const CalibrationData& projectorBaseline,
                                     int patternRows,
                                     int patternCols,
                                     double squareSizeMm,
                                     int projectorWidth,
                                     int projectorHeight,
                                     int projectorFrequency,
                                     int grayCodeBits,
                                     int phaseShiftSteps,
                                     int minValidPoseCount,
                                     double& outMeanRmse,
                                     int& outValidPoseCount,
                                     QString& errorMessage)
{
    outMeanRmse = -1.0;
    outValidPoseCount = 0;

    if (projectorBaseline.projMatrix.empty() || projectorBaseline.projDist.empty()) {
        errorMessage = "投影仪标定数据不完整(projMatrix/projDist缺失)";
        return false;
    }

    GcPsCalib calibrator;
    std::vector<double> poseErrors;

    for (const auto& poseGroup : poseGroups) {
        std::vector<cv::Point3f> worldPoints;
        std::vector<cv::Point2f> cameraPoints;
        std::vector<cv::Point2f> projectorPoints;
        const bool processed = calibrator.processPoseGroup(
            poseGroup,
            cv::Size(patternCols, patternRows),
            static_cast<float>(squareSizeMm),
            cv::Size(projectorWidth, projectorHeight),
            projectorFrequency,
            grayCodeBits,
            phaseShiftSteps,
            worldPoints,
            cameraPoints,
            projectorPoints
        );

        if (!processed) {
            continue;
        }

        if (worldPoints.size() < 4 || projectorPoints.size() < 4 || worldPoints.size() != projectorPoints.size()) {
            continue;
        }

        cv::Mat rvec, tvec;
        if (!cv::solvePnP(worldPoints, projectorPoints, projectorBaseline.projMatrix, projectorBaseline.projDist, rvec, tvec)) {
            continue;
        }

        std::vector<cv::Point2f> reprojPoints;
        cv::projectPoints(worldPoints, rvec, tvec, projectorBaseline.projMatrix, projectorBaseline.projDist, reprojPoints);
        const double poseRmse = computePointSetRmse(projectorPoints, reprojPoints);
        if (std::isfinite(poseRmse) && poseRmse >= 0.0) {
            poseErrors.push_back(poseRmse);
        }
    }

    outValidPoseCount = static_cast<int>(poseErrors.size());
    if (outValidPoseCount < minValidPoseCount) {
        errorMessage = QString("投影仪有效Pose不足: valid=%1, required=%2")
                           .arg(outValidPoseCount)
                           .arg(minValidPoseCount);
        return false;
    }

    outMeanRmse = std::accumulate(poseErrors.begin(), poseErrors.end(), 0.0) /
                  static_cast<double>(poseErrors.size());
    return true;
}
}

DailyQAWindow::DailyQAWindow(const DeviceConfig& config, QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::DailyQAWindow)
    , m_deviceConfig(config)
{
    ui->setupUi(this);
    setWindowTitle("QA检测");

    connect(ui->pushButton_StartQA, &QPushButton::clicked,
            this, &DailyQAWindow::onStartDailyQAClicked);

    ui->pushButton_StartQA->setText("开始日检");
    ui->comboBox_Standard->clear();
    ui->comboBox_Standard->addItem("日检：QAData批次Pose评估（相机+投影仪）");
    ui->comboBox_Standard->addItem("月检：完整重标定 + 阈值判定");
    ui->comboBox_Standard->setEnabled(false);

    m_buttonStartMonthlyQA = new QPushButton("开始月检", this);
    m_buttonStartMonthlyQA->setObjectName("pushButton_StartMonthlyQA");
    m_buttonStartMonthlyQA->setMinimumSize(120, 30);
    ui->horizontalLayout_qaControls->addWidget(m_buttonStartMonthlyQA);
    connect(m_buttonStartMonthlyQA, &QPushButton::clicked,
            this, &DailyQAWindow::onStartMonthlyQAClicked);

    updateDeviceInfo();
    loadQASettings();
    appendLog("QA检测窗口已启动");
}

DailyQAWindow::~DailyQAWindow()
{
    delete ui;
}

void DailyQAWindow::updateDeviceInfo()
{
    QString info;
    info += QString("左侧相机: %1\n").arg(m_deviceConfig.leftCameraSN.isEmpty() ? "未设置" : m_deviceConfig.leftCameraSN);
    info += QString("右侧相机: %1\n").arg(m_deviceConfig.rightCameraSN.isEmpty() ? "未设置" : m_deviceConfig.rightCameraSN);
    info += QString("左侧投影仪: %1\n").arg(m_deviceConfig.leftProjectorTag);
    info += QString("右侧投影仪: %1").arg(m_deviceConfig.rightProjectorTag);

    ui->label_DeviceInfo->setText(info);
}

void DailyQAWindow::loadQASettings()
{
    QSettings settings(QDir::currentPath() + "/config.ini", QSettings::IniFormat);

    m_qaPatternRows = settings.value("QA/PatternRows", 20).toInt();
    m_qaPatternCols = settings.value("QA/PatternCols", 20).toInt();
    m_qaSquareSizeMm = settings.value("QA/SquareSizeMm", 10.0).toDouble();

    const QString qaRootValue = settings.value("QA/DailyQARoot", "QAData").toString();
    m_dailyQARootPath = resolvePathFromAppDir(qaRootValue);

    m_cameraReprojFailThresholdPx = settings.value("QA/CameraReprojFailThresholdPx", 0.8).toDouble();
    m_projectorReprojFailThresholdPx = settings.value("QA/DailyRmsProjFailThreshold", 0.8).toDouble();
    m_minValidPoseCount = settings.value("QA/MinValidPoseCount", 1).toInt();

    m_projectorWidth = settings.value("QA/ProjectorWidth", 912).toInt();
    m_projectorHeight = settings.value("QA/ProjectorHeight", 1140).toInt();
    m_projectorFrequency = settings.value("QA/ProjectorFrequency", 16).toInt();
    m_grayCodeBits = settings.value("QA/GrayCodeBits", 5).toInt();
    m_phaseShiftSteps = settings.value("QA/PhaseShiftSteps", 4).toInt();

    m_monthlyRmsProjFailThreshold = settings.value("QA/MonthlyRmsProjFailThreshold", 0.8).toDouble();
    m_monthlyRmsStereoFailThreshold = settings.value("QA/MonthlyRmsStereoFailThreshold", 0.8).toDouble();
    m_monthlyEpiMeanFailThreshold = settings.value("QA/MonthlyEpiMeanFailThresholdPx", 1.0).toDouble();

    if (m_qaPatternRows < 2) m_qaPatternRows = 20;
    if (m_qaPatternCols < 2) m_qaPatternCols = 20;
    if (m_qaSquareSizeMm <= 0.0) m_qaSquareSizeMm = 10.0;
    if (m_cameraReprojFailThresholdPx <= 0.0) m_cameraReprojFailThresholdPx = 0.8;
    if (m_projectorReprojFailThresholdPx <= 0.0) m_projectorReprojFailThresholdPx = 0.8;
    if (m_minValidPoseCount < 1) m_minValidPoseCount = 1;

    if (m_projectorWidth <= 0) m_projectorWidth = 912;
    if (m_projectorHeight <= 0) m_projectorHeight = 1140;
    if (m_projectorFrequency <= 0) m_projectorFrequency = 16;
    if (m_grayCodeBits <= 0) m_grayCodeBits = 5;
    if (m_phaseShiftSteps <= 0) m_phaseShiftSteps = 4;

    if (m_monthlyRmsProjFailThreshold <= 0.0) m_monthlyRmsProjFailThreshold = 0.8;
    if (m_monthlyRmsStereoFailThreshold <= 0.0) m_monthlyRmsStereoFailThreshold = 0.8;
    if (m_monthlyEpiMeanFailThreshold <= 0.0) m_monthlyEpiMeanFailThreshold = 1.0;

    appendLog(QString("日检配置: QAData=%1, 棋盘=%2x%3, 方格=%4mm, 相机阈值=%5, 投影仪阈值=%6, 最少有效Pose=%7")
                  .arg(m_dailyQARootPath)
                  .arg(m_qaPatternRows)
                  .arg(m_qaPatternCols)
                  .arg(m_qaSquareSizeMm, 0, 'f', 3)
                  .arg(m_cameraReprojFailThresholdPx, 0, 'f', 3)
                  .arg(m_projectorReprojFailThresholdPx, 0, 'f', 3)
                  .arg(m_minValidPoseCount));
}

void DailyQAWindow::appendLog(const QString& message)
{
    const QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    ui->textEdit_QALog->append(QString("[%1] %2").arg(timestamp).arg(message));
}

void DailyQAWindow::onStartDailyQAClicked()
{
    appendLog("开始执行日检流程（QAData批次Pose，双分项检测）");

    int passCount = 0;
    int failCount = 0;

    runDailyQASide("left",
                   m_deviceConfig.leftCameraSN,
                   m_deviceConfig.leftProjectorTag,
                   "左侧",
                   passCount,
                   failCount);
    runDailyQASide("right",
                   m_deviceConfig.rightCameraSN,
                   m_deviceConfig.rightProjectorTag,
                   "右侧",
                   passCount,
                   failCount);

    appendLog(QString("日检完成: PASS=%1, FAIL=%2").arg(passCount).arg(failCount));
    QMessageBox::information(this, tr("日检完成"),
                             tr("日检完成。\nPASS: %1\nFAIL: %2").arg(passCount).arg(failCount));
}

bool DailyQAWindow::runDailyQASide(const QString& deviceSide,
                                   const QString& cameraSN,
                                   const QString& projectorTag,
                                   const QString& sideLabel,
                                   int& passCount,
                                   int& failCount)
{
    QString status = "fail";
    double reportRmsError = -1.0;

    const bool isLeft = (deviceSide == "left");
    const QString sideFolder = sideToFolder(deviceSide);
    const QString sideCode = sideToCode(deviceSide);

    const QString sideProjectorRoot = QDir(m_dailyQARootPath).absoluteFilePath(sideFolder + "/Projector");
    QString batchError;
    const QString batchPath = findLatestBatchDirectory(sideProjectorRoot, batchError);
    if (batchPath.isEmpty()) {
        const QString details = QString("mode=pose_batch;error=batch_not_found;qa_root=%1;msg=%2")
                                    .arg(sanitizeDetailsValue(sideProjectorRoot))
                                    .arg(sanitizeDetailsValue(batchError));
        QADbManager::instance().insertQAReport(QDate::currentDate(), deviceSide, "daily_qa",
                                               cameraSN, projectorTag, status, reportRmsError, details);
        appendLog(QString("%1日检失败: %2").arg(sideLabel).arg(batchError));
        ++failCount;
        return false;
    }

    const std::vector<std::string> imagePaths = collectBatchImagePaths(batchPath);
    GcPsCalib poseCalibrator;
    const std::vector<std::vector<std::string>> poseGroups = poseCalibrator.groupCalibrationImages(imagePaths);
    const int totalPoseCount = static_cast<int>(poseGroups.size());
    if (poseGroups.empty()) {
        const QString details = QString("mode=pose_batch;error=no_pose_groups;batch=%1")
                                    .arg(sanitizeDetailsValue(batchPath));
        QADbManager::instance().insertQAReport(QDate::currentDate(), deviceSide, "daily_qa",
                                               cameraSN, projectorTag, status, reportRmsError, details);
        appendLog(QString("%1日检失败: 批次目录中未识别到有效Pose组").arg(sideLabel));
        ++failCount;
        return false;
    }

    const QString cameraParamsPath = CalibrationUtils::getCalibrationTypePath(isLeft, "Camera") +
                                     QString("/CameraParams_%1.xml").arg(sideCode);
    const QString projectorParamsPath = CalibrationUtils::getCalibrationTypePath(isLeft, "Projector") +
                                        QString("/ProjectorParams_%1.xml").arg(sideCode);

    CalibrationData cameraBaseline;
    QString cameraLoadError;
    const bool cameraLoadOk = loadCalibrationFile(cameraParamsPath, cameraBaseline, cameraLoadError);

    CalibrationData projectorBaseline;
    QString projectorLoadError;
    const bool projectorLoadOk = loadCalibrationFile(projectorParamsPath, projectorBaseline, projectorLoadError);

    double cameraRmse = -1.0;
    int cameraValidPoseCount = 0;
    QString cameraMetricError;
    bool cameraMetricOk = false;
    if (cameraLoadOk) {
        cameraMetricOk = computeCameraPoseBatchMetric(
            poseGroups,
            cameraBaseline,
            m_qaPatternRows,
            m_qaPatternCols,
            m_qaSquareSizeMm,
            m_minValidPoseCount,
            cameraRmse,
            cameraValidPoseCount,
            cameraMetricError
        );
    } else {
        cameraMetricError = cameraLoadError;
    }

    double projectorRmse = -1.0;
    int projectorValidPoseCount = 0;
    QString projectorMetricError;
    bool projectorMetricOk = false;
    if (projectorLoadOk) {
        projectorMetricOk = computeProjectorPoseBatchMetric(
            poseGroups,
            projectorBaseline,
            m_qaPatternRows,
            m_qaPatternCols,
            m_qaSquareSizeMm,
            m_projectorWidth,
            m_projectorHeight,
            m_projectorFrequency,
            m_grayCodeBits,
            m_phaseShiftSteps,
            m_minValidPoseCount,
            projectorRmse,
            projectorValidPoseCount,
            projectorMetricError
        );
    } else {
        projectorMetricError = projectorLoadError;
    }

    const bool cameraPass = cameraMetricOk && cameraRmse <= m_cameraReprojFailThresholdPx;
    const bool projectorPass = projectorMetricOk && projectorRmse <= m_projectorReprojFailThresholdPx;
    const bool overallPass = cameraPass && projectorPass;

    status = overallPass ? "pass" : "fail";
    reportRmsError = projectorRmse;

    const QString details = QString(
        "mode=pose_batch;batch=%1;total_pose=%2;camera_params=%3;projector_params=%4;"
        "camera_valid_pose=%5;projector_valid_pose=%6;camera_rmse=%7;projector_rmse=%8;"
        "camera_threshold=%9;projector_threshold=%10;camera_status=%11;projector_status=%12;"
        "camera_error=%13;projector_error=%14")
        .arg(sanitizeDetailsValue(batchPath))
        .arg(totalPoseCount)
        .arg(sanitizeDetailsValue(cameraParamsPath))
        .arg(sanitizeDetailsValue(projectorParamsPath))
        .arg(cameraValidPoseCount)
        .arg(projectorValidPoseCount)
        .arg(cameraRmse, 0, 'f', 6)
        .arg(projectorRmse, 0, 'f', 6)
        .arg(m_cameraReprojFailThresholdPx, 0, 'f', 6)
        .arg(m_projectorReprojFailThresholdPx, 0, 'f', 6)
        .arg(cameraPass ? "pass" : "fail")
        .arg(projectorPass ? "pass" : "fail")
        .arg(sanitizeDetailsValue(cameraMetricError))
        .arg(sanitizeDetailsValue(projectorMetricError));

    const bool writeOk = QADbManager::instance().insertQAReport(
        QDate::currentDate(),
        deviceSide,
        "daily_qa",
        cameraSN,
        projectorTag,
        status,
        reportRmsError,
        details
    );

    if (!writeOk) {
        appendLog(QString("%1日检: 数据库写入失败").arg(sideLabel));
    }

    appendLog(QString("%1日检结果: %2 | 相机RMSE=%3(%4), 投影仪RMSE=%5(%6), 有效Pose: cam=%7/proj=%8")
                  .arg(sideLabel)
                  .arg(overallPass ? "PASS" : "FAIL")
                  .arg(cameraRmse, 0, 'f', 6)
                  .arg(cameraPass ? "PASS" : "FAIL")
                  .arg(projectorRmse, 0, 'f', 6)
                  .arg(projectorPass ? "PASS" : "FAIL")
                  .arg(cameraValidPoseCount)
                  .arg(projectorValidPoseCount));

    if (overallPass) {
        ++passCount;
    } else {
        ++failCount;
    }
    return overallPass;
}

void DailyQAWindow::onStartMonthlyQAClicked()
{
    if (m_monthlyCalibrationWindow) {
        m_monthlyCalibrationWindow->raise();
        m_monthlyCalibrationWindow->activateWindow();
        appendLog("月检标定窗口已存在，已切换到前台");
        return;
    }

    m_monthlyWorkflowStartAt = QDateTime::currentDateTime();
    appendLog(QString("开始月检流程，标定起始时间: %1")
                  .arg(m_monthlyWorkflowStartAt.toString("yyyy-MM-dd hh:mm:ss")));

    m_monthlyCalibrationWindow = new CalibrationWindow(m_deviceConfig, this);
    m_monthlyCalibrationWindow->setAttribute(Qt::WA_DeleteOnClose);
    connect(m_monthlyCalibrationWindow, &QObject::destroyed, this, [this]() {
        m_monthlyCalibrationWindow = nullptr;
        onMonthlyCalibrationWindowClosed();
    });
    m_monthlyCalibrationWindow->show();

    QMessageBox::information(this, tr("月检引导"),
                             tr("已打开标定窗口。\n请完成左右侧重标定后关闭标定窗口，系统将自动生成月检报告。"));
}

void DailyQAWindow::onMonthlyCalibrationWindowClosed()
{
    appendLog("月检标定窗口已关闭，开始生成月检报告");

    int passCount = 0;
    int failCount = 0;

    generateMonthlyReportForSide("left",
                                 m_deviceConfig.leftCameraSN,
                                 m_deviceConfig.leftProjectorTag,
                                 "左侧",
                                 passCount,
                                 failCount);
    generateMonthlyReportForSide("right",
                                 m_deviceConfig.rightCameraSN,
                                 m_deviceConfig.rightProjectorTag,
                                 "右侧",
                                 passCount,
                                 failCount);

    appendLog(QString("月检报告生成完成: PASS=%1, FAIL=%2").arg(passCount).arg(failCount));
    QMessageBox::information(this, tr("月检完成"),
                             tr("月检报告已生成。\nPASS: %1\nFAIL: %2").arg(passCount).arg(failCount));
}

void DailyQAWindow::generateMonthlyReportForSide(const QString& deviceSide,
                                                 const QString& defaultCameraSN,
                                                 const QString& defaultProjectorTag,
                                                 const QString& sideLabel,
                                                 int& passCount,
                                                 int& failCount)
{
    CalibrationRecord record;
    if (!QADbManager::instance().getLatestCalibrationRecord(deviceSide, record, m_monthlyWorkflowStartAt)) {
        QString details = QString("source=calibration_records;error=no_new_record;required_created_after=%1")
                              .arg(m_monthlyWorkflowStartAt.toString("yyyy-MM-dd hh:mm:ss"));
        QADbManager::instance().insertQAReport(
            QDate::currentDate(),
            deviceSide,
            "monthly_calib",
            defaultCameraSN,
            defaultProjectorTag,
            "fail",
            -1.0,
            details
        );
        appendLog(QString("%1月检失败: 本次流程未检测到新的标定记录").arg(sideLabel));
        ++failCount;
        return;
    }

    const bool epiValid = record.epiMeanPx >= 0.0;
    const bool pass = (record.rmsProj <= m_monthlyRmsProjFailThreshold) &&
                      (record.rmsStereo <= m_monthlyRmsStereoFailThreshold) &&
                      epiValid &&
                      (record.epiMeanPx <= m_monthlyEpiMeanFailThreshold);

    const QString status = pass ? "pass" : "fail";
    const QString cameraSN = record.cameraSN.isEmpty() ? defaultCameraSN : record.cameraSN;
    const QString projectorTag = record.projectorTag.isEmpty() ? defaultProjectorTag : record.projectorTag;

    QString details = QString("source_calibration_id=%1;calib_file=%2;rms_proj=%3;rms_stereo=%4;epi_mean_px=%5;epi_median_px=%6;"
                              "threshold_rms_proj=%7;threshold_rms_stereo=%8;threshold_epi_mean=%9;created_at=%10")
                          .arg(record.id)
                          .arg(record.calibFilePath)
                          .arg(record.rmsProj, 0, 'f', 6)
                          .arg(record.rmsStereo, 0, 'f', 6)
                          .arg(record.epiMeanPx, 0, 'f', 6)
                          .arg(record.epiMedianPx, 0, 'f', 6)
                          .arg(m_monthlyRmsProjFailThreshold, 0, 'f', 6)
                          .arg(m_monthlyRmsStereoFailThreshold, 0, 'f', 6)
                          .arg(m_monthlyEpiMeanFailThreshold, 0, 'f', 6)
                          .arg(record.createdAt.toString("yyyy-MM-dd hh:mm:ss"));

    bool writeOk = QADbManager::instance().insertQAReport(
        QDate::currentDate(),
        deviceSide,
        "monthly_calib",
        cameraSN,
        projectorTag,
        status,
        record.rmsStereo,
        details
    );

    if (!writeOk) {
        appendLog(QString("%1月检: 数据库写入失败").arg(sideLabel));
    }

    appendLog(QString("%1月检结果: %2, rmsProj=%3, rmsStereo=%4, epiMean=%5")
                  .arg(sideLabel)
                  .arg(pass ? "PASS" : "FAIL")
                  .arg(record.rmsProj, 0, 'f', 6)
                  .arg(record.rmsStereo, 0, 'f', 6)
                  .arg(record.epiMeanPx, 0, 'f', 6));

    if (pass) {
        ++passCount;
    } else {
        ++failCount;
    }
}
