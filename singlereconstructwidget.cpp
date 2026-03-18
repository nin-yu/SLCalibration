#include "singlereconstructwidget.h"
#include "ui_singlereconstructwidget.h"
#include "ProjectorController.h"
#include "CameraController.h"
#include "singlecameracontrolwidget.h"
#include "singleprojectorcontrolwidget.h"
#include "reconengine.h"
#include "reconworker.h"
#include "pointcloudwidget.h"

#include <QDebug>
#include <QMessageBox>
#include <QDir>
#include <QFileInfo>
#include <QCoreApplication>

SingleReconstructWidget::SingleReconstructWidget(ProjectorController* projCtrl,
                                                 CCameraController* camCtrl,
                                                 SingleCameraControlWidget* camWidget,
                                                 SingleProjectorControlWidget* projWidget,
                                                 const QString& projectorTag,
                                                 QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::SingleReconstructWidget)
    , m_projectorController(projCtrl)
    , m_cameraController(camCtrl)
    , m_cameraWidget(camWidget)
    , m_projectorWidget(projWidget)
    , m_projectorTag(projectorTag)
    , m_cameraHandle(nullptr)
    , m_reconEngine(nullptr)
    , m_grabWorker(nullptr)
    , m_processWorker(nullptr)
    , m_isReconstructing(false)
    , m_isInitialized(false)
    , m_wasGrabbingBeforeRecon(false)
    , m_nGrayCode(5)
    , m_nPhaseShift(4)
    , m_framesPerBatch(9)
    , m_projectorExposure(30)
    , m_projectorPattern(8)
    , m_currentFps(0.0)
    , m_currentPointCount(0)
{
    ui->setupUi(this);
    
    if (projCtrl && camCtrl && camWidget) {
        m_isInitialized = true;
    }
    
    updateButtonStates();
}

SingleReconstructWidget::~SingleReconstructWidget()
{
    // Stop reconstruction if running
    if (m_isReconstructing) {
        // Stop projector first
        stopProjector();
        
        // Stop camera capture
        if (m_cameraController && m_cameraHandle) {
            m_cameraController->StopImageCapture(m_cameraHandle);
        }
        
        // Clean up workers
        cleanupWorkers();
        
        // Restore camera to continuous mode
        switchCameraToSoftwareTrigger();
        
        m_isReconstructing = false;
    }
    
    // Clean up
    cleanupWorkers();
    
    if (m_reconEngine) {
        delete m_reconEngine;
        m_reconEngine = nullptr;
    }
    
    delete ui;
}

void SingleReconstructWidget::initialize(ProjectorController* projCtrl,
                                         CCameraController* camCtrl,
                                         SingleCameraControlWidget* camWidget,
                                         SingleProjectorControlWidget* projWidget,
                                         const QString& projectorTag)
{
    m_projectorController = projCtrl;
    m_cameraController = camCtrl;
    m_cameraWidget = camWidget;
    m_projectorWidget = projWidget;
    m_projectorTag = projectorTag;
    
    if (projCtrl && camCtrl && camWidget) {
        m_isInitialized = true;
    }
    
    updateButtonStates();
}

void SingleReconstructWidget::setCameraSerialNumber(const QString& serialNumber)
{
    m_cameraSerialNumber = serialNumber;
}

QString SingleReconstructWidget::getCalibrationFilePath() const
{
    // Get application directory
    QString appDir = QCoreApplication::applicationDirPath();
    QString calibPath;
    
    // Determine left or right based on device tag
    bool isLeft = m_deviceTag.contains("left", Qt::CaseInsensitive) || 
                  m_projectorTag.contains("left", Qt::CaseInsensitive);
    
    if (isLeft) {
        // 左侧标定文件路径
        calibPath = appDir + "/CalibrateData/LeftSL/SLCalibParams_Left.xml";
    } else {
        // 右侧标定文件路径
        calibPath = appDir + "/CalibrateData/RightSL/SLCalibParams_Right.xml";
    }
    
    // Check if file exists
    if (!QFileInfo::exists(calibPath)) {
        qDebug() << "Calibration file not found at:" << calibPath;
        // Try alternative paths (relative to source directory)
        QString altPath;
        if (isLeft) {
            altPath = appDir + "/../CalibrateData/LeftSL/SLCalibParams_Left.xml";
        } else {
            altPath = appDir + "/../CalibrateData/RightSL/SLCalibParams_Right.xml";
        }
        if (QFileInfo::exists(altPath)) {
            calibPath = altPath;
        }
    }
    
    return calibPath;
}

bool SingleReconstructWidget::initializeReconstruction()
{
    // Create reconstruction engine if not exists
    if (!m_reconEngine) {
        m_reconEngine = new ReconEngine(this);
        
        connect(m_reconEngine, &ReconEngine::errorOccurred,
                this, &SingleReconstructWidget::onWorkerError);
    }
    
    // Load calibration data
    QString calibPath = getCalibrationFilePath();
    qDebug() << "Loading calibration from:" << calibPath;
    
    if (!m_reconEngine->loadCalibration(calibPath)) {
        logMessage(QString("Failed to load calibration: %1").arg(calibPath));
        return false;
    }
    
    logMessage("Calibration loaded successfully");
    return true;
}

bool SingleReconstructWidget::initializeReconstructionForDevice(bool isLeftDevice)
{
    // Create reconstruction engine if not exists
    if (!m_reconEngine) {
        m_reconEngine = new ReconEngine(this);
        
        connect(m_reconEngine, &ReconEngine::errorOccurred,
                this, &SingleReconstructWidget::onWorkerError);
    }
    
    // Load device-specific calibration data
    QString calibPath = getCalibrationFilePathForDevice(isLeftDevice);
    QString deviceTypeStr = isLeftDevice ? "Left" : "Right";
    qDebug() << QString("Loading %1 device calibration from:").arg(deviceTypeStr) << calibPath;
    
    if (!m_reconEngine->loadCalibration(calibPath)) {
        logMessage(QString("Failed to load %1 device calibration: %2").arg(deviceTypeStr).arg(calibPath));
        return false;
    }
    
    logMessage(QString("%1 device calibration loaded successfully").arg(deviceTypeStr));
    return true;
}

QString SingleReconstructWidget::getCalibrationFilePathForDevice(bool isLeftDevice) const
{
    // Get application directory
    QString appDir = QCoreApplication::applicationDirPath();
    QString calibPath;
    
    if (isLeftDevice) {
        // 左侧标定文件路径
        calibPath = appDir + "/CalibrateData/LeftSL/SLCalibParams_Left.xml";
    } else {
        // 右侧标定文件路径
        calibPath = appDir + "/CalibrateData/RightSL/SLCalibParams_Right.xml";
    }
    
    // Check if file exists
    if (!QFileInfo::exists(calibPath)) {
        qDebug() << "Calibration file not found at:" << calibPath;
        // Try alternative paths (relative to source directory)
        QString altPath;
        if (isLeftDevice) {
            altPath = appDir + "/../CalibrateData/LeftSL/SLCalibParams_Left.xml";
        } else {
            altPath = appDir + "/../CalibrateData/RightSL/SLCalibParams_Right.xml";
        }
        if (QFileInfo::exists(altPath)) {
            calibPath = altPath;
        }
    }
    
    return calibPath;
}

bool SingleReconstructWidget::isOtherDeviceReconstructing() const
{
    // 通过父窗口或其他全局状态检查是否有其他设备正在重建
    // 这里使用信号/槽机制通知主窗口检查状态
    // 实际实现需要主窗口配合维护一个全局重建状态
    
    // 简化实现：发送信号给主窗口查询，或者通过共享状态
    // 这里返回false作为默认，实际应由主窗口管理
    
    // TODO: 需要与主窗口协调实现真正的互斥检查
    // 临时方案：通过信号通知主窗口，由主窗口决定是否允许开始
    
    return false; // 默认允许，实际互斥由主窗口控制
}

bool SingleReconstructWidget::startProjector()
{
    if (!m_projectorController || m_projectorTag.isEmpty()) {
        logMessage("Projector not configured");
        return false;
    }
    
    // 从投影仪控制组件获取参数
    int patternIndex = m_projectorPattern;
    int exposureTime = m_projectorExposure;
    
    if (m_projectorWidget) {
        patternIndex = m_projectorWidget->getPatternIndex();
        exposureTime = m_projectorWidget->getExposureTime();
        logMessage(QString("Using projector widget parameters - Pattern: %1, Exposure: %2μs")
                   .arg(patternIndex).arg(exposureTime));
    } else {
        logMessage("Warning: Projector widget not set, using default parameters");
    }
    
    // Send and play projector pattern
    // Parameters: serialNum, trigger(1=continuous loop), projTag, exposure
    bool success = m_projectorController->sendAndPlayProjector(
        m_projectorTag.toStdString(),
        1,  // Continuous loop
        patternIndex,
        exposureTime
    );
    
    if (!success) {
        logMessage("Failed to start projector");
        return false;
    }
    
    logMessage(QString("Projector started: Pattern %1, Exposure %2μs")
               .arg(patternIndex).arg(exposureTime));
    return true;
}

void SingleReconstructWidget::stopProjector()
{
    if (m_projectorController && !m_projectorTag.isEmpty()) {
        m_projectorController->pauseProjector(m_projectorTag.toStdString());
        logMessage("Projector stopped");
    }
}

bool SingleReconstructWidget::switchCameraToHardwareTrigger()
{
    if (!m_cameraController || !m_cameraHandle) {
        return false;
    }
    
    // Remember current grabbing state
    m_wasGrabbingBeforeRecon = m_cameraWidget->isGrabbing();
    
    // Stop current grabbing
    if (m_wasGrabbingBeforeRecon) {
        m_cameraController->StopImageCapture(m_cameraHandle);
    }
    
    // Set trigger mode to hardware (1 = on)
    if (!m_cameraController->SetTriggerMode(m_cameraHandle, 1)) {
        logMessage("Failed to set hardware trigger mode");
        return false;
    }
    
    // Set trigger source to Line0 (GPIO)
    if (!m_cameraController->SetTriggerSource(m_cameraHandle, "Line0")) {
        logMessage("Failed to set trigger source to Line0");
        return false;
    }
    
    // Start image capture (in hardware trigger mode)
    if (!m_cameraController->StartImageCapture(m_cameraHandle)) {
        logMessage("Failed to start image capture in trigger mode");
        return false;
    }
    
    logMessage("Camera switched to hardware trigger mode (Line0)");
    return true;
}

bool SingleReconstructWidget::switchCameraToSoftwareTrigger()
{
    if (!m_cameraController || !m_cameraHandle) {
        return false;
    }
    
    // Set trigger mode to off (0 = continuous)
    if (!m_cameraController->SetTriggerMode(m_cameraHandle, 0)) {
        logMessage("Failed to restore trigger mode");
        return false;
    }
    
    logMessage("Camera restored to continuous mode");
    
    // Restart grabbing if it was running before
    if (m_wasGrabbingBeforeRecon) {
        if (!m_cameraController->StartImageCapture(m_cameraHandle)) {
            logMessage("Warning: Failed to restart image capture");
        }
    }
    
    return true;
}

void SingleReconstructWidget::cleanupWorkers()
{
    // 先停止处理线程（消费者）
    if (m_processWorker) {
        m_processWorker->stop();
        m_processWorker->wait(3000);  // Wait max 3 seconds
        delete m_processWorker;
        m_processWorker = nullptr;
    }
    
    // 再停止采集线程（生产者）
    if (m_grabWorker) {
        m_grabWorker->stop();
        m_grabWorker->wait(3000);
        delete m_grabWorker;
        m_grabWorker = nullptr;
    }
    
    // 队列由 ReconGrabWorker 内部管理，无需手动清空
}

void SingleReconstructWidget::updateButtonStates()
{
    // 连续重建按钮状态由 toggled 信号自动管理
    // 这里只更新离线重建按钮状态
    ui->pushButton_offlineReconstruction->setEnabled(!m_isReconstructing);
}

void SingleReconstructWidget::logMessage(const QString& message)
{
    qDebug() << "[Recon]" << message;
    emit statusUpdated(message);
}

void SingleReconstructWidget::on_pushButton_continueReconstruction_toggled(bool checked)
{
    if (checked) {
        // 开始连续重建
        if (m_isReconstructing) {
            logMessage("Reconstruction already running");
            return;
        }
        
        // 检查是否有其他设备正在进行重建
        if (isOtherDeviceReconstructing()) {
            QMessageBox::warning(this, "Warning", "Another device is currently reconstructing. Please wait.");
            ui->pushButton_continueReconstruction->setChecked(false);
            return;
        }
        
        if (!m_isInitialized) {
            QMessageBox::warning(this, "Error", 
                "Widget not initialized. Please configure camera and projector first.");
            ui->pushButton_continueReconstruction->setChecked(false);
            return;
        }
        
        // Get camera handle
        if (m_cameraWidget) {
            m_cameraHandle = m_cameraWidget->getCameraHandle();
        }
        
        if (!m_cameraHandle) {
            QMessageBox::warning(this, "Error", 
                "Camera not opened. Please open camera first.");
            ui->pushButton_continueReconstruction->setChecked(false);
            return;
        }
        
        // 确定设备类型
        bool isLeftDevice = m_deviceTag.contains("left", Qt::CaseInsensitive) ||
                            m_projectorTag.contains("left", Qt::CaseInsensitive);
        QString deviceTypeStr = isLeftDevice ? "Left" : "Right";
        logMessage(QString("Starting %1 device continuous reconstruction...").arg(deviceTypeStr));
        
        // Step 1: Initialize reconstruction engine with device-specific calibration
        if (!initializeReconstructionForDevice(isLeftDevice)) {
            QMessageBox::warning(this, "Error", 
                QString("Failed to initialize reconstruction engine for %1 device.").arg(deviceTypeStr));
            ui->pushButton_continueReconstruction->setChecked(false);
            return;
        }
        
        // Step 2: Switch camera to hardware trigger mode
        if (!switchCameraToHardwareTrigger()) {
            QMessageBox::warning(this, "Error", "Failed to configure camera trigger mode.");
            ui->pushButton_continueReconstruction->setChecked(false);
            return;
        }
        
        // Step 3: Create and start worker threads
        cleanupWorkers();  // Clean up any existing workers
        
        // 创建图像采集工作线程
        m_grabWorker = new ReconGrabWorker(
            m_cameraController,
            m_cameraHandle,
            m_framesPerBatch,
            this
        );
        
        // 创建图像处理工作线程
        m_processWorker = new ReconProcessWorker(
            m_reconEngine,
            m_grabWorker->getImageQueue(),
            m_grabWorker->getQueueMutex(),
            m_grabWorker->getQueueCondition(),
            this
        );
        
        // Connect signals (Qt::QueuedConnection for thread safety)
        connect(m_processWorker, &ReconProcessWorker::pointCloudReady,
                this, &SingleReconstructWidget::onPointCloudReady,
                Qt::QueuedConnection);
        
        connect(m_processWorker, &ReconProcessWorker::errorOccurred,
                this, &SingleReconstructWidget::onWorkerError,
                Qt::QueuedConnection);
        
        connect(m_processWorker, &ReconProcessWorker::processingStats,
                this, &SingleReconstructWidget::onProcessingStats,
                Qt::QueuedConnection);
        
        connect(m_grabWorker, &ReconGrabWorker::errorOccurred,
                this, &SingleReconstructWidget::onWorkerError,
                Qt::QueuedConnection);
        
        // Start workers
        m_grabWorker->start();
        m_processWorker->start();
        
        // Step 4: Start projector
        if (!startProjector()) {
            // Cleanup on failure
            cleanupWorkers();
            switchCameraToSoftwareTrigger();
            QMessageBox::warning(this, "Error", "Failed to start projector.");
            ui->pushButton_continueReconstruction->setChecked(false);
            return;
        }
        
        m_isReconstructing = true;
        updateButtonStates();
        emit reconstructionStateChanged(true);
        
        // 更新按钮文本
        ui->pushButton_continueReconstruction->setText("停止重建");
        
        logMessage(QString("%1 device continuous reconstruction started successfully").arg(deviceTypeStr));
    } else {
        // 停止连续重建
        if (!m_isReconstructing) {
            return;
        }
        
        logMessage("Stopping reconstruction...");
        
        // Step 1: Stop projector first (no more triggers)
        stopProjector();
        
        // Step 2: Stop camera capture (unblocks GetImage)
        if (m_cameraController && m_cameraHandle) {
            m_cameraController->StopImageCapture(m_cameraHandle);
        }
        
        // Step 3: Clean up workers (stop and delete)
        cleanupWorkers();
        
        // Step 4: Restore camera to continuous mode
        switchCameraToSoftwareTrigger();
        
        m_isReconstructing = false;
        updateButtonStates();
        emit reconstructionStateChanged(false);
        
        // 更新按钮文本
        ui->pushButton_continueReconstruction->setText("连续重建");
        
        logMessage("Reconstruction stopped");
    }
}

void SingleReconstructWidget::on_pushButton_offlineReconstruction_clicked()
{
    if (m_isReconstructing) {
        QMessageBox::warning(this, "Warning", "Please stop current reconstruction first.");
        return;
    }
    
    // 检查是否有其他设备正在进行重建
    if (isOtherDeviceReconstructing()) {
        QMessageBox::warning(this, "Warning", "Another device is currently reconstructing. Please wait.");
        return;
    }
    
    logMessage("Starting offline reconstruction...");
    
    // Step 1: Determine device type (left or right)
    bool isLeftDevice = m_deviceTag.contains("left", Qt::CaseInsensitive) ||
                        m_projectorTag.contains("left", Qt::CaseInsensitive);
    
    QString deviceTypeStr = isLeftDevice ? "Left" : "Right";
    logMessage(QString("Device type: %1").arg(deviceTypeStr));
    
    // Step 2: Get offline calibration file path based on device type
    QString calibPath = getOfflineCalibrationFilePath(isLeftDevice);
    if (calibPath.isEmpty()) {
        QMessageBox::warning(this, "Error", 
            QString("Offline calibration file not found for %1 device.").arg(deviceTypeStr));
        return;
    }
    
    // Step 3: Initialize reconstruction engine with offline calibration
    if (!m_reconEngine) {
        m_reconEngine = new ReconEngine(this);
        connect(m_reconEngine, &ReconEngine::errorOccurred,
                this, &SingleReconstructWidget::onWorkerError);
    }
    
    if (!m_reconEngine->loadCalibration(calibPath)) {
        QMessageBox::warning(this, "Error", 
            QString("Failed to load offline calibration: %1").arg(calibPath));
        return;
    }
    
    logMessage(QString("Offline calibration loaded: %1").arg(calibPath));
    
    // Step 4: Load offline images based on device type
    QString appDir = QCoreApplication::applicationDirPath();
    QString imageDir = appDir + "/OfflineImages";
    
    QVector<cv::Mat> images = loadOfflineImages(imageDir, isLeftDevice);
    if (images.isEmpty()) {
        QMessageBox::warning(this, "Error", 
            QString("Failed to load offline images for %1 device.").arg(deviceTypeStr));
        return;
    }
    
    logMessage(QString("Loaded %1 offline images for %2 device").arg(images.size()).arg(deviceTypeStr));
    
    // Step 5: Perform reconstruction
    PointCloudT::Ptr cloud = m_reconEngine->reconstruct(images);
    
    if (!cloud || cloud->empty()) {
        QMessageBox::warning(this, "Error", "Offline reconstruction failed or produced empty point cloud.");
        return;
    }
    
    // Step 6: Display result
    onPointCloudReady(cloud);
    
    logMessage(QString("Offline reconstruction completed. Points: %1").arg(cloud->size()));
    QMessageBox::information(this, "Success", 
        QString("%1 device offline reconstruction completed successfully.\nPoints: %2")
            .arg(deviceTypeStr).arg(cloud->size()));
}

QString SingleReconstructWidget::getOfflineCalibrationFilePath(bool isLeftDevice) const
{
    QString appDir = QCoreApplication::applicationDirPath();
    QString calibPath;
    
    // 根据设备类型选择对应的标定文件
    if (isLeftDevice) {
        calibPath = appDir + "/CalibrateData/LeftSL/SLCalibParams_Left.xml";
    } else {
        calibPath = appDir + "/CalibrateData/RightSL/SLCalibParams_Right.xml";
    }
    
    if (QFileInfo::exists(calibPath)) {
        return calibPath;
    }
    
    // Try alternative path (relative to source directory)
    QString altPath;
    if (isLeftDevice) {
        altPath = appDir + "/../CalibrateData/LeftSL/SLCalibParams_Left.xml";
    } else {
        altPath = appDir + "/../CalibrateData/RightSL/SLCalibParams_Right.xml";
    }
    
    if (QFileInfo::exists(altPath)) {
        return altPath;
    }
    
    return QString();
}

QVector<cv::Mat> SingleReconstructWidget::loadOfflineImages(const QString& imageDir, bool isLeftDevice) const
{
    QVector<cv::Mat> images;
    
    // 根据设备类型选择图像前缀
    // 左设备: set_0img_*.bmp, 右设备: set_1img_*.bmp
    QString imagePrefix = isLeftDevice ? "set_0img_" : "set_1img_";
    
    // Expected image count: 5 gray code + 4 phase shift = 9 images
    const int expectedCount = m_nGrayCode + m_nPhaseShift;
    
    for (int i = 0; i < expectedCount; ++i) {
        QString fileName = QString("%1/%2%3.bmp").arg(imageDir).arg(imagePrefix).arg(i);
        
        if (!QFileInfo::exists(fileName)) {
            qDebug() << "Offline image not found:" << fileName;
            return QVector<cv::Mat>(); // Return empty if any image is missing
        }
        
        cv::Mat img = cv::imread(fileName.toStdString(), cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            qDebug() << "Failed to load image:" << fileName;
            return QVector<cv::Mat>();
        }
        
        images.push_back(img);
    }
    
    return images;
}

void SingleReconstructWidget::onPointCloudReady(PointCloudT::Ptr cloud)
{
    if (!cloud || cloud->empty()) {
        return;
    }
    
    // Update point cloud widget
    if (ui->pointcloudwidget) {
        ui->pointcloudwidget->updatePointCloud(cloud);
    }
}

void SingleReconstructWidget::onWorkerError(const QString& message)
{
    logMessage(QString("Error: %1").arg(message));
}

void SingleReconstructWidget::onProcessingStats(double fps, int pointCount)
{
    m_currentFps = fps;
    m_currentPointCount = pointCount;
    
    // Could update a status label here if needed
    // logMessage(QString("FPS: %1, Points: %2").arg(fps, 0, 'f', 1).arg(pointCount));
}

