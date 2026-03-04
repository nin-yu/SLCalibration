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

SingleReconstructWidget::SingleReconstructWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::SingleReconstructWidget)
    , m_projectorController(nullptr)
    , m_cameraController(nullptr)
    , m_cameraWidget(nullptr)
    , m_cameraHandle(nullptr)
    , m_reconEngine(nullptr)
    , m_grabWorker(nullptr)
    , m_processWorker(nullptr)
    , m_isReconstructing(false)
    , m_isInitialized(false)
    , m_wasGrabbingBeforeRecon(false)
    , m_nGrayCode(5)
    , m_nPhaseShift(4)
    , m_framesPerBatch(11)  // Pattern 8: 1 white + 1 dark + 5 GC + 4 PS
    , m_projectorExposure(41000)
    , m_projectorPattern(8)  // Pattern 8 by default
    , m_currentFps(0.0)
    , m_currentPointCount(0)
{
    ui->setupUi(this);
    updateButtonStates();
}

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
    , m_framesPerBatch(11)
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
        on_pushButton_stopReconstruct_clicked();
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
    if (m_grabWorker) {
        m_grabWorker->stop();
        m_grabWorker->wait(3000);  // Wait max 3 seconds
        delete m_grabWorker;
        m_grabWorker = nullptr;
    }
    
    if (m_processWorker) {
        m_processWorker->stop();
        m_processWorker->wait(3000);
        delete m_processWorker;
        m_processWorker = nullptr;
    }
    
    // Clear the queue
    QMutexLocker locker(&m_reconBufferMutex);
    m_reconImageQueue.clear();
}

void SingleReconstructWidget::updateButtonStates()
{
    bool canStart = m_isInitialized && !m_isReconstructing;
    bool canStop = m_isReconstructing;
    
    ui->pushButton_startReconstruct->setEnabled(canStart);
    ui->pushButton_stopReconstruct->setEnabled(canStop);
}

void SingleReconstructWidget::logMessage(const QString& message)
{
    qDebug() << "[Recon]" << message;
    emit statusUpdated(message);
}

void SingleReconstructWidget::on_pushButton_startReconstruct_clicked()
{
    if (m_isReconstructing) {
        logMessage("Reconstruction already running");
        return;
    }
    
    if (!m_isInitialized) {
        QMessageBox::warning(this, "Error", 
            "Widget not initialized. Please configure camera and projector first.");
        return;
    }
    
    // Get camera handle
    if (m_cameraWidget) {
        m_cameraHandle = m_cameraWidget->getCameraHandle();
    }
    
    if (!m_cameraHandle) {
        QMessageBox::warning(this, "Error", 
            "Camera not opened. Please open camera first.");
        return;
    }
    
    logMessage("Starting reconstruction...");
    
    // Step 1: Initialize reconstruction engine
    if (!initializeReconstruction()) {
        QMessageBox::warning(this, "Error", "Failed to initialize reconstruction engine.");
        return;
    }
    
    // Step 2: Switch camera to hardware trigger mode
    if (!switchCameraToHardwareTrigger()) {
        QMessageBox::warning(this, "Error", "Failed to configure camera trigger mode.");
        return;
    }
    
    // Step 3: Create and start worker threads
    cleanupWorkers();  // Clean up any existing workers
    
    m_grabWorker = new ReconGrabWorker(
        m_cameraController,
        m_cameraHandle,
        &m_reconImageQueue,
        &m_reconBufferMutex,
        this
    );
    m_grabWorker->setFramesPerBatch(m_framesPerBatch);
    m_grabWorker->setSkipFirstBatch(true);  // Skip first batch for sync
    
    m_processWorker = new ReconProcessWorker(
        m_reconEngine,
        &m_reconImageQueue,
        &m_reconBufferMutex,
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
        return;
    }
    
    m_isReconstructing = true;
    updateButtonStates();
    emit reconstructionStateChanged(true);
    
    logMessage("Reconstruction started successfully");
}

void SingleReconstructWidget::on_pushButton_stopReconstruct_clicked()
{
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
    
    // Step 3: Stop grab worker and wait
    if (m_grabWorker) {
        m_grabWorker->stop();
        if (!m_grabWorker->wait(5000)) {
            logMessage("Warning: Grab worker did not stop cleanly");
            m_grabWorker->terminate();
            m_grabWorker->wait();
        }
    }
    
    // Step 4: Stop process worker and wait
    if (m_processWorker) {
        m_processWorker->stop();
        if (!m_processWorker->wait(5000)) {
            logMessage("Warning: Process worker did not stop cleanly");
            m_processWorker->terminate();
            m_processWorker->wait();
        }
    }
    
    // Step 5: Clean up workers
    cleanupWorkers();
    
    // Step 6: Restore camera to continuous mode
    switchCameraToSoftwareTrigger();
    
    m_isReconstructing = false;
    updateButtonStates();
    emit reconstructionStateChanged(false);
    
    logMessage("Reconstruction stopped");
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
