#include "singlecalibrationwidget.h"
#include "ui_singlecalibrationwidget.h"
#include "singleprojectorcontrolwidget.h"
#include <QDateTime>
#include <QMessageBox>
#include <QApplication>
#include <QStandardPaths>
#include <QDir>
#include <QFileInfo>
#include <QDebug>
#include <QThread>
#include <QtConcurrent>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iomanip>
#include <QTextStream>
#include <QRegularExpression>
#include "qadbmanager.h"

SingleCalibrationWidget::SingleCalibrationWidget(ProjectorController* projectorCtrl,
                                               CCameraController* cameraCtrl,
                                               SingleCameraControlWidget* cameraWidget,
                                               SingleProjectorControlWidget* projectorWidget,
                                               const QString& tagName,
                                               QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::SingleCalibrationWidget)
    , m_projectorController(projectorCtrl)
    , m_cameraController(cameraCtrl)
    , m_cameraWidget(cameraWidget)
    , m_projectorWidget(projectorWidget)
    , m_deviceTag(tagName)
    , m_cameraHandle(nullptr)
    , m_pImageBuffer(nullptr)
    , m_imageBufferSize(0)
    , m_isCameraOpen(false)
    , m_isCalibrating(false)
    , m_patternRows(20)
    , m_patternCols(20)
    , m_calibrator(nullptr)
    , m_squareSizeParam(10.0f)
    , m_projSize(912, 1140)
    , m_projFreq(16)
    , m_nGrayCode(5)
    , m_nPhaseShift(4)
    , m_processTimer(nullptr)
    , m_capturedImageCount(0)
    , m_targetImageCount(10)
    , m_isCapturing(false)
    , m_nProjCaliPoseCounter(0)
    // 注意：m_sequenceIndex 和 m_totalSequences 已移除，未使用
{
    // 解析设备标识
    m_isLeftDevice = tagName.contains("left", Qt::CaseInsensitive);
    m_sideIdentifier = m_isLeftDevice ? "左侧" : "右侧";

    // 设置UI
    ui->setupUi(this);

    // 初始化界面元素
    // initializeUIElements();
    setupControllers();

    // 初始化标定器
    m_calibrator = new GcPsCalib(this);
    connect(m_calibrator, &GcPsCalib::calibrationFinished,
            this, &SingleCalibrationWidget::onCalibrationFinished);

    // 如果提供了相机控制器，初始化相机状态
    if (m_cameraController) {
        // 直接使用相机控制器查询相机状态
        logMessage("通过相机控制器初始化相机状态");
        // 注意：这里需要根据实际API调整，可能需要其他方式获取序列号
        m_isCameraOpen = false;  // 初始状态为关闭
        m_isCapturing = false;   // 初始状态为未采集
    }

    logMessage(QString("标定界面初始化完成 - %1 (%2)").arg(m_sideIdentifier).arg(tagName));
}

SingleCalibrationWidget::~SingleCalibrationWidget()
{
    // 停止正在进行的标定
    if (m_isCalibrating) {
        // stopCalibrationProcess();
    }

    // 清理标定器
    if (m_calibrator) {
        delete m_calibrator;
        m_calibrator = nullptr;
    }

    // 停止图像处理定时器
    if (m_processTimer && m_processTimer->isActive()) {
        m_processTimer->stop();
    }

    // 停止图像采集
    if (m_isCapturing && m_cameraController && m_cameraHandle) {
        m_cameraController->StopImageCapture(m_cameraHandle);
        m_isCapturing = false;
    }

    // 清理图像队列
    QMutexLocker locker(&m_imageMutex);
    while (!m_imageQueue.isEmpty()) {
        m_imageQueue.dequeue();
    }
    
    // 释放图像缓冲区
    if (m_pImageBuffer) {
        delete[] m_pImageBuffer;
        m_pImageBuffer = nullptr;
    }

    delete ui;
    logMessage(QString("标定界面已销毁 - %1").arg(m_sideIdentifier));
}

// ==================== 公共接口 ====================

void SingleCalibrationWidget::setCameraSerialNumber(const QString& serialNumber)
{
    if (m_cameraSerialNumber != serialNumber) {
        m_cameraSerialNumber = serialNumber;
        // 更新UI显示
        updateCameraSNDisplay();
        updateStatusDisplay();
        logMessage(QString("设置相机序列号: %1").arg(serialNumber));

        // 如果当前相机已打开，先关闭
        if (m_isCameraOpen && m_cameraController && m_cameraHandle) {
            // 通过相机控制器关闭相机
            if (m_cameraController->CloseCamera(m_cameraHandle)) {
                m_cameraHandle = nullptr;
                m_isCameraOpen = false;
                logMessage("相机已关闭");
                
                // 刷新关联的相机控制组件UI状态
                if (m_cameraWidget) {
                    m_cameraWidget->updateCameraStatus();
                    m_cameraWidget->updateParameterControls();
                }
            } else {
                logMessage("警告: 关闭相机失败");
            }
        }
    }
}

// ==================== 初始化函数 ====================

void SingleCalibrationWidget::initializeUIElements()
{
    // 初始化参数设置
    ui->spinBox_PatternRows->setValue(m_patternRows);
    ui->spinBox_PatternCols->setValue(m_patternCols);
    ui->doubleSpinBox_SquareSize->setValue(m_squareSizeParam);

    // 初始化进度条
    ui->progressBar_CameraCalibration->setValue(0);
    ui->progressBar_ProjectorCalibration->setValue(0);
    ui->progressBar_CoordinationCalibration->setValue(0);

    // 初始化按钮状态
    ui->pushButton_StartCalibrateCamera->setEnabled(false);
    ui->button_StartCalibrationProjector->setEnabled(false);
    ui->pushButton_StartCalibrationCoordination->setEnabled(false);

    updateStatusDisplay();
    updateCameraSNDisplay();
}

void SingleCalibrationWidget::setupControllers()
{
    if (!m_projectorController) {
        logMessage("警告: 投影仪控制器未初始化");
    }

    if (!m_cameraController) {
        logMessage("警告: 相机控制器未初始化");
    }
}

// ==================== 相机控制函数 ====================

bool SingleCalibrationWidget::ensureCameraOpened()
{
    if (m_cameraSerialNumber.isEmpty()) {
        logMessage("错误: 相机序列号未设置");
        return false;
    }

    if (m_isCameraOpen && m_cameraHandle) {
        // 相机已经打开
        return true;
    }

    // 通过相机控制器打开相机
    if (m_cameraController) {
        logMessage(QString("正在打开相机: %1").arg(m_cameraSerialNumber));
        void* handle = m_cameraController->OpenCameraBySN(m_cameraSerialNumber.toStdString().c_str());
        
        if (handle) {
            m_cameraHandle = handle;
            m_isCameraOpen = true;
            
            // 分配图像缓冲区
            if (!m_pImageBuffer) {
                m_pImageBuffer = new unsigned char[5 * 1024 * 1024];  // 5MB
                m_imageBufferSize = 5 * 1024 * 1024;
            }
            
            logMessage(QString("相机打开成功: %1").arg(m_cameraSerialNumber));
            
            // 刷新关联的相机控制组件UI状态
            if (m_cameraWidget) {
                m_cameraWidget->updateCameraStatus();
                m_cameraWidget->updateParameterControls();
            }
            
            return true;
        } else {
            logMessage(QString("相机打开失败: %1").arg(m_cameraSerialNumber));
            return false;
        }
    } else {
        logMessage("错误: 相机控制器未初始化");
        return false;
    }
}

// ==================== UI事件槽函数 ====================

void SingleCalibrationWidget::on_pushButton_StartCaptureCameraImage_clicked()
{
    // 检查相机序列号是否已设置
    if (m_cameraSerialNumber.isEmpty()) {
        showErrorMessage("请先设置相机序列号");
        logMessage("错误: 相机序列号未设置，无法进行图像采集");
        return;
    }

    logMessage(QString("开始执行相机图像采集流程 - 序列号: %1").arg(m_cameraSerialNumber));

    // 安全停止当前可能正在进行的图像采集
    if (m_isCapturing && m_cameraController && m_cameraHandle) {
        logMessage("检测到正在进行的图像采集，正在安全停止...");
        if (m_cameraController->StopImageCapture(m_cameraHandle)) {
            m_isCapturing = false;
            logMessage("图像采集已停止");
        } else {
            logMessage("警告: 停止图像采集失败");
        }
        // 短暂等待确保停止完成
        QThread::msleep(50);
    }

    // 确保相机已打开
    if (!ensureCameraOpened()) {
        showErrorMessage(QString("无法打开相机: %1").arg(m_cameraSerialNumber));
        logMessage(QString("相机打开失败: %1").arg(m_cameraSerialNumber));
        return;
    }


    // 执行软触发图像采集
    logMessage("执行软触发图像采集...");
    if (!captureSingleImageWithSoftwareTrigger()) {
        showErrorMessage("软触发图像采集失败");
        logMessage("软触发图像采集流程完成（含错误处理）");
        return;
    }

    showSuccessMessage("图像采集成功完成");
}

void SingleCalibrationWidget::on_pushButton_StartCalibrateCamera_clicked()
{
    // 检查是否已有标定图像
    QString calibrationPath = getCalibrationTypePath("Camera");
    if (calibrationPath.isEmpty()) {
        showErrorMessage("无法获取标定路径");
        return;
    }

    QDir dir(calibrationPath);
    QStringList filters;
    filters << "*.jpg" << "*.bmp" << "*.png";
    QFileInfoList fileList = dir.entryInfoList(filters, QDir::Files);

    if (fileList.isEmpty()) {
        showErrorMessage(QString("No calibration images found in path: %1. Please capture calibration images first.").arg(calibrationPath));
        return;
    }

    logMessage(QString("Starting camera calibration process - Found %1 calibration images").arg(fileList.size()));

    // 禁用按钮防止重复点击
    ui->pushButton_StartCalibrateCamera->setEnabled(false);
    ui->progressBar_CameraCalibration->setValue(0);

    // 执行标定
    bool success = calibrateCamera();

    // 重新启用按钮
    ui->pushButton_StartCalibrateCamera->setEnabled(true);

    if (success) {
        showSuccessMessage(QString("%1 camera calibration completed successfully").arg(m_sideIdentifier));
    } else {
        showErrorMessage(QString("%1 camera calibration failed").arg(m_sideIdentifier));
    }
}

void SingleCalibrationWidget::on_pushButton_StartCalibrationSequenceProjector_clicked()
{
    // --- 0. 获取投影仪标定路径，并通过目录扫描确定下一个 Pose 编号 ---
    QString sCalibPath = getCalibrationTypePath("Projector");
    if (sCalibPath.isEmpty()) {
        showErrorMessage("无法获取投影仪标定路径");
        logMessage("错误: 无法获取投影仪标定路径");
        return;
    }

    // 通过扫描目录确定下一个 Pose 编号，保证命名连续性
    int currentPoseNumber = getNextPoseNumber(sCalibPath);
    logMessage(formatPoseMsg(currentPoseNumber, "开始执行投影仪标定序列采集 (目录扫描确定编号)"));

    // 同步更新成员计数器，保持一致性
    m_nProjCaliPoseCounter = currentPoseNumber;

    // 检查相机和投影仪是否已准备好
    if (m_cameraSerialNumber.isEmpty()) {
        showErrorMessage("请先设置相机序列号");
        logMessage(formatPoseMsg(currentPoseNumber, "错误: 相机序列号未设置，采集取消"));
        return;
    }

    if (!m_projectorController) {
        showErrorMessage("投影仪控制器未初始化");
        logMessage(formatPoseMsg(currentPoseNumber, "错误: 投影仪控制器未初始化，采集取消"));
        return;
    }

    // 确保相机已打开
    if (!ensureCameraOpened()) {
        showErrorMessage(QString("无法打开相机: %1").arg(m_cameraSerialNumber));
        logMessage(formatPoseMsg(currentPoseNumber, QString("错误: 无法打开相机 %1，采集取消").arg(m_cameraSerialNumber)));
        return;
    }

    // 确保缓冲区已分配
    if (!m_pImageBuffer) {
        m_pImageBuffer = new unsigned char[5 * 1024 * 1024];
        m_imageBufferSize = 5 * 1024 * 1024;
    }

    // 保存原来的触发模式
    int nOldTriggerMode = 0;
    m_cameraController->GetTriggerMode(m_cameraHandle, nOldTriggerMode);
    char oldTriggerSource[64] = {0};
    m_cameraController->GetTriggerSource(m_cameraHandle, oldTriggerSource, sizeof(oldTriggerSource));

    // 记录是否之前在抓图
    bool bWasGrabbing = m_isCapturing;

    // 定义清理恢复函数
    auto cleanupAndRestore = [&]() {
        // 恢复相机到之前的触发模式
        m_cameraController->SetTriggerMode(m_cameraHandle, nOldTriggerMode);
        m_cameraController->SetTriggerSource(m_cameraHandle, oldTriggerSource);

        // 如果之前在连续抓图，重启
        if (bWasGrabbing) {
            m_cameraController->SetTriggerMode(m_cameraHandle, 0);  // 连续模式
            m_cameraController->StartImageCapture(m_cameraHandle);
            m_isCapturing = true;
        }

        // 刷新关联的相机控制组件UI状态
        if (m_cameraWidget) {
            m_cameraWidget->updateCameraStatus();
            m_cameraWidget->updateParameterControls();
        }

        logMessage(formatPoseMsg(currentPoseNumber, "投影仪标定序列采集流程结束"));
    };

    // --- 1. 停止连续抓图 (如果正在抓) ---
    if (bWasGrabbing) {
        logMessage(formatPoseMsg(currentPoseNumber, "停止正在进行的图像采集..."));
        m_cameraController->StopImageCapture(m_cameraHandle);
        m_isCapturing = false;
        QThread::msleep(100);
    }

    // --- 2. 抓取暗场 (Dark Field) - 使用软件触发 ---
    logMessage(formatPoseMsg(currentPoseNumber, "正在抓取暗场图像..."));

    // 确保投影仪是黑的
    m_projectorController->pauseProjector(m_deviceTag.toStdString());
    QThread::msleep(100);

    // 设置软件触发模式
    if (!m_cameraController->SetTriggerMode(m_cameraHandle, 1)) {
        logMessage(formatPoseMsg(currentPoseNumber, "失败: 设置相机软件触发失败，原因: 相机SetTriggerMode返回错误"));
        showErrorMessage(QString("Pose %1 采集失败: 设置相机软件触发失败!").arg(formatPoseNumber(currentPoseNumber)));
        cleanupAndRestore();
        return;
    }
    if (!m_cameraController->SetTriggerSource(m_cameraHandle, "Software")) {
        logMessage(formatPoseMsg(currentPoseNumber, "失败: 设置触发源为Software失败，原因: SetTriggerSource返回错误"));
        showErrorMessage(QString("Pose %1 采集失败: 设置触发源为Software失败!").arg(formatPoseNumber(currentPoseNumber)));
        cleanupAndRestore();
        return;
    }

    // 启动相机
    if (!m_cameraController->StartImageCapture(m_cameraHandle)) {
        logMessage(formatPoseMsg(currentPoseNumber, "失败: 启动相机软触发模式失败，原因: StartImageCapture返回错误"));
        showErrorMessage(QString("Pose %1 采集失败: 启动相机 (SW Trigger) 失败!").arg(formatPoseNumber(currentPoseNumber)));
        cleanupAndRestore();
        return;
    }

    // 发送软触发并抓取暗场图
    QThread::msleep(100);
    if (!m_cameraController->TriggerSoftware(m_cameraHandle)) {
        logMessage(formatPoseMsg(currentPoseNumber, "失败: 发送软触发信号失败，原因: TriggerSoftware返回错误"));
        showErrorMessage(QString("Pose %1 采集失败: 发送软触发失败!").arg(formatPoseNumber(currentPoseNumber)));
        m_cameraController->StopImageCapture(m_cameraHandle);
        cleanupAndRestore();
        return;
    }

    // 获取暗场图像 (3秒超时)
    {
        MV_FRAME_OUT_INFO_EX frameInfo = {0};
        // 使用带超时限制的 GetImage，超时时间为 3000ms
        bool gotImage = m_cameraController->GetImage(
            m_cameraHandle, m_pImageBuffer,
            static_cast<unsigned int>(m_imageBufferSize), &frameInfo, 3000);

        if (!gotImage || frameInfo.nHeight == 0) {
            logMessage(formatPoseMsg(currentPoseNumber, "失败: 暗场图像采集超时(3秒)，原因: 相机未在规定时间内返回图像"));
            showErrorMessage(QString("Pose %1 采集失败: 抓取暗场图超时(3秒)，请检查相机连接和曝光设置").arg(formatPoseNumber(currentPoseNumber)));
            m_cameraController->StopImageCapture(m_cameraHandle);
            cleanupAndRestore();
            return;
        }

        // 保存暗场图
        cv::Mat darkImage(frameInfo.nHeight, frameInfo.nWidth, CV_8UC1, m_pImageBuffer);
        QString sFileName = QString("%1/Pose_%2_Img_00_Dark.bmp")
            .arg(sCalibPath)
            .arg(currentPoseNumber, 2, 10, QLatin1Char('0'));
        if (!cv::imwrite(sFileName.toStdString(), darkImage)) {
            logMessage(formatPoseMsg(currentPoseNumber, "失败: 暗场图像保存失败，原因: cv::imwrite失败，请检查磁盘空间和路径权限"));
            showErrorMessage(QString("Pose %1 采集失败: 暗场图像保存失败，请检查磁盘空间").arg(formatPoseNumber(currentPoseNumber)));
            m_cameraController->StopImageCapture(m_cameraHandle);
            cleanupAndRestore();
            return;
        }
        logMessage(formatPoseMsg(currentPoseNumber, QString("暗场图已保存: %1").arg(sFileName)));
    }

    // 停止软触发抓图，准备切换到硬件触发
    m_cameraController->StopImageCapture(m_cameraHandle);

    // --- 3. 配置相机为硬件触发 ---
    logMessage(formatPoseMsg(currentPoseNumber, "配置相机为硬件触发模式..."));

    if (!m_cameraController->SetTriggerMode(m_cameraHandle, 1)) {
        logMessage(formatPoseMsg(currentPoseNumber, "失败: 设置相机硬件触发失败，原因: SetTriggerMode返回错误"));
        showErrorMessage(QString("Pose %1 采集失败: 设置相机硬件触发失败!").arg(formatPoseNumber(currentPoseNumber)));
        cleanupAndRestore();
        return;
    }
    if (!m_cameraController->SetTriggerSource(m_cameraHandle, "Line0")) {
        logMessage(formatPoseMsg(currentPoseNumber, "失败: 设置触发源为Line0失败，原因: SetTriggerSource返回错误"));
        showErrorMessage(QString("Pose %1 采集失败: 设置触发源为Line0失败!").arg(formatPoseNumber(currentPoseNumber)));
        cleanupAndRestore();
        return;
    }
    // 设置下降沿触发
    m_cameraController->SetTriggerActivation(m_cameraHandle, 1);  // 1 = 下降沿

    // 启动相机 (在硬件触发模式下 "待命")
    if (!m_cameraController->StartImageCapture(m_cameraHandle)) {
        logMessage(formatPoseMsg(currentPoseNumber, "失败: 启动相机硬触发模式失败，原因: StartImageCapture返回错误"));
        showErrorMessage(QString("Pose %1 采集失败: 启动相机 (HW Trigger) 失败!").arg(formatPoseNumber(currentPoseNumber)));
        cleanupAndRestore();
        return;
    }

    // --- 4. 抓取19张格雷码+相移图 (pattern 6 = Cali) ---
    logMessage(formatPoseMsg(currentPoseNumber, "正在抓取19张标定图..."));

    // 从投影仪控制组件获取参数
    int triggerMode = 1;  // 硬件触发模式
    int patternIndex = 6; // 默认标定序列
    int exposureTime = 500000; // 默认曝光时间500ms

    if (m_projectorWidget) {
        patternIndex = m_projectorWidget->getPatternIndex();
        exposureTime = m_projectorWidget->getExposureTime();
        logMessage(formatPoseMsg(currentPoseNumber, QString("使用投影仪参数 - 图案: %1, 曝光: %2μs").arg(patternIndex).arg(exposureTime)));
    } else {
        logMessage(formatPoseMsg(currentPoseNumber, "警告: 投影仪控制组件未设置，使用默认参数"));
    }

    if (!m_projectorController->sendAndPlayProjector(m_deviceTag.toStdString(), triggerMode, patternIndex, exposureTime)) {
        logMessage(formatPoseMsg(currentPoseNumber, QString("失败: 发送投影仪序列失败，原因: sendAndPlayProjector(Pattern=%1)返回错误").arg(patternIndex)));
        showErrorMessage(QString("Pose %1 采集失败: 发送投影仪序列 (Pattern=%2) 失败!").arg(formatPoseNumber(currentPoseNumber)).arg(patternIndex));
        m_cameraController->StopImageCapture(m_cameraHandle);
        cleanupAndRestore();
        return;
    }

    // 循环抓取19张图 (1白 + 18张格雷码/相移)，每张设置3秒超时
    for (int i = 0; i < 19; i++) {
        logMessage(formatPoseMsg(currentPoseNumber, QString("正在抓取标定图 %1/19...").arg(i + 1)));

        // 命名规则:
        // 0: White (白场)
        // 1-5: GC_H_0~4 (5张水平格雷码)
        // 6-10: GC_V_0~4 (5张垂直格雷码)
        // 11-14: PS_V_0~3 (4张垂直相移)
        // 15-18: PS_H_0~3 (4张水平相移)
        QString sPatternName;
        if (i < 1)        sPatternName = QString("White");
        else if (i < 6)   sPatternName = QString("GC_H_%1").arg(i - 1);
        else if (i < 11)  sPatternName = QString("GC_V_%1").arg(i - 6);
        else if (i < 15)  sPatternName = QString("PS_V_%1").arg(i - 11);
        else              sPatternName = QString("PS_H_%1").arg(i - 15);

        QString sFileName = QString("%1/Pose_%2_Img_%3_%4.bmp")
            .arg(sCalibPath)
            .arg(formatPoseNumber(currentPoseNumber))
            .arg(i + 1, 2, 10, QLatin1Char('0'))
            .arg(sPatternName);

        // 获取图像，超时时间 3000ms
        MV_FRAME_OUT_INFO_EX frameInfo = {0};
        bool gotImage = m_cameraController->GetImage(
            m_cameraHandle, m_pImageBuffer,
            static_cast<unsigned int>(m_imageBufferSize), &frameInfo, 3000);

        if (!gotImage || frameInfo.nHeight == 0) {
            m_projectorController->pauseProjector(m_deviceTag.toStdString());
            logMessage(formatPoseMsg(currentPoseNumber, QString("失败: 第 %1/19 张标定图采集超时(3秒)，原因: 图案[%2]相机未返回图像，请检查投影仪触发信号和相机连接").arg(i + 1).arg(sPatternName)));
            showErrorMessage(QString("Pose %1 采集失败!\n"
                                     "第 %2/19 张图像(%3)采集超时(3秒)\n"
                                     "可能原因:\n"
                                     "  - 投影仪触发信号未到达相机\n"
                                     "  - 相机曝光时间超过3秒\n"
                                     "  - 硬件连接异常\n"
                                     "本次 Pose 采集已终止，已采集的图像将被保留。")
                             .arg(formatPoseNumber(currentPoseNumber))
                             .arg(i + 1).arg(sPatternName));
            m_cameraController->StopImageCapture(m_cameraHandle);
            cleanupAndRestore();
            return;
        }

        // 保存图像
        cv::Mat image(frameInfo.nHeight, frameInfo.nWidth, CV_8UC1, m_pImageBuffer);
        if (!cv::imwrite(sFileName.toStdString(), image)) {
            m_projectorController->pauseProjector(m_deviceTag.toStdString());
            logMessage(formatPoseMsg(currentPoseNumber, QString("失败: 第 %1/19 张图像(%2)保存失败，原因: cv::imwrite失败，请检查磁盘空间和路径权限").arg(i + 1).arg(sPatternName)));
            showErrorMessage(QString("Pose %1 采集失败: 第 %2/19 张图像(%3)保存失败，请检查磁盘空间")
                             .arg(formatPoseNumber(currentPoseNumber))
                             .arg(i + 1).arg(sPatternName));
            m_cameraController->StopImageCapture(m_cameraHandle);
            cleanupAndRestore();
            return;
        }
        logMessage(formatPoseMsg(currentPoseNumber, QString("图像已保存(%1/19): %2").arg(i + 1).arg(sFileName)));

        // 更新进度
        ui->progressBar_ProjectorCalibration->setValue((i + 1) * 100 / 19);
    }

    // 19张图全部完成后暂停投影仪
    m_projectorController->pauseProjector(m_deviceTag.toStdString());

    // --- 5. 成功 ---
    m_cameraController->StopImageCapture(m_cameraHandle);

    QString successMsg = QString("Pose %1 采集成功!\n共20张图像 (1暗+1白+18格雷码/相移)\n已保存到:\n%2")
        .arg(formatPoseNumber(currentPoseNumber)).arg(sCalibPath);
    logMessage(successMsg);
    showSuccessMessage(successMsg);

    // 更新进度条
    ui->progressBar_ProjectorCalibration->setValue(100);

    // --- 6. 清理和恢复 ---
    cleanupAndRestore();
}

void SingleCalibrationWidget::on_button_LoadImagesProjector_clicked()
{
    if (m_isCapturing) {
        showErrorMessage("请等待当前采集完成");
        return;
    }

    logMessage("读取投影仪图像...");

    // 禁用按钮防止重复点击
    ui->button_LoadImagesProjector->setEnabled(false);

    if (loadProjectorImages()) {
        ui->button_StartCalibrationProjector->setEnabled(true);
        showSuccessMessage("图像读取完成");
    } else {
        showErrorMessage("图像读取失败");
    }

    // 重新启用按钮
    ui->button_LoadImagesProjector->setEnabled(true);
}

void SingleCalibrationWidget::on_button_StartCalibrationProjector_clicked()
{
    // 检查是否有标定图像
    if (m_imagePaths.empty()) {
        QMessageBox::warning(this, tr("警告"), tr("请先加载标定图片"));
        return;
    }

    logMessage("开始执行投影仪标定...");

    // 根据设备侧向动态选择相机标定参数文件
    QString sideCode = m_isLeftDevice ? "Left" : "Right";
    QString calibrationPath = getCalibrationTypePath("Camera");
    QString cameraParamsPath = calibrationPath + QString("/CameraParams_%1.xml").arg(sideCode);
    QFile cameraParamsFile(cameraParamsPath);

    if (cameraParamsFile.exists()) {
        // 尝试加载相机参数
        CalibrationData tempCalibData;
        if (m_calibrator->loadCalibrationData(tempCalibData, cameraParamsPath.toStdString())) {
            // 使用加载的相机参数
            m_calibData.camMatrix = tempCalibData.camMatrix;
            m_calibData.camDist = tempCalibData.camDist;
            updateStatus(QString("已从%1侧相机标定文件加载参数").arg(m_sideIdentifier));
            logMessage(QString("成功加载%1侧相机参数: %2").arg(m_sideIdentifier).arg(cameraParamsPath));
        }
        else {
            QMessageBox::warning(this, tr("警告"),
                QString(tr("无法从%1侧相机标定文件加载参数，将使用默认值")).arg(m_sideIdentifier));
            logMessage(QString("警告: 无法加载%1侧相机参数，使用默认值").arg(m_sideIdentifier));
        }
    }
    else {
        QMessageBox::warning(this, tr("警告"),
            QString(tr("未找到%1侧相机标定文件(%2)，将使用默认相机参数")).arg(m_sideIdentifier).arg(cameraParamsPath));
        logMessage(QString("警告: 未找到%1侧相机标定文件: %2，使用默认相机参数").arg(m_sideIdentifier).arg(cameraParamsPath));
    }

    // 禁用标定按钮防止重复点击
    ui->button_StartCalibrationProjector->setEnabled(false);

    // 开始标定（同步执行）
    // 标定完成后会通过 calibrationFinished 信号自动触发 onCalibrationFinished 槽
    logMessage("开始执行投影仪标定计算...");
    m_calibrator->calibrate(
        m_imagePaths,
        cv::Size(m_patternCols, m_patternRows),  // 动态构建棋盘格尺寸
        m_squareSizeParam,
        m_projSize,
        m_projFreq,
        m_nGrayCode,
        m_nPhaseShift,
        m_calibData
    );
}

void SingleCalibrationWidget::on_pushButton_StartCalibrationCoordination_clicked()
{
    // 检查相机序列号是否已设置
    if (m_cameraSerialNumber.isEmpty()) {
        showErrorMessage("请先设置相机序列号");
        logMessage("错误: 相机序列号未设置，无法进行坐标系标定");
        return;
    }

    logMessage(QString("开始执行坐标系标定 - 序列号: %1").arg(m_cameraSerialNumber));

    // 安全停止当前可能正在进行的图像采集
    if (m_isCapturing && m_cameraController && m_cameraHandle) {
        logMessage("检测到正在进行的图像采集，正在安全停止...");
        if (m_cameraController->StopImageCapture(m_cameraHandle)) {
            m_isCapturing = false;
        }
        QThread::msleep(50);
    }

    // 确保相机已打开
    if (!ensureCameraOpened()) {
        showErrorMessage(QString("无法打开相机: %1").arg(m_cameraSerialNumber));
        return;
    }

    // --- 1. 清空 Coordinate 目录，准备本次标定 ---
    logMessage("清空坐标系标定目录...");
    if (!clearCoordinateDirectory()) {
        showErrorMessage("无法清空坐标系标定目录，请检查文件权限");
        return;
    }

    // --- 2. 获取当前相机图像 ---
    cv::Mat currentImage;
    if (!captureImageAndReturn(currentImage)) {
        showErrorMessage("无法获取当前相机图像");
        return;
    }

    // --- 3. 保存采集到的图像到 Coordinate 目录 ---
    QString savedImagePath;
    if (!saveCoordinateImage(currentImage, savedImagePath)) {
        showErrorMessage("保存坐标系标定图像失败");
        return;
    }

    // --- 4. 转换为灰度图 ---
    cv::Mat currentImageGray;
    if (currentImage.channels() == 3) {
        cv::cvtColor(currentImage, currentImageGray, cv::COLOR_BGR2GRAY);
    } else {
        currentImageGray = currentImage;
    }

    // --- 5. 加载相机内参 (来自 CameraParams.xml) ---
    QString sideCode = m_isLeftDevice ? "Left" : "Right";
    QString cameraCalibPath = getCalibrationTypePath("Camera");
    QString cameraParamsPath = cameraCalibPath + QString("/CameraParams_%1.xml").arg(sideCode);
    QFile cameraParamsFile(cameraParamsPath);

    if (!cameraParamsFile.exists()) {
        showErrorMessage("无法找到 CameraParams.xml 文件。请先运行相机标定。");
        logMessage(QString("错误: 相机参数文件不存在: %1").arg(cameraParamsPath));
        return;
    }

    cv::FileStorage fsCam(cameraParamsPath.toStdString(), cv::FileStorage::READ);
    if (!fsCam.isOpened()) {
        showErrorMessage(QString("无法打开 CameraParams_%1.xml 文件").arg(sideCode));
        logMessage(QString("错误: 无法打开相机参数文件: %1").arg(cameraParamsPath));
        return;
    }

    cv::Mat camMatrix, distCoeffs;
    fsCam["cameraMatrix"] >> camMatrix;
    fsCam["distCoeffs"] >> distCoeffs;
    fsCam.release();

    if (camMatrix.empty() || distCoeffs.empty()) {
        showErrorMessage("CameraParams.xml 文件中缺少相机内参");
        logMessage("错误: 相机内参数据不完整");
        return;
    }

    logMessage("成功加载相机内参");

    // --- 6. 查找角点 ---
    std::vector<cv::Point2f> imageCorners;
    cv::Size boardSize(m_patternCols, m_patternRows);

    bool found = cv::findChessboardCorners(currentImageGray,
        boardSize,
        imageCorners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK);

    if (!found) {
        // 即使未找到角点，也将失败结果保存到 Coordinate/result 目录
        saveCornerDetectionResult(savedImagePath, currentImage, imageCorners, false,
                                  getCalibrationTypePath("Coordinate"));
        showErrorMessage("在当前图像中未找到标定板角点。请确保标定板清晰可见。");
        logMessage("错误: 未检测到棋盘格角点");
        return;
    }

    // 亚像素精化
    cv::cornerSubPix(currentImageGray, imageCorners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));

    logMessage(QString("成功检测到 %1 个角点").arg(imageCorners.size()));

    // 保存角点检测结果图像到 Coordinate/result 目录
    saveCornerDetectionResult(savedImagePath, currentImage, imageCorners, true,
                              getCalibrationTypePath("Coordinate"));

    // --- 7. 创建居中的 3D 世界坐标 (原点在标定板中心) ---
    std::vector<cv::Point3f> objectPoints;
    float offsetX = ((float)m_patternCols - 1.0f) * m_squareSizeParam / 2.0f;
    float offsetY = ((float)m_patternRows - 1.0f) * m_squareSizeParam / 2.0f;

    for (int i = 0; i < m_patternRows; ++i) {
        for (int j = 0; j < m_patternCols; ++j) {
            objectPoints.push_back(cv::Point3f(
                j * m_squareSizeParam - offsetX,
                i * m_squareSizeParam - offsetY,
                0.0f
            ));
        }
    }

    // --- 8. 解算 PnP ---
    cv::Mat rvec, tvec, R_cam_to_board;
    cv::solvePnP(objectPoints, imageCorners, camMatrix, distCoeffs, rvec, tvec);
    cv::Rodrigues(rvec, R_cam_to_board); // rvec -> R

    // --- 9. 计算逆变换 (R_board_to_cam, T_board_to_cam) ---
    cv::Mat R_board_to_cam = R_cam_to_board.t();
    cv::Mat T_board_to_cam = -R_cam_to_board.t() * tvec;

    logMessage("成功计算坐标变换矩阵");

    // --- 10. 将新坐标系覆盖写入到投影仪标定文件 ---
    QString projCalibPath = getCalibrationTypePath("Projector") + QString("/ProjectorParams_%1.xml").arg(sideCode);
    QFile projCalibFile(projCalibPath);

    if (!projCalibFile.exists()) {
        showErrorMessage("calibration_data.xml 不存在。请先运行投影仪标定。");
        logMessage(QString("错误: 投影仪标定文件不存在: %1").arg(projCalibPath));
        return;
    }

    // 生成新的 R_BoardToCam 和 T_BoardToCam XML 片段
    QString rBoardToCamXml;
    QString tBoardToCamXml;
    {
        // 使用临时内存缓冲区生成 XML 片段
        cv::FileStorage fsTmp(".tmp_calib.xml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
        fsTmp << "R_BoardToCam" << R_board_to_cam;
        fsTmp << "T_BoardToCam" << T_board_to_cam;
        std::string allXml = fsTmp.releaseAndGetString();
        
        // 从完整 XML 中提取两个节点的片段
        QString fullXml = QString::fromStdString(allXml);
        QRegularExpression rRegex("<R_BoardToCam[\\s\\S]*?</R_BoardToCam>");
        QRegularExpression tRegex("<T_BoardToCam[\\s\\S]*?</T_BoardToCam>");
        
        QRegularExpressionMatch rMatch = rRegex.match(fullXml);
        QRegularExpressionMatch tMatch = tRegex.match(fullXml);
        
        if (rMatch.hasMatch()) {
            rBoardToCamXml = rMatch.captured(0);
        }
        if (tMatch.hasMatch()) {
            tBoardToCamXml = tMatch.captured(0);
        }
    }

    // 读取现有文件内容
    QFile inFile(projCalibPath);
    if (!inFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
        showErrorMessage("无法读取投影仪标定文件");
        logMessage(QString("错误: 无法读取投影仪标定文件: %1").arg(projCalibPath));
        return;
    }
    QString fileContent = QString::fromUtf8(inFile.readAll());
    inFile.close();

    // 使用正则表达式搜索并替换旧的坐标系数据
    QRegularExpression rOldRegex("<R_BoardToCam[\\s\\S]*?</R_BoardToCam>");
    QRegularExpression tOldRegex("<T_BoardToCam[\\s\\S]*?</T_BoardToCam>");
    
    bool hasOldR = rOldRegex.match(fileContent).hasMatch();
    bool hasOldT = tOldRegex.match(fileContent).hasMatch();

    if (hasOldR && !rBoardToCamXml.isEmpty()) {
        fileContent.replace(rOldRegex, rBoardToCamXml);
        logMessage("已覆盖旧的 R_BoardToCam 数据");
    }
    if (hasOldT && !tBoardToCamXml.isEmpty()) {
        fileContent.replace(tOldRegex, tBoardToCamXml);
        logMessage("已覆盖旧的 T_BoardToCam 数据");
    }

    // 如果原来没有这些节点，追加到 </opencv_storage> 之前
    if (!hasOldR && !rBoardToCamXml.isEmpty()) {
        fileContent.replace("</opencv_storage>", rBoardToCamXml + "\n</opencv_storage>");
        logMessage("已追加新的 R_BoardToCam 数据");
    }
    if (!hasOldT && !tBoardToCamXml.isEmpty()) {
        fileContent.replace("</opencv_storage>", tBoardToCamXml + "\n</opencv_storage>");
        logMessage("已追加新的 T_BoardToCam 数据");
    }

    // 写回文件
    QFile outFile(projCalibPath);
    if (!outFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        showErrorMessage("无法写入投影仪标定文件");
        logMessage(QString("错误: 无法写入投影仪标定文件: %1").arg(projCalibPath));
        return;
    }
    outFile.write(fileContent.toUtf8());
    outFile.close();

    logMessage(QString("坐标系标定完成，结果已覆盖保存到: %1").arg(projCalibPath));

    // 生成带侧向标识的文件名
    QString targetFileName = QString("SLCalibParams_%1.xml").arg(sideCode); 
    QString finalTargetPath = getDevicePath() + "/" + targetFileName;

    // 如果目标文件已存在，先删除它
    QFile targetFile(finalTargetPath);
    if (targetFile.exists()) {
        if (!targetFile.remove()) {
            logMessage(QString("警告: 无法删除已存在的文件: %1").arg(finalTargetPath));
        } else {
            logMessage(QString("已删除已存在的文件: %1").arg(finalTargetPath));
        }
    }

    // 复制文件（现在目标文件肯定不存在）
    if (QFile::copy(projCalibPath, finalTargetPath)) {
        logMessage(QString("ProjectorParams.xml已成功复制并重命名为: %1").arg(finalTargetPath));
    }
    else {
        logMessage("警告: ProjectorParams.xml复制失败");
    }

    // 更新进度条
    ui->progressBar_CoordinationCalibration->setValue(100);

    // 显示成功消息
    showSuccessMessage(QString("坐标系标定成功！\n原点已设置为标定板中心。\n\n结果已保存到: %1\n\nProjectorParams.xml已复制到RightSL和LeftSL目录\n\n[重要]\n请停止并重新开始重建，以加载和应用新的坐标系。")
                      .arg(projCalibPath));

    updateStatus("坐标系标定完成");

    // 发射标定完成信号
    emit calibrationCompleted(true, "Coordination calibration completed successfully");
}

void SingleCalibrationWidget::on_pushButton_deleteImagesCamera_clicked()
{
    // 获取相机标定图像路径
    QString cameraCalibPath = getCalibrationTypePath("Camera");
    if (cameraCalibPath.isEmpty()) {
        showErrorMessage("无法获取相机标定路径");
        logMessage("错误: 无法获取相机标定路径");
        return;
    }

    QDir dir(cameraCalibPath);
    if (!dir.exists()) {
        showErrorMessage("相机标定目录不存在");
        logMessage(QString("相机标定目录不存在: %1").arg(cameraCalibPath));
        return;
    }

    // 确认删除操作
    QMessageBox::StandardButton reply = QMessageBox::question(
        this,
        tr("确认删除"),
        tr("确定要删除所有相机标定图像吗？\n此操作不可恢复！"),
        QMessageBox::Yes | QMessageBox::No);

    if (reply != QMessageBox::Yes) {
        logMessage("用户取消删除相机标定图像操作");
        return;
    }

    // 定义图像文件过滤器
    QStringList imageFilters;
    imageFilters << "*.jpg" << "*.bmp" << "*.png" << "*.tiff" << "*.tif";

    // 获取所有图像文件
    QFileInfoList imageFiles = dir.entryInfoList(imageFilters, QDir::Files);

    int deletedCount = 0;
    int failedCount = 0;

    // 删除图像文件
    for (const QFileInfo& fileInfo : imageFiles) {
        QString filePath = fileInfo.absoluteFilePath();
        if (QFile::remove(filePath)) {
            deletedCount++;
            logMessage(QString("已删除: %1").arg(fileInfo.fileName()));
        }
        else {
            failedCount++;
            logMessage(QString("删除失败: %1").arg(fileInfo.fileName()));
        }
    }

    // 删除result子目录中的角点检测结果图像
    QString resultPath = cameraCalibPath + "/result";
    QDir resultDir(resultPath);
    if (resultDir.exists()) {
        QFileInfoList resultFiles = resultDir.entryInfoList(imageFilters, QDir::Files);
        for (const QFileInfo& fileInfo : resultFiles) {
            QString filePath = fileInfo.absoluteFilePath();
            if (QFile::remove(filePath)) {
                deletedCount++;
                logMessage(QString("已删除结果图: %1").arg(fileInfo.fileName()));
            }
        }
    }

    // 重置相关状态变量
    m_capturedImageCount = 0;

    // 更新进度条
    ui->progressBar_CameraCalibration->setValue(0);

    // 显示结果
    QString resultMsg = QString("相机标定图像清理完成\n删除: %1 个文件\n失败: %2 个文件")
        .arg(deletedCount).arg(failedCount);
    showSuccessMessage(resultMsg);
    logMessage(QString("相机标定图像清理完成 - 删除: %1, 失败: %2").arg(deletedCount).arg(failedCount));

    updateStatus("相机标定图像已清空");
}

void SingleCalibrationWidget::on_pushButton_deleteImagesProjector_clicked()
{
    // 获取投影仪标定图像路径
    QString projectorCalibPath = getCalibrationTypePath("Projector");
    if (projectorCalibPath.isEmpty()) {
        showErrorMessage("无法获取投影仪标定路径");
        logMessage("错误: 无法获取投影仪标定路径");
        return;
    }

    QDir dir(projectorCalibPath);
    if (!dir.exists()) {
        showErrorMessage("投影仪标定目录不存在");
        logMessage(QString("投影仪标定目录不存在: %1").arg(projectorCalibPath));
        return;
    }

    // 确认删除操作
    QMessageBox::StandardButton reply = QMessageBox::question(
        this,
        tr("确认删除"),
        tr("确定要删除所有投影仪标定图像吗？\n此操作不可恢复！"),
        QMessageBox::Yes | QMessageBox::No);

    if (reply != QMessageBox::Yes) {
        logMessage("用户取消删除投影仪标定图像操作");
        return;
    }

    // 定义图像文件过滤器
    QStringList imageFilters;
    imageFilters << "*.jpg" << "*.bmp" << "*.png" << "*.tiff" << "*.tif";

    // 获取所有图像文件
    QFileInfoList imageFiles = dir.entryInfoList(imageFilters, QDir::Files);

    int deletedCount = 0;
    int failedCount = 0;

    // 删除图像文件
    for (const QFileInfo& fileInfo : imageFiles) {
        QString filePath = fileInfo.absoluteFilePath();
        if (QFile::remove(filePath)) {
            deletedCount++;
            logMessage(QString("已删除: %1").arg(fileInfo.fileName()));
        }
        else {
            failedCount++;
            logMessage(QString("删除失败: %1").arg(fileInfo.fileName()));
        }
    }

    // 删除result子目录中的角点检测结果图像
    QString resultPath = projectorCalibPath + "/result";
    QDir resultDir(resultPath);
    if (resultDir.exists()) {
        QFileInfoList resultFiles = resultDir.entryInfoList(imageFilters, QDir::Files);
        for (const QFileInfo& fileInfo : resultFiles) {
            QString filePath = fileInfo.absoluteFilePath();
            if (QFile::remove(filePath)) {
                deletedCount++;
                logMessage(QString("已删除结果图: %1").arg(fileInfo.fileName()));
            }
        }
    }

    // 清空图像路径列表
    m_imagePaths.clear();

    // 重置投影仪标定Pose计数器
    m_nProjCaliPoseCounter = 0;

    // 更新进度条
    ui->progressBar_ProjectorCalibration->setValue(0);

    // 禁用标定按钮（因为图像已清空）
    ui->button_StartCalibrationProjector->setEnabled(false);

    // 显示结果
    QString resultMsg = QString("投影仪标定图像清理完成\n删除: %1 个文件\n失败: %2 个文件")
        .arg(deletedCount).arg(failedCount);
    showSuccessMessage(resultMsg);
    logMessage(QString("投影仪标定图像清理完成 - 删除: %1, 失败: %2").arg(deletedCount).arg(failedCount));

    updateStatus("投影仪标定图像已清空");
}

// ==================== 标定控制函数 ====================

bool SingleCalibrationWidget::captureCameraImages()
{
    return true;
}

bool SingleCalibrationWidget::calibrateCamera()
{
    logMessage(QString("开始执行%1相机标定").arg(m_sideIdentifier));

    // --- 1. 定义 3D 世界坐标 (Object Points) ---
    std::vector<cv::Point3f> objP;

    for (int i = 0; i < m_patternRows; ++i) // 行数
    {
        for (int j = 0; j < m_patternCols; ++j) // 列数
        {
            objP.push_back(cv::Point3f(j * m_squareSizeParam, i * m_squareSizeParam, 0));
        }
    }

    logMessage(QString("标定板规格: %1x%2, 方格尺寸: %3mm")
               .arg(m_patternCols).arg(m_patternRows).arg(m_squareSizeParam));

    // --- 2. 查找 Camera 标定文件夹中的所有图像 ---

    // 2.1. 获取标定图像存储路径
    QString calibrationPath = getCalibrationTypePath("Camera");
    if (calibrationPath.isEmpty()) {
        logMessage("错误: 无法获取相机标定路径");
        showErrorMessage("无法获取相机标定路径");
        return false;
    }

    QString searchPath = calibrationPath + "/*.jpg";
    logMessage(QString("搜索路径: %1").arg(searchPath));

    // 2.2. 遍历文件夹，查找所有图像文件
    QDir dir(calibrationPath);
    QStringList filters;
    filters << "*.jpg" << "*.bmp" << "*.png";
    QFileInfoList fileList = dir.entryInfoList(filters, QDir::Files, QDir::Name);

    std::vector<QString> imagePaths;
    for (const QFileInfo& fileInfo : fileList) {
        imagePaths.push_back(fileInfo.absoluteFilePath());
    }

    // --- 3. 显示找到的图像总数 ---
    int totalImages = static_cast<int>(imagePaths.size());
    if (totalImages == 0) {
        logMessage("错误: 在标定文件夹中未找到任何图像");
        showErrorMessage(QString("在 %1 路径下未找到任何标定图像").arg(calibrationPath));
        return false;
    }

    logMessage(QString("在标定路径下共找到 %1 张图像").arg(totalImages));

    // --- 4. 初始化标定所需变量 ---
    std::vector<std::vector<cv::Point3f>> allObjectPoints; // 存储所有图像的 3D 点
    std::vector<std::vector<cv::Point2f>> allImagePoints;  // 存储所有图像的 2D 角点

    int successCount = 0;
    int failCount = 0;

    // --- 5. 循环处理每张图像 ---
    for (int i = 0; i < totalImages; ++i)
    {
        QString imagePath = imagePaths[i];
        logMessage(QString("正在处理图像 %1/%2: %3").arg(i+1).arg(totalImages).arg(imagePath));

        cv::Mat img = cv::imread(imagePath.toStdString());
        if (img.empty())
        {
            logMessage(QString("警告: 无法加载图像: %1").arg(imagePath));
            failCount++;
            continue;
        }

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        // 查找棋盘格角点
        std::vector<cv::Point2f> imageCorners;
        bool found = cv::findChessboardCorners(gray,
            cv::Size(m_patternCols, m_patternRows), imageCorners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found)
        {
            successCount++;
            // 亚像素级精确化
            cv::cornerSubPix(gray, imageCorners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));

            allImagePoints.push_back(imageCorners);
            allObjectPoints.push_back(objP);

            logMessage(QString("成功检测到角点: %1个").arg(imageCorners.size()));
            
            // 保存角点检测结果图像
            saveCornerDetectionResult(imagePath, img, imageCorners, true, calibrationPath);
        }
        else
        {
            failCount++;
            logMessage(QString("未检测到棋盘格角点"));
            
            // 即使检测失败，也保存结果图像（标记为失败）
            saveCornerDetectionResult(imagePath, img, imageCorners, false, calibrationPath);
        }
    }

    if (allImagePoints.empty())
    {
        logMessage("错误: 未在任何图像中成功检测到角点，标定失败");
        showErrorMessage("未在任何图像中成功检测到角点，标定失败");
        return false;
    }

    // --- 6. 执行相机标定 ---
    logMessage("开始执行相机标定计算...");

    // 获取第一张图像的尺寸作为标定尺寸
    QString firstImagePath = imagePaths[0];
    cv::Mat firstImg = cv::imread(firstImagePath.toStdString());
    if (firstImg.empty()) {
        logMessage("错误: 无法加载第一张图像");
        showErrorMessage("无法加载第一张标定图像");
        return false;
    }
    cv::Size imageSize = firstImg.size();

    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(allObjectPoints, allImagePoints, imageSize,
        m_cameraMatrix, m_distCoeffs, rvecs, tvecs);

    // --- 7. 保存标定结果到文件 ---
    QString sideCode = m_isLeftDevice ? "Left" : "Right";
    QString savePath = calibrationPath + QString("/CameraParams_%1.xml")
                      .arg(sideCode);
    
    // 使用固定小数格式保存，避免科学计数法
    // 手动构建XML内容以确保格式正确
    bool bSaveSuccess = false;
    
    {
        std::ofstream outFile(savePath.toStdString());
        if (outFile.is_open())
        {
            outFile << std::fixed << std::setprecision(15);
            outFile << "<?xml version=\"1.0\"?>\n";
            outFile << "<opencv_storage>\n";
            
            // 写入相机内参矩阵 (3x3)
            outFile << "<cameraMatrix type_id=\"opencv-matrix\">\n";
            outFile << "  <rows>3</rows>\n";
            outFile << "  <cols>3</cols>\n";
            outFile << "  <dt>d</dt>\n";
            outFile << "  <data>\n";
            for (int i = 0; i < m_cameraMatrix.rows; i++) {
                for (int j = 0; j < m_cameraMatrix.cols; j++) {
                    outFile << m_cameraMatrix.at<double>(i, j) << " ";
                }
                outFile << "\n";
            }
            outFile << "  </data>\n";
            outFile << "</cameraMatrix>\n";
            
            // 确保畸变系数为5行1列格式
            cv::Mat distCoeffsCol;
            if (m_distCoeffs.rows == 1 && m_distCoeffs.cols >= 5) {
                // 转换为5x1列向量
                distCoeffsCol = m_distCoeffs.colRange(0, 5).t();
            } else if (m_distCoeffs.rows >= 5 && m_distCoeffs.cols == 1) {
                distCoeffsCol = m_distCoeffs.rowRange(0, 5).clone();
            } else if (m_distCoeffs.rows == 5 && m_distCoeffs.cols == 1) {
                distCoeffsCol = m_distCoeffs.clone();
            } else {
                // 默认取前5个元素
                distCoeffsCol = cv::Mat::zeros(5, 1, CV_64F);
                int count = std::min(5, m_distCoeffs.rows * m_distCoeffs.cols);
                for (int i = 0; i < count; i++) {
                    distCoeffsCol.at<double>(i, 0) = m_distCoeffs.at<double>(i);
                }
            }
            
            // 写入畸变系数 (5行1列)
            outFile << "<distCoeffs type_id=\"opencv-matrix\">\n";
            outFile << "  <rows>5</rows>\n";
            outFile << "  <cols>1</cols>\n";
            outFile << "  <dt>d</dt>\n";
            outFile << "  <data>\n";
            for (int i = 0; i < 5; i++) {
                outFile << distCoeffsCol.at<double>(i, 0) << "\n";
            }
            outFile << "  </data>\n";
            outFile << "</distCoeffs>\n";
            
            // 写入其他参数
            outFile << "<rms>" << rms << "</rms>\n";
            outFile << "<imageSize_width>" << imageSize.width << "</imageSize_width>\n";
            outFile << "<imageSize_height>" << imageSize.height << "</imageSize_height>\n";
            outFile << "<patternRows>" << m_patternRows << "</patternRows>\n";
            outFile << "<patternCols>" << m_patternCols << "</patternCols>\n";
            outFile << "<squareSize>" << m_squareSizeParam << "</squareSize>\n";
            outFile << "<successfulImages>" << successCount << "</successfulImages>\n";
            outFile << "<failedImages>" << failCount << "</failedImages>\n";
            outFile << "<totalImages>" << totalImages << "</totalImages>\n";
            outFile << "</opencv_storage>\n";
            
            outFile.close();
            bSaveSuccess = true;
            logMessage(QString("标定结果已保存到: %1").arg(savePath));
        }
        else
        {
            logMessage(QString("警告: 无法保存标定结果到: %1").arg(savePath));
        }
    }

    // --- 8. 显示标定结果 ---
    QString resultMsg = QString("Camera calibration completed!\n\n"
                               "--- Statistics ---\n"
                               "Total images: %1\n"
                               "Successful detections: %2\n"
                               "Failed detections: %3\n\n"
                               "--- Calibration Results ---\n"
                               "Reprojection error (RMS): %4\n\n"
                               "Camera intrinsic matrix:\n")
                       .arg(totalImages).arg(successCount).arg(failCount).arg(rms, 0, 'f', 3);

    // 格式化相机内参矩阵
    for (int i = 0; i < m_cameraMatrix.rows; i++)
    {
        for (int j = 0; j < m_cameraMatrix.cols; j++)
        {
            resultMsg += QString("%1 ").arg(m_cameraMatrix.at<double>(i, j), 10, 'f', 4);
        }
        resultMsg += "\n";
    }

    // 格式化畸变系数
    resultMsg += "\nDistortion coefficients:\n";
    for (int i = 0; i < m_distCoeffs.rows && i < 5; i++)  // 显示前5个畸变系数
    {
        resultMsg += QString("k%1: %2\n").arg(i+1).arg(m_distCoeffs.at<double>(i, 0), 0, 'f', 6);
    }

    if (bSaveSuccess)
    {
        resultMsg += QString("\n--- Save Info ---\nCalibration results saved to:\n%1").arg(savePath);
    }
    else
    {
        resultMsg += "\n--- Save Failed ---\nUnable to write calibration results file!";
    }

    logMessage(QString("Calibration completed - RMS error: %1, Success rate: %2%%")
               .arg(rms, 0, 'f', 3).arg((double)successCount/totalImages*100, 0, 'f', 1));

    showSuccessMessage(resultMsg);

    // 更新进度条
    ui->progressBar_CameraCalibration->setValue(100);

    // 发射标定完成信号
    emit calibrationCompleted(true, QString("Camera calibration completed successfully - RMS: %1").arg(rms, 0, 'f', 3));

    return true;
}

bool SingleCalibrationWidget::captureProjectorSequence()
{

    return true;
}

bool SingleCalibrationWidget::loadProjectorImages()
{
    logMessage(QString("读取%1投影仪图像").arg(m_sideIdentifier));

    // 获取投影仪标定图像路径
    QString calibrationPath = getCalibrationTypePath("Projector");
    if (calibrationPath.isEmpty()) {
        logMessage("错误: 无法获取投影仪标定路径");
        return false;
    }

    logMessage(QString("搜索投影仪标定图像路径: %1").arg(calibrationPath));

    // 查找所有图像文件
    QDir dir(calibrationPath);
    QStringList filters;
    filters << "*.jpg" << "*.bmp" << "*.png" << "*.tiff" << "*.tif";
    QFileInfoList fileList = dir.entryInfoList(filters, QDir::Files, QDir::Name);

    if (fileList.isEmpty()) {
        logMessage(QString("在路径 %1 中未找到任何图像文件").arg(calibrationPath));
        return false;
    }

    // 清空之前的图像路径
    m_imagePaths.clear();

    // 添加所有找到的图像路径
    for (const QFileInfo& fileInfo : fileList) {
        m_imagePaths.push_back(fileInfo.absoluteFilePath().toStdString());
        logMessage(QString("添加图像: %1").arg(fileInfo.fileName()));
    }

    logMessage(QString("成功加载 %1 张投影仪标定图像").arg(m_imagePaths.size()));



    return !m_imagePaths.empty();
}


// ==================== 参数设置槽函数 ====================

void SingleCalibrationWidget::on_spinBox_PatternRows_valueChanged(int arg1)
{
    m_patternRows = arg1;
}

void SingleCalibrationWidget::on_spinBox_PatternCols_valueChanged(int arg1)
{
    m_patternCols = arg1;
}

void SingleCalibrationWidget::on_doubleSpinBox_SquareSize_valueChanged(double arg1)
{
    m_squareSizeParam = arg1;
}

// ==================== 状态更新函数 ====================

void SingleCalibrationWidget::updateStatus(const QString& message)
{
    logMessage(message);

    // 更新UI状态显示
    if (ui->label_Status) {
        ui->label_Status->setText(QString("状态: %1 - %2").arg(m_sideIdentifier).arg(message));
    }
}

void SingleCalibrationWidget::onCalibrationFinished(bool success)
{
    // 重新启用标定按钮
    ui->button_StartCalibrationProjector->setEnabled(true);

    if (success) {
        constexpr double kEpipolarWarnThresholdPx = 1.0;

        // 保存标定结果到投影仪标定文件
        QString sideCode = m_isLeftDevice ? "Left" : "Right";
        QString resultPath = getCalibrationTypePath("Projector") + QString("/ProjectorParams_%1.xml").arg(sideCode);
        QFileInfo fileInfo(resultPath);
        QDir().mkpath(fileInfo.absolutePath());

        // 尝试保存标定结果
        bool saveSuccess = m_calibrator->saveCalibrationData(m_calibData, resultPath.toStdString());

        if (saveSuccess) {
            QString epiSummary;
            QString epiWarning;
            if (m_calibData.epiValidCount > 0) {
                epiSummary = QString("极线误差(像素): mean=%1, median=%2, P95=%3, max=%4, N=%5")
                                .arg(m_calibData.epiMeanPx, 0, 'f', 6)
                                .arg(m_calibData.epiMedianPx, 0, 'f', 6)
                                .arg(m_calibData.epiP95Px, 0, 'f', 6)
                                .arg(m_calibData.epiMaxPx, 0, 'f', 6)
                                .arg(m_calibData.epiValidCount);
                if (m_calibData.epiMeanPx > kEpipolarWarnThresholdPx) {
                    epiWarning = QString("\n[告警] 极线误差均值超过阈值(%1 px)，建议重采高质量姿态后重新标定。")
                                    .arg(kEpipolarWarnThresholdPx, 0, 'f', 1);
                    logMessage(QString("告警: 极线误差均值过高(mean=%1 px, threshold=%2 px)")
                                   .arg(m_calibData.epiMeanPx, 0, 'f', 6)
                                   .arg(kEpipolarWarnThresholdPx, 0, 'f', 1));
                }
            } else {
                epiSummary = "极线误差(像素): 无有效点，未输出统计";
                logMessage("警告: 极线误差统计失败或有效点数量不足");
            }

            showSuccessMessage(QString("投影仪标定成功完成！\n结果已保存到: %1\n投影仪RMS误差: %2\n立体标定RMS误差: %3\n%4%5")
                                  .arg(resultPath)
                                  .arg(m_calibData.rmsProj, 0, 'f', 6)
                                  .arg(m_calibData.rmsStereo, 0, 'f', 6)
                                  .arg(epiSummary)
                                  .arg(epiWarning));
            logMessage(QString("标定结果已保存到: %1").arg(resultPath));

            QString deviceSide = m_isLeftDevice ? "left" : "right";
            QString calibDetails = QString("epi_p95_px=%1;epi_max_px=%2;epi_valid_count=%3;pattern_rows=%4;pattern_cols=%5;square_size_mm=%6")
                                       .arg(m_calibData.epiP95Px, 0, 'f', 6)
                                       .arg(m_calibData.epiMaxPx, 0, 'f', 6)
                                       .arg(m_calibData.epiValidCount)
                                       .arg(m_patternRows)
                                       .arg(m_patternCols)
                                       .arg(m_squareSizeParam, 0, 'f', 3);

            bool recordOk = QADbManager::instance().insertCalibrationRecord(
                QDate::currentDate(),
                deviceSide,
                m_cameraSerialNumber,
                m_deviceTag,
                resultPath,
                m_calibData.rmsProj,
                m_calibData.rmsStereo,
                m_calibData.epiMeanPx,
                m_calibData.epiMedianPx,
                calibDetails
            );
            if (recordOk) {
                logMessage(QString("标定记录已写入数据库: side=%1, rmsProj=%2, rmsStereo=%3")
                               .arg(deviceSide)
                               .arg(m_calibData.rmsProj, 0, 'f', 6)
                               .arg(m_calibData.rmsStereo, 0, 'f', 6));
            } else {
                logMessage("警告: 标定记录写入数据库失败");
            }
        } else {
            showErrorMessage("标定完成但保存结果失败");
            logMessage("警告: 标定结果保存失败");
        }
    } else {
        showErrorMessage(QString("%1 投影仪标定失败，请检查图像质量和参数设置"));
    }
}

void SingleCalibrationWidget::updateCameraSNDisplay()
{
    if (ui->lineEdit_CameraSN) {
        ui->lineEdit_CameraSN->setText(m_cameraSerialNumber.isEmpty() ? "未设置" : m_cameraSerialNumber);
    }
}

void SingleCalibrationWidget::updateStatusDisplay()
{
    QString statusText;
    if (!m_cameraSerialNumber.isEmpty()) {
        statusText = QString("状态: %1 - 就绪").arg(m_sideIdentifier);
    } else {
        statusText = QString("状态: %1 - 待机").arg(m_sideIdentifier);
    }

    if (ui->label_Status) {
        ui->label_Status->setText(statusText);
    }
}

void SingleCalibrationWidget::showErrorMessage(const QString& message)
{
    QMessageBox::critical(this, "错误", message);
}

void SingleCalibrationWidget::showSuccessMessage(const QString& message)
{
    QMessageBox::information(this, "成功", message);
}

void SingleCalibrationWidget::logMessage(const QString& message)
{
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString logEntry = QString("[%1] %2").arg(timestamp).arg(message);

    if (ui->textEdit_Log) {
        ui->textEdit_Log->append(logEntry);

        // 限制日志行数
        QTextCursor cursor = ui->textEdit_Log->textCursor();
        cursor.movePosition(QTextCursor::End);
        while (ui->textEdit_Log->document()->blockCount() > 100) {
            cursor.movePosition(QTextCursor::Start);
            cursor.select(QTextCursor::BlockUnderCursor);
            cursor.removeSelectedText();
            cursor.deleteChar();
        }
    }

    // 发射状态更新信号
    emit statusUpdated(message);
}

// ==================== 新增工具函数 ====================

void SingleCalibrationWidget::logAndShow(const QString& message, bool isError)
{
    logMessage(message);
    if (isError) {
        showErrorMessage(message);
    } else {
        showSuccessMessage(message);
    }
}

QString SingleCalibrationWidget::formatPoseMsg(int poseNumber, const QString& message) const
{
    return QString("[Pose %1] %2").arg(formatPoseNumber(poseNumber)).arg(message);
}

QString SingleCalibrationWidget::formatPoseNumber(int poseNumber) const
{
    return CalibrationUtils::formatPoseNumber(poseNumber);
}

// ==================== 软触发图像采集函数 ====================

bool SingleCalibrationWidget::captureSingleImageWithSoftwareTrigger()
{
    cv::Mat image;
    if (!captureImageAndReturn(image)) {
        return false;
    }

    // 保存图像到相机标定目录
    QString calibrationPath = getCalibrationTypePath("Camera");
    if (calibrationPath.isEmpty()) {
        logMessage("错误: 无法获取标定路径");
        return false;
    }

    if (!saveImageToCalibrationPath(image, calibrationPath)) {
        logMessage("错误: 图像保存失败");
        return false;
    }

    logMessage("软触发图像采集完成");
    return true;
}

bool SingleCalibrationWidget::captureImageAndReturn(cv::Mat& outImage)
{
    if (!m_isCameraOpen || !m_cameraHandle) {
        logMessage("错误: 相机未打开，无法执行软触发");
        return false;
    }

    if (!m_cameraController) {
        logMessage("错误: 相机控制器未初始化");
        return false;
    }

    // 确保缓冲区已分配
    if (!m_pImageBuffer) {
        m_pImageBuffer = new unsigned char[5 * 1024 * 1024];
        m_imageBufferSize = 5 * 1024 * 1024;
    }

    logMessage("执行软触发图像采集...");
    
    // 启动采集流
    if (!m_cameraController->StartImageCapture(m_cameraHandle)) {
        logMessage("错误: 启动采集流失败");
        return false;
    }
    
    // 设置软触发模式
    m_cameraController->SetTriggerMode(m_cameraHandle, 1);
    m_cameraController->SetTriggerSource(m_cameraHandle, "Software");
    
    // 发送软件触发信号
    if (!m_cameraController->TriggerSoftware(m_cameraHandle)) {
        logMessage("错误: 发送软件触发信号失败");
        m_cameraController->StopImageCapture(m_cameraHandle);
        return false;
    }
    
    logMessage("软件触发信号已发送，等待图像返回...");

    // 获取图像帧
    MV_FRAME_OUT_INFO_EX frameInfo = {0};
    if (!m_cameraController->GetImage(m_cameraHandle, m_pImageBuffer, static_cast<unsigned int>(m_imageBufferSize), &frameInfo)) {
        logMessage("错误: 获取图像帧失败");
        m_cameraController->StopImageCapture(m_cameraHandle);
        return false;
    }
    
    // 停止采集流
    m_cameraController->StopImageCapture(m_cameraHandle);
    
    // 将原始数据转换为cv::Mat（需要深拷贝，因为缓冲区会被复用）
    cv::Mat tempImage(frameInfo.nHeight, frameInfo.nWidth, CV_8UC1, m_pImageBuffer);
    outImage = tempImage.clone();

    // 检查图像是否有效
    if (outImage.empty()) {
        logMessage("错误: 获取到空图像");
        return false;
    }

    // 成功获取图像，记录详细信息
    logMessage(QString("软触发取图成功 - 图像尺寸: %1x%2")
               .arg(outImage.cols).arg(outImage.rows));

    // 刷新关联的相机控制组件UI状态
    if (m_cameraWidget) {
        m_cameraWidget->updateCameraStatus();
        m_cameraWidget->updateParameterControls();
    }

    return true;
}

// ==================== 图像存储函数 ====================
// 注意：以下路径管理函数现在委托给 CalibrationUtils

QString SingleCalibrationWidget::getBaseCalibrationPath() const
{
    return CalibrationUtils::getBaseCalibrationPath();
}

QString SingleCalibrationWidget::getDevicePath() const
{
    return CalibrationUtils::getDevicePath(m_isLeftDevice);
}

QString SingleCalibrationWidget::getCalibrationTypePath(const QString& calibType) const
{
    return CalibrationUtils::getCalibrationTypePath(m_isLeftDevice, calibType);
}

bool SingleCalibrationWidget::createDirectoryIfNotExists(const QString& path) const
{
    return CalibrationUtils::createDirectoryIfNotExists(path);
}

bool SingleCalibrationWidget::saveCalibrationImage(const cv::Mat& image, const QString& calibType, int imageIndex)
{
    if (image.empty()) {
        logMessage("错误: 尝试保存空图像");
        return false;
    }

    bool success = CalibrationUtils::saveCalibrationImage(image, calibType, imageIndex, m_isLeftDevice);
    
    if (success) {
        QString typePath = getCalibrationTypePath(calibType);
        QString fileName = QString("image_%1_%2.jpg")
                          .arg(imageIndex, 3, 10, QLatin1Char('0'))
                          .arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
        logMessage(QString("图像已保存: %1/%2").arg(typePath).arg(fileName));
    } else {
        logMessage("图像保存失败");
    }
    
    return success;
}

bool SingleCalibrationWidget::saveImageToCalibrationPath(const cv::Mat& image, const QString& calibrationPath)
{
    if (image.empty()) {
        logMessage("错误: 尝试保存空图像");
        return false;
    }

    // 使用现有的标定图像保存函数，传入固定的图像索引
    static int softwareTriggerImageIndex = 0;
    softwareTriggerImageIndex++;

    // 调用已有的存图函数，保存到Camera标定目录
    bool success = saveCalibrationImage(image, "Camera", softwareTriggerImageIndex);

    if (success) {
        logMessage(QString("软触发图像已通过标准流程保存，索引: %1").arg(softwareTriggerImageIndex));
    } else {
        logMessage("软触发图像保存失败");
    }

    return success;
}

// ==================== 角点检测结果存储函数 ====================

bool SingleCalibrationWidget::saveCornerDetectionResult(
    const QString& imagePath,
    const cv::Mat& image,
    const std::vector<cv::Point2f>& corners,
    bool detectionSuccess,
    const QString& calibrationPath)
{
    // 确保 result 目录存在
    QFileInfo imageFileInfo(imagePath);
    QString resultDirPath = imageFileInfo.absolutePath() + "/result";
    QDir resultDir(resultDirPath);
    if (!resultDir.exists()) {
        if (!resultDir.mkpath(".")) {
            logMessage(QString("错误: 无法创建结果目录: %1").arg(resultDirPath));
            return false;
        }
        logMessage(QString("已创建结果目录: %1").arg(resultDirPath));
    }
    
    // 委托给 CalibrationUtils 保存角点检测结果
    bool success = CalibrationUtils::saveCornerDetectionResult(
        imagePath, image, corners, detectionSuccess, m_patternCols, m_patternRows);
    
    if (success) {
        QString resultFilePath = resultDirPath + "/" + imageFileInfo.completeBaseName() + "_corners.jpg";
        logMessage(QString("角点检测结果图像已保存: %1").arg(resultFilePath));
    } else {
        logMessage("角点检测结果图像保存失败");
    }
    
    return success;
}

// ==================== 坐标系标定目录管理函数 ====================

bool SingleCalibrationWidget::clearCoordinateDirectory()
{
    bool success = CalibrationUtils::clearCoordinateDirectory(m_isLeftDevice);
    if (success) {
        logMessage("坐标系标定目录已清空");
    }
    return success;
}

bool SingleCalibrationWidget::saveCoordinateImage(const cv::Mat& image, QString& savedImagePath)
{
    if (image.empty()) {
        logMessage("错误: 尝试保存空图像到坐标系标定目录");
        return false;
    }

    bool success = CalibrationUtils::saveCoordinateImage(image, m_isLeftDevice, savedImagePath);
    
    if (success) {
        logMessage(QString("坐标系标定图像已保存: %1").arg(savedImagePath));
    } else {
        logMessage("坐标系标定图像保存失败");
    }
    
    return success;
}

// ==================== 投影仪标定图像采集辅助函数 ====================

int SingleCalibrationWidget::getNextPoseNumber(const QString& projectorCalibPath) const
{
    return CalibrationUtils::getNextPoseNumber(projectorCalibPath);
}

