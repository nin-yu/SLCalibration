#include "singlecameracontrolwidget.h"
#include "ui_singlecameracontrolwidget.h"
#include <QDateTime>
#include <QFileDialog>
#include <QStandardPaths>
#include <QTimerEvent>
#include <QApplication>

SingleCameraControlWidget::SingleCameraControlWidget(CCameraController* controller, QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::SingleCameraControlWidget)
    , m_cameraController(controller)
    , m_statusTimer(nullptr)
    , m_grabTimerId(0)
    , m_cameraHandle(nullptr)
    , m_pImageBuffer(nullptr)
    , m_imageBufferSize(0)
    , m_frameCounter(0)
    , m_lastFpsTime(QTime::currentTime())
    , m_isContinuousMode(false)
{
    ui->setupUi(this);

    // 初始化组件
    initializeUI();
    
    // 启动状态定时器
    m_statusTimer = new QTimer(this);
    connect(m_statusTimer, &QTimer::timeout, this, &SingleCameraControlWidget::onTimeout_UpdateStatus);
    m_statusTimer->start(5000); // 每5秒更新一次状态

    logMessage("单相机控制组件初始化完成");
}

SingleCameraControlWidget::~SingleCameraControlWidget()
{
    // 停止图像采集定时器
    if (m_grabTimerId != 0) {
        killTimer(m_grabTimerId);
        m_grabTimerId = 0;
    }
    
    // 清理资源
    if (m_cameraController && m_cameraHandle) {
        if (isGrabbing()) {
            m_cameraController->StopImageCapture(m_cameraHandle);
            logMessage("图像采集已停止");
        }
        if (isCameraOpen()) {
            m_cameraController->CloseCamera(m_cameraHandle);
            logMessage("相机已关闭");
        }
    }
    
    // 释放图像缓冲区
    if (m_pImageBuffer) {
        delete[] m_pImageBuffer;
        m_pImageBuffer = nullptr;
    }

    // 注意：不删除m_cameraController，因为它是由外部传入的单例

    delete ui;
}

// ==================== 状态查询函数 ====================

bool SingleCameraControlWidget::isCameraOpen() const
{
    if (!m_cameraController || !m_cameraHandle) {
        return false;
    }
    return m_cameraController->IsCameraConnected(m_cameraHandle);
}

bool SingleCameraControlWidget::isGrabbing() const
{
    if (!m_cameraController || !m_cameraHandle) {
        return false;
    }
    return m_cameraController->IsCapturing(m_cameraHandle);
}

// ==================== 初始化函数 ====================

void SingleCameraControlWidget::initializeUI()
{
    // 设置默认值
    ui->spinBox_Exposure->setValue(25000);  // 25.00ms
    ui->doubleSpinBox_Gain->setValue(0.0);   // 0dB
    ui->comboBox_TriggerMode->setCurrentIndex(0);  // 内触发
    ui->comboBox_TriggerEdge->setCurrentIndex(1);  // 上升沿
    ui->doubleSpinBox_TriggerDelay->setValue(0.0); // 无延迟

    // 初始化触发源下拉框（UI文件中已定义选项，这里设置默认选择）
    ui->comboBox_TriggerSource->setCurrentIndex(1);  // 默认选择 Line0

    // 更新UI状态
    updateCameraStatus();
    updateParameterControls();
}


// ==================== UI事件槽函数 ====================

void SingleCameraControlWidget::on_pushButton_OpenCamera_clicked()
{
    QString serialNumber = ui->lineEdit_SN->text().trimmed();
    if (serialNumber.isEmpty()) {
        showErrorMessage("请输入相机序列号");
        return;
    }

    if (isCameraOpen()) {
        showErrorMessage("相机已经打开");
        return;
    }

    logMessage(QString("尝试打开相机: %1").arg(serialNumber));

    if (!openCameraBySerialNumber(serialNumber)) {
        return;
    }

    // 更新UI
    updateCameraStatus();
    updateParameterControls();

    logMessage("相机打开完成", true);  // 通知主窗口
}

void SingleCameraControlWidget::on_pushButton_CloseCamera_clicked()
{
    if (!isCameraOpen()) {
        showErrorMessage("相机未打开");
        return;
    }

    if (isGrabbing()) {
        showErrorMessage("请先停止图像采集");
        return;
    }

    logMessage("正在关闭相机...");

    if (m_cameraController && m_cameraHandle) {
        if (m_cameraController->CloseCamera(m_cameraHandle)) {
            m_cameraHandle = nullptr;

            updateCameraStatus();
            updateParameterControls();

            logMessage("相机已关闭", true);  // 通知主窗口
        } else {
            showErrorMessage("关闭相机失败");
            logMessage("相机关闭失败", true);  // 通知主窗口
        }
    }
}

void SingleCameraControlWidget::on_pushButton_StartGrabbing_clicked()
{
    if (!m_cameraController) {
        showErrorMessage("相机控制器未初始化");
        return;
    }

    if (!isCameraOpen()) {
        showErrorMessage("请先打开相机");
        return;
    }

    if (isGrabbing()) {
        showErrorMessage("图像采集已在运行");
        return;
    }

    logMessage("配置相机参数...");
    if (!configureCameraParameters()) {
        return;
    }
    
    // 设置为连续采集模式（TriggerMode=0）
    m_cameraController->SetTriggerMode(m_cameraHandle, 0);
    logMessage("设置触发模式: 连续采集(TriggerMode=0)");
    
    logMessage("启动连续采集模式...");
    if (m_cameraController->StartImageCapture(m_cameraHandle)) {
        // 启动主动取图定时器（20ms间隔）
        m_grabTimerId = startTimer(20);
        m_frameCounter = 0;
        m_lastFpsTime = QTime::currentTime();
        m_isContinuousMode = true;  // 标记为连续采集模式
        
        updateCameraStatus();
        updateParameterControls();
        logMessage("连续采集已启动", true);  // 通知主窗口
    } else {
        showErrorMessage("启动图像采集失败");
        logMessage("图像采集启动失败", true);  // 通知主窗口
    }
}

void SingleCameraControlWidget::on_pushButton_StopGrabbing_clicked()
{
    if (!isGrabbing()) {
        showErrorMessage("图像采集未运行");
        return;
    }

    logMessage("正在停止图像采集...");

    // 停止定时器
    if (m_grabTimerId != 0) {
        killTimer(m_grabTimerId);
        m_grabTimerId = 0;
    }
    
    // 清除连续采集模式标记
    m_isContinuousMode = false;

    if (m_cameraController && m_cameraHandle) {
        if (m_cameraController->StopImageCapture(m_cameraHandle)) {
            updateCameraStatus();
            updateParameterControls();
            logMessage("连续采集已停止", true);  // 通知主窗口
        } else {
            showErrorMessage("停止图像采集失败");
            logMessage("图像采集停止失败", true);  // 通知主窗口
        }
    }
}

void SingleCameraControlWidget::on_pushButton_SoftWareTrigger_clicked()
{
    // 检查相机控制器
    if (!m_cameraController) {
        showErrorMessage("相机控制器未初始化");
        return;
    }

    // 检查相机状态
    if (!isCameraOpen()) {
        showErrorMessage("请先打开相机");
        return;
    }

    // 检查是否处于连续采集模式
    if (isGrabbing()) {
        showErrorMessage("请先停止连续采集模式");
        return;
    }

    logMessage("开始软触发采图...");

    // Step 1: 启动采集流
    if (!m_cameraController->StartImageCapture(m_cameraHandle)) {
        showErrorMessage("启动采集流失败");
        logMessage("启动采集流失败");
        return;
    }
    logMessage("采集流已启动");

    // Step 2: 设置软触发模式（TriggerMode=1, TriggerSource=Software）
    if (!m_cameraController->SetTriggerMode(m_cameraHandle, 1)) {
        logMessage("警告: 设置触发模式失败");
    } else {
        logMessage("触发模式设置为: 外触发(TriggerMode=1)");
    }
    
    if (!m_cameraController->SetTriggerSource(m_cameraHandle, "Software")) {
        logMessage("警告: 设置触发源失败");
    } else {
        logMessage("触发源设置为: Software");
    }

    // Step 3: 发送软触发指令
    if (!m_cameraController->TriggerSoftware(m_cameraHandle)) {
        showErrorMessage("发送软件触发信号失败");
        logMessage("软件触发信号发送失败");
        m_cameraController->StopImageCapture(m_cameraHandle);
        return;
    }
    logMessage("软触发信号已发送，等待图像...");

    // Step 4: 获取图像
    // 确保缓冲区已分配
    if (!m_pImageBuffer) {
        m_pImageBuffer = new unsigned char[DEFAULT_BUFFER_SIZE];
        m_imageBufferSize = DEFAULT_BUFFER_SIZE;
    }
    
    MV_FRAME_OUT_INFO_EX frameInfo = {0};
    if (!m_cameraController->GetImage(m_cameraHandle, m_pImageBuffer, static_cast<unsigned int>(m_imageBufferSize), &frameInfo)) {
        showErrorMessage("获取图像帧失败");
        logMessage("图像帧获取失败");
        m_cameraController->StopImageCapture(m_cameraHandle);
        return;
    }

    // Step 5: 转换图像
    cv::Mat capturedImage(frameInfo.nHeight, frameInfo.nWidth, CV_8UC1, m_pImageBuffer);
    if (capturedImage.empty()) {
        showErrorMessage("获取到空图像");
        logMessage("获取到空图像");
        m_cameraController->StopImageCapture(m_cameraHandle);
        return;
    }

    // Step 6: 缩放图像并发送信号到主窗口
    cv::Mat scaledImage;
    cv::resize(capturedImage, scaledImage, cv::Size(), 0.25, 0.25, cv::INTER_AREA);
    
    QImage qImage;
    if (scaledImage.channels() == 1) {
        qImage = QImage(scaledImage.data, scaledImage.cols, scaledImage.rows,
                        scaledImage.step, QImage::Format_Grayscale8);
    } else if (scaledImage.channels() == 3) {
        cv::Mat rgbImage;
        cv::cvtColor(scaledImage, rgbImage, cv::COLOR_BGR2RGB);
        qImage = QImage(rgbImage.data, rgbImage.cols, rgbImage.rows,
                        rgbImage.step, QImage::Format_RGB888);
    }
    
    if (!qImage.isNull()) {
        emit imageReceived(qImage.copy());
    }

    // Step 7: 停止采集流
    m_cameraController->StopImageCapture(m_cameraHandle);
    logMessage("采集流已停止");

    // Step 8: 更新UI显示的触发模式和触发源状态
    ui->comboBox_TriggerMode->blockSignals(true);
    ui->comboBox_TriggerMode->setCurrentIndex(1);  // 显示为外触发
    ui->comboBox_TriggerMode->blockSignals(false);
    
    ui->comboBox_TriggerSource->blockSignals(true);
    ui->comboBox_TriggerSource->setCurrentIndex(0);  // 显示为Software
    ui->comboBox_TriggerSource->blockSignals(false);

    logMessage(QString("软触发采图成功 - 图像尺寸: %1x%2")
               .arg(capturedImage.cols).arg(capturedImage.rows), true);

    // 保存图像到文件（保留原有功能）
    saveCapturedImage(capturedImage);
    
    // 更新UI状态
    updateCameraStatus();
    updateParameterControls();
}

// ==================== 公共接口函数 ====================

bool SingleCameraControlWidget::setCameraSerialNumber(const QString& serialNumber)
{
    QString trimmedSN = serialNumber.trimmed();
    if (trimmedSN.isEmpty()) {
        logMessage("错误: 传入的相机序列号为空");
        return false;
    }
    
    logMessage(QString("通过公共接口设置相机序列号: %1").arg(trimmedSN));
    
    // 检查相机控制器是否有效
    if (!m_cameraController) {
        logMessage("错误: 相机控制器未初始化");
        return false;
    }
    
    // 如果序列号相同，无需重复设置
    if (trimmedSN == m_currentSerialNumber && m_cameraHandle) {
        logMessage(QString("序列号未变更: %1").arg(trimmedSN));
        return true;
    }
    
    // 如果相机正在采集，先安全停止
    if (isGrabbing()) {
        logMessage("检测到正在进行的图像采集，正在安全停止...");
        // 停止定时器
        if (m_grabTimerId != 0) {
            killTimer(m_grabTimerId);
            m_grabTimerId = 0;
        }
        if (m_cameraController && m_cameraHandle) {
            m_cameraController->StopImageCapture(m_cameraHandle);
            logMessage("图像采集已停止");
        }
        // 等待一段时间确保停止完成
        QThread::msleep(100);
    }
    
    // 如果相机已打开，先安全关闭
    if (isCameraOpen()) {
        logMessage("检测到已打开的相机，正在安全关闭...");
        if (m_cameraController && m_cameraHandle) {
            m_cameraController->CloseCamera(m_cameraHandle);
            m_cameraHandle = nullptr;
            logMessage("相机已关闭");
        }
        // 等待一段时间确保关闭完成
        QThread::msleep(100);
    }
    
    // 更新内部状态
    m_currentSerialNumber = trimmedSN;
    
    // 更新UI显示
    if (ui && ui->lineEdit_SN) {
        ui->lineEdit_SN->setText(trimmedSN);
        logMessage(QString("UI序列号显示已更新为: %1").arg(trimmedSN));
    }
    
    // 刷新状态显示
    updateCameraStatus();
    
    logMessage(QString("相机序列号设置完成: %1").arg(trimmedSN));
    return true;
}

// ==================== 参数设置槽函数 ====================
void SingleCameraControlWidget::on_pushButton_SetSN_clicked() // 取消自动开始采集
{
    QString serialNumber = ui->lineEdit_SN->text().trimmed();
    if (serialNumber.isEmpty()) {
        showErrorMessage("请输入相机序列号");
        return;
    }

    logMessage(QString("用户点击配置相机: %1").arg(serialNumber));

    // 检查相机控制器是否有效
    if (!m_cameraController) {
        showErrorMessage("相机控制器未初始化");
        logMessage("相机控制器为空指针");
        return;
    }

    // 如果相机已经在采集，先停止采集
    if (isGrabbing()) {
        logMessage("停止当前图像采集");
        // 停止定时器
        if (m_grabTimerId != 0) {
            killTimer(m_grabTimerId);
            m_grabTimerId = 0;
        }
        m_cameraController->StopImageCapture(m_cameraHandle);
    }

    // 如果相机已打开，先关闭
    if (isCameraOpen()) {
        logMessage("关闭当前相机");
        m_cameraController->CloseCamera(m_cameraHandle);
        m_cameraHandle = nullptr;
    }

    // 打开新的相机
    if (openCameraBySerialNumber(serialNumber)) {
        logMessage(QString("打开相机成功: %1").arg(serialNumber));
    } else {
        showErrorMessage(QString("无法打开相机: %1").arg(serialNumber));
    }
    updateParameterControls();
}

void SingleCameraControlWidget::on_spinBox_Exposure_valueChanged(int arg1)
{
    if (isCameraOpen() && m_cameraHandle) {
        if (m_cameraController->SetExposureTime(m_cameraHandle, static_cast<float>(arg1))) {
            logMessage(QString("曝光时间设置为: %1μs").arg(arg1));
        } else {
            logMessage(QString("曝光时间设置失败: %1μs").arg(arg1));
        }
    }
}

void SingleCameraControlWidget::on_doubleSpinBox_Gain_valueChanged(double arg1)
{
    if (isCameraOpen() && m_cameraHandle) {
        if (m_cameraController->SetGain(m_cameraHandle, static_cast<float>(arg1))) {
            logMessage(QString("增益设置为: %1dB").arg(arg1));
        } else {
            logMessage(QString("增益设置失败: %1dB").arg(arg1));
        }
    }
}

void SingleCameraControlWidget::on_comboBox_TriggerMode_currentIndexChanged(int index)
{
    if (isCameraOpen() && m_cameraHandle) {
        if (m_cameraController->SetTriggerMode(m_cameraHandle, index)) {
            logMessage(QString("触发模式设置为: %1").arg(index == 0 ? "内触发" : "外触发"));
            updateParameterControls();  // 刷新UI状态
        } else {
            logMessage(QString("触发模式设置失败: %1").arg(index));
        }
    }
}

void SingleCameraControlWidget::on_comboBox_TriggerEdge_currentIndexChanged(int index)
{
    if (isCameraOpen() && m_cameraHandle) {
        if (m_cameraController->SetTriggerActivation(m_cameraHandle, static_cast<unsigned int>(index))) {
            logMessage(QString("触发边沿设置为: %1").arg(index == 0 ? "上升沿" : "下降沿"));
            updateParameterControls();  // 刷新UI状态
        } else {
            logMessage(QString("触发边沿设置失败: %1").arg(index));
        }
    }
}

void SingleCameraControlWidget::on_doubleSpinBox_TriggerDelay_valueChanged(double arg1)
{
    if (isCameraOpen()) {
        // 注意：CCameraController中没有直接的触发延迟设置函数
        // 这里暂时留空，可以根据实际需要实现
        logMessage(QString("触发延迟设置请求: %1μs（暂未实现）").arg(arg1));
    }
}

// ==================== 定时器槽函数 ====================

void SingleCameraControlWidget::onTimeout_UpdateStatus()
{
    updateCameraStatus();
}

// ==================== 定时器事件处理 ====================

void SingleCameraControlWidget::timerEvent(QTimerEvent* event)
{
    if (event->timerId() == m_grabTimerId && isGrabbing() && isCameraOpen() && m_cameraController) {
        grabAndProcessImage();
    }
    QWidget::timerEvent(event);
}

void SingleCameraControlWidget::grabAndProcessImage()
{
    // 防御性检查：确保控制器和相机状态有效
    if (!m_cameraController || !isCameraOpen() || !m_cameraHandle) {
        return;
    }

    // 确保缓冲区已分配
    if (!m_pImageBuffer) {
        m_pImageBuffer = new unsigned char[DEFAULT_BUFFER_SIZE];
        m_imageBufferSize = DEFAULT_BUFFER_SIZE;
    }

    MV_FRAME_OUT_INFO_EX frameInfo = {0};
    if (m_cameraController->GetImage(m_cameraHandle, m_pImageBuffer, static_cast<unsigned int>(m_imageBufferSize), &frameInfo)) {
        // 将原始数据转换为cv::Mat
        cv::Mat image(frameInfo.nHeight, frameInfo.nWidth, CV_8UC1, m_pImageBuffer);
        if (!image.empty()) {
            // 计算FPS
            m_frameCounter++;
            QTime currentTime = QTime::currentTime();
            int elapsedMs = m_lastFpsTime.msecsTo(currentTime);
            if (elapsedMs >= 1000) {  // 每秒更新一次FPS
                float fps = (m_frameCounter * 1000.0f) / elapsedMs;
                m_frameCounter = 0;
                m_lastFpsTime = currentTime;
                
                // 更新FPS显示（使用线程安全方式）
                QMetaObject::invokeMethod(ui->label_FPS, "setText", Qt::QueuedConnection,
                                          Q_ARG(QString, QString("FPS: %1").arg(fps, 0, 'f', 1)));
            }
            
            // 图像缩放：将宽度和高度缩小为原来的1/4
            cv::Mat scaledImage;
            cv::resize(image, scaledImage, cv::Size(), 0.25, 0.25, cv::INTER_AREA);
            
            // 将OpenCV Mat转换为QImage
            QImage qImage;
            if (scaledImage.channels() == 3) {
                // 彩色图像 (BGR)
                cv::Mat rgbImage;
                cv::cvtColor(scaledImage, rgbImage, cv::COLOR_BGR2RGB);
                qImage = QImage(rgbImage.data, rgbImage.cols, rgbImage.rows,
                                rgbImage.step, QImage::Format_RGB888);
            } else if (scaledImage.channels() == 1) {
                // 灰度图像
                qImage = QImage(scaledImage.data, scaledImage.cols, scaledImage.rows,
                                scaledImage.step, QImage::Format_Grayscale8);
            } else {
                logMessage("不支持的图像格式");
                return;
            }
            
            // 发送图像信号
            if (!qImage.isNull()) {
                emit imageReceived(qImage.copy());  // 发送副本以避免数据竞争
            }
        }
    }
}

// ==================== 工具函数 ====================

void SingleCameraControlWidget::updateCameraStatus()
{
    QString statusText;
    QString cameraInfoText;

    bool cameraOpen = isCameraOpen();
    bool grabbing = isGrabbing();

    if (cameraOpen) {
        statusText = QString("状态: 相机已打开");
        if (grabbing) {
            statusText += " [采集中]";
        } else {
            statusText += " [空闲]";
        }

        cameraInfoText = QString("相机信息: %1").arg(m_currentSerialNumber);
    } else {
        statusText = "状态: 相机未打开";
        cameraInfoText = "相机信息: 无";
    }

    ui->label_Status->setText(statusText);
    ui->label_CameraInfo->setText(cameraInfoText);
}

void SingleCameraControlWidget::updateParameterControls()
{
    bool cameraOpen = isCameraOpen();
    bool grabbing = isGrabbing();
    bool enableControls = cameraOpen && !grabbing;

    // 参数设置控件的启用状态
    ui->spinBox_Exposure->setEnabled(enableControls);
    ui->doubleSpinBox_Gain->setEnabled(enableControls);
    ui->comboBox_TriggerMode->setEnabled(enableControls);
    ui->comboBox_TriggerEdge->setEnabled(enableControls);
    ui->comboBox_TriggerSource->setEnabled(enableControls);
    ui->doubleSpinBox_TriggerDelay->setEnabled(enableControls);

    // 按钮状态
    ui->pushButton_OpenCamera->setEnabled(!cameraOpen && !ui->lineEdit_SN->text().isEmpty());
    ui->pushButton_CloseCamera->setEnabled(cameraOpen && !grabbing);
    ui->pushButton_StartGrabbing->setEnabled(cameraOpen && !grabbing);
    ui->pushButton_StopGrabbing->setEnabled(grabbing);
    ui->pushButton_SoftWareTrigger->setEnabled(cameraOpen && !grabbing);
}

void SingleCameraControlWidget::showErrorMessage(const QString& message)
{
    QMessageBox::critical(this, "错误", message);
}

void SingleCameraControlWidget::logMessage(const QString& message, bool notifyMainWindow)
{
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString logEntry = QString("[%1] %2").arg(timestamp).arg(message);

    ui->textEdit_Log->append(logEntry);

    // 仅对关键操作发送信号到主窗口
    if (notifyMainWindow) {
        emit statusUpdated(message);
    }

    // 限制日志行数
    QTextCursor cursor = ui->textEdit_Log->textCursor();
    cursor.movePosition(QTextCursor::End);
    while (ui->textEdit_Log->document()->blockCount() > 100) {
        cursor.movePosition(QTextCursor::Start);
        cursor.select(QTextCursor::BlockUnderCursor);
        cursor.removeSelectedText();
        cursor.deleteChar(); // 删除换行符
    }
}

// ==================== 相机控制函数 ====================

bool SingleCameraControlWidget::openCameraBySerialNumber(const QString& serialNumber)
{
    if (!m_cameraController) return false;

    void* handle = m_cameraController->OpenCameraBySN(serialNumber.toStdString().c_str());
    if (!handle) {
        showErrorMessage(QString("无法打开相机: %1").arg(serialNumber));
        logMessage(QString("打开相机失败: %1").arg(serialNumber), true);  // 通知主窗口
        return false;
    }

    m_cameraHandle = handle;
    m_currentSerialNumber = serialNumber;
    
    // 分配图像缓冲区
    if (!m_pImageBuffer) {
        m_pImageBuffer = new unsigned char[DEFAULT_BUFFER_SIZE];
        m_imageBufferSize = DEFAULT_BUFFER_SIZE;
    }

    logMessage(QString("相机打开成功: %1").arg(serialNumber), true);  // 通知主窗口
    return true;
}

bool SingleCameraControlWidget::configureCameraParameters()
{
    if (!isCameraOpen() || !m_cameraHandle) return false;

    bool allSuccess = true;
    
    // 设置曝光时间
    int exposure = ui->spinBox_Exposure->value();
    if (!m_cameraController->SetExposureTime(m_cameraHandle, static_cast<float>(exposure))) {
        logMessage(QString("曝光时间设置失败: %1μs").arg(exposure), false);
        allSuccess = false;
    }
    
    // 设置增益
    float gain = static_cast<float>(ui->doubleSpinBox_Gain->value());
    if (!m_cameraController->SetGain(m_cameraHandle, gain)) {
        logMessage(QString("增益设置失败: %1dB").arg(gain), false);
        allSuccess = false;
    }
    
    // 设置触发模式
    int triggerMode = ui->comboBox_TriggerMode->currentIndex();
    if (!m_cameraController->SetTriggerMode(m_cameraHandle, triggerMode)) {
        logMessage(QString("触发模式设置失败: %1").arg(triggerMode), false);
        allSuccess = false;
    }
    
    // 仅在触发模式(TriggerMode=1)下设置触发源和触发激活方式
    if (triggerMode == 1) {
        QString triggerSourceStr = ui->comboBox_TriggerSource->currentText();
        if (!m_cameraController->SetTriggerSource(m_cameraHandle, triggerSourceStr.toStdString().c_str())) {
            logMessage(QString("触发源设置失败: %1").arg(triggerSourceStr), false);
            allSuccess = false;
        }
        
        // 设置触发激活方式（仅硬件模式需要）
        if (triggerSourceStr != "Software")
        {
            int triggerActivation = ui->comboBox_TriggerEdge->currentIndex();
            if (!m_cameraController->SetTriggerActivation(m_cameraHandle, static_cast<unsigned int>(triggerActivation))) {
                logMessage(QString("触发激活方式设置失败: %1").arg(triggerActivation), false);
                allSuccess = false;
            }
        }
    }

    if (allSuccess) {
        logMessage("相机参数配置成功", true);  // 配置成功时通知主窗口
    } else {
        logMessage("警告: 相机参数配置可能不完全成功", true);
    }

    return allSuccess;
}


// ==================== 参数获取函数 ====================

int SingleCameraControlWidget::getCurrentExposure() const
{
    if (ui && ui->spinBox_Exposure) {
        return ui->spinBox_Exposure->value();
    }
    return 15000; // 默认值
}

float SingleCameraControlWidget::getCurrentGain() const
{
    if (ui && ui->doubleSpinBox_Gain) {
        return static_cast<float>(ui->doubleSpinBox_Gain->value());
    }
    return 2.0f; // 默认值
}

int SingleCameraControlWidget::getCurrentTriggerMode() const
{
    if (ui && ui->comboBox_TriggerMode) {
        return ui->comboBox_TriggerMode->currentIndex();
    }
    return 0; // 默认内触发
}

int SingleCameraControlWidget::getCurrentTriggerEdge() const
{
    if (ui && ui->comboBox_TriggerEdge) {
        return ui->comboBox_TriggerEdge->currentIndex();
    }
    return 1; // 默认上升沿
}

QString SingleCameraControlWidget::getCurrentTriggerSource() const
{
    if (ui && ui->comboBox_TriggerSource) {
        return ui->comboBox_TriggerSource->currentText();
    }
    return "Line0"; // 默认触发源
}

double SingleCameraControlWidget::getCurrentTriggerDelay() const
{
    if (ui && ui->doubleSpinBox_TriggerDelay) {
        return ui->doubleSpinBox_TriggerDelay->value();
    }
    return 0.0; // 默认无延迟
}

// ==================== 图像保存函数 ====================

void SingleCameraControlWidget::saveCapturedImage(const cv::Mat& image)
{
    if (image.empty()) return;
    
    // 生成文件名
    QString fileName = QString("software_trigger_image_%1.jpg")
                      .arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmsszzz"));
    QString fullPath = QCoreApplication::applicationDirPath() + "/" + fileName;
    
    // 保存图像
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(95); // JPEG质量95%
    
    try {
        bool success = cv::imwrite(fullPath.toStdString(), image, compression_params);
        if (success) {
            logMessage(QString("图像已保存: %1").arg(fullPath));
        } else {
            logMessage(QString("图像保存失败: %1").arg(fullPath));
            showErrorMessage("图像保存失败");
        }
    } catch (const cv::Exception& ex) {
        logMessage(QString("OpenCV保存图像异常: %1").arg(ex.what()));
        showErrorMessage(QString("保存图像时发生错误: %1").arg(ex.what()));
    }
}
