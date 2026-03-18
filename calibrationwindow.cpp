#include "calibrationwindow.h"
#include "ui_calibrationwindow.h"
#include "singleprojectorcontrolwidget.h"
#include "singlecameracontrolwidget.h"
#include "CameraController.h"
#include <QMessageBox>
#include <QDebug>
#include <QDateTime>
#include <QSettings>
#include <QStandardPaths>
#include <QDir>
#include <QFile>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <map>
#include <vector>
#include <algorithm>


CalibrationWindow::CalibrationWindow(const DeviceConfig& config, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::CalibrationWindow)
    , m_projectorController(new ProjectorController())
    , m_cameraController(new CCameraController())
    , m_leftProjectorWidget(nullptr)
    , m_rightProjectorWidget(nullptr)
    , m_leftCameraWidget(nullptr)
    , m_rightCameraWidget(nullptr)
    , m_leftCalibrationWidget(nullptr)
    , m_rightCalibrationWidget(nullptr)
    , m_leftReconstructWidget(nullptr)
    , m_rightReconstructWidget(nullptr)
    , m_deviceConfig(config)
{
    ui->setupUi(this);

    // 初始化投影仪控制器USB
    if (!m_projectorController->loadDll()) {
        QMessageBox::critical(this, tr("错误"), tr("无法加载投影仪控制DLL"));
        return;
    }

    // 相机控制器已直接集成，无需加载DLL

    // 连接检测投影仪按钮的点击信号
    connect(ui->pushButton_ProjectorDetect, &QPushButton::clicked,
            this, &CalibrationWindow::onProjectorDetectClicked);

    // 连接QTabWidget标签切换信号，实现三个区域的同步切换
    setupTabSynchronization();

    // 直接初始化点云重建界面（不依赖标定完成）
    if (initializeReconstructWidgets()) {
        m_reconstructWidgetInitialized = true;
    }

    // 记录配置信息到日志
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString logMessage = QString("[%1] 标定窗口已启动\n左侧相机SN: %2\n右侧相机SN: %3\n左侧投影仪Tag: %4\n右侧投影仪Tag: %5")
                        .arg(timestamp)
                        .arg(m_deviceConfig.leftCameraSN.isEmpty() ? "未设置" : m_deviceConfig.leftCameraSN)
                        .arg(m_deviceConfig.rightCameraSN.isEmpty() ? "未设置" : m_deviceConfig.rightCameraSN)
                        .arg(m_deviceConfig.leftProjectorTag)
                        .arg(m_deviceConfig.rightProjectorTag);
    if (ui && ui->textEdit_CalibrationResults) {
        ui->textEdit_CalibrationResults->append(logMessage);
    }
}

CalibrationWindow::~CalibrationWindow()
{
    // 清理投影仪控制组件
    if (m_leftProjectorWidget) {
        delete m_leftProjectorWidget;
        m_leftProjectorWidget = nullptr;
    }
    
    if (m_rightProjectorWidget) {
        delete m_rightProjectorWidget;
        m_rightProjectorWidget = nullptr;
    }
    
    // 清理标定子界面
    if (m_leftCalibrationWidget) {
        delete m_leftCalibrationWidget;
        m_leftCalibrationWidget = nullptr;
    }
    
    if (m_rightCalibrationWidget) {
        delete m_rightCalibrationWidget;
        m_rightCalibrationWidget = nullptr;
    }
    
    // 清理点云重建界面
    if (m_leftReconstructWidget) {
        delete m_leftReconstructWidget;
        m_leftReconstructWidget = nullptr;
    }
    
    if (m_rightReconstructWidget) {
        delete m_rightReconstructWidget;
        m_rightReconstructWidget = nullptr;
    }
    
    // 清理相机控制组件
    if (m_leftCameraWidget) {
        delete m_leftCameraWidget;
        m_leftCameraWidget = nullptr;
    }
    
    if (m_rightCameraWidget) {
        delete m_rightCameraWidget;
        m_rightCameraWidget = nullptr;
    }
    
    // 清理相机控制器
    if (m_cameraController) {
        delete m_cameraController;
        m_cameraController = nullptr;
    }
    
    // 清理投影仪控制器
    if (m_projectorController) {
        delete m_projectorController;
        m_projectorController = nullptr;
    }
    
    delete ui;
}

void CalibrationWindow::closeEvent(QCloseEvent *event)
{
    QMessageBox::StandardButton reply = QMessageBox::question(
        this, tr("确认关闭"),
        tr("确定要关闭标定窗口吗？\n关闭后所有硬件连接将断开。"),
        QMessageBox::Yes | QMessageBox::No);
    
    if (reply == QMessageBox::Yes) {
        event->accept();
    } else {
        event->ignore();
    }
}

/* ****************************************** */
/* *****************投影仪的操作**************** */
/* ****************************************** */
bool CalibrationWindow::initializeProjectorControlWidgets()
{
    // 查找左右投影仪控制区域
    QWidget* leftProjectorControlArea = findChild<QWidget*>("tabWidget_LeftProjectorControl");
    QWidget* rightProjectorControlArea = findChild<QWidget*>("tabWidget_RightProjectorControl");
    
    // 辅助lambda函数：将投影仪控件添加到指定的Tab区域
    auto addToTabArea = [&](SingleProjectorControlWidget* widget, const QString& tag, QWidget* leftArea, QWidget* rightArea) -> bool {
        QWidget* targetArea = nullptr;
        QString targetName;
        
        if (tag.contains("left", Qt::CaseInsensitive)) {
            targetArea = leftArea;
            targetName = "tabWidget_LeftProjectorControl";
        } else if (tag.contains("right", Qt::CaseInsensitive)) {
            targetArea = rightArea;
            targetName = "tabWidget_RightProjectorControl";
        } else {
            // 默认添加到左Tab
            targetArea = leftArea;
            targetName = "tabWidget_LeftProjectorControl";
        }
        
        if (targetArea) {
            QVBoxLayout* layout = qobject_cast<QVBoxLayout*>(targetArea->layout());
            if (!layout) {
                layout = new QVBoxLayout(targetArea);
                layout->setContentsMargins(0, 0, 0, 0);
            }
            layout->addWidget(widget);
            return true;
        } else {
            qDebug() << "[" << __FILE__ << ":" << __LINE__ << "] " << QString("警告：未找到名为%1的Tab控件").arg(targetName);
            return false;
        }
    };
    
    // 创建两个投影仪控制组件，使用配置文件中的tagname
    SingleProjectorControlWidget* projectorControlWidget0 = new SingleProjectorControlWidget(m_projectorController, m_deviceConfig.leftProjectorTag, this);
    SingleProjectorControlWidget* projectorControlWidget1 = new SingleProjectorControlWidget(m_projectorController, m_deviceConfig.rightProjectorTag, this);
    
    QString tag0 = m_deviceConfig.leftProjectorTag;
    QString tag1 = m_deviceConfig.rightProjectorTag;
    
    // 处理第一个投影仪控件
    if (!addToTabArea(projectorControlWidget0, tag0, leftProjectorControlArea, rightProjectorControlArea)) {
        delete projectorControlWidget0;
        delete projectorControlWidget1;
        return false;
    }
    
    // 处理第二个投影仪控件
    if (!addToTabArea(projectorControlWidget1, tag1, leftProjectorControlArea, rightProjectorControlArea)) {
        delete projectorControlWidget0;
        delete projectorControlWidget1;
        return false;
    }

    // 保存左投影仪控件指针以便后续管理
    m_leftProjectorWidget = projectorControlWidget0;
    m_rightProjectorWidget = projectorControlWidget1;
    
    // 记录投影仪配置信息
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString logMessage = QString("[%1] 投影仪控件初始化完成\n左侧投影仪Tag: %2\n右侧投影仪Tag: %3")
                        .arg(timestamp)
                        .arg(m_deviceConfig.leftProjectorTag)
                        .arg(m_deviceConfig.rightProjectorTag);
    if (ui && ui->textEdit_CalibrationResults) {
        ui->textEdit_CalibrationResults->append(logMessage);
    }
    
    
    // 连接状态更新信号
    connect(m_leftProjectorWidget, &SingleProjectorControlWidget::statusUpdated,
            this, &CalibrationWindow::onProjectorStatusUpdated);
    connect(m_rightProjectorWidget, &SingleProjectorControlWidget::statusUpdated,
            this, &CalibrationWindow::onProjectorStatusUpdated);
    
    return true;
}

void CalibrationWindow::onProjectorDetectClicked()
{
    if (!m_projectorController) {
        QMessageBox::warning(this, tr("警告"), tr("投影仪控制器未初始化"));
        return;
    }
    
    // 清空之前的投影仪列表
    ui->comboBox_ProjectorSelect->clear();
    
    // 调用ProjectorController的detectProjectors方法检测投影仪
    int projectorCount = m_projectorController->detectProjectors();
    
    if (projectorCount <= 0) {
        QMessageBox::information(this, tr("提示"), tr("未检测到投影仪设备"));
        return;
    }
    
    // 获取每个投影仪的序列号并添加到下拉框
    for (int i = 0; i < projectorCount; ++i) {
        std::string serial = m_projectorController->getProjectorSerial(i);
        if (!serial.empty()) {
            QString projectorName = QString::fromStdString(serial);
            ui->comboBox_ProjectorSelect->addItem(projectorName, QVariant(i));
        }
    }
    
    // 在textEdit_CalibrationResults中记录检测结果
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString logMessage = QString("[%1] 检测到 %2 个投影仪设备\n").arg(timestamp).arg(projectorCount);
    ui->textEdit_CalibrationResults->append(logMessage);
    
    if (!m_projectorControlWidgetInitialized) {
        if (initializeProjectorControlWidgets()) {
            m_projectorControlWidgetInitialized = true;
        }
    }
}

void CalibrationWindow::onProjectorStatusUpdated(const QString& message)
{
    // 在textEdit_CalibrationResults中显示投影仪状态信息
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString logMessage = QString("[%1] %2\n").arg(timestamp).arg(message);
    ui->textEdit_CalibrationResults->append(logMessage);
    
}

/* ****************************************** */
/* *****************相机的操作**************** */
/* ****************************************** */
bool CalibrationWindow::initializeCameraControlWidgets()
{
    // 查找左右相机控制区域
    QWidget* leftCameraControlArea = findChild<QWidget*>("tabWidget_LeftCameraControl");
    QWidget* rightCameraControlArea = findChild<QWidget*>("tabWidget_RightCameraControl");

    if (!leftCameraControlArea || !rightCameraControlArea) {
        qDebug() << "[" << __FILE__ << ":" << __LINE__ << "] " << "警告：未找到相机控制Tab区域";
        return false;
    }

    // 创建两个独立的相机控制组件，传入相机控制器
    SingleCameraControlWidget* leftCameraWidget = new SingleCameraControlWidget(m_cameraController, this);
    SingleCameraControlWidget* rightCameraWidget = new SingleCameraControlWidget(m_cameraController, this);

    // 设置相机序列号（来自配置文件）
    if (!m_deviceConfig.leftCameraSN.isEmpty()) {
        leftCameraWidget->setCameraSerialNumber(m_deviceConfig.leftCameraSN);
        QString logMessage = QString("[%1] 已为左侧相机控件设置序列号: %2")
                            .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"))
                            .arg(m_deviceConfig.leftCameraSN);
        if (ui && ui->textEdit_CalibrationResults) {
            ui->textEdit_CalibrationResults->append(logMessage);
        }
    }
    
    if (!m_deviceConfig.rightCameraSN.isEmpty()) {
        rightCameraWidget->setCameraSerialNumber(m_deviceConfig.rightCameraSN);
        QString logMessage = QString("[%1] 已为右侧相机控件设置序列号: %2")
                            .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"))
                            .arg(m_deviceConfig.rightCameraSN);
        if (ui && ui->textEdit_CalibrationResults) {
            ui->textEdit_CalibrationResults->append(logMessage);
        }
    }

    // 添加到左Tab区域
    QVBoxLayout* leftLayout = qobject_cast<QVBoxLayout*>(leftCameraControlArea->layout());
    if (!leftLayout) {
        leftLayout = new QVBoxLayout(leftCameraControlArea);
        leftLayout->setContentsMargins(0, 0, 0, 0);
    }
    leftLayout->addWidget(leftCameraWidget);

    // 添加到右Tab区域
    QVBoxLayout* rightLayout = qobject_cast<QVBoxLayout*>(rightCameraControlArea->layout());
    if (!rightLayout) {
        rightLayout = new QVBoxLayout(rightCameraControlArea);
        rightLayout->setContentsMargins(0, 0, 0, 0);
    }
    rightLayout->addWidget(rightCameraWidget);

    // 保存相机控件指针以便后续管理
    m_leftCameraWidget = leftCameraWidget;
    m_rightCameraWidget = rightCameraWidget;

    // 连接状态更新信号
    connect(m_leftCameraWidget, &SingleCameraControlWidget::statusUpdated,
            this, &CalibrationWindow::onCameraStatusUpdated);
    connect(m_rightCameraWidget, &SingleCameraControlWidget::statusUpdated,
            this, &CalibrationWindow::onCameraStatusUpdated);

    // 连接图像接收信号
    connect(m_leftCameraWidget, &SingleCameraControlWidget::imageReceived,
            this, &CalibrationWindow::onLeftCameraImageReceived);
    connect(m_rightCameraWidget, &SingleCameraControlWidget::imageReceived,
            this, &CalibrationWindow::onRightCameraImageReceived);

    return true;
}

void CalibrationWindow::onCameraStatusUpdated(const QString& message)
{
    // 在textEdit_CalibrationResults中显示相机状态信息
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString logMessage = QString("[%1] 相机状态: %2\n").arg(timestamp).arg(message);
    ui->textEdit_CalibrationResults->append(logMessage);
}

void CalibrationWindow::onLeftCameraImageReceived(const QImage& image)
{
    // 在左相机显示区域显示图像
    if (!image.isNull()) {
        QPixmap pixmap = QPixmap::fromImage(image);
        QPixmap scaledPixmap = pixmap.scaled(
            ui->label_CameraImageDisplayLeft->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation);
        
        ui->label_CameraImageDisplayLeft->setAlignment(Qt::AlignCenter);
        ui->label_CameraImageDisplayLeft->setPixmap(scaledPixmap);
        
        ui->label_ImageResolutionLeft->setText(
            QString("分辨率: %1x%2").arg(image.width()).arg(image.height()));
    }
}

void CalibrationWindow::onCalibrationStatusUpdated(const QString& message)
{
    // 在textEdit_CalibrationResults中显示标定状态信息
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString logMessage = QString("[%1] 标定状态: %2\n").arg(timestamp).arg(message);
    ui->textEdit_CalibrationResults->append(logMessage);
}

void CalibrationWindow::onCalibrationCompleted(bool success, const QString& result)
{
    // 在textEdit_CalibrationResults中显示标定完成信息
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString status = success ? "成功" : "失败";
    QString logMessage = QString("[%1] 标定完成[%2]: %3").arg(timestamp).arg(status).arg(result);
    ui->textEdit_CalibrationResults->append(logMessage);
}

void CalibrationWindow::onRightCameraImageReceived(const QImage& image)
{
    // 在右相机显示区域显示图像
    if (!image.isNull()) {
        QPixmap pixmap = QPixmap::fromImage(image);
        QPixmap scaledPixmap = pixmap.scaled(
            ui->label_CameraImageDisplayRight->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation);
        
        ui->label_CameraImageDisplayRight->setAlignment(Qt::AlignCenter);
        ui->label_CameraImageDisplayRight->setPixmap(scaledPixmap);
        
        ui->label_ImageResolutionRight->setText(
            QString("分辨率: %1x%2").arg(image.width()).arg(image.height()));
    }
}

void CalibrationWindow::on_pushButton_CameraDetect_clicked()
{
    if (!m_cameraController) {
        QMessageBox::warning(this, tr("警告"), tr("相机控制器未初始化"));
        return;
    }
    
    // 记录检测开始时间
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString logMessage = QString("[%1] 开始检测相机设备").arg(timestamp);
    ui->textEdit_CalibrationResults->append(logMessage);
    
    // 清空之前的相机列表
    ui->comboBox_CameraSelect->clear();
    
    // 调用CCameraController的EnumerateCameras方法检测相机
    std::vector<std::string> serialNumbers = m_cameraController->EnumerateCameras();
    int cameraCount = static_cast<int>(serialNumbers.size());
    
    if (cameraCount <= 0) {
        logMessage = QString("[%1] 未检测到相机设备").arg(timestamp);
        ui->textEdit_CalibrationResults->append(logMessage);
        QMessageBox::information(this, tr("提示"), tr("未检测到相机设备"));
        return;
    }
    
    // 获取每个相机的序列号并添加到下拉框
    for (int i = 0; i < cameraCount; ++i) {
        QString cameraSN = QString::fromStdString(serialNumbers[i]);
        if (!cameraSN.isEmpty()) {
            ui->comboBox_CameraSelect->addItem(cameraSN, QVariant(i));
            logMessage = QString("[%1] 发现相机: %2")
                        .arg(timestamp)
                        .arg(cameraSN);
            ui->textEdit_CalibrationResults->append(logMessage);
        }
    }
    
    // 记录检测结果
    logMessage = QString("[%1] 相机检测完成，共发现 %2 个设备").arg(timestamp).arg(cameraCount);
    ui->textEdit_CalibrationResults->append(logMessage);
    
    
    // 若尚未初始化相机控制界面，则进行初始化
    if (!m_cameraControlWidgetInitialized) {
        if (initializeCameraControlWidgets()) {
            m_cameraControlWidgetInitialized = true;
            logMessage = QString("[%1] 相机控制界面初始化成功").arg(timestamp);
            ui->textEdit_CalibrationResults->append(logMessage);
        } else {
            logMessage = QString("[%1] 相机控制界面初始化失败").arg(timestamp);
            ui->textEdit_CalibrationResults->append(logMessage);
            QMessageBox::critical(this, tr("错误"), tr("相机控制界面初始化失败"));
        }
    }
}

/* ****************************************** */
/* *****************标定的操作**************** */
/* ****************************************** */
bool CalibrationWindow::initializeCalibrationWidgets()
{
    // 查找左右标定控制区域
    QWidget* leftCalibrationArea = findChild<QWidget*>("tabWidget_LeftSLCalibration");
    QWidget* rightCalibrationArea = findChild<QWidget*>("tabWidget_RightSLCalibration");

    if (!leftCalibrationArea || !rightCalibrationArea) {
        qDebug() << "[" << __FILE__ << ":" << __LINE__ << "] " << "警告：未找到标定控制Tab区域";
        return false;
    }

    // 如果控件已存在，先清理再重新初始化
    if (m_leftCalibrationWidget) {
        delete m_leftCalibrationWidget;
        m_leftCalibrationWidget = nullptr;
    }
    
    if (m_rightCalibrationWidget) {
        delete m_rightCalibrationWidget;
        m_rightCalibrationWidget = nullptr;
    }

    // 创建两个标定子界面组件，使用配置文件中的参数
    SingleCalibrationWidget* leftCalibrationWidget = new SingleCalibrationWidget(
        m_projectorController, m_cameraController, m_leftCameraWidget, m_leftProjectorWidget, m_deviceConfig.leftProjectorTag, this);
    SingleCalibrationWidget* rightCalibrationWidget = new SingleCalibrationWidget(
        m_projectorController, m_cameraController, m_rightCameraWidget, m_rightProjectorWidget, m_deviceConfig.rightProjectorTag, this);
    
    // 设置相机序列号（来自配置文件）
    if (!m_deviceConfig.leftCameraSN.isEmpty()) {
        leftCalibrationWidget->setCameraSerialNumber(m_deviceConfig.leftCameraSN);
    }
    
    if (!m_deviceConfig.rightCameraSN.isEmpty()) {
        rightCalibrationWidget->setCameraSerialNumber(m_deviceConfig.rightCameraSN);
    }

    // 添加到左Tab区域
    QVBoxLayout* leftLayout = qobject_cast<QVBoxLayout*>(leftCalibrationArea->layout());
    if (!leftLayout) {
        leftLayout = new QVBoxLayout(leftCalibrationArea);
        leftLayout->setContentsMargins(0, 0, 0, 0);
    }
    leftLayout->addWidget(leftCalibrationWidget);

    // 添加到右Tab区域
    QVBoxLayout* rightLayout = qobject_cast<QVBoxLayout*>(rightCalibrationArea->layout());
    if (!rightLayout) {
        rightLayout = new QVBoxLayout(rightCalibrationArea);
        rightLayout->setContentsMargins(0, 0, 0, 0);
    }
    rightLayout->addWidget(rightCalibrationWidget);

    // 保存标定控件指针以便后续管理
    m_leftCalibrationWidget = leftCalibrationWidget;
    m_rightCalibrationWidget = rightCalibrationWidget;

    // 连接状态更新信号
    connect(m_leftCalibrationWidget, &SingleCalibrationWidget::statusUpdated,
            this, &CalibrationWindow::onCalibrationStatusUpdated);
    connect(m_rightCalibrationWidget, &SingleCalibrationWidget::statusUpdated,
            this, &CalibrationWindow::onCalibrationStatusUpdated);

    // 连接标定完成信号
    connect(m_leftCalibrationWidget, &SingleCalibrationWidget::calibrationCompleted,
            this, &CalibrationWindow::onCalibrationCompleted);
    connect(m_rightCalibrationWidget, &SingleCalibrationWidget::calibrationCompleted,
            this, &CalibrationWindow::onCalibrationCompleted);

    // 更新主窗口显示信息
    updateCalibrationDisplayInfo();

    return true;
}

void CalibrationWindow::updateCalibrationDisplayInfo()
{
    // 更新左侧标定信息显示
    if (m_leftCalibrationWidget && ui->label_LeftSL) {
        QString leftCameraSN = m_leftCalibrationWidget->getCameraSerialNumber();
        QString leftTag = m_leftCalibrationWidget->getDeviceTag();
        QString leftInfo = QString("相机SN: %1\n投影仪Tag: %2")
                          .arg(leftCameraSN.isEmpty() ? "未设置" : leftCameraSN)
                          .arg(leftTag);
        ui->label_LeftSL->setText(leftInfo);
    }
    
    // 更新右侧标定信息显示
    if (m_rightCalibrationWidget && ui->label_RightSL) {
        QString rightCameraSN = m_rightCalibrationWidget->getCameraSerialNumber();
        QString rightTag = m_rightCalibrationWidget->getDeviceTag();
        QString rightInfo = QString("相机SN: %1\n投影仪Tag: %2")
                           .arg(rightCameraSN.isEmpty() ? "未设置" : rightCameraSN)
                           .arg(rightTag);
        ui->label_RightSL->setText(rightInfo);
    }
}

void CalibrationWindow::on_pushButton_InitCalibration_clicked()
{
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString logMessage;
    
    // 检查相机控制器是否初始化
    if (!m_cameraController) {
        logMessage = QString("[%1] 错误: 相机控制器未初始化，请先检测相机").arg(timestamp);
        ui->textEdit_CalibrationResults->append(logMessage);
        QMessageBox::warning(this, tr("警告"), tr("请先检测相机设备"));
        return;
    }
    
    // 检查相机控制界面是否已初始化
    if (!m_cameraControlWidgetInitialized) {
        logMessage = QString("[%1] 错误: 相机控制界面未初始化，请先检测相机").arg(timestamp);
        ui->textEdit_CalibrationResults->append(logMessage);
        QMessageBox::warning(this, tr("警告"), tr("请先检测相机设备并初始化相机控制界面"));
        return;
    }
    
    // 检查投影仪控制界面是否已初始化
    if (!m_projectorControlWidgetInitialized) {
        logMessage = QString("[%1] 错误: 投影仪控制界面未初始化，请先检测投影仪").arg(timestamp);
        ui->textEdit_CalibrationResults->append(logMessage);
        QMessageBox::warning(this, tr("警告"), tr("请先检测投影仪设备"));
        return;
    }
    
    logMessage = QString("[%1] 开始初始化标定界面").arg(timestamp);
    ui->textEdit_CalibrationResults->append(logMessage);
    
    // 初始化标定界面
    if (!m_calibrationWidgetInitialized) {
        if (initializeCalibrationWidgets()) {
            m_calibrationWidgetInitialized = true;
            logMessage = QString("[%1] 标定界面初始化成功").arg(timestamp);
            ui->textEdit_CalibrationResults->append(logMessage);
        } else {
            logMessage = QString("[%1] 标定界面初始化失败").arg(timestamp);
            ui->textEdit_CalibrationResults->append(logMessage);
            QMessageBox::critical(this, tr("错误"), tr("标定界面初始化失败"));
        }
    } else {
        logMessage = QString("[%1] 标定界面已初始化").arg(timestamp);
        ui->textEdit_CalibrationResults->append(logMessage);
    }
}

/* ****************************************** */
/* ***************点云重建的操作*************** */
/* ****************************************** */
bool CalibrationWindow::initializeReconstructWidgets()
{
    // 查找左右点云重建区域
    QWidget* leftReconstructArea = findChild<QWidget*>("groupBox_pointcloudLeft");
    QWidget* rightReconstructArea = findChild<QWidget*>("groupBox_pointcloudRight");

    if (!leftReconstructArea || !rightReconstructArea) {
        qDebug() << "[" << __FILE__ << ":" << __LINE__ << "] " << "警告：未找到点云重建控制区域";
        return false;
    }

    // 如果控件已存在，先清理再重新初始化
    if (m_leftReconstructWidget) {
        delete m_leftReconstructWidget;
        m_leftReconstructWidget = nullptr;
    }
    
    if (m_rightReconstructWidget) {
        delete m_rightReconstructWidget;
        m_rightReconstructWidget = nullptr;
    }

    // 创建两个点云重建界面组件，使用配置文件中的参数
    SingleReconstructWidget* leftReconstructWidget = new SingleReconstructWidget(
        m_projectorController, m_cameraController, m_leftCameraWidget, m_leftProjectorWidget, m_deviceConfig.leftProjectorTag, this);
    SingleReconstructWidget* rightReconstructWidget = new SingleReconstructWidget(
        m_projectorController, m_cameraController, m_rightCameraWidget, m_rightProjectorWidget, m_deviceConfig.rightProjectorTag, this);
    
    // 设置设备标识（用于区分左右）
    leftReconstructWidget->setDeviceTag("dibh_left");
    rightReconstructWidget->setDeviceTag("dibh_right");
    
    // 设置相机序列号（来自配置文件）
    if (!m_deviceConfig.leftCameraSN.isEmpty()) {
        leftReconstructWidget->setCameraSerialNumber(m_deviceConfig.leftCameraSN);
    }
    
    if (!m_deviceConfig.rightCameraSN.isEmpty()) {
        rightReconstructWidget->setCameraSerialNumber(m_deviceConfig.rightCameraSN);
    }

    // 添加到左点云区域
    QVBoxLayout* leftLayout = qobject_cast<QVBoxLayout*>(leftReconstructArea->layout());
    if (!leftLayout) {
        leftLayout = new QVBoxLayout(leftReconstructArea);
        leftLayout->setContentsMargins(0, 0, 0, 0);
    }
    leftLayout->addWidget(leftReconstructWidget);

    // 添加到右点云区域
    QVBoxLayout* rightLayout = qobject_cast<QVBoxLayout*>(rightReconstructArea->layout());
    if (!rightLayout) {
        rightLayout = new QVBoxLayout(rightReconstructArea);
        rightLayout->setContentsMargins(0, 0, 0, 0);
    }
    rightLayout->addWidget(rightReconstructWidget);

    // 保存重建控件指针以便后续管理
    m_leftReconstructWidget = leftReconstructWidget;
    m_rightReconstructWidget = rightReconstructWidget;

    // 连接状态更新信号
    connect(m_leftReconstructWidget, &SingleReconstructWidget::statusUpdated,
            this, &CalibrationWindow::onReconstructionStatusUpdated);
    connect(m_rightReconstructWidget, &SingleReconstructWidget::statusUpdated,
            this, &CalibrationWindow::onReconstructionStatusUpdated);

    // 连接重建状态变化信号
    connect(m_leftReconstructWidget, &SingleReconstructWidget::reconstructionStateChanged,
            this, &CalibrationWindow::onReconstructionStateChanged);
    connect(m_rightReconstructWidget, &SingleReconstructWidget::reconstructionStateChanged,
            this, &CalibrationWindow::onReconstructionStateChanged);

    // 记录日志
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString logMessage = QString("[%1] 点云重建界面初始化完成\n左侧投影仪Tag: %2\n右侧投影仪Tag: %3")
                        .arg(timestamp)
                        .arg(m_deviceConfig.leftProjectorTag)
                        .arg(m_deviceConfig.rightProjectorTag);
    if (ui && ui->textEdit_CalibrationResults) {
        ui->textEdit_CalibrationResults->append(logMessage);
    }

    return true;
}

void CalibrationWindow::onReconstructionStatusUpdated(const QString& message)
{
    // 在textEdit_CalibrationResults中显示重建状态信息
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString logMessage = QString("[%1] 重建状态: %2").arg(timestamp).arg(message);
    ui->textEdit_CalibrationResults->append(logMessage);
}

void CalibrationWindow::onReconstructionStateChanged(bool running)
{
    // 记录重建状态变化
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString status = running ? "开始" : "停止";
    QString logMessage = QString("[%1] 点云重建%2").arg(timestamp).arg(status);
    ui->textEdit_CalibrationResults->append(logMessage);
}

// ==================== QTabWidget标签同步功能 ====================

void CalibrationWindow::setupTabSynchronization()
{
    // 连接相机控制QTabWidget的currentChanged信号
    connect(ui->tabWidget_CameraControl, QOverload<int>::of(&QTabWidget::currentChanged),
            this, &CalibrationWindow::onCameraTabChanged);

    // 连接投影仪控制QTabWidget的currentChanged信号
    connect(ui->tabWidget_ProjectorControl, QOverload<int>::of(&QTabWidget::currentChanged),
            this, &CalibrationWindow::onProjectorTabChanged);

    // 连接结构光标定QTabWidget的currentChanged信号
    connect(ui->tabWidget_, QOverload<int>::of(&QTabWidget::currentChanged),
            this, &CalibrationWindow::onCalibrationTabChanged);
}

void CalibrationWindow::onCameraTabChanged(int index)
{
    // 防止循环触发
    if (m_isSyncingTabs) return;
    m_isSyncingTabs = true;

    if (index == 0) {
        ui->tabWidget_ProjectorControl->setCurrentIndex(0);
        ui->tabWidget_->setCurrentIndex(0);
    } else if (index == 1) {
        ui->tabWidget_ProjectorControl->setCurrentIndex(1);
        ui->tabWidget_->setCurrentIndex(1);
    }

    m_isSyncingTabs = false;
}

void CalibrationWindow::onProjectorTabChanged(int index)
{
    // 防止循环触发
    if (m_isSyncingTabs) return;
    m_isSyncingTabs = true;

    if (index == 0) {
        ui->tabWidget_CameraControl->setCurrentIndex(0);
        ui->tabWidget_->setCurrentIndex(0);
    } else if (index == 1) {
        ui->tabWidget_CameraControl->setCurrentIndex(1);
        ui->tabWidget_->setCurrentIndex(1);
    }

    m_isSyncingTabs = false;
}

void CalibrationWindow::onCalibrationTabChanged(int index)
{
    // 防止循环触发
    if (m_isSyncingTabs) return;
    m_isSyncingTabs = true;

    if (index == 0) {
        ui->tabWidget_CameraControl->setCurrentIndex(0);
        ui->tabWidget_ProjectorControl->setCurrentIndex(0);
    } else if (index == 1) {
        ui->tabWidget_CameraControl->setCurrentIndex(1);
        ui->tabWidget_ProjectorControl->setCurrentIndex(1);
    }

    m_isSyncingTabs = false;
}
