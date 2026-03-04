#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ProjectorController.h"
#include "CameraController.h"
#include "singleprojectorcontrolwidget.h"
#include "singlecameracontrolwidget.h"
#include "singlecalibrationwidget.h"
#include "singlereconstructwidget.h"
#include <QMessageBox>
#include <QDebug>
#include <QDateTime>
#include <QSettings>


namespace Ui {
class MainWindow;
}

// 配置文件相关
struct DeviceConfig {
    QString leftCameraSN;
    QString rightCameraSN;
    QString leftProjectorTag;
    QString rightProjectorTag;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    
    // 获取设备配置
    const DeviceConfig& getDeviceConfig() const { return m_deviceConfig; }

private slots:
    // 投影仪检测槽函数
    void onProjectorDetectClicked();
    
    // 投影仪状态更新槽函数
    void onProjectorStatusUpdated(const QString& message);

    // 相机检测槽函数
    void on_pushButton_CameraDetect_clicked();

    // 相机状态更新槽函数
    void onCameraStatusUpdated(const QString& message);

    // 相机图像接收槽函数
    void onLeftCameraImageReceived(const QImage& image);
    void onRightCameraImageReceived(const QImage& image);

    // 标定状态更新槽函数
    void onCalibrationStatusUpdated(const QString& message);
    void onCalibrationCompleted(bool success, const QString& result);

    // 标定初始化按钮槽函数
    void on_pushButton_InitCalibration_clicked();

    // 点云重建状态更新槽函数
    void onReconstructionStatusUpdated(const QString& message);
    void onReconstructionStateChanged(bool running);

private:
    
    // 加载配置文件
    bool loadConfiguration();
    
    // 创建默认配置文件
    bool createDefaultConfigFile(const QString& filePath);

    void saveDefaultConfiguration();
    // 初始化投影仪控制组件
    bool initializeProjectorControlWidgets();

    // 初始化相机控制组件
    bool initializeCameraControlWidgets();

    // 初始化标定子界面
    bool initializeCalibrationWidgets();

    // 初始化点云重建界面
    bool initializeReconstructWidgets();

    // 更新标定信息显示
    void updateCalibrationDisplayInfo();

    Ui::MainWindow *ui;

    // 投影仪控制器
    ProjectorController* m_projectorController;

    // 相机控制器
    CCameraController* m_cameraController;

    // 左右投影仪控制组件
    SingleProjectorControlWidget* m_leftProjectorWidget;
    SingleProjectorControlWidget* m_rightProjectorWidget;
    bool m_projectorControlWidgetInitialized = false; // 标记投影仪控制组件是否已初始化

    // 左右相机控制组件
    SingleCameraControlWidget* m_leftCameraWidget;
    SingleCameraControlWidget* m_rightCameraWidget;
    bool m_cameraControlWidgetInitialized = false; // 标记相机控制组件是否已初始化

    // 左右标定子界面
    SingleCalibrationWidget* m_leftCalibrationWidget;
    SingleCalibrationWidget* m_rightCalibrationWidget;
    bool m_calibrationWidgetInitialized = false; // 标记标定组件是否已初始化

    // 左右点云重建界面
    SingleReconstructWidget* m_leftReconstructWidget;
    SingleReconstructWidget* m_rightReconstructWidget;
    bool m_reconstructWidgetInitialized = false; // 标记重建组件是否已初始化
    
    // 设备配置信息
    DeviceConfig m_deviceConfig;
};
#endif // MAINWINDOW_H
