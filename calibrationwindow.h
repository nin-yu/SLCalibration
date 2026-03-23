#ifndef CALIBRATIONWINDOW_H
#define CALIBRATIONWINDOW_H

#include <QMainWindow>
#include <QCloseEvent>
#include <QMessageBox>
#include <QDebug>
#include <QDateTime>
#include <QSettings>
#include <QTimer>

#include "deviceconfig.h"
#include "ProjectorController.h"
#include "CameraController.h"
#include "singleprojectorcontrolwidget.h"
#include "singlecameracontrolwidget.h"
#include "singlecalibrationwidget.h"
#include "singlereconstructwidget.h"

namespace Ui {
class CalibrationWindow;
}

class CalibrationWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit CalibrationWindow(const DeviceConfig& config, QWidget *parent = nullptr);
    ~CalibrationWindow();

    const DeviceConfig& getDeviceConfig() const { return m_deviceConfig; }

protected:
    void closeEvent(QCloseEvent *event) override;

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

    // QTabWidget标签同步槽函数
    void onCameraTabChanged(int index);
    void onProjectorTabChanged(int index);
    void onCalibrationTabChanged(int index);

    // 定时刷新重建界面按钮状态
    void onRefreshReconstructButtonStates();

private:
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

    // 设置QTabWidget标签同步
    void setupTabSynchronization();

    Ui::CalibrationWindow *ui;

    // 投影仪控制器
    ProjectorController* m_projectorController;

    // 相机控制器
    CCameraController* m_cameraController;

    // 左右投影仪控制组件
    SingleProjectorControlWidget* m_leftProjectorWidget;
    SingleProjectorControlWidget* m_rightProjectorWidget;
    bool m_projectorControlWidgetInitialized = false;

    // 左右相机控制组件
    SingleCameraControlWidget* m_leftCameraWidget;
    SingleCameraControlWidget* m_rightCameraWidget;
    bool m_cameraControlWidgetInitialized = false;

    // 左右标定子界面
    SingleCalibrationWidget* m_leftCalibrationWidget;
    SingleCalibrationWidget* m_rightCalibrationWidget;
    bool m_calibrationWidgetInitialized = false;

    // 左右点云重建界面
    SingleReconstructWidget* m_leftReconstructWidget;
    SingleReconstructWidget* m_rightReconstructWidget;
    bool m_reconstructWidgetInitialized = false;

    // 设备配置信息
    DeviceConfig m_deviceConfig;

    // 标签同步标志，防止循环触发
    bool m_isSyncingTabs = false;

    // 定时器用于刷新重建按钮状态
    QTimer* m_reconstructButtonRefreshTimer = nullptr;
};

#endif // CALIBRATIONWINDOW_H
