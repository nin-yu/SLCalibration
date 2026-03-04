#ifndef SINGLECAMERACONTROLWIDGET_H
#define SINGLECAMERACONTROLWIDGET_H

#include <QWidget>
#include <QTimer>
#include <QMessageBox>
#include <QThread>
#include <QMutex>
#include <QTime>
#include "CameraController.h"
#include <opencv2/opencv.hpp>

namespace Ui {
class SingleCameraControlWidget;
}

class SingleCameraControlWidget : public QWidget
{
    Q_OBJECT

signals:
    void statusUpdated(const QString& message);
    void imageReceived(const QImage& image);

public:
    // 设置相机序列号（公共接口）
    bool setCameraSerialNumber(const QString& serialNumber);
    
    // 获取当前设置的相机序列号
    QString getCameraSerialNumber() const { return m_currentSerialNumber; }
    
    // 相机状态检查函数 - 从控制器查询状态
    bool isCameraOpen() const;
    bool isGrabbing() const;

    // 获取相机参数值
    int getCurrentExposure() const;
    float getCurrentGain() const;
    int getCurrentTriggerMode() const;
    int getCurrentTriggerEdge() const;
    QString getCurrentTriggerSource() const;
    double getCurrentTriggerDelay() const;
    
    // 获取相机控制器指针
    CCameraController* getCameraController() const { return m_cameraController; }
    
    // 获取相机句柄
    void* getCameraHandle() const { return m_cameraHandle; }
    
    // 刷新UI状态的公有接口
    void updateCameraStatus();
    void updateParameterControls();

public:
    explicit SingleCameraControlWidget(CCameraController* controller, QWidget *parent = nullptr);
    ~SingleCameraControlWidget();

private slots:
    // UI事件槽函数
    void on_pushButton_SetSN_clicked();
    void on_pushButton_OpenCamera_clicked();
    void on_pushButton_CloseCamera_clicked();
    void on_pushButton_StartGrabbing_clicked();
    void on_pushButton_StopGrabbing_clicked();
    void on_pushButton_SoftWareTrigger_clicked();

    // 参数设置槽函数
    void on_spinBox_Exposure_valueChanged(int arg1);
    void on_doubleSpinBox_Gain_valueChanged(double arg1);
    void on_comboBox_TriggerMode_currentIndexChanged(int index);
    void on_comboBox_TriggerEdge_currentIndexChanged(int index);
    void on_doubleSpinBox_TriggerDelay_valueChanged(double arg1);

    // 定时器槽函数
    void onTimeout_UpdateStatus();

private:
    // 初始化函数
    void initializeUI();

    // 工具函数
    void showErrorMessage(const QString& message);
    void logMessage(const QString& message, bool notifyMainWindow = false);
    void saveCapturedImage(const cv::Mat& image);

    // 相机控制函数
    bool openCameraBySerialNumber(const QString& serialNumber);
    bool configureCameraParameters();

private:
    Ui::SingleCameraControlWidget *ui;
    CCameraController* m_cameraController;
    QTimer* m_statusTimer;
    int m_grabTimerId;          // 主动取图定时器ID
    QString m_currentSerialNumber;  // 当前控件绑定的相机SN
    void* m_cameraHandle;       // 当前相机句柄
    
    // 图像缓冲区
    unsigned char* m_pImageBuffer;
    size_t m_imageBufferSize;
    
    // FPS计算相关
    int m_frameCounter;
    QTime m_lastFpsTime;
    
    // 采集模式标记
    bool m_isContinuousMode;  // 标记当前是否为连续采集模式

protected:
    // 定时器事件处理
    void timerEvent(QTimerEvent* event) override;
    
private:
    // 主动取图处理函数
    void grabAndProcessImage();
    
    // 默认缓冲区大小
    static const size_t DEFAULT_BUFFER_SIZE = 5 * 1024 * 1024;
};

#endif // SINGLECAMERACONTROLWIDGET_H