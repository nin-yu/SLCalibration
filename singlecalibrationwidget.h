#ifndef SINGLECALIBRATIONWIDGET_H
#define SINGLECALIBRATIONWIDGET_H

#include <QWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QTextEdit>
#include <QPushButton>
#include <QDir>
#include <QFileInfo>
#include <QDateTime>
#include <QGroupBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QSpinBox>
#include <QMessageBox>
#include <QMutex>
#include <QQueue>
#include <QTimer>
#include <QtConcurrent>
#include "ProjectorController.h"
#include "CameraController.h"
#include "singlecameracontrolwidget.h"
#include "gcpscalib.h"
#include <QProgressDialog>
#include <QFile>

class SingleCameraControlWidget;  // 前向声明
class SingleProjectorControlWidget;  // 前向声明

namespace Ui {
    class SingleCalibrationWidget;
}

class SingleCalibrationWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SingleCalibrationWidget(ProjectorController* projectorCtrl, 
                                   CCameraController* cameraCtrl,
                                   SingleCameraControlWidget* cameraWidget,
                                   SingleProjectorControlWidget* projectorWidget,
                                   const QString& tagName,
                                   QWidget *parent = nullptr);
    ~SingleCalibrationWidget();

    // 设置相机序列号
    void setCameraSerialNumber(const QString& serialNumber);
    
    // 获取当前设置的相机序列号
    QString getCameraSerialNumber() const { return m_cameraSerialNumber; }
    
    // 获取设备标识（左/右）
    QString getDeviceTag() const { return m_deviceTag; }


signals:
    // 状态更新信号
    void statusUpdated(const QString& message);
    
    // 标定完成信号
    void calibrationCompleted(bool success, const QString& result);

private slots:
    // UI事件槽函数
    void on_pushButton_StartCalibrateCamera_clicked();
    void on_pushButton_StartCaptureCameraImage_clicked();

    void on_pushButton_StartCalibrationSequenceProjector_clicked();
    void on_button_LoadImagesProjector_clicked();
    void on_button_StartCalibrationProjector_clicked();

    void on_pushButton_StartCalibrationCoordination_clicked();

    // 参数设置槽函数
    void on_spinBox_PatternRows_valueChanged(int arg1);
    void on_spinBox_PatternCols_valueChanged(int arg1);
    void on_doubleSpinBox_SquareSize_valueChanged(double arg1);
    


    void on_pushButton_deleteImagesCamera_clicked();

    void on_pushButton_deleteImagesProjector_clicked();

private:
    Ui::SingleCalibrationWidget *ui;

    // 初始化函数
    void initializeUIElements();
    void setupControllers();
    
    // 标定图像采集相关
    bool captureSingleImageWithSoftwareTrigger();
    bool captureImageAndReturn(cv::Mat& outImage);
    
    // 相机控制函数
    bool ensureCameraOpened();

    
    // 工具函数
    void updateCameraSNDisplay();
    void updateStatusDisplay();
    void showErrorMessage(const QString& message);
    void showSuccessMessage(const QString& message);
    void logMessage(const QString& message);
    
    // 标定控制函数
    bool captureCameraImages();
    bool calibrateCamera();
    bool captureProjectorSequence();
    bool loadProjectorImages();

    
    // 图像存储函数
    QString getBaseCalibrationPath() const;
    QString getDevicePath() const;
    QString getCalibrationTypePath(const QString& calibType) const;
    bool createDirectoryIfNotExists(const QString& path) const;
    bool saveCalibrationImage(const cv::Mat& image, const QString& calibType, int imageIndex);
    bool saveImageToCalibrationPath(const cv::Mat& image, const QString& calibrationPath);
    
    // 角点检测结果存储函数
    bool saveCornerDetectionResult(const QString& imagePath,
                                    const cv::Mat& image,
                                    const std::vector<cv::Point2f>& corners,
                                    bool detectionSuccess,
                                    const QString& calibrationPath);

    // 坐标系标定目录管理函数
    bool clearCoordinateDirectory();
    bool saveCoordinateImage(const cv::Mat& image, QString& savedImagePath);

    // 投影仪标定图像采集辅助函数
    // 扫描目录，返回下一个可用的 Pose 编号（从1开始，若目录中存在文件则续接编号）
    int getNextPoseNumber(const QString& projectorCalibPath) const;
    
    // 控制器指针
    ProjectorController* m_projectorController;
    CCameraController* m_cameraController;
    SingleCameraControlWidget* m_cameraWidget;  // 相机控制组件指针
    SingleProjectorControlWidget* m_projectorWidget;  // 投影仪控制组件指针
    
    // 相机句柄
    void* m_cameraHandle;
    
    // 图像缓冲区
    unsigned char* m_pImageBuffer;
    size_t m_imageBufferSize;

    // 设备标识信息
    QString m_deviceTag;        // "dibh_left" 或 "dibh_right"
    QString m_sideIdentifier;   // "左侧" 或 "右侧"
    bool m_isLeftDevice;        // 是否为左侧设备
    
    // 相机信息
    QString m_cameraSerialNumber;
    bool m_isCameraOpen;
    bool m_isCalibrating;
    
    // 标定参数
    int m_patternRows;
    int m_patternCols;
    
    // 相机标定结果参数
    cv::Mat m_cameraMatrix;     // 相机内参矩阵
    cv::Mat m_distCoeffs;       // 相机畸变系数
    
    // 图像采集相关
    QMutex m_imageMutex;
    QQueue<cv::Mat> m_imageQueue;
    QTimer* m_processTimer;
    int m_capturedImageCount;
    int m_targetImageCount;
    bool m_isCapturing;
    
    // 投影仪标定相关
    int m_sequenceIndex;
    int m_totalSequences;
    QStringList m_calibrationPatterns;
    int m_nProjCaliPoseCounter;  // 投影仪标定Pose计数器
    
    // UI控件
    QVBoxLayout* m_mainLayout;
    QLabel* m_label_Title;
    QLabel* m_label_CameraInfo;
    QLabel* m_label_Status;
    QTextEdit* m_textEdit_Log;


    
    // 参数设置控件
    QSpinBox* m_spinBox_PatternRows;
    QSpinBox* m_spinBox_PatternCols;
    QDoubleSpinBox* m_doubleSpinBox_SquareSize;
    
    // 投影仪标定相关成员变量
    GcPsCalib* m_calibrator;           // GcPsCalib标定器实例
    std::vector<std::string> m_imagePaths;  // 标定图像路径列表
    float m_squareSizeParam;           // 棋盘格方块大小(mm)
    cv::Size m_projSize;               // 投影仪分辨率
    int m_projFreq;                    // 投影仪频率
    int m_nGrayCode;                   // 格雷码位数
    int m_nPhaseShift;                 // 相移步数
    CalibrationData m_calibData;       // 标定数据
    
    // 状态更新函数
    void updateStatus(const QString& message);
    void onCalibrationFinished(bool success);
};

#endif // SINGLECALIBRATIONWIDGET_H
