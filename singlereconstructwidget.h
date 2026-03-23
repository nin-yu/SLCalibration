#ifndef SINGLERECONSTRUCTWIDGET_H
#define SINGLERECONSTRUCTWIDGET_H

#include <QWidget>
#include <QVector>
#include <QTimer>
#include <QElapsedTimer>
#include <QLabel>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "configmanager.h"

// Forward declarations
class ProjectorController;
class CCameraController;
class SingleCameraControlWidget;
class SingleProjectorControlWidget;
class ReconEngine;
class ReconGrabWorker;
class ReconProcessWorker;

// Point cloud types
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Image batch type
typedef QVector<cv::Mat> ReconImageBatch;

namespace Ui {
class SingleReconstructWidget;
}

/**
 * @class SingleReconstructWidget
 * @brief Widget for real-time 3D reconstruction
 * 
 * This widget provides UI for starting/stopping real-time 3D reconstruction.
 * It coordinates camera, projector, and reconstruction pipeline.
 */
class SingleReconstructWidget : public QWidget
{
    Q_OBJECT

public:
    /**
     * @brief Default constructor (for Qt Designer compatibility)
     */
    /**
     * @brief Full constructor with dependency injection
     * @param projCtrl Projector controller
     * @param camCtrl Camera controller
     * @param camWidget Camera control widget (for getting handle)
     * @param projWidget Projector control widget (for getting parameters)
     * @param projectorTag Projector serial number/tag
     * @param parent Parent widget
     */
    SingleReconstructWidget(ProjectorController* projCtrl,
                           CCameraController* camCtrl,
                           SingleCameraControlWidget* camWidget,
                           SingleProjectorControlWidget* projWidget,
                           const QString& projectorTag,
                           QWidget *parent = nullptr);

    ~SingleReconstructWidget();

    /**
     * @brief Initialize the widget with controllers
     * 
     * Call this after default construction to set up dependencies.
     */
    void initialize(ProjectorController* projCtrl,
                   CCameraController* camCtrl,
                   SingleCameraControlWidget* camWidget,
                   SingleProjectorControlWidget* projWidget,
                   const QString& projectorTag);

    /**
     * @brief Set camera serial number
     */
    void setCameraSerialNumber(const QString& serialNumber);

    /**
     * @brief Set device tag (left/right identifier)
     */
    void setDeviceTag(const QString& tag) { m_deviceTag = tag; }

    /**
     * @brief Set camera control widget (for online reconstruction)
     * @param camWidget Camera control widget pointer
     */
    void setCameraControlWidget(SingleCameraControlWidget* camWidget) { m_cameraWidget = camWidget; }

    /**
     * @brief Refresh button states - call this when camera open/close state changes
     */
    void refreshButtonStates() { updateReconstructionButtonStates(); }

    /**
     * @brief Get device tag
     */
    QString getDeviceTag() const { return m_deviceTag; }

    /**
     * @brief Check if reconstruction is running
     */
    bool isReconstructing() const { return m_isReconstructing; }

    /**
     * @brief Set projector exposure time (ms)
     */
    void setProjectorExposure(int exposureMs) { m_projectorExposure = exposureMs; }

    /**
     * @brief Check if camera is ready for online reconstruction
     */
    bool isCameraReady() const;

    /**
     * @brief Update UI button states based on current status
     */
    void updateReconstructionButtonStates();

signals:
    /**
     * @brief Emitted when status changes
     */
    void statusUpdated(const QString& message);

    /**
     * @brief Emitted when reconstruction starts/stops
     */
    void reconstructionStateChanged(bool running);

private slots:
    /**
     * @brief Continuous reconstruction button toggled (start/stop)
     * @param checked true if button is checked (running)
     */
    void on_pushButton_continueReconstruction_toggled(bool checked);

    /**
     * @brief Offline reconstruction button clicked
     */
    void on_pushButton_offlineReconstruction_clicked();

    /**
     * @brief Handle point cloud ready signal from process worker
     */
    void onPointCloudReady(PointCloudT::Ptr cloud);

    /**
     * @brief Handle error from workers
     */
    void onWorkerError(const QString& message);

    /**
     * @brief Handle processing stats
     */
    void onProcessingStats(double fps, int pointCount);

    /**
     * @brief Handle batch enqueued signal
     */
    void onBatchEnqueued(int queueSize);

    /**
     * @brief Status monitor timeout handler
     */
    void onStatusMonitorTimeout();

private:
    /**
     * @brief Initialize reconstruction engine and workers
     * @return true if successful
     */
    bool initializeReconstruction();

    /**
     * @brief Initialize reconstruction engine for specific device (left/right)
     * @param isLeftDevice true for left device, false for right device
     * @return true if successful
     */
    bool initializeReconstructionForDevice(bool isLeftDevice);

    /**
     * @brief Get calibration file path for specific device
     * @param isLeftDevice true for left device, false for right device
     * @return Path to calibration file
     */
    QString getCalibrationFilePathForDevice(bool isLeftDevice) const;

    /**
     * @brief Start the projector pattern sequence
     * @return true if successful
     */
    bool startProjector();

    /**
     * @brief Stop the projector
     */
    void stopProjector();

    /**
     * @brief Switch camera to hardware trigger mode
     * @return true if successful
     */
    bool switchCameraToHardwareTrigger();

    /**
     * @brief Switch camera back to continuous/software trigger mode
     * @return true if successful
     */
    bool switchCameraToSoftwareTrigger();

    /**
     * @brief Get calibration file path for current device
     * @return Path to calibration_data.xml
     */
    QString getCalibrationFilePath() const;

    /**
     * @brief Get offline calibration file path based on device type
     * @param isLeftDevice true for left device, false for right device
     * @return Path to calibration file
     */
    QString getOfflineCalibrationFilePath(bool isLeftDevice) const;

    /**
     * @brief Load offline images from directory based on device type
     * @param imageDir Directory containing offline images
     * @param isLeftDevice true for left device (set_0img_*.bmp), false for right device (set_1img_*.bmp)
     * @return Vector of loaded images
     */
    QVector<cv::Mat> loadOfflineImages(const QString& imageDir, bool isLeftDevice) const;

    /**
     * @brief Check if another device is currently reconstructing
     * @return true if another device is busy
     */
    bool isOtherDeviceReconstructing() const;

    /**
     * @brief Clean up worker threads
     */
    void cleanupWorkers();

    /**
     * @brief Update UI button states
     */
    void updateButtonStates();

    /**
     * @brief Log message to status
     */
    void logMessage(const QString& message);

    Ui::SingleReconstructWidget *ui;

    // Controllers (injected)
    ProjectorController* m_projectorController;
    CCameraController* m_cameraController;
    SingleCameraControlWidget* m_cameraWidget;
    SingleProjectorControlWidget* m_projectorWidget;  // 投影仪控制组件指针

    // Device identification
    QString m_projectorTag;
    QString m_cameraSerialNumber;
    QString m_deviceTag;  // "dibh_left" or "dibh_right"
    void* m_cameraHandle;

    // Reconstruction components
    ReconEngine* m_reconEngine;
    ReconGrabWorker* m_grabWorker;
    ReconProcessWorker* m_processWorker;

    // State
    bool m_isReconstructing;
    bool m_isInitialized;
    bool m_wasGrabbingBeforeRecon;  // Remember previous state

    // Parameters
    int m_nGrayCode;
    int m_nPhaseShift;
    int m_framesPerBatch;
    int m_projectorExposure;  // in milliseconds
    int m_projectorPattern;   // Pattern 8 by default

    // Statistics
    double m_currentFps;
    int m_currentPointCount;

    // Error handling and monitoring
    int m_consecutiveErrorCount;
    static const int MAX_CONSECUTIVE_ERRORS = 5;
    QTimer* m_statusMonitorTimer;
    QElapsedTimer m_reconTimer;
    int m_totalProcessedFrames;

    // UI elements for status display
    QLabel* m_statusLabel;
    QLabel* m_fpsLabel;
    QLabel* m_pointCountLabel;
};

#endif // SINGLERECONSTRUCTWIDGET_H
