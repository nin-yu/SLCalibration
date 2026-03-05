#ifndef SINGLERECONSTRUCTWIDGET_H
#define SINGLERECONSTRUCTWIDGET_H

#include <QWidget>
#include <QQueue>
#include <QMutex>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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
     * @brief Start reconstruction button clicked
     */
    void on_pushButton_startReconstruct_clicked();

    /**
     * @brief Stop reconstruction button clicked
     */
    void on_pushButton_stopReconstruct_clicked();

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

private:
    /**
     * @brief Initialize reconstruction engine and workers
     * @return true if successful
     */
    bool initializeReconstruction();

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

    // Shared data structures (producer-consumer)
    QQueue<ReconImageBatch> m_reconImageQueue;
    QMutex m_reconBufferMutex;

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
};

#endif // SINGLERECONSTRUCTWIDGET_H
