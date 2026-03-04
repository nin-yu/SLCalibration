#ifndef RECONWORKER_H
#define RECONWORKER_H

#include <QThread>
#include <QQueue>
#include <QMutex>
#include <QVector>
#include <QMetaType>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "CameraController.h"
#include "reconengine.h"

// Image batch type
typedef QVector<cv::Mat> ReconImageBatch;

// Point cloud types
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Register metatype for cross-thread signal/slot
Q_DECLARE_METATYPE(PointCloudT::Ptr)

/**
 * @class ReconGrabWorker
 * @brief Worker thread for grabbing images from hardware-triggered camera
 * 
 * This worker continuously grabs images when the camera is in hardware trigger mode.
 * It collects images into batches and pushes them to a shared queue for processing.
 */
class ReconGrabWorker : public QThread
{
    Q_OBJECT

public:
    /**
     * @brief Constructor
     * @param cameraCtrl Camera controller instance
     * @param cameraHandle Camera handle
     * @param imageQueue Shared queue for image batches
     * @param queueMutex Mutex protecting the queue
     * @param parent Parent object
     */
    ReconGrabWorker(CCameraController* cameraCtrl,
                   void* cameraHandle,
                   QQueue<ReconImageBatch>* imageQueue,
                   QMutex* queueMutex,
                   QObject* parent = nullptr);

    ~ReconGrabWorker();

    /**
     * @brief Stop the worker thread
     * 
     * Call this method to signal the thread to stop.
     * After calling stop(), the caller should call wait() to ensure clean exit.
     */
    void stop();

    /**
     * @brief Set skip first batch flag
     * 
     * The first batch after starting may be incomplete due to synchronization.
     * This flag tells the worker to skip the first batch.
     */
    void setSkipFirstBatch(bool skip) { m_skipFirstBatch = skip; }

    /**
     * @brief Set frames per batch
     * @param count Number of frames per batch (default: 11 for Pattern 8)
     */
    void setFramesPerBatch(int count) { m_framesPerBatch = count; }

signals:
    /**
     * @brief Signal emitted when an error occurs
     */
    void errorOccurred(const QString& message);

    /**
     * @brief Signal emitted when a batch is enqueued
     */
    void batchEnqueued(int queueSize);

protected:
    void run() override;

private:
    CCameraController* m_cameraCtrl;
    void* m_cameraHandle;
    QQueue<ReconImageBatch>* m_imageQueue;
    QMutex* m_queueMutex;

    std::atomic<bool> m_stopFlag;
    std::atomic<bool> m_skipFirstBatch;
    int m_framesPerBatch;

    // Image buffer
    unsigned char* m_pImageBuffer;
    size_t m_bufferSize;
};

/**
 * @class ReconProcessWorker
 * @brief Worker thread for processing image batches and reconstructing point clouds
 * 
 * This worker takes image batches from the queue, processes them using ReconEngine,
 * and emits the resulting point cloud.
 */
class ReconProcessWorker : public QThread
{
    Q_OBJECT

public:
    /**
     * @brief Constructor
     * @param reconEngine Reconstruction engine instance
     * @param imageQueue Shared queue for image batches
     * @param queueMutex Mutex protecting the queue
     * @param parent Parent object
     */
    ReconProcessWorker(ReconEngine* reconEngine,
                      QQueue<ReconImageBatch>* imageQueue,
                      QMutex* queueMutex,
                      QObject* parent = nullptr);

    ~ReconProcessWorker();

    /**
     * @brief Stop the worker thread
     */
    void stop();

signals:
    /**
     * @brief Signal emitted when a point cloud is ready
     */
    void pointCloudReady(PointCloudT::Ptr cloud);

    /**
     * @brief Signal emitted when an error occurs
     */
    void errorOccurred(const QString& message);

    /**
     * @brief Signal emitted with processing statistics
     */
    void processingStats(double fps, int pointCount);

protected:
    void run() override;

private:
    ReconEngine* m_reconEngine;
    QQueue<ReconImageBatch>* m_imageQueue;
    QMutex* m_queueMutex;

    std::atomic<bool> m_stopFlag;
};

#endif // RECONWORKER_H
