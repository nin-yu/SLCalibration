#include "reconworker.h"
#include <QDebug>
#include <QElapsedTimer>

// Register metatype at static initialization time
static struct RegisterMetaTypes {
    RegisterMetaTypes() {
        qRegisterMetaType<PointCloudT::Ptr>("PointCloudT::Ptr");
    }
} s_registerMetaTypes;

// ============================================================================
// ReconGrabWorker Implementation
// ============================================================================

ReconGrabWorker::ReconGrabWorker(CCameraController* cameraCtrl,
                                 void* cameraHandle,
                                 QQueue<ReconImageBatch>* imageQueue,
                                 QMutex* queueMutex,
                                 QObject* parent)
    : QThread(parent)
    , m_cameraCtrl(cameraCtrl)
    , m_cameraHandle(cameraHandle)
    , m_imageQueue(imageQueue)
    , m_queueMutex(queueMutex)
    , m_stopFlag(false)
    , m_skipFirstBatch(true)
    , m_framesPerBatch(11)  // Pattern 8: 1 white + 1 dark + 5 GC + 4 PS
    , m_pImageBuffer(nullptr)
    , m_bufferSize(0)
{
    // Allocate buffer for camera images (assume max 4K resolution with 16-bit depth)
    m_bufferSize = 4096 * 4096 * 2;  // 32MB buffer
    m_pImageBuffer = new unsigned char[m_bufferSize];
}

ReconGrabWorker::~ReconGrabWorker()
{
    if (m_pImageBuffer) {
        delete[] m_pImageBuffer;
        m_pImageBuffer = nullptr;
    }
}

void ReconGrabWorker::stop()
{
    m_stopFlag = true;
}

void ReconGrabWorker::run()
{
    qDebug() << "ReconGrabWorker started, frames per batch:" << m_framesPerBatch;

    ReconImageBatch currentBatch;
    currentBatch.reserve(m_framesPerBatch);
    int batchCount = 0;
    bool isFirstBatch = true;

    while (!m_stopFlag) {
        // Get image from camera (blocking call with timeout)
        MV_FRAME_OUT_INFO_EX frameInfo = {0};
        
        // GetImage blocks until a hardware trigger is received or timeout
        bool success = m_cameraCtrl->GetImage(
            m_cameraHandle,
            m_pImageBuffer,
            static_cast<unsigned int>(m_bufferSize),
            &frameInfo
        );

        if (m_stopFlag) {
            break;
        }

        if (!success) {
            // Timeout or error - continue waiting
            continue;
        }

        // Convert to cv::Mat and clone (important: must clone to avoid buffer overwrite)
        cv::Mat rawImage;
        
        // Determine image format based on pixel type
        if (frameInfo.enPixelType == PixelType_Gvsp_Mono8) {
            rawImage = cv::Mat(frameInfo.nHeight, frameInfo.nWidth, CV_8UC1, m_pImageBuffer).clone();
        } else if (frameInfo.enPixelType == PixelType_Gvsp_Mono10 ||
                   frameInfo.enPixelType == PixelType_Gvsp_Mono12 ||
                   frameInfo.enPixelType == PixelType_Gvsp_Mono16) {
            // For 10/12/16-bit images, convert to 8-bit
            cv::Mat rawMat(frameInfo.nHeight, frameInfo.nWidth, CV_16UC1, m_pImageBuffer);
            rawMat.convertTo(rawImage, CV_8UC1, 1.0 / 256.0);
        } else {
            // Assume 8-bit grayscale for other formats
            rawImage = cv::Mat(frameInfo.nHeight, frameInfo.nWidth, CV_8UC1, m_pImageBuffer).clone();
        }

        // Add to current batch
        currentBatch.append(rawImage);

        // Check if batch is complete
        if (currentBatch.size() >= m_framesPerBatch) {
            if (isFirstBatch && m_skipFirstBatch) {
                // Skip the first batch (may be incomplete due to sync issues)
                qDebug() << "Skipping first batch (sync)";
                currentBatch.clear();
                currentBatch.reserve(m_framesPerBatch);
                isFirstBatch = false;
                continue;
            }

            // Enqueue the batch
            {
                QMutexLocker locker(m_queueMutex);
                m_imageQueue->enqueue(currentBatch);
                int queueSize = m_imageQueue->size();
                emit batchEnqueued(queueSize);
                
                if (queueSize > 5) {
                    qDebug() << "Warning: queue growing large:" << queueSize << "batches";
                }
            }

            ++batchCount;
            qDebug() << "Batch" << batchCount << "enqueued";

            // Reset for next batch
            currentBatch.clear();
            currentBatch.reserve(m_framesPerBatch);
            isFirstBatch = false;
        }
    }

    qDebug() << "ReconGrabWorker stopped, total batches:" << batchCount;
}

// ============================================================================
// ReconProcessWorker Implementation
// ============================================================================

ReconProcessWorker::ReconProcessWorker(ReconEngine* reconEngine,
                                       QQueue<ReconImageBatch>* imageQueue,
                                       QMutex* queueMutex,
                                       QObject* parent)
    : QThread(parent)
    , m_reconEngine(reconEngine)
    , m_imageQueue(imageQueue)
    , m_queueMutex(queueMutex)
    , m_stopFlag(false)
{
}

ReconProcessWorker::~ReconProcessWorker()
{
}

void ReconProcessWorker::stop()
{
    m_stopFlag = true;
}

void ReconProcessWorker::run()
{
    qDebug() << "ReconProcessWorker started";

    QElapsedTimer timer;
    int processedCount = 0;
    double totalTime = 0.0;

    while (!m_stopFlag) {
        ReconImageBatch batch;
        bool hasBatch = false;

        // Try to get a batch from the queue
        {
            QMutexLocker locker(m_queueMutex);
            if (!m_imageQueue->isEmpty()) {
                batch = m_imageQueue->dequeue();
                hasBatch = true;
            }
        }

        if (!hasBatch) {
            // No batch available, wait a bit
            if (m_stopFlag) {
                break;
            }
            msleep(10);
            continue;
        }

        // Process the batch
        timer.start();
        
        PointCloudT::Ptr cloud = m_reconEngine->reconstruct(batch);
        
        qint64 elapsed = timer.elapsed();
        totalTime += elapsed;
        ++processedCount;

        if (cloud && !cloud->empty()) {
            // Calculate FPS
            double avgTime = totalTime / processedCount;
            double fps = 1000.0 / avgTime;

            qDebug() << "Batch processed:" << elapsed << "ms,"
                     << cloud->points.size() << "points,"
                     << "avg FPS:" << QString::number(fps, 'f', 1);

            // Emit the point cloud (use queued connection for thread safety)
            emit pointCloudReady(cloud);
            emit processingStats(fps, static_cast<int>(cloud->points.size()));
        } else {
            qDebug() << "Reconstruction failed or empty cloud";
            emit errorOccurred("Reconstruction produced empty result");
        }
    }

    qDebug() << "ReconProcessWorker stopped, processed" << processedCount << "batches";
    
    if (processedCount > 0) {
        double avgTime = totalTime / processedCount;
        qDebug() << "Average processing time:" << avgTime << "ms";
    }
}
