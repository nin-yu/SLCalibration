#include "reconworker.h"
#include "CameraController.h"
#include "reconengine.h"
#include <QDebug>
#include <QElapsedTimer>

// ============================================================================
// ReconGrabWorker 实现
// ============================================================================

ReconGrabWorker::ReconGrabWorker(CCameraController* cameraController,
                                 void* cameraHandle,
                                 int framesPerBatch,
                                 QObject* parent)
    : QThread(parent)
    , m_cameraController(cameraController)
    , m_cameraHandle(cameraHandle)
    , m_framesPerBatch(framesPerBatch)
    , m_running(false)
    , m_grabbedCount(0)
{
}

ReconGrabWorker::~ReconGrabWorker()
{
    stop();
}

void ReconGrabWorker::stop()
{
    m_running = false;
    m_queueCondition.wakeAll();
}

int ReconGrabWorker::getQueueDepth() const
{
    QMutexLocker locker(const_cast<QMutex*>(&m_queueMutex));
    return m_imageQueue.size();
}

void ReconGrabWorker::clearQueue()
{
    QMutexLocker locker(&m_queueMutex);
    while (!m_imageQueue.isEmpty()) {
        m_imageQueue.dequeue();
    }
}

void ReconGrabWorker::run()
{
    m_running = true;
    
    // 分配图像缓冲区
    size_t bufferSize = 4096 * 4096 * 2;
    unsigned char* imageBuffer = new unsigned char[bufferSize];
    
    ReconImageBatch batch;
    batch.reserve(m_framesPerBatch);
    
    while (m_running)
    {
        batch.clear();
        
        // 采集一批图像
        int collectedFrames = 0;
        while (collectedFrames < m_framesPerBatch && m_running)
        {
            MV_FRAME_OUT_INFO_EX frameInfo = {0};
            
            bool success = m_cameraController->GetImage(
                m_cameraHandle,
                imageBuffer,
                static_cast<unsigned int>(bufferSize),
                &frameInfo
            );
            
            if (!success)
            {
                continue;
            }
            
            // 转换为cv::Mat
            cv::Mat rawImage;
            if (frameInfo.enPixelType == PixelType_Gvsp_Mono8)
            {
                rawImage = cv::Mat(frameInfo.nHeight, frameInfo.nWidth, CV_8UC1, imageBuffer).clone();
            }
            else if (frameInfo.enPixelType == PixelType_Gvsp_Mono10 ||
                     frameInfo.enPixelType == PixelType_Gvsp_Mono12 ||
                     frameInfo.enPixelType == PixelType_Gvsp_Mono16)
            {
                cv::Mat rawMat(frameInfo.nHeight, frameInfo.nWidth, CV_16UC1, imageBuffer);
                rawMat.convertTo(rawImage, CV_8UC1, 1.0 / 256.0);
            }
            else
            {
                rawImage = cv::Mat(frameInfo.nHeight, frameInfo.nWidth, CV_8UC1, imageBuffer).clone();
            }
            
            batch.append(rawImage);
            ++collectedFrames;
        }
        
        // 将批次放入队列（带流量控制）
        if (batch.size() == m_framesPerBatch)
        {
            QMutexLocker locker(&m_queueMutex);
            
            // 队列深度限制：如果队列已满，丢弃最旧的批次
            if (m_imageQueue.size() >= MAX_QUEUE_DEPTH) {
                m_imageQueue.dequeue();  // 丢弃旧批次
                qDebug() << "[ReconGrabWorker] Queue full, dropped oldest batch";
            }
            
            m_imageQueue.enqueue(batch);
            m_grabbedCount++;
            m_queueCondition.wakeOne();
            emit batchEnqueued(m_imageQueue.size());
        }
    }
    
    delete[] imageBuffer;
}

// ============================================================================
// ReconProcessWorker 实现
// ============================================================================

ReconProcessWorker::ReconProcessWorker(ReconEngine* reconEngine,
                                       QQueue<ReconImageBatch>& imageQueue,
                                       QMutex& queueMutex,
                                       QWaitCondition& queueCondition,
                                       QObject* parent)
    : QThread(parent)
    , m_reconEngine(reconEngine)
    , m_imageQueue(imageQueue)
    , m_queueMutex(queueMutex)
    , m_queueCondition(queueCondition)
    , m_running(false)
    , m_processedCount(0)
    , m_lastFps(0.0)
{
}

ReconProcessWorker::~ReconProcessWorker()
{
    stop();
}

void ReconProcessWorker::stop()
{
    m_running = false;
    m_queueCondition.wakeAll();
}

void ReconProcessWorker::run()
{
    m_running = true;
    m_processedCount = 0;
    
    QElapsedTimer fpsTimer;
    fpsTimer.start();
    
    while (m_running)
    {
        ReconImageBatch batch;
        
        // 从队列获取批次
        {
            QMutexLocker locker(&m_queueMutex);
            while (m_imageQueue.isEmpty() && m_running)
            {
                m_queueCondition.wait(&m_queueMutex, 100);
            }
            
            if (!m_running)
            {
                break;
            }
            
            if (!m_imageQueue.isEmpty())
            {
                batch = m_imageQueue.dequeue();
            }
        }
        
        if (batch.isEmpty())
        {
            continue;
        }
        
        // 重建点云
        PointCloudT::Ptr cloud;
        try {
            cloud = m_reconEngine->reconstruct(batch);
        } catch (const std::exception& e) {
            emit errorOccurred(QString("Reconstruction exception: %1").arg(e.what()));
            continue;
        } catch (...) {
            emit errorOccurred("Unknown exception during reconstruction");
            continue;
        }
        
        if (cloud && !cloud->empty())
        {
            m_processedCount++;
            
            // 计算帧率
            qint64 elapsedMs = fpsTimer.elapsed();
            if (elapsedMs > 1000)
            {
                m_lastFps = m_processedCount * 1000.0 / elapsedMs;
                m_processedCount = 0;
                fpsTimer.restart();
            }
            
            emit pointCloudReady(cloud);
            emit processingStats(m_lastFps, static_cast<int>(cloud->size()));
        }
    }
}
