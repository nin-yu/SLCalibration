#ifndef RECONWORKER_H
#define RECONWORKER_H

#include <QThread>
#include <QQueue>
#include <QMutex>
#include <QWaitCondition>
#include "recontypes.h"

// 前向声明
class CCameraController;
class ReconEngine;

/**
 * @brief 图像采集工作线程
 * 
 * 负责从相机采集图像批次并放入队列
 */
class ReconGrabWorker : public QThread
{
    Q_OBJECT

public:
    static const int MAX_QUEUE_DEPTH = 3;  // 最大队列深度，防止内存溢出
    /**
     * @brief 构造函数
     * @param cameraController 相机控制器
     * @param cameraHandle 相机句柄
     * @param framesPerBatch 每批次的帧数
     * @param parent 父对象
     */
    explicit ReconGrabWorker(CCameraController* cameraController,
                            void* cameraHandle,
                            int framesPerBatch,
                            QObject* parent = nullptr);
    ~ReconGrabWorker();

    /**
     * @brief 获取图像队列
     */
    QQueue<ReconImageBatch>& getImageQueue() { return m_imageQueue; }

    /**
     * @brief 获取队列互斥锁
     */
    QMutex& getQueueMutex() { return m_queueMutex; }

    /**
     * @brief 获取队列条件变量
     */
    QWaitCondition& getQueueCondition() { return m_queueCondition; }

    /**
     * @brief 停止采集
     */
    void stop();

    /**
     * @brief 获取当前队列深度
     */
    int getQueueDepth() const;

    /**
     * @brief 获取已采集批次计数
     */
    int getGrabbedCount() const { return m_grabbedCount; }

    /**
     * @brief 清空队列
     */
    void clearQueue();

signals:
    /**
     * @brief 错误发生信号
     */
    void errorOccurred(const QString& message);

    /**
     * @brief 批次入队信号
     */
    void batchEnqueued(int queueSize);

protected:
    void run() override;

private:
    CCameraController* m_cameraController;
    void* m_cameraHandle;
    int m_framesPerBatch;
    std::atomic<bool> m_running;

    QQueue<ReconImageBatch> m_imageQueue;
    QMutex m_queueMutex;
    QWaitCondition m_queueCondition;
    int m_grabbedCount;  // 已采集批次计数
};

/**
 * @brief 图像处理工作线程
 * 
 * 负责从队列取出图像批次并重建点云
 */
class ReconProcessWorker : public QThread
{
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param reconEngine 重建引擎
     * @param imageQueue 图像队列
     * @param queueMutex 队列互斥锁
     * @param queueCondition 队列条件变量
     * @param parent 父对象
     */
    explicit ReconProcessWorker(ReconEngine* reconEngine,
                               QQueue<ReconImageBatch>& imageQueue,
                               QMutex& queueMutex,
                               QWaitCondition& queueCondition,
                               QObject* parent = nullptr);
    ~ReconProcessWorker();

    /**
     * @brief 停止处理
     */
    void stop();

signals:
    /**
     * @brief 点云就绪信号
     */
    void pointCloudReady(PointCloudT::Ptr cloud);

    /**
     * @brief 错误发生信号
     */
    void errorOccurred(const QString& message);

    /**
     * @brief 处理统计信号
     */
    void processingStats(double fps, int pointCount);

protected:
    void run() override;

private:
    ReconEngine* m_reconEngine;
    QQueue<ReconImageBatch>& m_imageQueue;
    QMutex& m_queueMutex;
    QWaitCondition& m_queueCondition;
    std::atomic<bool> m_running;

    // 统计信息
    int m_processedCount;
    double m_lastFps;
};

#endif // RECONWORKER_H
