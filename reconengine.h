#ifndef RECONENGINE_H
#define RECONENGINE_H

#include <QObject>
#include "recontypes.h"
#include "structuredlightdecoderRecon.h"

/**
 * @brief 三维重建引擎
 * 
 * 整合标定数据加载、结构光解码和三维点云重建功能
 */
class ReconEngine : public QObject
{
    Q_OBJECT

public:
    explicit ReconEngine(QObject* parent = nullptr);
    ~ReconEngine();

    /**
     * @brief 加载标定数据
     * @param calibFilePath 标定文件路径（XML格式）
     * @param scale 缩放因子（默认1.0）
     * @return 成功返回true
     */
    bool loadCalibration(const QString& calibFilePath, double scale = 1.0);

    /**
     * @brief 从图像批次重建点云
     * @param imageBatch 图像批次（5张格雷码 + 4张相移 = 9张）
     * @return 重建的点云
     */
    PointCloudT::Ptr reconstruct(const ReconImageBatch& imageBatch);

    /**
     * @brief 设置调制阈值
     * @param threshold 调制阈值（默认70）
     */
    void setModulationThreshold(double threshold);

    /**
     * @brief 设置深度范围
     * @param minDepth 最小深度（mm）
     * @param maxDepth 最大深度（mm）
     */
    void setDepthRange(float minDepth, float maxDepth);

    /**
     * @brief 获取标定数据
     */
    const CalibData& getCalibData() const { return m_calibData; }

    /**
     * @brief 检查是否已加载标定数据
     */
    bool isCalibrationLoaded() const { return m_calibrationLoaded; }

signals:
    /**
     * @brief 错误发生信号
     */
    void errorOccurred(const QString& message);

private:
    /**
     * @brief 创建调制掩码
     * @param ps_imgs 4张相移图像
     * @param threshold 调制阈值
     * @return 有效像素掩码 (CV_8U)
     */
    cv::Mat createModulationMask(const std::vector<cv::Mat>& ps_imgs, int threshold);

    /**
     * @brief 重建三维点
     * @param map_up 投影仪X坐标图 (CV_32F)
     * @param valid_mask 有效像素掩码 (CV_8U)
     * @param out_points_3d 输出的三维点 (CV_32FC3)
     * @return 成功返回true
     */
    bool reconstruct3DPoints(const cv::Mat& map_up,
        const cv::Mat& valid_mask,
        cv::Mat& out_points_3d);

    /**
     * @brief 预计算重建参数
     */
    void precomputeReconstructionParams();

private:
    CalibData m_calibData;              // 标定数据
    ReconParams m_params;               // 重建参数
    StructuredLightDecoderRecon* m_decoder;  // 结构光解码器
    bool m_calibrationLoaded;           // 标定数据是否已加载
};

#endif // RECONENGINE_H
