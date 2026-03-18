#ifndef STRUCTUREDLIGHTDECODERRECON_H
#define STRUCTUREDLIGHTDECODERRECON_H

#include <QObject>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <map>

#include "graycode.h" // use centralized GrayCode

/**
 * @brief 结构光解码器
 * 
 * 用于解码格雷码和相移图案，计算绝对相位
 */
class StructuredLightDecoderRecon : public QObject
{
    Q_OBJECT

public:
    /**
     * @brief 初始化解码器
     * @param N_GC 格雷码位数（例如5）
     * @param N_PS 相移步数（例如4）
     * @param freq 相移图案的周期数（例如16）
     * @param proj_res 投影仪分辨率 (width, height)
     * @param direction "vertical"（编码X）或 "horizontal"（编码Y）
     * @param parent Qt父对象
     */
    StructuredLightDecoderRecon(int N_GC, int N_PS, int freq,
        cv::Size proj_res, std::string direction, QObject* parent = nullptr);
    ~StructuredLightDecoderRecon();

    /**
     * @brief 解码原始图像得到绝对相位
     * @param gc_imgs_raw 原始格雷码图像（N_GC张）
     * @param ps_imgs_raw 原始相移图像（N_PS张）
     * @param dark_img 暗场图像
     * @return 绝对相位图 (CV_32F)
     */
    cv::Mat decode(const std::vector<cv::Mat>& gc_imgs_raw,
        const std::vector<cv::Mat>& ps_imgs_raw,
        const cv::Mat& dark_img);

    /**
     * @brief 解码原始图像（无暗场图像）
     */
    cv::Mat decode_without_dark_img(const std::vector<cv::Mat>& gc_imgs_raw,
        const std::vector<cv::Mat>& ps_imgs_raw);

    /**
     * @brief 将绝对相位图转换为投影仪像素坐标
     */
    cv::Mat phase_to_pixels(const cv::Mat& unwrapped_phase);

private:
    int m_nN_GC;            // 格雷码位数
    int m_nN_PS;            // 相移步数
    int m_nFreq;            // 相移频率
    cv::Size m_projSize;    // 投影仪分辨率
    std::string m_sDirection; // 编码方向

    GrayCode m_g_k1;        // N_GC-1 位的格雷码
    GrayCode m_g_k2;        // N_GC 位的格雷码

    cv::Mat _compute_threshold(const std::vector<cv::Mat>& ps_imgs);
    std::vector<cv::Mat> _binarize_gc(const std::vector<cv::Mat>& gc_imgs,
        const cv::Mat& threshold);
    cv::Mat _compute_wrapped_phase(const std::vector<cv::Mat>& ps_imgs);
    bool _get_k1_k2(const std::vector<cv::Mat>& binarized_gc_imgs,
        cv::Mat& k1, cv::Mat& k2);
    cv::Mat _compute_unwrapped_phase(const cv::Mat& wrapped_pha,
        const cv::Mat& k1, const cv::Mat& k2);
};

std::vector<cv::Point2f> get_subpixel_values(const cv::Mat& map_u,
    const cv::Mat& map_v,
    const std::vector<cv::Point2f>& corners_cam);

#endif // STRUCTUREDLIGHTDECODERRECON_H
