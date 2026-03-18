#include <cmath>
#include <iostream>
#include "structuredlightdecoderRecon.h"

// ============================================================================
// StructuredLightDecoderRecon 实现
// ============================================================================

StructuredLightDecoderRecon::StructuredLightDecoderRecon(int N_GC, int N_PS, int freq,
    cv::Size proj_res, std::string direction, QObject* parent)
    : QObject(parent)
    , m_nN_GC(N_GC)
    , m_nN_PS(N_PS)
    , m_nFreq(freq)
    , m_projSize(proj_res)
    , m_sDirection(direction)
    , m_g_k1(N_GC - 1)
    , m_g_k2(N_GC)
{
    if (N_PS != 4)
    {
        std::cerr << "Warning: Decoder currently only supports 4-step phase shift." << std::endl;
    }
}

StructuredLightDecoderRecon::~StructuredLightDecoderRecon() {}

cv::Mat StructuredLightDecoderRecon::_compute_threshold(const std::vector<cv::Mat>& ps_imgs)
{
    cv::Mat i_th_f(ps_imgs[0].size(), CV_32F, cv::Scalar(0));
    for (const auto& img : ps_imgs)
    {
        cv::Mat f_img;
        img.convertTo(f_img, CV_32F);
        i_th_f += f_img;
    }
    i_th_f /= (float)m_nN_PS;

    cv::Mat i_th_8u;
    i_th_f.convertTo(i_th_8u, CV_8U);
    return i_th_8u;
}

std::vector<cv::Mat> StructuredLightDecoderRecon::_binarize_gc(
    const std::vector<cv::Mat>& gc_imgs, const cv::Mat& threshold)
{
    std::vector<cv::Mat> binarized_imgs;
    for (const auto& img_raw : gc_imgs)
    {
        cv::Mat bin_img;
        cv::compare(img_raw, threshold, bin_img, cv::CMP_GT);
        bin_img = bin_img / 255;
        binarized_imgs.push_back(bin_img);
    }
    return binarized_imgs;
}

cv::Mat StructuredLightDecoderRecon::_compute_wrapped_phase(const std::vector<cv::Mat>& ps_imgs)
{
    cv::Mat i0, i1, i2, i3;
    ps_imgs[0].convertTo(i0, CV_32F);
    ps_imgs[1].convertTo(i1, CV_32F);
    ps_imgs[2].convertTo(i2, CV_32F);
    ps_imgs[3].convertTo(i3, CV_32F);

    cv::Mat numerator = i3 - i1;
    cv::Mat denominator = i0 - i2;

    cv::Mat pha_degrees(numerator.size(), CV_32F);

    #pragma omp parallel for
    for (int r = 0; r < pha_degrees.rows; ++r)
    {
        for (int c = 0; c < pha_degrees.cols; ++c)
        {
            pha_degrees.at<float>(r, c) = cv::fastAtan2(
                numerator.at<float>(r, c),
                denominator.at<float>(r, c)
            );
        }
    }

    cv::Mat pha_radians;
    cv::multiply(pha_degrees, (float)(CV_PI / 180.0), pha_radians);

    return pha_radians;
}

bool StructuredLightDecoderRecon::_get_k1_k2(const std::vector<cv::Mat>& binarized_gc_imgs,
    cv::Mat& k1, cv::Mat& k2)
{
    int rows = binarized_gc_imgs[0].rows;
    int cols = binarized_gc_imgs[0].cols;
    k1.create(rows, cols, CV_8U);
    k2.create(rows, cols, CV_8U);

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            int code1_int = 0;
            int code2_int = 0;

            // 构建 code1_int (N_GC-1 位)
            for (int i = 0; i < m_nN_GC - 1; ++i)
            {
                int bit = binarized_gc_imgs[i].at<uchar>(r, c);
                code1_int |= (bit << ((m_nN_GC - 2) - i));
            }

            // 构建 code2_int (N_GC 位)
            int last_bit = binarized_gc_imgs[m_nN_GC - 1].at<uchar>(r, c);
            code2_int = (code1_int << 1) | last_bit;

            auto it1 = m_g_k1.m_code2k_int.find(code1_int);
            auto it2 = m_g_k2.m_code2k_int.find(code2_int);

            if (it1 == m_g_k1.m_code2k_int.end() || it2 == m_g_k2.m_code2k_int.end())
            {
                k1.at<uchar>(r, c) = 0;
                k2.at<uchar>(r, c) = 0;
                return false;
            }

            int k1_val = it1->second;
            int k2_val_raw = it2->second;

            k1.at<uchar>(r, c) = static_cast<uchar>(k1_val);
            k2.at<uchar>(r, c) = static_cast<uchar>(std::floor((k2_val_raw + 1) / 2.0));
        }
    }
    return true;
}

cv::Mat StructuredLightDecoderRecon::_compute_unwrapped_phase(const cv::Mat& wrapped_pha,
    const cv::Mat& k1, const cv::Mat& k2)
{
    cv::Mat unwrapped_pha = cv::Mat::zeros(wrapped_pha.size(), CV_32F);

    cv::Mat k1_f, k2_f;
    k1.convertTo(k1_f, CV_32F);
    k2.convertTo(k2_f, CV_32F);

    const float pi_half = (float)(CV_PI / 2.0);
    const float pi_3_half = (float)(3.0 * CV_PI / 2.0);
    const float two_pi = (float)(2.0 * CV_PI);

    #pragma omp parallel for
    for (int r = 0; r < wrapped_pha.rows; ++r)
    {
        for (int c = 0; c < wrapped_pha.cols; ++c)
        {
            float pha = wrapped_pha.at<float>(r, c);
            float k1_val = k1_f.at<float>(r, c);
            float k2_val = k2_f.at<float>(r, c);

            if (pha <= pi_half)
            {
                unwrapped_pha.at<float>(r, c) = pha + k2_val * two_pi;
            }
            else if (pha > pi_half && pha < pi_3_half)
            {
                unwrapped_pha.at<float>(r, c) = pha + k1_val * two_pi;
            }
            else
            {
                unwrapped_pha.at<float>(r, c) = pha + (k2_val - 1) * two_pi;
            }
        }
    }
    return unwrapped_pha;
}

cv::Mat StructuredLightDecoderRecon::decode(const std::vector<cv::Mat>& gc_imgs_raw,
    const std::vector<cv::Mat>& ps_imgs_raw,
    const cv::Mat& dark_img)
{
    std::vector<cv::Mat> ps_imgs, gc_imgs;
    cv::Mat dark_s16;
    dark_img.convertTo(dark_s16, CV_16S);

    // 减去暗场
    for (const auto& img : ps_imgs_raw)
    {
        cv::Mat s16_img, sub_img;
        img.convertTo(s16_img, CV_16S);
        cv::subtract(s16_img, dark_s16, sub_img);
        cv::Mat u8_img;
        sub_img.convertTo(u8_img, CV_8U, 1.0, 0);
        ps_imgs.push_back(u8_img);
    }
    for (const auto& img : gc_imgs_raw)
    {
        cv::Mat s16_img, sub_img;
        img.convertTo(s16_img, CV_16S);
        cv::subtract(s16_img, dark_s16, sub_img);
        cv::Mat u8_img;
        sub_img.convertTo(u8_img, CV_8U, 1.0, 0);
        gc_imgs.push_back(u8_img);
    }

    // 计算阈值
    cv::Mat threshold = _compute_threshold(ps_imgs);

    // 二值化格雷码
    std::vector<cv::Mat> binarized_gc = _binarize_gc(gc_imgs, threshold);

    // 计算包裹相位
    cv::Mat wrapped_phase = _compute_wrapped_phase(ps_imgs);

    // 获取 K1, K2
    cv::Mat k1, k2;
    if (!_get_k1_k2(binarized_gc, k1, k2))
    {
        std::cerr << "Decoding error: Failed to find GrayCode in dictionary." << std::endl;
        return cv::Mat();
    }

    // 解包裹相位
    cv::Mat unwrapped_phase = _compute_unwrapped_phase(wrapped_phase, k1, k2);

    return unwrapped_phase;
}

cv::Mat StructuredLightDecoderRecon::decode_without_dark_img(const std::vector<cv::Mat>& gc_imgs_raw,
    const std::vector<cv::Mat>& ps_imgs_raw)
{
    std::vector<cv::Mat> ps_imgs, gc_imgs;

    // 转换图像
    for (const auto& img : ps_imgs_raw)
    {
        cv::Mat u8_img;
        img.convertTo(u8_img, CV_8U);
        ps_imgs.push_back(u8_img);
    }
    for (const auto& img : gc_imgs_raw)
    {
        cv::Mat u8_img;
        img.convertTo(u8_img, CV_8U);
        gc_imgs.push_back(u8_img);
    }

    // 计算阈值
    cv::Mat threshold = _compute_threshold(ps_imgs);

    // 二值化格雷码
    std::vector<cv::Mat> binarized_gc = _binarize_gc(gc_imgs, threshold);

    // 获取 k1/k2
    cv::Mat k1_cpu, k2_cpu;
    if (!_get_k1_k2(binarized_gc, k1_cpu, k2_cpu))
    {
        std::cerr << "Decoding error (No Dark): Failed to find GrayCode in dictionary." << std::endl;
        return cv::Mat();
    }

    // 计算包裹相位
    cv::Mat wrapped_phase = _compute_wrapped_phase(ps_imgs);

    // 解包裹相位
    cv::Mat unwrapped_phase = _compute_unwrapped_phase(wrapped_phase, k1_cpu, k2_cpu);

    return unwrapped_phase;
}

cv::Mat StructuredLightDecoderRecon::phase_to_pixels(const cv::Mat& unwrapped_phase)
{
    float res = (m_sDirection == "vertical") ? (float)m_projSize.width : (float)m_projSize.height;

    cv::Mat pixels;
    cv::multiply(unwrapped_phase, cv::Scalar(res / (2.0 * CV_PI * m_nFreq)), pixels);

    return pixels;
}

// ============================================================================
// 辅助函数实现
// ============================================================================

std::vector<cv::Point2f> get_subpixel_values(const cv::Mat& map_u,
    const cv::Mat& map_v,
    const std::vector<cv::Point2f>& corners_cam)
{
    if (corners_cam.empty())
    {
        return std::vector<cv::Point2f>();
    }

    int n_corners = static_cast<int>(corners_cam.size());

    cv::Mat map_x(1, n_corners, CV_32F);
    cv::Mat map_y(1, n_corners, CV_32F);
    for (int i = 0; i < n_corners; ++i)
    {
        map_x.at<float>(0, i) = corners_cam[i].x;
        map_y.at<float>(0, i) = corners_cam[i].y;
    }

    cv::Mat u_p_mat, v_p_mat;
    cv::remap(map_u, u_p_mat, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::remap(map_v, v_p_mat, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));

    std::vector<cv::Point2f> corners_proj;
    corners_proj.reserve(n_corners);
    for (int i = 0; i < n_corners; ++i)
    {
        corners_proj.push_back(cv::Point2f(
            u_p_mat.at<float>(0, i),
            v_p_mat.at<float>(0, i)
        ));
    }
    return corners_proj;
}
