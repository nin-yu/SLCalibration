#include "structuredlightdecoder.h"
#include <iostream>
#include <cmath>
#include <map>

// GrayCode implementation
GrayCode::GrayCode(int n) : m_n(n)
{
    if (n < 1) return;
    std::vector<std::string> code_temp = __createGrayCode(n);
    __formCodes(n, code_temp);

    // Populate maps
    for (int k = 0; k < static_cast<int>(pow(2, n)); ++k)
    {
        // Convert k to binary string, then convert that string to base-2 integer (e.g., "111" -> 7)
        int gray_value_as_int = __k2v(k);

        // This creates a "binary value -> k index" mapping
        m_code2k_int[gray_value_as_int] = k; // NEW MAP: e.g., 7 -> 5

        m_k2v[k] = gray_value_as_int; // OLD MAP: e.g., 5 -> 7 (This is still needed)
    }
}

GrayCode::~GrayCode() {}

std::vector<std::string> GrayCode::__createGrayCode(int n)
{
    if (n < 1)
    {
        return std::vector<std::string>();
    }
    if (n == 1)
    {
        return { "0", "1" };
    }

    std::vector<std::string> prev_code = __createGrayCode(n - 1);
    std::vector<std::string> code;
    for (const auto& s : prev_code)
    {
        code.push_back("0" + s);
    }
    for (int i = prev_code.size() - 1; i >= 0; --i)
    {
        code.push_back("1" + prev_code[i]);
    }
    return code;
}

void GrayCode::__formCodes(int n, const std::vector<std::string>& code_temp)
{
    int num_codes = static_cast<int>(code_temp.size()); // 2^n
    m_codes.create(n, num_codes, CV_8U);

    for (int row = 0; row < n; ++row)
    {
        for (int col = 0; col < static_cast<int>(num_codes); ++col)
        {
            m_codes.at<uchar>(row, col) = (code_temp[col][row] == '1' ? 1 : 0);
        }
    }
}

std::string GrayCode::__code2k(int k)
{
    std::string code = "";
    for (int i = 0; i < m_n; ++i)
    {
        code += std::to_string(m_codes.at<uchar>(i, k));
    }
    return code;
}

int GrayCode::__k2v(int k)
{
    std::string code = __code2k(k);
    return std::stoi(code, nullptr, 2);
}

// StructuredLightDecoder implementation
StructuredLightDecoder::StructuredLightDecoder(int nGrayCode,
                                               int nPhaseShift,
                                               int projFreq,
                                               cv::Size projSize,
                                               const std::string& direction)
    : m_nGrayCode(nGrayCode), m_nPhaseShift(nPhaseShift),
    m_projFreq(projFreq), m_projSize(projSize), m_direction(direction),
    m_g_k1(nGrayCode - 1), m_g_k2(nGrayCode) // Initialize GrayCode helpers
{
    if (nPhaseShift != 4) {
        std::cerr << "Warning: Decoder currently only supports 4-step phase shift." << std::endl;
    }
}

cv::Mat StructuredLightDecoder::decode(const std::vector<cv::Mat>& gc_imgs_raw,
                                       const std::vector<cv::Mat>& ps_imgs_raw,
                                       const cv::Mat& dark_img)
{
    std::vector<cv::Mat> ps_imgs, gc_imgs;
    cv::Mat dark_s16;
    dark_img.convertTo(dark_s16, CV_16S);

    // 1. Subtract dark field
    for (const auto& img : ps_imgs_raw)
    {
        cv::Mat s16_img, sub_img;
        img.convertTo(s16_img, CV_16S);
        cv::subtract(s16_img, dark_s16, sub_img);
        cv::Mat u8_img;
        sub_img.convertTo(u8_img, CV_8U, 1.0, 0); // Clamp at 0
        ps_imgs.push_back(u8_img);
    }
    for (const auto& img : gc_imgs_raw)
    {
        cv::Mat s16_img, sub_img;
        img.convertTo(s16_img, CV_16S);
        cv::subtract(s16_img, dark_s16, sub_img);
        cv::Mat u8_img;
        sub_img.convertTo(u8_img, CV_8U, 1.0, 0); // Clamp at 0
        gc_imgs.push_back(u8_img);
    }

    // 2. Compute threshold
    cv::Mat threshold = _compute_threshold(ps_imgs);

    // 3. Binarize GrayCode
    std::vector<cv::Mat> binarized_gc = _binarize_gc(gc_imgs, threshold);

    // 4. Compute wrapped phase
    cv::Mat wrapped_phase = _compute_wrapped_phase(ps_imgs);

    // 5. Get K1, K2
    cv::Mat k1, k2;
    if (!_get_k1_k2(binarized_gc, k1, k2))
    {
        std::cerr << "Decoding error: Failed to find GrayCode in dictionary. Binarization may have failed." << std::endl;
        return cv::Mat(); // Return empty mat on failure
    }

    // 6. Unwrap phase
    cv::Mat unwrapped_phase = _compute_unwrapped_phase(wrapped_phase, k1, k2);

    return unwrapped_phase;
}

cv::Mat StructuredLightDecoder::phaseToPixels(const cv::Mat& phaseMap)
{
    // 根据方向选择正确的分辨率
    // vertical 方向使用 width，horizontal 方向使用 height
    float res = (m_direction == "vertical") ? (float)m_projSize.width 
                                            : (float)m_projSize.height;
    cv::Mat pixels;
    cv::multiply(phaseMap, cv::Scalar(res / (2.0 * CV_PI * m_projFreq)), pixels);

    return pixels;
}
// Helper functions implementation

cv::Mat StructuredLightDecoder::_compute_threshold(const std::vector<cv::Mat>& ps_imgs)
{
    cv::Mat i_th_f(ps_imgs[0].size(), CV_32F, cv::Scalar(0));
    for (const auto& img : ps_imgs)
    {
        cv::Mat f_img;
        img.convertTo(f_img, CV_32F);
        i_th_f += f_img;
    }
    i_th_f /= (float)m_nPhaseShift;

    cv::Mat i_th_8u;
    i_th_f.convertTo(i_th_8u, CV_8U); // rint() is implicit in conversion
    return i_th_8u;
}

std::vector<cv::Mat> StructuredLightDecoder::_binarize_gc(
    const std::vector<cv::Mat>& gc_imgs, const cv::Mat& threshold)
{
    std::vector<cv::Mat> binarized_imgs;
    for (const auto& img_raw : gc_imgs)
    {
        cv::Mat bin_img;
        // bin_img[img_raw > threshold] = 1
        cv::compare(img_raw, threshold, bin_img, cv::CMP_GT);
        bin_img = bin_img / 255; // Convert from 0/255 to 0/1
        binarized_imgs.push_back(bin_img);
    }
    return binarized_imgs;
}

cv::Mat StructuredLightDecoder::_compute_wrapped_phase(const std::vector<cv::Mat>& ps_imgs)
{
    cv::Mat i0, i1, i2, i3;
    ps_imgs[0].convertTo(i0, CV_32F);
    ps_imgs[1].convertTo(i1, CV_32F);
    ps_imgs[2].convertTo(i2, CV_32F);
    ps_imgs[3].convertTo(i3, CV_32F);

    cv::Mat numerator = i3 - i1;
    cv::Mat denominator = i0 - i2;

    cv::Mat pha_degrees(numerator.size(), CV_32F);

    // Calculate the output using cv::fastAtan2(y, x)
    for (int r = 0; r < pha_degrees.rows; ++r)
    {
        for (int c = 0; c < pha_degrees.cols; ++c)
        {
            pha_degrees.at<float>(r, c) = cv::fastAtan2(
                numerator.at<float>(r, c),   // param y
                denominator.at<float>(r, c)  // param x
            );
        }
    }

    // Convert degrees (0-360) to radians (0-2*pi)
    cv::Mat pha_radians;
    cv::multiply(pha_degrees, (float)(CV_PI / 180.0), pha_radians);

    return pha_radians;
}

bool StructuredLightDecoder::_get_k1_k2(const std::vector<cv::Mat>& binarized_gc_imgs,
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

            // Build code1_int (N_GC-1 bits)
            for (int i = 0; i < m_nGrayCode - 1; ++i)
            {
                // Get (0 or 1)
                int bit = binarized_gc_imgs[i].at<uchar>(r, c);
                // i=0, bit << (3-0) -> MSB
                // i=1, bit << (3-1)
                // i=2, bit << (3-2)
                // i=3, bit << (3-3) -> LSB
                code1_int |= (bit << ((m_nGrayCode - 2) - i));
            }

            // Build code2_int (N_GC bits)
            int last_bit = binarized_gc_imgs[m_nGrayCode - 1].at<uchar>(r, c);
            // code1_int shifted left by 1 bit, then last bit is added
            code2_int = (code1_int << 1) | last_bit;

            // Use int mapping to find k values
            std::map<int, int>::iterator it1 = m_g_k1.m_code2k_int.find(code1_int);
            std::map<int, int>::iterator it2 = m_g_k2.m_code2k_int.find(code2_int);

            // Check if decoding was successful
            if (it1 == m_g_k1.m_code2k_int.end() || it2 == m_g_k2.m_code2k_int.end())
            {
                // Error: code not found. Binarization likely failed.
                k1.at<uchar>(r, c) = 0;
                k2.at<uchar>(r, c) = 0;
                return false; // Fail fast
            }

            // Get k values
            int k1_val = it1->second;
            int k2_val_raw = it2->second;

            k1.at<uchar>(r, c) = static_cast<uchar>(k1_val);
            k2.at<uchar>(r, c) = static_cast<uchar>(floor((k2_val_raw + 1) / 2.0));
        }
    }
    return true;
}

cv::Mat StructuredLightDecoder::_compute_unwrapped_phase(const cv::Mat& wrapped_pha,
    const cv::Mat& k1, const cv::Mat& k2)
{
    cv::Mat unwrapped_pha = cv::Mat::zeros(wrapped_pha.size(), CV_32F);

    cv::Mat k1_f, k2_f;
    k1.convertTo(k1_f, CV_32F);
    k2.convertTo(k2_f, CV_32F);

    const float pi_half = (float)(CV_PI / 2.0);
    const float pi_3_half = (float)(3.0 * CV_PI / 2.0);
    const float two_pi = (float)(2.0 * CV_PI);
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
            else // pha >= pi_3_half
            {
                unwrapped_pha.at<float>(r, c) = pha + (k2_val - 1) * two_pi;
            }
        }
    }
    return unwrapped_pha;
}

// Decode without dark image

cv::Mat StructuredLightDecoder::decode_without_dark_img(const std::vector<cv::Mat>& gc_imgs_raw,
    const std::vector<cv::Mat>& ps_imgs_raw)
{
    std::vector<cv::Mat> ps_imgs, gc_imgs;

    // Convert images to 8-bit
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

    // Compute threshold
    cv::Mat threshold = _compute_threshold(ps_imgs);

    // Binarize GrayCode
    std::vector<cv::Mat> binarized_gc = _binarize_gc(gc_imgs, threshold);

    // Get K1, K2
    cv::Mat k1_cpu, k2_cpu;
    if (!_get_k1_k2(binarized_gc, k1_cpu, k2_cpu))
    {
        std::cerr << "Decoding error (No Dark): Failed to find GrayCode in dictionary." << std::endl;
        return cv::Mat();
    }

    // Compute wrapped phase
    cv::Mat wrapped_phase = _compute_wrapped_phase(ps_imgs);

    // Unwrap phase
    cv::Mat unwrapped_phase = _compute_unwrapped_phase(wrapped_phase, k1_cpu, k2_cpu);

    return unwrapped_phase;
}

// 静态函数实现



cv::Mat StructuredLightDecoder::phaseToPixels(const cv::Mat& phaseMap, int projResolution, double projFreq)
{
    cv::Mat pixelMap = cv::Mat::zeros(phaseMap.size(), CV_32F);

    // 将相位转换为像素坐标
    for (int y = 0; y < phaseMap.rows; ++y) {
        for (int x = 0; x < phaseMap.cols; ++x) {
            float phase = phaseMap.at<float>(y, x);
            // 转换为像素坐标
            pixelMap.at<float>(y, x) = (phase * projResolution) / (2 * CV_PI * projFreq);
        }
    }

    return pixelMap;
}



cv::Mat StructuredLightDecoder::createModulationMask(const std::vector<cv::Mat>& phaseImages,
                                                     const cv::Mat& darkImage,
                                                     double threshold)
{
    if (phaseImages.empty()) {
        return cv::Mat();
    }

    // 计算调制强度掩膜
    cv::Mat modulation = cv::Mat::zeros(phaseImages[0].size(), CV_32F);

    for (int y = 0; y < phaseImages[0].rows; ++y) {
        for (int x = 0; x < phaseImages[0].cols; ++x) {
            std::vector<double> intensities(phaseImages.size());
            for (size_t i = 0; i < phaseImages.size(); ++i) {
                intensities[i] = static_cast<double>(phaseImages[i].at<uchar>(y, x));
            }

            // 计算调制强度
            double maxVal = *std::max_element(intensities.begin(), intensities.end());
            double minVal = *std::min_element(intensities.begin(), intensities.end());

            double modVal = (maxVal - minVal) / (maxVal + minVal + 1e-10); // 避免除零
            modulation.at<float>(y, x) = static_cast<float>(modVal);
        }
    }

    // 应用阈值创建掩膜
   cv::Mat mask;
    cv::threshold(modulation, mask, threshold, 255, cv::THRESH_BINARY);
    mask.convertTo(mask, CV_8UC1);

    return mask;
}

// Implementation of getSubpixelValues function based on CalibrationTools.cpp
std::vector<cv::Point2f> StructuredLightDecoder::getSubpixelValues(const cv::Mat& map_u,
    const cv::Mat& map_v,
    const std::vector<cv::Point2f>& corners_cam)
{
    if (corners_cam.empty())
    {
        return std::vector<cv::Point2f>();
    }

    int n_corners = static_cast<int>(corners_cam.size());

    // Create 1xN maps of the x and y coordinates to sample
    cv::Mat map_x(1, n_corners, CV_32F);
    cv::Mat map_y(1, n_corners, CV_32F);
    for (int i = 0; i < n_corners; ++i)
    {
        map_x.at<float>(0, i) = corners_cam[i].x;
        map_y.at<float>(0, i) = corners_cam[i].y;
    }

    // Use cv::remap for bilinear interpolation
    cv::Mat u_p_mat, v_p_mat;
    cv::remap(map_u, u_p_mat, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::remap(map_v, v_p_mat, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));

    // Convert the 1xN result mats back to a vector
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
