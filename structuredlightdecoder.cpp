#include "structuredlightdecoder.h"
#include "structuredlightdecoderRecon.h"
#include <cmath>
#include <iostream>

StructuredLightDecoder::StructuredLightDecoder(int nGrayCode, int nPhaseShift, int projFreq, cv::Size projSize, const std::string& direction)
    : QObject(nullptr)
    , m_nGrayCode(nGrayCode)
    , m_nPhaseShift(nPhaseShift)
    , m_projFreq(projFreq)
    , m_projSize(projSize)
    , m_direction(direction)
    , m_g_k1(nGrayCode - 1)
    , m_g_k2(nGrayCode)
{
}

cv::Mat StructuredLightDecoder::decode(const std::vector<cv::Mat>& grayCodeImages,
    const std::vector<cv::Mat>& phaseShiftImages,
    const cv::Mat& darkImage)
{
    // forward to Recon implementation
    StructuredLightDecoderRecon recon(m_nGrayCode, m_nPhaseShift, m_projFreq, m_projSize, m_direction);
    return recon.decode(grayCodeImages, phaseShiftImages, darkImage);
}

cv::Mat StructuredLightDecoder::phaseToPixels(const cv::Mat& phaseMap)
{
    // same formula as recon
    float res = (m_direction == "vertical") ? (float)m_projSize.width : (float)m_projSize.height;
    cv::Mat pixels;
    cv::multiply(phaseMap, cv::Scalar(res / (2.0 * CV_PI * m_projFreq)), pixels);
    return pixels;
}

std::vector<cv::Point2f> StructuredLightDecoder::getSubpixelValues(const cv::Mat& mapU, const cv::Mat& mapV, const std::vector<cv::Point2f>& points)
{
    return get_subpixel_values(mapU, mapV, points);
}

cv::Mat StructuredLightDecoder::createModulationMask(const std::vector<cv::Mat>& phaseImages,
    const cv::Mat& darkImage,
    double threshold)
{
    if (phaseImages.empty()) return cv::Mat();

    cv::Mat meanImg = cv::Mat::zeros(phaseImages[0].size(), CV_32F);
    for (const auto& img : phaseImages) {
        cv::Mat f; img.convertTo(f, CV_32F);
        meanImg += f;
    }
    meanImg /= static_cast<float>(phaseImages.size());

    cv::Mat darkF; darkImage.convertTo(darkF, CV_32F);
    cv::Mat mod = meanImg - darkF;

    cv::Mat mask;
    cv::threshold(mod, mask, threshold, 255, cv::THRESH_BINARY);
    mask.convertTo(mask, CV_8U);
    return mask;
}

// Minimal stubs for other static helpers (optional)
cv::Mat StructuredLightDecoder::decodeGrayCode(const std::vector<cv::Mat>& grayCodeImages, int nBits)
{
    // Simple placeholder: return empty
    return cv::Mat();
}

cv::Mat StructuredLightDecoder::decodePhaseShift(const std::vector<cv::Mat>& phaseImages, int nSteps, double initialPhase)
{
    return cv::Mat();
}

cv::Mat StructuredLightDecoder::combineGrayCodeAndPhase(const cv::Mat& grayCodePhase, const cv::Mat& phase)
{
    return cv::Mat();
}

cv::Mat StructuredLightDecoder::phaseToPixels(const cv::Mat& phaseMap, int projResolution, double projFreq)
{
    float res = static_cast<float>(projResolution);
    cv::Mat pixels;
    cv::multiply(phaseMap, cv::Scalar(res / (2.0 * CV_PI * projFreq)), pixels);
    return pixels;
}

cv::Mat StructuredLightDecoder::decode_without_dark_img(const std::vector<cv::Mat>& gc_imgs_raw,
    const std::vector<cv::Mat>& ps_imgs_raw)
{
    StructuredLightDecoderRecon recon(m_nGrayCode, m_nPhaseShift, m_projFreq, m_projSize, m_direction);
    return recon.decode_without_dark_img(gc_imgs_raw, ps_imgs_raw);
}
