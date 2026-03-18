#ifndef STRUCTUREDLIGHTDECODER_H
#define STRUCTUREDLIGHTDECODER_H

#include <QObject>
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <string>

#include "graycode.h" // use centralized GrayCode

class StructuredLightDecoder : public QObject
{
    Q_OBJECT

public:
    StructuredLightDecoder(int nGrayCode, int nPhaseShift, int projFreq, cv::Size projSize, const std::string& direction);
    
    // 实例方法
    cv::Mat decode(const std::vector<cv::Mat>& grayCodeImages, 
                  const std::vector<cv::Mat>& phaseShiftImages, 
                  const cv::Mat& darkImage);
    
    cv::Mat phaseToPixels(const cv::Mat& phaseMap);
    
    // 静态辅助方法
    static cv::Mat decodeGrayCode(const std::vector<cv::Mat>& grayCodeImages, int nBits);
    static cv::Mat decodePhaseShift(const std::vector<cv::Mat>& phaseImages, int nSteps, double initialPhase = 0.0);
    static cv::Mat combineGrayCodeAndPhase(const cv::Mat& grayCodePhase, const cv::Mat& phase);
    static cv::Mat phaseToPixels(const cv::Mat& phaseMap, int projResolution, double projFreq);
    
    // 获取亚像素值
    static std::vector<cv::Point2f> getSubpixelValues(const cv::Mat& mapU, const cv::Mat& mapV, const std::vector<cv::Point2f>& points);
    
    // 创建调制掩膜
    static cv::Mat createModulationMask(const std::vector<cv::Mat>& phaseImages, 
                                const cv::Mat& darkImage, 
                                double threshold);
    
    // New methods based on CalibrationTools.cpp
    cv::Mat decode_without_dark_img(const std::vector<cv::Mat>& gc_imgs_raw,
        const std::vector<cv::Mat>& ps_imgs_raw);

private:
    int m_nGrayCode;
    int m_nPhaseShift;
    int m_projFreq;
    cv::Size m_projSize;
    std::string m_direction; // "horizontal" or "vertical"
    
    // GrayCode helpers
    GrayCode m_g_k1;
    GrayCode m_g_k2;
    
    // Helper functions based on CalibrationTools.cpp
    cv::Mat _compute_threshold(const std::vector<cv::Mat>& ps_imgs);
    std::vector<cv::Mat> _binarize_gc(const std::vector<cv::Mat>& gc_imgs, const cv::Mat& threshold);
    cv::Mat _compute_wrapped_phase(const std::vector<cv::Mat>& ps_imgs);
    bool _get_k1_k2(const std::vector<cv::Mat>& binarized_gc_imgs, cv::Mat& k1, cv::Mat& k2);
    cv::Mat _compute_unwrapped_phase(const cv::Mat& wrapped_pha, const cv::Mat& k1, const cv::Mat& k2);
};

#endif // STRUCTUREDLIGHTDECODER_H
