#ifndef RECONTYPES_H
#define RECONTYPES_H

#include <QVector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include "configmanager.h"

// 点云类型定义
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// 图像批次类型
using ReconImageBatch = QVector<cv::Mat>;

// 标定数据结构
struct CalibData
{
    cv::Mat camMatrix;          // 相机内参矩阵
    cv::Mat camDist;            // 相机畸变系数
    cv::Mat projMatrix;         // 投影仪内参矩阵
    cv::Mat projDist;           // 投影仪畸变系数
    cv::Mat R_CamToProj;        // 相机到投影仪的旋转矩阵
    cv::Mat T_CamToProj;        // 相机到投影仪的平移向量
    cv::Size camRes;            // 相机分辨率
    cv::Size projRes;           // 投影仪分辨率
    int proj_frequency;         // 投影仪频率

    // 预计算的去畸变映射
    cv::Mat camMatrixNew;       // 新相机内参（去畸变后）
    cv::Rect roi;               // 有效区域
    cv::Mat mapx, mapy;         // 去畸变映射

    // 预计算的重建参数
    float fx_cam, fy_cam, cx_cam, cy_cam;
    float fx_proj, cx_proj;
    float tx, tz;

    // 预计算的雅可比矩阵基
    cv::Mat J_x_base, J_z_base;
    cv::Mat x_c_norm, y_c_norm;

    // 标定板坐标系到相机坐标系的变换
    cv::Mat R_BoardToCam;       // 旋转矩阵
    cv::Mat T_BoardToCam;       // 平移向量
    cv::Mat R_BoardToCam_32f;   // CV_32F版本的R
    cv::Mat T_BoardToCam_32f;   // CV_32F版本的T

    CalibData() : proj_frequency(0) {}
};

// 重建参数结构
struct ReconParams
{
    double modulationThreshold;     // 调制阈值
    float minDepth;                 // 最小深度
    float maxDepth;                 // 最大深度
    int nGrayCode;                  // 格雷码图像数量
    int nPhaseShift;                // 相移图像数量
    int projFrequency;              // 投影仪频率

    // 默认构造函数（作为安全回退）
    ReconParams()
        : modulationThreshold(70.0)
        , minDepth(200.0f)
        , maxDepth(2000.0f)
        , nGrayCode(5)
        , nPhaseShift(4)
        , projFrequency(16)
    {}

    // 从 ConfigManager 创建 ReconParams
    static ReconParams fromConfig() {
        auto& cfg = ConfigManager::instance();
        ReconParams params;
        params.modulationThreshold = cfg.modulationThreshold();
        params.minDepth = static_cast<float>(cfg.minDepth());
        params.maxDepth = static_cast<float>(cfg.maxDepth());
        params.nGrayCode = cfg.grayCodeBits();
        params.nPhaseShift = cfg.phaseShiftSteps();
        params.projFrequency = cfg.projectorFrequency();
        return params;
    }
};

#endif // RECONTYPES_H
