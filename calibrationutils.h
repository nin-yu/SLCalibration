#ifndef CALIBRATIONUTILS_H
#define CALIBRATIONUTILS_H

#include <QString>
#include <QDir>
#include <QFileInfo>
#include <QFileInfoList>
#include <QRegularExpression>
#include <opencv2/opencv.hpp>

/**
 * @brief 标定工具类
 * 
 * 提供标定相关的通用工具函数，包括路径管理、目录操作、文件保存等
 */
class CalibrationUtils
{
public:
    // ==================== 路径管理 ====================
    
    /**
     * @brief 获取基础标定数据路径
     * @return 基础标定路径 (应用程序目录/CalibrateData)
     */
    static QString getBaseCalibrationPath();
    
    /**
     * @brief 获取设备路径
     * @param isLeftDevice 是否为左侧设备
     * @return 设备路径 (LeftSL 或 RightSL)
     */
    static QString getDevicePath(bool isLeftDevice);
    
    /**
     * @brief 获取标定类型路径
     * @param isLeftDevice 是否为左侧设备
     * @param calibType 标定类型 ("Camera", "Projector", "Coordinate")
     * @return 标定类型路径
     */
    static QString getCalibrationTypePath(bool isLeftDevice, const QString& calibType);
    
    /**
     * @brief 确保目录存在，不存在则创建
     * @param path 目录路径
     * @return 是否成功
     */
    static bool createDirectoryIfNotExists(const QString& path);
    
    // ==================== 投影仪标定辅助 ====================
    
    /**
     * @brief 获取下一个可用的 Pose 编号
     * @param projectorCalibPath 投影仪标定路径
     * @return 下一个 Pose 编号（从1开始，若目录中存在文件则续接编号）
     */
    static int getNextPoseNumber(const QString& projectorCalibPath);
    
    /**
     * @brief 格式化 Pose 编号为两位字符串
     * @param poseNumber Pose 编号
     * @return 格式化后的字符串 (如 "01", "12")
     */
    static QString formatPoseNumber(int poseNumber);
    
    // ==================== 图像保存 ====================
    
    /**
     * @brief 保存标定图像
     * @param image 图像数据
     * @param calibType 标定类型
     * @param imageIndex 图像索引
     * @param isLeftDevice 是否为左侧设备
     * @return 是否保存成功
     */
    static bool saveCalibrationImage(const cv::Mat& image, 
                                      const QString& calibType, 
                                      int imageIndex,
                                      bool isLeftDevice);
    
    /**
     * @brief 保存角点检测结果图像
     * @param imagePath 原始图像路径
     * @param image 原始图像数据
     * @param corners 检测到的角点
     * @param detectionSuccess 检测是否成功
     * @param patternCols 棋盘格列数
     * @param patternRows 棋盘格行数
     * @return 是否保存成功
     */
    static bool saveCornerDetectionResult(const QString& imagePath,
                                          const cv::Mat& image,
                                          const std::vector<cv::Point2f>& corners,
                                          bool detectionSuccess,
                                          int patternCols,
                                          int patternRows);
    
    /**
     * @brief 保存坐标系标定图像
     * @param image 图像数据
     * @param isLeftDevice 是否为左侧设备
     * @param savedImagePath 保存后的完整路径输出
     * @return 是否保存成功
     */
    static bool saveCoordinateImage(const cv::Mat& image, 
                                    bool isLeftDevice, 
                                    QString& savedImagePath);
    
    // ==================== 目录清理 ====================
    
    /**
     * @brief 清空坐标系标定目录
     * @param isLeftDevice 是否为左侧设备
     * @return 是否成功
     */
    static bool clearCoordinateDirectory(bool isLeftDevice);
    
    /**
     * @brief 清空指定类型的标定目录
     * @param isLeftDevice 是否为左侧设备
     * @param calibType 标定类型
     * @return 删除的文件数量
     */
    static int clearCalibrationDirectory(bool isLeftDevice, const QString& calibType);
    
    // ==================== 文件查找 ====================
    
    /**
     * @brief 获取标定目录中的图像文件列表
     * @param isLeftDevice 是否为左侧设备
     * @param calibType 标定类型
     * @return 图像文件信息列表
     */
    static QFileInfoList getCalibrationImages(bool isLeftDevice, const QString& calibType);
    
private:
    CalibrationUtils() = delete;  // 禁止实例化
    
    // 图像文件过滤器
    static QStringList getImageFilters();
};

#endif // CALIBRATIONUTILS_H
