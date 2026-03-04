#ifndef POINTCLOUDWIDGET_H
#define POINTCLOUDWIDGET_H

#include <QWidget>
#include <QVTKOpenGLStereoWidget.h>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkAutoInit.h>

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

// 定义点类型
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

/**
 * @class PointCloudWidget
 * @brief 点云可视化组件 - 简化版
 * 
 * 仅提供点云显示和基本视图交互功能（放大、缩小、旋转、平移等）
 */
class PointCloudWidget : public QVTKOpenGLStereoWidget
{
    Q_OBJECT
public:
    /**
     * @brief 构造函数
     */
    explicit PointCloudWidget(QWidget* parent = nullptr);

    /**
     * @brief 析构函数
     */
    ~PointCloudWidget();

    /**
     * @brief 更新点云数据
     * @param cloud3D 新的点云数据
     */
    void updatePointCloud(PointCloudT::Ptr cloud3D);

    /**
     * @brief 获取可视化器
     */
    pcl::visualization::PCLVisualizer::Ptr getVisualizer() { return m_visualizer; }

private:
    /**
     * @brief 初始化窗口部件
     */
    void initializeWidget();

    // 可视化器和点云数据
    pcl::visualization::PCLVisualizer::Ptr m_visualizer;  // 可视化器
    PointCloudT::Ptr m_pointCloud;                        // 当前显示的点云
};

#endif // POINTCLOUDWIDGET_H