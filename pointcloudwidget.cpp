#include "pointcloudwidget.h"
#include <QDebug>

/**
 * @brief PointCloudWidget构造函数
 * @param parent 父窗口部件
 */
PointCloudWidget::PointCloudWidget(QWidget* parent)
    : QVTKOpenGLStereoWidget{parent}
    , m_pointCloud(new PointCloudT)
    , m_visualizer(new pcl::visualization::PCLVisualizer("viewer", false))
{
    initializeWidget();
    qDebug() << "[PointCloudWidget] 构造函数 - 初始化完成";
}

/**
 * @brief PointCloudWidget析构函数
 */
PointCloudWidget::~PointCloudWidget()
{
    m_visualizer.reset();
    m_pointCloud.reset();
    qDebug() << "[PointCloudWidget] 析构函数 - 资源释放完成";
}

/**
 * @brief 初始化窗口部件
 * 配置VTK渲染环境，设置相机位置和交互方式
 * 
 * 基本视图交互说明：
 * - 鼠标左键拖拽：旋转视图
 * - 鼠标中键拖拽：平移视图
 * - 鼠标滚轮：放大/缩小
 * - 鼠标右键拖拽：缩放视图
 */
void PointCloudWidget::initializeWidget()
{
    QVTKInteractor::SetGlobalWarningDisplay(0);

    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    m_visualizer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "base", false));

    // 设置交互器
    this->setRenderWindow(m_visualizer->getRenderWindow());
    m_visualizer->setupInteractor(this->interactor(), this->renderWindow());
    this->setMouseTracking(true);

    // 设置渲染属性
    m_visualizer->getRenderWindow()->SetDoubleBuffer(0);
    m_visualizer->addCoordinateSystem(50, "camera", 0);
    
    // 相机位置设置
    m_visualizer->setCameraPosition(7000, -5000, 819, 0, 0, 1);
    qDebug() << "[PointCloudWidget] 设置相机位置: (7000, -5000, 819), 目标点: (0, 0, 1)";
    
    // 相机裁剪距离设置
    m_visualizer->setCameraClipDistances(500, 12000);
    qDebug() << "[PointCloudWidget] 设置相机裁剪距离: 500 - 12000";
}

/**
 * @brief 刷新实时重建的点云
 * @param cloud3D 新的点云数据
 */
void PointCloudWidget::updatePointCloud(PointCloudT::Ptr cloud3D)
{
    m_pointCloud = cloud3D;
    m_visualizer->removeAllPointClouds();

    m_visualizer->addPointCloud(cloud3D, "cloud");
    qDebug() << "[PointCloudWidget] 更新点云，点云大小:" << cloud3D->size();

    m_visualizer->getRenderWindow()->Render();
}