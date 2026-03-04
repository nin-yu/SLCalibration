#ifndef SINGLEPROJECTORCONTROLWIDGET_H
#define SINGLEPROJECTORCONTROLWIDGET_H

#include <QWidget>
#include <QLineEdit>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QGridLayout>
#include <QGroupBox>
#include <QCheckBox>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include "ProjectorController.h"  // 添加ProjectorController类

// 前置声明UI类
namespace Ui {
    class SingleProjectorControlWidget;
}


// 单个投影仪控制组件，用于控制单个投影仪设备的参数和状态
class SingleProjectorControlWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SingleProjectorControlWidget(ProjectorController* controller, QString tagName, QWidget* parent = nullptr);
    ~SingleProjectorControlWidget();

    // 设置当前控制的投影仪ID（tag）
    void setProjectorTag(const QString& projectId);

    // 获取当前投影仪tag
    QString getProjectorTag() const;

    // 更新UI控件状态
    void updateUIControls();

    // Getter方法
    int getExposureTime() const { return m_exposureTime; }
    int getTriggerMode() const { return m_triggerMode; }
    int getPatternIndex() const { return m_patternIndex; }

    // 应用参数到投影仪
    void applyParameters();


signals:
    // 状态更新信号
    void statusUpdated(const QString& message);

    // 错误信号
    void errorOccurred(const QString& error);

private slots:
    // UI事件处理槽函数
    void on_pushButton_ProjectorOpen_clicked();
    void on_pushButton_ProjectorPlay_clicked();
    void on_pushButton_ProjectorPause_clicked();

    // 参数改变槽函数
    void on_lineEdit_ProjectorExposureTime_textChanged(const QString& text);
    void on_lineEdit_ProjectorBrightness_textChanged(const QString& text);
    void on_comboBox_ProjectorMode_currentIndexChanged(int index);
    void on_comboBox_ProjectorPattern_currentIndexChanged(int index);
    void on_comboBox_ProjectorTrigger_currentIndexChanged(int index);

private:
    // UI组件 - 通过ui指针直接访问UI文件中的控件
    QPushButton* pushButton_ProjectorOpen;
    QPushButton* pushButton_ProjectorClose;
    QPushButton* pushButton_ProjectorPlay;
    QPushButton* pushButton_ProjectorPause;
    QLineEdit* lineEdit_ProjectorExposureTime;
    QLineEdit* lineEdit_ProjectorBrightness;
    QComboBox* comboBox_ProjectorMode;
    QComboBox* comboBox_ProjectorPattern;
    QComboBox* comboBox_ProjectorTrigger;
    QLabel* m_labelStatus;

    // 投影仪相关
    Ui::SingleProjectorControlWidget* ui;

    int m_exposureTime;   // 曝光时间（单位：微秒）
    int m_ledBrightness;  // 亮度
    int m_triggerMode;    // 触发模式索引
    int m_patternIndex;   // 投影图案索引

    // ProjectorController管理类
    ProjectorController* m_controller;  // DLL控制器实例
    QString m_tagName;  // 投影仪标签

    // 创建UI控件
    void createUIControls();

    // 更新投影仪参数
    void updateProjectorParameters();

    // 保存配置到文件
    void saveConfigToFile();

    // 从文件加载配置
    bool loadConfigFromFile();
};

#endif // SINGLEPROJECTORCONTROLWIDGET_H
