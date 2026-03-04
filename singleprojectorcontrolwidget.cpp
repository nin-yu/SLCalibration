#include "singleprojectorcontrolwidget.h"
#include "ui_singleprojectorcontrolwidget.h"  // Generated from .ui file
#include <QMessageBox>
#include <QDebug>
#include <QApplication>
#include <QMetaObject>
#include <QThread>
#include <opencv2/opencv.hpp>  // 添加OpenCV支持

// 构造函数
SingleProjectorControlWidget::SingleProjectorControlWidget(ProjectorController* controller, QString tagName, QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::SingleProjectorControlWidget)
    , m_exposureTime(41000)  // 初始化曝光时间为41000微秒
    , m_triggerMode(0)       // 初始化触发模式为0 内触发
    , m_patternIndex(0)      // 初始化投影图案索引为0
    , m_controller(controller)  // 初始化DLL控制器实例
    , m_tagName(tagName)  // 通过index获取投影仪序列号作为tag
{
    ui->setupUi(this);

    // 初始化控件指针 - 直接从UI类获取
    pushButton_ProjectorOpen = ui->pushButton_ProjectorOpen;
    pushButton_ProjectorPlay = ui->pushButton_ProjectorPlay;
    pushButton_ProjectorPause = ui->pushButton_ProjectorPause;
    lineEdit_ProjectorExposureTime = ui->lineEdit_ProjectorExposureTime;
    lineEdit_ProjectorBrightness = ui->lineEdit_ProjectorBrightness;
    comboBox_ProjectorMode = ui->comboBox_ProjectorMode;
    comboBox_ProjectorPattern = ui->comboBox_ProjectorPattern;
    comboBox_ProjectorTrigger = ui->comboBox_ProjectorTrigger;
    
    // 状态标签 - 使用UI中的label_3控件（根据UI文件定义）
    m_labelStatus = findChild<QLabel*>("label_Status");
    if (!m_labelStatus) {
        m_labelStatus = ui->label_3;  // 根据UI文件，label_3是"连接模式"的标签
        if (!m_labelStatus) {
            m_labelStatus = ui->label;  // 或者使用label
        }
    }

    bool ok = m_controller->initProjector(m_tagName.toStdString(), 7);
    if (ok) {
        emit statusUpdated(QString("投影仪 %1 初始化成功").arg(m_tagName));
    } else {
        emit statusUpdated(QString("投影仪 %1 初始化失败").arg(m_tagName));
    }
    // 初始化UI控件状态
    updateUIControls();
}

// 析构函数
SingleProjectorControlWidget::~SingleProjectorControlWidget()
{
    // 在析构函数中统一关闭投影仪
    if (m_controller) {
        m_controller->closeProjector(m_tagName.toStdString());
    }
    delete ui;
}

// 设置当前控制的投影仪ID
void SingleProjectorControlWidget::setProjectorTag(const QString& projectId)
{
    if (projectId.isEmpty()) return;
    m_tagName = projectId;
    updateUIControls();
}

// 获取当前投影仪tag
QString SingleProjectorControlWidget::getProjectorTag() const
{
    return m_tagName;
}

// 更新UI控件状态
void SingleProjectorControlWidget::updateUIControls()
{
    // 使用当前管理的控制器实例
    if (!m_controller) {
        if (pushButton_ProjectorOpen) pushButton_ProjectorOpen->setEnabled(false);
        if (pushButton_ProjectorPlay) pushButton_ProjectorPlay->setEnabled(false);
        if (pushButton_ProjectorPause) pushButton_ProjectorPause->setEnabled(false);

        if (lineEdit_ProjectorExposureTime) lineEdit_ProjectorExposureTime->setEnabled(false);
        if (lineEdit_ProjectorBrightness) lineEdit_ProjectorBrightness->setEnabled(false);
        if (comboBox_ProjectorMode) comboBox_ProjectorMode->setEnabled(false);
        if (comboBox_ProjectorPattern) comboBox_ProjectorPattern->setEnabled(false);
        if (comboBox_ProjectorTrigger) comboBox_ProjectorTrigger->setEnabled(false);

        if (m_labelStatus) m_labelStatus->setText(tr("投影仪不存在 - ") + m_tagName);
        emit statusUpdated("投影仪不存在");
        return;
    }

    // 获取当前投影仪状态（这里需要通过Projector4500的一些方法来判断）
    // 检查连接状态作为是否打开的近似
    bool isConnected = m_controller->checkConnection(m_tagName.toStdString());
    bool isProjectorOpen = isConnected; // 简化处理
    if (pushButton_ProjectorOpen) pushButton_ProjectorOpen->setEnabled(!isProjectorOpen);
    if (pushButton_ProjectorPlay) pushButton_ProjectorPlay->setEnabled(isProjectorOpen);
    if (pushButton_ProjectorPause) pushButton_ProjectorPause->setEnabled(isProjectorOpen);

    if (lineEdit_ProjectorExposureTime) lineEdit_ProjectorExposureTime->setEnabled(isProjectorOpen);
    if (lineEdit_ProjectorBrightness) lineEdit_ProjectorBrightness->setEnabled(isProjectorOpen);
    if (comboBox_ProjectorMode) comboBox_ProjectorMode->setEnabled(isProjectorOpen);
    if (comboBox_ProjectorPattern) comboBox_ProjectorPattern->setEnabled(isProjectorOpen);
    if (comboBox_ProjectorTrigger) comboBox_ProjectorTrigger->setEnabled(isProjectorOpen);

    if (isProjectorOpen) {
        // 从投影仪获取当前配置并更新UI（仅在投影仪连接时）
        

        if (lineEdit_ProjectorExposureTime) {
            lineEdit_ProjectorExposureTime->setText(QString::number(m_exposureTime));
        }
        if (lineEdit_ProjectorBrightness) {
            lineEdit_ProjectorBrightness->setText(QString::number(100));
        }
        if (comboBox_ProjectorMode) {
            comboBox_ProjectorMode->setCurrentIndex(0); // 默认USB模式
        }
        if (comboBox_ProjectorTrigger) {
            comboBox_ProjectorTrigger->setCurrentIndex(m_triggerMode);
        }
    } else {
        // 当投影仪未连接时，清空参数显示
        if (lineEdit_ProjectorExposureTime) {
            lineEdit_ProjectorExposureTime->setText("N/A");
        }
        if (lineEdit_ProjectorBrightness) {
            lineEdit_ProjectorBrightness->setText("N/A");
        }
        if (comboBox_ProjectorMode) {
            comboBox_ProjectorMode->setCurrentIndex(0); // 默认USB模式
        }
        if (comboBox_ProjectorTrigger) {
            comboBox_ProjectorTrigger->setCurrentIndex(0); // 默认触发模式
        }
    }

    // 更新状态标签
    QString statusText = QString("投影仪 - ");
    statusText += isProjectorOpen ? "已连接" : "未连接";

    // 在状态中包含tagName
    if (!m_tagName.isEmpty()) {
        statusText += " (" + m_tagName + ")";
    }

    if (m_labelStatus) m_labelStatus->setText(statusText);
}

// 打开投影仪按钮点击事件
void SingleProjectorControlWidget::on_pushButton_ProjectorOpen_clicked() //TODO 需要重写跟检测函数按钮有冲突
{
    if (!m_controller) {
        emit statusUpdated("控制器未初始化");
        return;
    }

    ProjectorController* controller = m_controller;
    if (!controller) {
        emit statusUpdated("投影仪不存在");
        return;
    }

    // 使用m_tagName初始化投影仪, 使用默认亮度设置
    if (m_tagName.isEmpty()) {
        emit statusUpdated("未指定投影仪Tag");
        return;
    }

    bool ok = m_controller->initProjector(m_tagName.toStdString(), 7);
    if (ok) {
        emit statusUpdated(QString("投影仪 %1 初始化成功").arg(m_tagName));
    } else {
        emit statusUpdated(QString("投影仪 %1 初始化失败").arg(m_tagName));
    }

    updateUIControls();
}

// 投影图案按钮点击事件
void SingleProjectorControlWidget::on_pushButton_ProjectorPlay_clicked()
{
    if (!m_controller) {
        emit statusUpdated("控制器未初始化");
        return;
    }

    ProjectorController* controller = m_controller;
    if (!controller) {
        emit statusUpdated("投影仪不存在");
        return;
    }

    // 使用内部成员变量中的参数值
    int exposureTime = m_exposureTime;
    int patternIndex = m_patternIndex;
    int triggerMode = m_triggerMode;

    // 使用ProjectorController的DLL函数开始投影
    m_controller->sendAndPlayProjector(m_tagName.toStdString(), triggerMode, patternIndex, exposureTime);

    emit statusUpdated(QString("投影仪开始投影 (%1)").arg(m_tagName));
    updateUIControls();
}

// 暂停投影按钮点击事件
void SingleProjectorControlWidget::on_pushButton_ProjectorPause_clicked()
{
    if (!m_controller) {
        emit statusUpdated("控制器未初始化");
        return;
    }

    ProjectorController* controller = m_controller;
    if (!controller) {
        emit statusUpdated("投影仪不存在");
        return;
    }

    // 使用ProjectorController的DLL函数暂停投影
    m_controller->pauseProjector(m_tagName.toStdString());

    emit statusUpdated(QString("投影仪暂停投影 (%1)").arg(m_tagName));
    updateUIControls();
}

// 曝光时间文本改变事件
void SingleProjectorControlWidget::on_lineEdit_ProjectorExposureTime_textChanged(const QString& text)
{
    // 更新曝光时间成员变量
    bool ok;
    int newExposureTime = text.toInt(&ok);
    if (ok && newExposureTime >= 0) {
        m_exposureTime = newExposureTime;
    }
}

// LED亮度文本改变事件
void SingleProjectorControlWidget::on_lineEdit_ProjectorBrightness_textChanged(const QString& text)
{
    // 更新LED亮度成员变量
    bool ok;
    int newBrightness = text.toInt(&ok);
    if (ok && newBrightness >= 0) {
        m_ledBrightness = newBrightness;
    }
}

// 连接模式下拉框改变事件
void SingleProjectorControlWidget::on_comboBox_ProjectorMode_currentIndexChanged(int index)
{
    Q_UNUSED(index);

}

// 投影图案下拉框改变事件
void SingleProjectorControlWidget::on_comboBox_ProjectorPattern_currentIndexChanged(int index)
{
    // 更新投影图案索引成员变量
    m_patternIndex = index;
    // 图案改变时的处理
}

// 触发模式下拉框改变事件
void SingleProjectorControlWidget::on_comboBox_ProjectorTrigger_currentIndexChanged(int index)
{
    // 更新触发模式成员变量
    m_triggerMode = index;
    // 触发模式改变时的处理
}

