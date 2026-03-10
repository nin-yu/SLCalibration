#include "dailyqawindow.h"
#include "ui_dailyqawindow.h"
#include <QMessageBox>
#include <QDateTime>

DailyQAWindow::DailyQAWindow(const DeviceConfig& config, QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::DailyQAWindow)
    , m_deviceConfig(config)
{
    ui->setupUi(this);

    // 连接按钮信号
    connect(ui->pushButton_StartQA, &QPushButton::clicked,
            this, &DailyQAWindow::onStartQAClicked);

    // 显示设备信息
    updateDeviceInfo();

    // 初始日志
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    ui->textEdit_QALog->append(QString("[%1] 每日QA检测窗口已启动").arg(timestamp));
}

DailyQAWindow::~DailyQAWindow()
{
    delete ui;
}

void DailyQAWindow::updateDeviceInfo()
{
    QString info;
    info += QString("左侧相机: %1\n").arg(m_deviceConfig.leftCameraSN.isEmpty() ? "未设置" : m_deviceConfig.leftCameraSN);
    info += QString("右侧相机: %1\n").arg(m_deviceConfig.rightCameraSN.isEmpty() ? "未设置" : m_deviceConfig.rightCameraSN);
    info += QString("左侧投影仪: %1\n").arg(m_deviceConfig.leftProjectorTag);
    info += QString("右侧投影仪: %1").arg(m_deviceConfig.rightProjectorTag);
    
    ui->label_DeviceInfo->setText(info);
}

void DailyQAWindow::onStartQAClicked()
{
    QMessageBox::information(this, tr("提示"), 
                            tr("每日QA检测功能正在开发中，敬请期待。"));
    
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    ui->textEdit_QALog->append(QString("[%1] QA检测功能尚未实现").arg(timestamp));
}
