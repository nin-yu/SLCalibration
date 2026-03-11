#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "calibrationwindow.h"
#include "dailyqawindow.h"
#include "qareportwindow.h"
#include "qadbmanager.h"

#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QSettings>
#include <QDateTime>
#include <QDebug>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // 加载设备配置
    if (!loadConfiguration()) {
        ui->label_ConfigStatus->setText("配置状态: 已使用默认配置");
        ui->label_ConfigStatus->setStyleSheet("color: #FF9800; font-size: 12px; padding: 5px;");
    } else {
        QString statusText = QString("配置状态: 已加载 | 左相机: %1 | 右相机: %2")
                            .arg(m_deviceConfig.leftCameraSN.isEmpty() ? "未设置" : m_deviceConfig.leftCameraSN)
                            .arg(m_deviceConfig.rightCameraSN.isEmpty() ? "未设置" : m_deviceConfig.rightCameraSN);
        ui->label_ConfigStatus->setText(statusText);
        ui->label_ConfigStatus->setStyleSheet("color: #4CAF50; font-size: 12px; padding: 5px;");
    }

    // 初始化SQLite数据库
    QString dbPath = QDir::currentPath() + "/qa_reports.db";
    QADbManager::instance().initialize(dbPath);

    // 连接三个导航按钮
    connect(ui->pushButton_DailyQA, &QPushButton::clicked,
            this, &MainWindow::onDailyQAClicked);
    connect(ui->pushButton_Calibration, &QPushButton::clicked,
            this, &MainWindow::onCalibrationClicked);
    connect(ui->pushButton_QAReport, &QPushButton::clicked,
            this, &MainWindow::onQAReportClicked);
}

MainWindow::~MainWindow()
{
    delete ui;
}

bool MainWindow::createDefaultConfigFile(const QString& filePath)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return false;
    }
    
    QTextStream out(&file);
    out << "[Camera]\n";
    out << "# 左侧相机序列号\n";
    out << "LeftSerialNumber=\n";
    out << "# 右侧相机序列号\n";
    out << "RightSerialNumber=\n\n";
    out << "[Projector]\n";
    out << "# 左侧投影仪tagname\n";
    out << "LeftTagName=LCR2@dibh_left\n";
    out << "# 右侧投影仪tagname\n";
    out << "RightTagName=LCR2@dibh_right\n\n";

    out << "[QA]\n";
    out << "# 日检数据目录（相对路径将基于程序目录）\n";
    out << "DailyQARoot=QAData\n";
    out << "# 日检模式：pose_batch（手动拷图）\n";
    out << "DailyMode=pose_batch\n";
    out << "# 日检棋盘格参数\n";
    out << "PatternRows=20\n";
    out << "PatternCols=20\n";
    out << "SquareSizeMm=10.0\n";
    out << "# 日检分项阈值\n";
    out << "CameraReprojFailThresholdPx=0.8\n";
    out << "DailyRmsProjFailThreshold=0.8\n";
    out << "MinValidPoseCount=1\n";
    out << "# 结构光参数（应与采集模板一致）\n";
    out << "ProjectorWidth=912\n";
    out << "ProjectorHeight=1140\n";
    out << "ProjectorFrequency=16\n";
    out << "GrayCodeBits=5\n";
    out << "PhaseShiftSteps=4\n";
    out << "# 月检失败阈值\n";
    out << "MonthlyRmsProjFailThreshold=0.8\n";
    out << "MonthlyRmsStereoFailThreshold=0.8\n";
    out << "MonthlyEpiMeanFailThresholdPx=1.0\n";
    
    file.close();
    return true;
}

bool MainWindow::loadConfiguration()
{
    QString configFile = QDir::currentPath() + "/config.ini";
    
    // 检查配置文件是否存在，如果不存在则创建默认文件
    if (!QFile::exists(configFile)) {
        if (!createDefaultConfigFile(configFile)) {
            qDebug() << "无法创建默认配置文件:" << configFile;
            return false;
        }
    }
    
    // 加载配置文件
    QSettings settings(configFile, QSettings::IniFormat);
    
    // 读取相机序列号
    m_deviceConfig.leftCameraSN = settings.value("Camera/LeftSerialNumber", "").toString();
    m_deviceConfig.rightCameraSN = settings.value("Camera/RightSerialNumber", "").toString();
    
    // 读取投影仪tagname
    m_deviceConfig.leftProjectorTag = settings.value("Projector/LeftTagName", "LCR2@dibh_left").toString();
    m_deviceConfig.rightProjectorTag = settings.value("Projector/RightTagName", "LCR2@dibh_right").toString();
    
    // 使用默认值填充空字段
    if (m_deviceConfig.leftProjectorTag.isEmpty()) {
        m_deviceConfig.leftProjectorTag = "LCR2@dibh_left";
    }
    if (m_deviceConfig.rightProjectorTag.isEmpty()) {
        m_deviceConfig.rightProjectorTag = "LCR2@dibh_right";
    }
    
    qDebug() << "[配置加载] 左相机SN:" << m_deviceConfig.leftCameraSN
             << "右相机SN:" << m_deviceConfig.rightCameraSN
             << "左投影仪Tag:" << m_deviceConfig.leftProjectorTag
             << "右投影仪Tag:" << m_deviceConfig.rightProjectorTag;
    
    return true;
}

void MainWindow::onDailyQAClicked()
{
    if (!m_dailyQAWin) {
        m_dailyQAWin = new DailyQAWindow(m_deviceConfig, this);
        m_dailyQAWin->setAttribute(Qt::WA_DeleteOnClose);
        connect(m_dailyQAWin, &QObject::destroyed, this, [this]{ m_dailyQAWin = nullptr; });
        m_dailyQAWin->show();
    } else {
        m_dailyQAWin->raise();
        m_dailyQAWin->activateWindow();
    }
}

void MainWindow::onCalibrationClicked()
{
    if (!m_calibrationWin) {
        m_calibrationWin = new CalibrationWindow(m_deviceConfig, this);
        m_calibrationWin->setAttribute(Qt::WA_DeleteOnClose);
        connect(m_calibrationWin, &QObject::destroyed, this, [this]{ m_calibrationWin = nullptr; });
        m_calibrationWin->show();
    } else {
        m_calibrationWin->raise();
        m_calibrationWin->activateWindow();
    }
}

void MainWindow::onQAReportClicked()
{
    if (!m_qaReportWin) {
        m_qaReportWin = new QAReportWindow(this);
        m_qaReportWin->setAttribute(Qt::WA_DeleteOnClose);
        connect(m_qaReportWin, &QObject::destroyed, this, [this]{ m_qaReportWin = nullptr; });
        m_qaReportWin->show();
    } else {
        m_qaReportWin->raise();
        m_qaReportWin->activateWindow();
    }
}
