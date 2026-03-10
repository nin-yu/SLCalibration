#include "qareportwindow.h"
#include "ui_qareportwindow.h"
#include "qadbmanager.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>
#include <QDateTime>
#include <QDebug>

QAReportWindow::QAReportWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::QAReportWindow)
{
    ui->setupUi(this);

    // 设置默认日期范围（最近30天）
    ui->dateEdit_To->setDate(QDate::currentDate());
    ui->dateEdit_From->setDate(QDate::currentDate().addDays(-30));

    // 连接信号
    connect(ui->pushButton_Query, &QPushButton::clicked,
            this, &QAReportWindow::onQueryClicked);
    connect(ui->pushButton_Export, &QPushButton::clicked,
            this, &QAReportWindow::onExportClicked);

    // 自动加载数据
    loadReports();
}

QAReportWindow::~QAReportWindow()
{
    delete m_reportModel;
    delete ui;
}

void QAReportWindow::loadReports()
{
    QDate fromDate = ui->dateEdit_From->date();
    QDate toDate = ui->dateEdit_To->date();

    // 获取筛选条件
    QString deviceSide;
    int sideIndex = ui->comboBox_DeviceSide->currentIndex();
    if (sideIndex == 1) deviceSide = "left";
    else if (sideIndex == 2) deviceSide = "right";

    QString reportType;
    int typeIndex = ui->comboBox_ReportType->currentIndex();
    if (typeIndex == 1) reportType = "daily_qa";
    else if (typeIndex == 2) reportType = "monthly_calib";

    // 清理旧模型
    if (m_reportModel) {
        delete m_reportModel;
        m_reportModel = nullptr;
    }

    // 查询数据
    m_reportModel = QADbManager::instance().createReportModel(fromDate, toDate, deviceSide, reportType);
    if (m_reportModel) {
        ui->tableView_Reports->setModel(m_reportModel);
        ui->tableView_Reports->resizeColumnsToContents();

        int rowCount = m_reportModel->rowCount();
        ui->label_RecordCount->setText(QString("共 %1 条记录").arg(rowCount));
    } else {
        ui->label_RecordCount->setText("查询失败");
    }
}

void QAReportWindow::onQueryClicked()
{
    loadReports();
}

void QAReportWindow::onExportClicked()
{
    if (!m_reportModel || m_reportModel->rowCount() == 0) {
        QMessageBox::information(this, tr("提示"), tr("没有可导出的数据"));
        return;
    }

    QString fileName = QFileDialog::getSaveFileName(
        this, tr("导出CSV"),
        QString("qa_report_%1.csv").arg(QDate::currentDate().toString("yyyyMMdd")),
        tr("CSV文件 (*.csv)"));

    if (fileName.isEmpty()) return;

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(this, tr("错误"), tr("无法创建文件: %1").arg(fileName));
        return;
    }

    QTextStream out(&file);

    // 写入表头
    int colCount = m_reportModel->columnCount();
    for (int col = 0; col < colCount; ++col) {
        if (col > 0) out << ",";
        out << "\"" << m_reportModel->headerData(col, Qt::Horizontal).toString() << "\"";
    }
    out << "\n";

    // 写入数据
    int rowCount = m_reportModel->rowCount();
    for (int row = 0; row < rowCount; ++row) {
        for (int col = 0; col < colCount; ++col) {
            if (col > 0) out << ",";
            out << "\"" << m_reportModel->data(m_reportModel->index(row, col)).toString() << "\"";
        }
        out << "\n";
    }

    file.close();
    QMessageBox::information(this, tr("成功"), tr("数据已导出到: %1").arg(fileName));
}
