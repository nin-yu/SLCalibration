#ifndef QAREPORTWINDOW_H
#define QAREPORTWINDOW_H

#include <QMainWindow>
#include <QSqlQueryModel>

namespace Ui {
class QAReportWindow;
}

class QAReportWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit QAReportWindow(QWidget *parent = nullptr);
    ~QAReportWindow();

private slots:
    void onQueryClicked();
    void onExportClicked();

private:
    void loadReports();

    Ui::QAReportWindow *ui;
    QSqlQueryModel* m_reportModel = nullptr;
};

#endif // QAREPORTWINDOW_H
