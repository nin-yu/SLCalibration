#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "deviceconfig.h"

class CalibrationWindow;
class DailyQAWindow;
class QAReportWindow;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onDailyQAClicked();
    void onCalibrationClicked();
    void onQAReportClicked();

private:
    bool loadConfiguration();
    bool createDefaultConfigFile(const QString& filePath);

    Ui::MainWindow *ui;
    DeviceConfig m_deviceConfig;

    CalibrationWindow* m_calibrationWin = nullptr;
    DailyQAWindow* m_dailyQAWin = nullptr;
    QAReportWindow* m_qaReportWin = nullptr;
};

#endif // MAINWINDOW_H
