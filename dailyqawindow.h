#ifndef DAILYQAWINDOW_H
#define DAILYQAWINDOW_H

#include <QDialog>
#include "deviceconfig.h"

namespace Ui {
class DailyQAWindow;
}

class DailyQAWindow : public QDialog
{
    Q_OBJECT

public:
    explicit DailyQAWindow(const DeviceConfig& config, QWidget *parent = nullptr);
    ~DailyQAWindow();

private slots:
    void onStartQAClicked();

private:
    void updateDeviceInfo();

    Ui::DailyQAWindow *ui;
    DeviceConfig m_deviceConfig;
};

#endif // DAILYQAWINDOW_H
