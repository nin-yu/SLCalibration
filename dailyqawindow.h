#ifndef DAILYQAWINDOW_H
#define DAILYQAWINDOW_H

#include <QDialog>
#include <QDateTime>
#include "deviceconfig.h"

namespace Ui {
class DailyQAWindow;
}

class CalibrationWindow;
class QPushButton;

class DailyQAWindow : public QDialog
{
    Q_OBJECT

public:
    explicit DailyQAWindow(const DeviceConfig& config, QWidget *parent = nullptr);
    ~DailyQAWindow();

private slots:
    void onStartDailyQAClicked();
    void onStartMonthlyQAClicked();

private:
    void updateDeviceInfo();
    void loadQASettings();
    void appendLog(const QString& message);
    bool runDailyQASide(const QString& deviceSide,
                        const QString& cameraSN,
                        const QString& projectorTag,
                        const QString& sideLabel,
                        int& passCount,
                        int& failCount);
    void onMonthlyCalibrationWindowClosed();
    void generateMonthlyReportForSide(const QString& deviceSide,
                                      const QString& defaultCameraSN,
                                      const QString& defaultProjectorTag,
                                      const QString& sideLabel,
                                      int& passCount,
                                      int& failCount);

    Ui::DailyQAWindow *ui;
    DeviceConfig m_deviceConfig;
    QPushButton* m_buttonStartMonthlyQA = nullptr;
    CalibrationWindow* m_monthlyCalibrationWindow = nullptr;
    QDateTime m_monthlyWorkflowStartAt;

    int m_qaPatternRows = 20;
    int m_qaPatternCols = 20;
    double m_qaSquareSizeMm = 10.0;
    QString m_dailyQARootPath;
    double m_cameraReprojFailThresholdPx = 0.8;
    double m_projectorReprojFailThresholdPx = 0.8;
    int m_minValidPoseCount = 1;
    int m_projectorWidth = 912;
    int m_projectorHeight = 1140;
    int m_projectorFrequency = 16;
    int m_grayCodeBits = 5;
    int m_phaseShiftSteps = 4;
    double m_monthlyRmsProjFailThreshold = 0.8;
    double m_monthlyRmsStereoFailThreshold = 0.8;
    double m_monthlyEpiMeanFailThreshold = 1.0;
};

#endif // DAILYQAWINDOW_H
