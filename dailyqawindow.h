#ifndef DAILYQAWINDOW_H
#define DAILYQAWINDOW_H

#include <QDialog>
#include <QString>
#include <QStringList>
#include "deviceconfig.h"

class ProjectorController;
class CCameraController;

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
    void onStartQAClicked();
    void onStartComputeClicked();

private:
    enum class SideRunResult {
        Skipped,
        Success,
        Failed
    };

    enum class SideComputeResult {
        Skipped,
        Success,
        Failed
    };

    struct SideConfig {
        QString displayName;
        QString folderName;
        QString cameraSN;
        QString projectorTag;
    };

    void updateDeviceInfo();
    void logMessage(const QString& message);
    QString getQABasePath() const;
    QString getQATypedPath(const QString& sideFolder, const QString& typeFolder) const;
    SideRunResult runSideDailyQA(const SideConfig& side);
    bool captureDarkImage(const QString& projectorTag,
                          void* cameraHandle,
                          unsigned char* imageBuffer,
                          unsigned int imageBufferSize,
                          const QString& projectorPath,
                          int poseNumber);
    bool capturePatternSequence(const QString& projectorTag,
                                void* cameraHandle,
                                unsigned char* imageBuffer,
                                unsigned int imageBufferSize,
                                const QString& projectorPath,
                                const QString& cameraPath,
                                int poseNumber);
    bool saveFrame(void* cameraHandle,
                   unsigned char* imageBuffer,
                   unsigned int imageBufferSize,
                   const QString& filePath,
                   int timeoutMs);
    bool copyWhiteFrameToCameraPath(const QString& projectorFilePath, const QString& cameraPath);
    int getNextPoseNumber(const QString& projectorPath) const;
    QString formatPoseNumber(int poseNumber) const;
    bool collectSinglePoseImages(const QString& projectorPath,
                                 QString& poseNumber,
                                 QStringList& orderedPoseFiles,
                                 QString& reason) const;
    QString getQACalibFilePath(const SideConfig& side) const;
    SideComputeResult computeSideProjectionError(const SideConfig& side, QString& summaryMessage);

    Ui::DailyQAWindow *ui;
    DeviceConfig m_deviceConfig;
    ProjectorController* m_projectorController;
    CCameraController* m_cameraController;
    bool m_isRunning;
};

#endif // DAILYQAWINDOW_H
