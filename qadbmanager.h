#ifndef QADBMANAGER_H
#define QADBMANAGER_H

#include <QString>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlQueryModel>
#include <QDate>
#include <QDateTime>

struct CalibrationRecord
{
    int id = -1;
    QDate calibrationDate;
    QString deviceSide;
    QString cameraSN;
    QString projectorTag;
    QString calibFilePath;
    double rmsProj = 0.0;
    double rmsStereo = 0.0;
    double epiMeanPx = -1.0;
    double epiMedianPx = -1.0;
    QString details;
    QDateTime createdAt;
};

class QADbManager
{
public:
    static QADbManager& instance();

    bool initialize(const QString& dbPath);
    bool isInitialized() const { return m_initialized; }

    // 每日QA报告操作（双误差模型）
    bool insertDailyQAReport(const QDate& reportDate,
                             const QString& deviceSide,
                             const QString& reportType,
                             const QString& cameraSN,
                             const QString& projectorTag,
                             const QString& resultStatus,
                             const QString& poseNumber,
                             int pointCount,
                             double cameraMeanPx,
                             double cameraRmsPx,
                             double cameraP95Px,
                             double cameraMaxPx,
                             double projectorMeanPx,
                             double projectorRmsPx,
                             double projectorP95Px,
                             double projectorMaxPx,
                             const QString& details = QString());

    // 查询
    QSqlQueryModel* createReportModel(const QDate& fromDate,
                                      const QDate& toDate,
                                      const QString& deviceSide = QString(),
                                      const QString& reportType = QString());

private:
    QADbManager();
    ~QADbManager();
    QADbManager(const QADbManager&) = delete;
    QADbManager& operator=(const QADbManager&) = delete;

    bool createTables();

    QSqlDatabase m_db;
    bool m_initialized = false;
};

#endif // QADBMANAGER_H
