#include "qadbmanager.h"
#include <QSqlError>
#include <QDebug>
#include <QDateTime>

QADbManager::QADbManager()
{
}

QADbManager::~QADbManager()
{
    if (m_db.isOpen()) {
        m_db.close();
    }
}

QADbManager& QADbManager::instance()
{
    static QADbManager inst;
    return inst;
}

bool QADbManager::initialize(const QString& dbPath)
{
    if (m_initialized) {
        return true;
    }

    m_db = QSqlDatabase::addDatabase("QSQLITE", "qa_connection");
    m_db.setDatabaseName(dbPath);

    if (!m_db.open()) {
        qDebug() << "无法打开QA数据库:" << m_db.lastError().text();
        return false;
    }

    if (!createTables()) {
        qDebug() << "无法创建数据库表";
        return false;
    }

    m_initialized = true;
    qDebug() << "QA数据库初始化成功:" << dbPath;
    return true;
}

bool QADbManager::createTables()
{
    QSqlQuery query(m_db);

    // 创建QA报告表
    bool ok = query.exec(
        "CREATE TABLE IF NOT EXISTS qa_reports ("
        "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  report_date TEXT NOT NULL,"
        "  device_side TEXT NOT NULL,"
        "  report_type TEXT NOT NULL,"
        "  camera_sn TEXT,"
        "  projector_tag TEXT,"
        "  result_status TEXT NOT NULL,"
        "  rms_error REAL,"
        "  details TEXT,"
        "  created_at TEXT DEFAULT (datetime('now','localtime'))"
        ")"
    );

    if (!ok) {
        qDebug() << "创建qa_reports表失败:" << query.lastError().text();
        return false;
    }

    // 创建标定记录表
    ok = query.exec(
        "CREATE TABLE IF NOT EXISTS calibration_records ("
        "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  calibration_date TEXT NOT NULL,"
        "  device_side TEXT NOT NULL,"
        "  camera_sn TEXT,"
        "  projector_tag TEXT,"
        "  calib_file_path TEXT,"
        "  rms_proj REAL,"
        "  rms_stereo REAL,"
        "  epi_mean_px REAL,"
        "  epi_median_px REAL,"
        "  details TEXT,"
        "  created_at TEXT DEFAULT (datetime('now','localtime'))"
        ")"
    );

    if (!ok) {
        qDebug() << "创建calibration_records表失败:" << query.lastError().text();
        return false;
    }

    return true;
}

bool QADbManager::insertQAReport(const QDate& reportDate,
                                  const QString& deviceSide,
                                  const QString& reportType,
                                  const QString& cameraSN,
                                  const QString& projectorTag,
                                  const QString& resultStatus,
                                  double rmsError,
                                  const QString& details)
{
    if (!m_initialized) return false;

    QSqlQuery query(m_db);
    query.prepare(
        "INSERT INTO qa_reports (report_date, device_side, report_type, camera_sn, "
        "projector_tag, result_status, rms_error, details) "
        "VALUES (:date, :side, :type, :camera, :projector, :status, :rms, :details)"
    );
    query.bindValue(":date", reportDate.toString(Qt::ISODate));
    query.bindValue(":side", deviceSide);
    query.bindValue(":type", reportType);
    query.bindValue(":camera", cameraSN);
    query.bindValue(":projector", projectorTag);
    query.bindValue(":status", resultStatus);
    query.bindValue(":rms", rmsError);
    query.bindValue(":details", details);

    if (!query.exec()) {
        qDebug() << "插入QA报告失败:" << query.lastError().text();
        return false;
    }
    return true;
}

bool QADbManager::insertCalibrationRecord(const QDate& calibDate,
                                           const QString& deviceSide,
                                           const QString& cameraSN,
                                           const QString& projectorTag,
                                           const QString& calibFilePath,
                                           double rmsProj,
                                           double rmsStereo,
                                           double epiMeanPx,
                                           double epiMedianPx,
                                           const QString& details)
{
    if (!m_initialized) return false;

    QSqlQuery query(m_db);
    query.prepare(
        "INSERT INTO calibration_records (calibration_date, device_side, camera_sn, "
        "projector_tag, calib_file_path, rms_proj, rms_stereo, epi_mean_px, epi_median_px, details) "
        "VALUES (:date, :side, :camera, :projector, :path, :rmsP, :rmsS, :epiMean, :epiMedian, :details)"
    );
    query.bindValue(":date", calibDate.toString(Qt::ISODate));
    query.bindValue(":side", deviceSide);
    query.bindValue(":camera", cameraSN);
    query.bindValue(":projector", projectorTag);
    query.bindValue(":path", calibFilePath);
    query.bindValue(":rmsP", rmsProj);
    query.bindValue(":rmsS", rmsStereo);
    query.bindValue(":epiMean", epiMeanPx);
    query.bindValue(":epiMedian", epiMedianPx);
    query.bindValue(":details", details);

    if (!query.exec()) {
        qDebug() << "插入标定记录失败:" << query.lastError().text();
        return false;
    }
    return true;
}

QSqlQueryModel* QADbManager::createReportModel(const QDate& fromDate,
                                                const QDate& toDate,
                                                const QString& deviceSide,
                                                const QString& reportType)
{
    if (!m_initialized) return nullptr;

    QString sql = "SELECT id, report_date AS '日期', device_side AS '设备侧', "
                  "report_type AS '类型', camera_sn AS '相机SN', "
                  "projector_tag AS '投影仪Tag', result_status AS '结果', "
                  "rms_error AS 'RMS误差', created_at AS '创建时间' "
                  "FROM qa_reports WHERE report_date >= :from AND report_date <= :to";

    if (!deviceSide.isEmpty()) {
        sql += " AND device_side = :side";
    }
    if (!reportType.isEmpty()) {
        sql += " AND report_type = :type";
    }
    sql += " ORDER BY report_date DESC, id DESC";

    QSqlQuery query(m_db);
    query.prepare(sql);
    query.bindValue(":from", fromDate.toString(Qt::ISODate));
    query.bindValue(":to", toDate.toString(Qt::ISODate));
    if (!deviceSide.isEmpty()) {
        query.bindValue(":side", deviceSide);
    }
    if (!reportType.isEmpty()) {
        query.bindValue(":type", reportType);
    }
    query.exec();

    QSqlQueryModel* model = new QSqlQueryModel();
    model->setQuery(std::move(query));
    return model;
}

int QADbManager::getReportCount() const
{
    if (!m_initialized) return 0;

    QSqlQuery query(m_db);
    query.exec("SELECT COUNT(*) FROM qa_reports");
    if (query.next()) {
        return query.value(0).toInt();
    }
    return 0;
}
