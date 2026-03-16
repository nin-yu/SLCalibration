#include "qadbmanager.h"
#include <QSqlError>
#include <QDebug>
#include <QDateTime>
#include <QSet>

namespace {
const QSet<QString> kRequiredDailyQAColumns = {
    "id",
    "report_date",
    "device_side",
    "report_type",
    "camera_sn",
    "projector_tag",
    "result_status",
    "pose_number",
    "point_count",
    "camera_mean_px",
    "camera_rms_px",
    "camera_p95_px",
    "camera_max_px",
    "projector_mean_px",
    "projector_rms_px",
    "projector_p95_px",
    "projector_max_px",
    "details",
    "created_at"
};

QSet<QString> getTableColumns(QSqlDatabase& db, const QString& tableName, bool& ok)
{
    ok = false;
    QSet<QString> columns;

    QSqlQuery query(db);
    if (!query.exec(QString("PRAGMA table_info(%1)").arg(tableName))) {
        qDebug() << "读取表结构失败:" << tableName << query.lastError().text();
        return columns;
    }

    while (query.next()) {
        columns.insert(query.value(1).toString());
    }
    ok = true;
    return columns;
}

bool containsAllColumns(const QSet<QString>& columns, const QSet<QString>& requiredColumns)
{
    for (const QString& name : requiredColumns) {
        if (!columns.contains(name)) {
            return false;
        }
    }
    return true;
}
}  // namespace

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

    // 检查并迁移每日QA报告表（旧版单RMS模型 -> 新版双误差模型）
    bool needCreateDailyQATable = true;
    if (m_db.tables().contains("qa_reports")) {
        bool schemaReadOk = false;
        const QSet<QString> existingColumns = getTableColumns(m_db, "qa_reports", schemaReadOk);
        if (!schemaReadOk) {
            return false;
        }

        const bool hasLegacyRmsColumn = existingColumns.contains("rms_error");
        const bool hasRequiredColumns = containsAllColumns(existingColumns, kRequiredDailyQAColumns);
        if (hasLegacyRmsColumn || !hasRequiredColumns) {
            const QString backupTableName =
                QString("qa_reports_backup_%1").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
            if (!query.exec(QString("ALTER TABLE qa_reports RENAME TO %1").arg(backupTableName))) {
                qDebug() << "备份旧qa_reports表失败:" << query.lastError().text();
                return false;
            }
            qDebug() << "已备份旧qa_reports表为:" << backupTableName;
            needCreateDailyQATable = true;
        } else {
            needCreateDailyQATable = false;
        }
    }

    if (needCreateDailyQATable) {
        const bool ok = query.exec(
            "CREATE TABLE IF NOT EXISTS qa_reports ("
            "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
            "  report_date TEXT NOT NULL,"
            "  device_side TEXT NOT NULL,"
            "  report_type TEXT NOT NULL,"
            "  camera_sn TEXT,"
            "  projector_tag TEXT,"
            "  result_status TEXT NOT NULL,"
            "  pose_number TEXT,"
            "  point_count INTEGER DEFAULT 0,"
            "  camera_mean_px REAL,"
            "  camera_rms_px REAL,"
            "  camera_p95_px REAL,"
            "  camera_max_px REAL,"
            "  projector_mean_px REAL,"
            "  projector_rms_px REAL,"
            "  projector_p95_px REAL,"
            "  projector_max_px REAL,"
            "  details TEXT,"
            "  created_at TEXT DEFAULT (datetime('now','localtime'))"
            ")"
        );
        if (!ok) {
            qDebug() << "创建qa_reports表失败:" << query.lastError().text();
            return false;
        }
    }

    // 创建标定记录表
    const bool ok = query.exec(
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

bool QADbManager::insertDailyQAReport(const QDate& reportDate,
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
                                      const QString& details)
{
    if (!m_initialized) return false;

    QSqlQuery query(m_db);
    query.prepare(
        "INSERT INTO qa_reports (report_date, device_side, report_type, camera_sn, "
        "projector_tag, result_status, pose_number, point_count, "
        "camera_mean_px, camera_rms_px, camera_p95_px, camera_max_px, "
        "projector_mean_px, projector_rms_px, projector_p95_px, projector_max_px, details) "
        "VALUES (:date, :side, :type, :camera, :projector, :status, :pose, :pointCount, "
        ":camMean, :camRms, :camP95, :camMax, :projMean, :projRms, :projP95, :projMax, :details)"
    );
    query.bindValue(":date", reportDate.toString(Qt::ISODate));
    query.bindValue(":side", deviceSide);
    query.bindValue(":type", reportType);
    query.bindValue(":camera", cameraSN);
    query.bindValue(":projector", projectorTag);
    query.bindValue(":status", resultStatus);
    query.bindValue(":pose", poseNumber);
    query.bindValue(":pointCount", pointCount);
    query.bindValue(":camMean", cameraMeanPx);
    query.bindValue(":camRms", cameraRmsPx);
    query.bindValue(":camP95", cameraP95Px);
    query.bindValue(":camMax", cameraMaxPx);
    query.bindValue(":projMean", projectorMeanPx);
    query.bindValue(":projRms", projectorRmsPx);
    query.bindValue(":projP95", projectorP95Px);
    query.bindValue(":projMax", projectorMaxPx);
    query.bindValue(":details", details);

    if (!query.exec()) {
        qDebug() << "插入每日QA报告失败:" << query.lastError().text();
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
                  "projector_tag AS '投影仪Tag', pose_number AS 'Pose号', "
                  "point_count AS '点数', camera_mean_px AS '相机Mean(px)', "
                  "camera_rms_px AS '相机RMS(px)', camera_p95_px AS '相机P95(px)', "
                  "camera_max_px AS '相机Max(px)', projector_mean_px AS '投影Mean(px)', "
                  "projector_rms_px AS '投影RMS(px)', projector_p95_px AS '投影P95(px)', "
                  "projector_max_px AS '投影Max(px)', result_status AS '结果', "
                  "created_at AS '创建时间' "
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
