#include "configmanager.h"
#include <QSettings>
#include <QFileInfo>
#include <QDebug>

ConfigManager::ConfigManager(QObject* parent)
    : QObject(parent)
{
    setDefaults();
}

ConfigManager::~ConfigManager()
{
}

ConfigManager& ConfigManager::instance()
{
    static ConfigManager instance;
    return instance;
}

void ConfigManager::setDefaults()
{
    // A. 设备标识默认值（空字符串）
    m_leftCameraSN.clear();
    m_rightCameraSN.clear();
    m_leftProjectorTag = "LCR2@dibh_left";
    m_rightProjectorTag = "LCR2@dibh_right";

    // B. 标定模块参数默认值
    m_projectorWidth = 912;
    m_projectorHeight = 1140;
    m_projectorFrequency = 16;
    m_grayCodeBits = 5;
    m_phaseShiftSteps = 4;
    m_patternRows = 20;
    m_patternCols = 20;
    m_squareSizeMm = 10.0;

    // C. 重建参数默认值
    m_modulationThreshold = 70.0;
    m_minDepth = 200.0;
    m_maxDepth = 1500.0;

    // D. QA 模块参数默认值
    m_qaPatternRows = 20;
    m_qaPatternCols = 20;
    m_qaSquareSizeMm = 10.0;
    m_qaProjectorWidth = 912;
    m_qaProjectorHeight = 1140;
    m_qaProjectorFrequency = 16;
    m_qaGrayCodeBits = 5;
    m_qaPhaseShiftSteps = 4;
    m_cameraReprojFailThreshold = 0.8;
    m_dailyRmsProjFailThreshold = 0.8;
    m_minValidPoseCount = 1;
    m_monthlyRmsProjFailThreshold = 0.8;
    m_monthlyRmsStereoFailThreshold = 0.8;
    m_monthlyEpiMeanFailThreshold = 1.0;
    m_dailyQARoot = "QAData";
    m_dailyMode = "pose_batch";

    // E. QA 模式设备预设默认值
    m_qaCameraExposureUs = 45000.0f;
    m_qaCameraGain = 0.0f;
    m_qaCameraTriggerMode = 1;
    m_qaProjectorExposureUs = 500000;
    m_qaProjectorPattern = 6;
    m_qaProjectorTriggerMode = 0;
    m_qaCameraTriggerSource = "Line0";
    m_qaImageTimeoutMs = 3000;
    m_qaImageBufferSize = 5 * 1024 * 1024;  // 5MB
    m_qaExpectedPoseImageCount = 20;
    m_qaMinValidPointCount = 320;
}

bool ConfigManager::loadFromFile(const QString& filePath)
{
    if (filePath.isEmpty()) {
        qWarning() << "ConfigManager: 配置文件路径为空";
        return false;
    }

    m_configFilePath = filePath;
    QSettings settings(filePath, QSettings::IniFormat);
    // Qt 6 默认使用 UTF-8 编码，无需设置 setIniCodec

    bool needSave = false;  // 标记是否需要回写缺失的配置项

    // ===== A. 设备标识 [Camera] 和 [Projector] =====
    settings.beginGroup("Camera");
    m_leftCameraSN = settings.value("LeftSerialNumber", "").toString();
    m_rightCameraSN = settings.value("RightSerialNumber", "").toString();
    settings.endGroup();

    settings.beginGroup("Projector");
    if (!settings.contains("LeftTagName")) {
        settings.setValue("LeftTagName", m_leftProjectorTag);
        needSave = true;
    } else {
        m_leftProjectorTag = settings.value("LeftTagName").toString();
    }
    if (!settings.contains("RightTagName")) {
        settings.setValue("RightTagName", m_rightProjectorTag);
        needSave = true;
    } else {
        m_rightProjectorTag = settings.value("RightTagName").toString();
    }
    settings.endGroup();

    // ===== B. 标定模块参数 [Calibration] =====
    settings.beginGroup("Calibration");
    
    // 辅助宏：读取配置项，不存在则使用默认值并标记需要保存
    #define LOAD_VALUE(key, member, defaultVal) \
        if (!settings.contains(key)) { \
            settings.setValue(key, defaultVal); \
            needSave = true; \
        } else { \
            member = settings.value(key, defaultVal).value<decltype(member)>(); \
        }

    LOAD_VALUE("ProjectorWidth", m_projectorWidth, 912)
    LOAD_VALUE("ProjectorHeight", m_projectorHeight, 1140)
    LOAD_VALUE("ProjectorFrequency", m_projectorFrequency, 16)
    LOAD_VALUE("GrayCodeBits", m_grayCodeBits, 5)
    LOAD_VALUE("PhaseShiftSteps", m_phaseShiftSteps, 4)
    LOAD_VALUE("PatternRows", m_patternRows, 20)
    LOAD_VALUE("PatternCols", m_patternCols, 20)
    LOAD_VALUE("SquareSizeMm", m_squareSizeMm, 10.0)
    
    settings.endGroup();

    // ===== C. 重建参数 [Reconstruction] =====
    settings.beginGroup("Reconstruction");
    LOAD_VALUE("ModulationThreshold", m_modulationThreshold, 70.0)
    LOAD_VALUE("MinDepth", m_minDepth, 200.0)
    LOAD_VALUE("MaxDepth", m_maxDepth, 1500.0)
    settings.endGroup();

    // ===== D. QA 模块参数 [QA] =====
    settings.beginGroup("QA");
    
    // QA 棋盘格参数
    LOAD_VALUE("PatternRows", m_qaPatternRows, 20)
    LOAD_VALUE("PatternCols", m_qaPatternCols, 20)
    LOAD_VALUE("SquareSizeMm", m_qaSquareSizeMm, 10.0)
    
    // QA 结构光参数
    LOAD_VALUE("ProjectorWidth", m_qaProjectorWidth, 912)
    LOAD_VALUE("ProjectorHeight", m_qaProjectorHeight, 1140)
    LOAD_VALUE("ProjectorFrequency", m_qaProjectorFrequency, 16)
    LOAD_VALUE("GrayCodeBits", m_qaGrayCodeBits, 5)
    LOAD_VALUE("PhaseShiftSteps", m_qaPhaseShiftSteps, 4)
    
    // QA 日检阈值
    LOAD_VALUE("CameraReprojFailThresholdPx", m_cameraReprojFailThreshold, 0.8)
    LOAD_VALUE("DailyRmsProjFailThreshold", m_dailyRmsProjFailThreshold, 0.8)
    LOAD_VALUE("MinValidPoseCount", m_minValidPoseCount, 1)
    
    // QA 月检阈值
    LOAD_VALUE("MonthlyRmsProjFailThreshold", m_monthlyRmsProjFailThreshold, 0.8)
    LOAD_VALUE("MonthlyRmsStereoFailThreshold", m_monthlyRmsStereoFailThreshold, 0.8)
    LOAD_VALUE("MonthlyEpiMeanFailThresholdPx", m_monthlyEpiMeanFailThreshold, 1.0)
    
    // QA 其他参数
    if (!settings.contains("DailyQARoot")) {
        settings.setValue("DailyQARoot", m_dailyQARoot);
        needSave = true;
    } else {
        m_dailyQARoot = settings.value("DailyQARoot").toString();
    }
    if (!settings.contains("DailyMode")) {
        settings.setValue("DailyMode", m_dailyMode);
        needSave = true;
    } else {
        m_dailyMode = settings.value("DailyMode").toString();
    }
    
    settings.endGroup();

    // ===== E. QA 模式设备预设 [QAPreset] =====
    settings.beginGroup("QAPreset");
    LOAD_VALUE("CameraExposureUs", m_qaCameraExposureUs, 45000.0f)
    LOAD_VALUE("CameraGain", m_qaCameraGain, 0.0f)
    LOAD_VALUE("CameraTriggerMode", m_qaCameraTriggerMode, 1)
    LOAD_VALUE("ProjectorExposureUs", m_qaProjectorExposureUs, 500000)
    LOAD_VALUE("ProjectorPattern", m_qaProjectorPattern, 6)
    LOAD_VALUE("ProjectorTriggerMode", m_qaProjectorTriggerMode, 0)
    
    if (!settings.contains("CameraTriggerSource")) {
        settings.setValue("CameraTriggerSource", m_qaCameraTriggerSource);
        needSave = true;
    } else {
        m_qaCameraTriggerSource = settings.value("CameraTriggerSource").toString();
    }
    
    LOAD_VALUE("ImageTimeoutMs", m_qaImageTimeoutMs, 3000)
    LOAD_VALUE("ImageBufferSize", m_qaImageBufferSize, 5u * 1024 * 1024)
    LOAD_VALUE("ExpectedPoseImageCount", m_qaExpectedPoseImageCount, 20)
    LOAD_VALUE("MinValidPointCount", m_qaMinValidPointCount, 320)
    settings.endGroup();

    #undef LOAD_VALUE

    // 如果有缺失配置项，同步写入文件
    if (needSave) {
        settings.sync();
        qDebug() << "ConfigManager: 已补全缺失的配置项到" << filePath;
    }

    qDebug() << "ConfigManager: 配置加载完成 -" << filePath;
    return true;
}

bool ConfigManager::saveToFile(const QString& filePath)
{
    QString targetPath = filePath.isEmpty() ? m_configFilePath : filePath;
    if (targetPath.isEmpty()) {
        qWarning() << "ConfigManager: 保存路径为空";
        return false;
    }

    QSettings settings(targetPath, QSettings::IniFormat);
    // Qt 6 默认使用 UTF-8 编码，无需设置 setIniCodec

    // ===== A. 设备标识 =====
    settings.beginGroup("Camera");
    settings.setValue("LeftSerialNumber", m_leftCameraSN);
    settings.setValue("RightSerialNumber", m_rightCameraSN);
    settings.endGroup();

    settings.beginGroup("Projector");
    settings.setValue("LeftTagName", m_leftProjectorTag);
    settings.setValue("RightTagName", m_rightProjectorTag);
    settings.endGroup();

    // ===== B. 标定模块参数 =====
    settings.beginGroup("Calibration");
    settings.setValue("ProjectorWidth", m_projectorWidth);
    settings.setValue("ProjectorHeight", m_projectorHeight);
    settings.setValue("ProjectorFrequency", m_projectorFrequency);
    settings.setValue("GrayCodeBits", m_grayCodeBits);
    settings.setValue("PhaseShiftSteps", m_phaseShiftSteps);
    settings.setValue("PatternRows", m_patternRows);
    settings.setValue("PatternCols", m_patternCols);
    settings.setValue("SquareSizeMm", m_squareSizeMm);
    settings.endGroup();

    // ===== C. 重建参数 =====
    settings.beginGroup("Reconstruction");
    settings.setValue("ModulationThreshold", m_modulationThreshold);
    settings.setValue("MinDepth", m_minDepth);
    settings.setValue("MaxDepth", m_maxDepth);
    settings.endGroup();

    // ===== D. QA 模块参数 =====
    settings.beginGroup("QA");
    settings.setValue("PatternRows", m_qaPatternRows);
    settings.setValue("PatternCols", m_qaPatternCols);
    settings.setValue("SquareSizeMm", m_qaSquareSizeMm);
    settings.setValue("ProjectorWidth", m_qaProjectorWidth);
    settings.setValue("ProjectorHeight", m_qaProjectorHeight);
    settings.setValue("ProjectorFrequency", m_qaProjectorFrequency);
    settings.setValue("GrayCodeBits", m_qaGrayCodeBits);
    settings.setValue("PhaseShiftSteps", m_qaPhaseShiftSteps);
    settings.setValue("CameraReprojFailThresholdPx", m_cameraReprojFailThreshold);
    settings.setValue("DailyRmsProjFailThreshold", m_dailyRmsProjFailThreshold);
    settings.setValue("MinValidPoseCount", m_minValidPoseCount);
    settings.setValue("MonthlyRmsProjFailThreshold", m_monthlyRmsProjFailThreshold);
    settings.setValue("MonthlyRmsStereoFailThreshold", m_monthlyRmsStereoFailThreshold);
    settings.setValue("MonthlyEpiMeanFailThresholdPx", m_monthlyEpiMeanFailThreshold);
    settings.setValue("DailyQARoot", m_dailyQARoot);
    settings.setValue("DailyMode", m_dailyMode);
    settings.endGroup();

    // ===== E. QA 模式设备预设 =====
    settings.beginGroup("QAPreset");
    settings.setValue("CameraExposureUs", m_qaCameraExposureUs);
    settings.setValue("CameraGain", m_qaCameraGain);
    settings.setValue("CameraTriggerMode", m_qaCameraTriggerMode);
    settings.setValue("ProjectorExposureUs", m_qaProjectorExposureUs);
    settings.setValue("ProjectorPattern", m_qaProjectorPattern);
    settings.setValue("ProjectorTriggerMode", m_qaProjectorTriggerMode);
    settings.setValue("CameraTriggerSource", m_qaCameraTriggerSource);
    settings.setValue("ImageTimeoutMs", m_qaImageTimeoutMs);
    settings.setValue("ImageBufferSize", m_qaImageBufferSize);
    settings.setValue("ExpectedPoseImageCount", m_qaExpectedPoseImageCount);
    settings.setValue("MinValidPointCount", m_qaMinValidPointCount);
    settings.endGroup();

    settings.sync();
    qDebug() << "ConfigManager: 配置已保存到" << targetPath;
    return true;
}

// ========== B. 标定模块参数 setter ==========

void ConfigManager::setProjectorWidth(int value)
{
    if (m_projectorWidth != value) {
        m_projectorWidth = value;
        saveToFile();
        emit configChanged("Calibration", "ProjectorWidth");
        emit calibParamsChanged();
    }
}

void ConfigManager::setProjectorHeight(int value)
{
    if (m_projectorHeight != value) {
        m_projectorHeight = value;
        saveToFile();
        emit configChanged("Calibration", "ProjectorHeight");
        emit calibParamsChanged();
    }
}

void ConfigManager::setProjectorFrequency(int value)
{
    if (m_projectorFrequency != value) {
        m_projectorFrequency = value;
        saveToFile();
        emit configChanged("Calibration", "ProjectorFrequency");
        emit calibParamsChanged();
    }
}

void ConfigManager::setGrayCodeBits(int value)
{
    if (m_grayCodeBits != value) {
        m_grayCodeBits = value;
        saveToFile();
        emit configChanged("Calibration", "GrayCodeBits");
        emit calibParamsChanged();
    }
}

void ConfigManager::setPhaseShiftSteps(int value)
{
    if (m_phaseShiftSteps != value) {
        m_phaseShiftSteps = value;
        saveToFile();
        emit configChanged("Calibration", "PhaseShiftSteps");
        emit calibParamsChanged();
    }
}

void ConfigManager::setPatternRows(int value)
{
    if (m_patternRows != value) {
        m_patternRows = value;
        saveToFile();
        emit configChanged("Calibration", "PatternRows");
        emit calibParamsChanged();
    }
}

void ConfigManager::setPatternCols(int value)
{
    if (m_patternCols != value) {
        m_patternCols = value;
        saveToFile();
        emit configChanged("Calibration", "PatternCols");
        emit calibParamsChanged();
    }
}

void ConfigManager::setSquareSizeMm(double value)
{
    if (qAbs(m_squareSizeMm - value) > 1e-6) {
        m_squareSizeMm = value;
        saveToFile();
        emit configChanged("Calibration", "SquareSizeMm");
        emit calibParamsChanged();
    }
}

// ========== C. 重建参数 setter ==========

void ConfigManager::setModulationThreshold(double value)
{
    if (qAbs(m_modulationThreshold - value) > 1e-6) {
        m_modulationThreshold = value;
        saveToFile();
        emit configChanged("Reconstruction", "ModulationThreshold");
        emit reconParamsChanged();
    }
}

void ConfigManager::setMinDepth(double value)
{
    if (qAbs(m_minDepth - value) > 1e-6) {
        m_minDepth = value;
        saveToFile();
        emit configChanged("Reconstruction", "MinDepth");
        emit reconParamsChanged();
    }
}

void ConfigManager::setMaxDepth(double value)
{
    if (qAbs(m_maxDepth - value) > 1e-6) {
        m_maxDepth = value;
        saveToFile();
        emit configChanged("Reconstruction", "MaxDepth");
        emit reconParamsChanged();
    }
}
