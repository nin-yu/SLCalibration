#ifndef CONFIGMANAGER_H
#define CONFIGMANAGER_H

#include <QObject>
#include <QString>
#include <opencv2/core.hpp>

/**
 * @brief 配置管理器单例类
 * 
 * 集中管理项目配置参数，从 config.ini 读取和写入。
 * 标定模块和QA模块参数完全独立。
 */
class ConfigManager : public QObject
{
    Q_OBJECT

public:
    /**
     * @brief 获取单例实例
     */
    static ConfigManager& instance();

    /**
     * @brief 从配置文件加载参数
     * @param filePath 配置文件路径
     * @return 加载是否成功
     */
    bool loadFromFile(const QString& filePath);

    /**
     * @brief 保存配置到文件
     * @param filePath 文件路径，为空则使用上次加载的路径
     * @return 保存是否成功
     */
    bool saveToFile(const QString& filePath = QString());

    // ========== A. 设备标识（只读）==========
    QString leftCameraSN() const { return m_leftCameraSN; }
    QString rightCameraSN() const { return m_rightCameraSN; }
    QString leftProjectorTag() const { return m_leftProjectorTag; }
    QString rightProjectorTag() const { return m_rightProjectorTag; }

    // ========== B. 标定模块参数 ==========
    int projectorWidth() const { return m_projectorWidth; }
    int projectorHeight() const { return m_projectorHeight; }
    cv::Size projectorSize() const { return cv::Size(m_projectorWidth, m_projectorHeight); }
    int projectorFrequency() const { return m_projectorFrequency; }
    int grayCodeBits() const { return m_grayCodeBits; }
    int phaseShiftSteps() const { return m_phaseShiftSteps; }
    int patternRows() const { return m_patternRows; }
    int patternCols() const { return m_patternCols; }
    double squareSizeMm() const { return m_squareSizeMm; }

    // 标定模块参数 setter
    void setProjectorWidth(int value);
    void setProjectorHeight(int value);
    void setProjectorFrequency(int value);
    void setGrayCodeBits(int value);
    void setPhaseShiftSteps(int value);
    void setPatternRows(int value);
    void setPatternCols(int value);
    void setSquareSizeMm(double value);

    // ========== C. 重建参数 ==========
    double modulationThreshold() const { return m_modulationThreshold; }
    double minDepth() const { return m_minDepth; }
    double maxDepth() const { return m_maxDepth; }

    // 重建参数 setter
    void setModulationThreshold(double value);
    void setMinDepth(double value);
    void setMaxDepth(double value);

    // ========== D. QA 模块参数（独立于标定）==========
    // QA 棋盘格参数
    int qaPatternRows() const { return m_qaPatternRows; }
    int qaPatternCols() const { return m_qaPatternCols; }
    double qaSquareSizeMm() const { return m_qaSquareSizeMm; }

    // QA 结构光参数
    int qaProjectorWidth() const { return m_qaProjectorWidth; }
    int qaProjectorHeight() const { return m_qaProjectorHeight; }
    cv::Size qaProjectorSize() const { return cv::Size(m_qaProjectorWidth, m_qaProjectorHeight); }
    int qaProjectorFrequency() const { return m_qaProjectorFrequency; }
    int qaGrayCodeBits() const { return m_qaGrayCodeBits; }
    int qaPhaseShiftSteps() const { return m_qaPhaseShiftSteps; }

    // QA 日检阈值
    double cameraReprojFailThreshold() const { return m_cameraReprojFailThreshold; }
    double dailyRmsProjFailThreshold() const { return m_dailyRmsProjFailThreshold; }
    int minValidPoseCount() const { return m_minValidPoseCount; }

    // QA 月检阈值
    double monthlyRmsProjFailThreshold() const { return m_monthlyRmsProjFailThreshold; }
    double monthlyRmsStereoFailThreshold() const { return m_monthlyRmsStereoFailThreshold; }
    double monthlyEpiMeanFailThreshold() const { return m_monthlyEpiMeanFailThreshold; }

    // QA 其他参数
    QString dailyQARoot() const { return m_dailyQARoot; }
    QString dailyMode() const { return m_dailyMode; }

    // ========== E. QA 模式设备预设 ==========
    float qaCameraExposureUs() const { return m_qaCameraExposureUs; }
    float qaCameraGain() const { return m_qaCameraGain; }
    int qaCameraTriggerMode() const { return m_qaCameraTriggerMode; }
    int qaProjectorExposureUs() const { return m_qaProjectorExposureUs; }
    int qaProjectorPattern() const { return m_qaProjectorPattern; }
    int qaProjectorTriggerMode() const { return m_qaProjectorTriggerMode; }
    QString qaCameraTriggerSource() const { return m_qaCameraTriggerSource; }
    int qaImageTimeoutMs() const { return m_qaImageTimeoutMs; }
    unsigned int qaImageBufferSize() const { return m_qaImageBufferSize; }
    int qaExpectedPoseImageCount() const { return m_qaExpectedPoseImageCount; }
    int qaMinValidPointCount() const { return m_qaMinValidPointCount; }

signals:
    /**
     * @brief 配置项变更信号
     * @param section 配置节名称
     * @param key 配置键名称
     */
    void configChanged(const QString& section, const QString& key);

    /**
     * @brief 标定参数变更信号
     */
    void calibParamsChanged();

    /**
     * @brief 重建参数变更信号
     */
    void reconParamsChanged();

private:
    // 私有构造函数（单例模式）
    explicit ConfigManager(QObject* parent = nullptr);
    ~ConfigManager();

    // 禁止拷贝
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

    // 内部辅助方法
    void setDefaults();

    // 配置文件路径
    QString m_configFilePath;

    // ===== A. 设备标识 =====
    QString m_leftCameraSN;
    QString m_rightCameraSN;
    QString m_leftProjectorTag;
    QString m_rightProjectorTag;

    // ===== B. 标定模块参数 =====
    int m_projectorWidth;
    int m_projectorHeight;
    int m_projectorFrequency;
    int m_grayCodeBits;
    int m_phaseShiftSteps;
    int m_patternRows;
    int m_patternCols;
    double m_squareSizeMm;

    // ===== C. 重建参数 =====
    double m_modulationThreshold;
    double m_minDepth;
    double m_maxDepth;

    // ===== D. QA 模块参数 =====
    // QA 棋盘格参数
    int m_qaPatternRows;
    int m_qaPatternCols;
    double m_qaSquareSizeMm;

    // QA 结构光参数
    int m_qaProjectorWidth;
    int m_qaProjectorHeight;
    int m_qaProjectorFrequency;
    int m_qaGrayCodeBits;
    int m_qaPhaseShiftSteps;

    // QA 日检阈值
    double m_cameraReprojFailThreshold;
    double m_dailyRmsProjFailThreshold;
    int m_minValidPoseCount;

    // QA 月检阈值
    double m_monthlyRmsProjFailThreshold;
    double m_monthlyRmsStereoFailThreshold;
    double m_monthlyEpiMeanFailThreshold;

    // QA 其他参数
    QString m_dailyQARoot;
    QString m_dailyMode;

    // ===== E. QA 模式设备预设 =====
    float m_qaCameraExposureUs;
    float m_qaCameraGain;
    int m_qaCameraTriggerMode;
    int m_qaProjectorExposureUs;
    int m_qaProjectorPattern;
    int m_qaProjectorTriggerMode;
    QString m_qaCameraTriggerSource;
    int m_qaImageTimeoutMs;
    unsigned int m_qaImageBufferSize;
    int m_qaExpectedPoseImageCount;
    int m_qaMinValidPointCount;
};

#endif // CONFIGMANAGER_H
