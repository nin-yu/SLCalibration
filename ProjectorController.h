#ifndef PROJECTORCONTROLLDDL_H
#define PROJECTORCONTROLLDDL_H

#include "Projector/ProjectorControlDll.h"
#include <iostream>
#include <vector>
#include <string>
#include <map>

class ProjectorController {
private:
    // 管理所有投影仪实例（序列号 -> 控制器）
    static std::map<std::string, ProjectorControlDll*> s_projections;
    std::string m_currentSerial;

public:
    ProjectorController();
    ~ProjectorController();

    // 接口保持与原DLL版本完全兼容
    // dllPath参数保留但不再使用，改为初始化USB
    bool loadDll(const std::string& dllPath = "");

    // 检测投影仪
    int detectProjectors();

    // 获取投影仪序列号
    std::string getProjectorSerial(int index);

    // 初始化投影仪
    bool initProjector(const std::string& serialNum, int ledSetting = 7);

    // 发送并播放投影
    bool sendAndPlayProjector(const std::string& serialNum, int trigger, int projTag, int expose);

    // 暂停投影
    bool pauseProjector(const std::string& serialNum);

    // 关闭投影仪
    bool closeProjector(const std::string& serialNum);

    // 检查连接状态
    bool checkConnection(const std::string& serialNum);

private:
    // 获取或创建投影仪实例
    ProjectorControlDll* GetInstance(const std::string& sn);
};

#endif // PROJECTORCONTROLLDDL_H