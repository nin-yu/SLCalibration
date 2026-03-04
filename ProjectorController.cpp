#include "ProjectorController.h"
#include <windows.h>

// 静态成员初始化
std::map<std::string, ProjectorControlDll*> ProjectorController::s_projections;

ProjectorController::ProjectorController() {}

ProjectorController::~ProjectorController()
{
    // 清理所有投影仪实例
    for (auto& pair : s_projections) {
        if (pair.second) {
            pair.second->CloseProjector();
            delete pair.second;
        }
    }
    s_projections.clear();
    DLPC350_USB_Exit();
}

ProjectorControlDll* ProjectorController::GetInstance(const std::string& sn)
{
    if (s_projections.find(sn) == s_projections.end()) {
        ProjectorControlDll* p = new ProjectorControlDll();
        p->m_serialNumber = sn;
        s_projections[sn] = p;
    }
    return s_projections[sn];
}

bool ProjectorController::loadDll(const std::string& dllPath)
{
    // 不再加载DLL，改为初始化USB
    return (DLPC350_USB_Init() == 0);
}

int ProjectorController::detectProjectors()
{
    return DLPC350_USB_UpdateConnectionList();
}

std::string ProjectorController::getProjectorSerial(int index)
{
    char buffer[64] = {0};
    if (DLPC350_USB_GetSerialAtIndex(index, buffer, 64) == 0) {
        return std::string(buffer);
    }
    return "";
}

bool ProjectorController::initProjector(const std::string& serialNum, int ledSetting)
{
    ProjectorControlDll* proj = GetInstance(serialNum);
    if (!proj) return false;

    proj->m_lrTag = false;  // 默认不翻转
    proj->m_udTag = false;
    proj->Activate();
    proj->SetProjectorUSB();
    proj->m_playTag = false;
    proj->m_openTag = true;
    return true;
}

bool ProjectorController::sendAndPlayProjector(const std::string& serialNum,
    int trigger, int projTag, int expose)
{
    ProjectorControlDll* proj = GetInstance(serialNum);
    if (!proj || !proj->m_openTag) return false;

    if (proj->m_playTag && trigger == proj->m_triggerTag &&
        projTag == proj->m_projTag && expose == proj->m_expose) {
        proj->PlayProjector();
    } else {
        if (!proj->SendProjector(trigger, projTag, expose)) return false;
        Sleep(400);
        if (!proj->ValidProjector()) return false;
        Sleep(300);
        proj->PlayProjector();
    }
    return true;
}

bool ProjectorController::pauseProjector(const std::string& serialNum)
{
    ProjectorControlDll* proj = GetInstance(serialNum);
    if (proj) {
        proj->PauseProjector();
        return true;
    }
    return false;
}

bool ProjectorController::closeProjector(const std::string& serialNum)
{
    ProjectorControlDll* proj = GetInstance(serialNum);
    if (proj) {
        proj->CloseProjector();
        return true;
    }
    return false;
}

bool ProjectorController::checkConnection(const std::string& serialNum)
{
    ProjectorControlDll* proj = GetInstance(serialNum);
    if (proj) {
        return proj->IsProjectorConnected();
    }
    return false;
}