#include "CameraController.h"
#include <iostream>
#include <algorithm>
#include <cstring>

CCameraController::CCameraController()
    : m_bSdkInitialized(false)
    , m_nCameraCounter(0)
{
    InitializeMVSdk();
    InitializeErrorMessages();
}

CCameraController::~CCameraController()
{
    for (auto& pair : m_cameraMap) {
        if (pair.second && pair.second->handle) {
            StopImageCapture(pair.second->handle);
            CloseCamera(pair.second->handle);
        }
    }
    m_cameraMap.clear();
    UninitializeMVSdk();
}

bool CCameraController::InitializeMVSdk()
{
    if (m_bSdkInitialized) {
        return true;
    }
    int nRet = MV_CC_Initialize();
    if (MV_OK != nRet) {
        std::cerr << "Initialize MV SDK failed! nRet = " << nRet << std::endl;
        return false;
    }
    m_bSdkInitialized = true;
    return true;
}

void CCameraController::UninitializeMVSdk()
{
    if (m_bSdkInitialized) {
        MV_CC_Finalize();
        m_bSdkInitialized = false;
    }
}

void CCameraController::InitializeErrorMessages()
{
    m_errorMessages[MV_OK] = "Success";
    m_errorMessages[MV_E_HANDLE] = "Error or invalid handle";
    m_errorMessages[MV_E_SUPPORT] = "Not supported function";
    m_errorMessages[MV_E_BUFOVER] = "Cache is full";
    m_errorMessages[MV_E_CALLORDER] = "Function calling order error";
    m_errorMessages[MV_E_PARAMETER] = "Incorrect parameter";
    m_errorMessages[MV_E_RESOURCE] = "Applying resource failed";
    m_errorMessages[MV_E_NODATA] = "No data";
    m_errorMessages[MV_E_PRECONDITION] = "Precondition error";
    m_errorMessages[MV_E_VERSION] = "Version mismatches";
    m_errorMessages[MV_E_NOENOUGH_BUF] = "Insufficient memory";
    m_errorMessages[MV_E_ABNORMAL_IMAGE] = "Abnormal image";
    m_errorMessages[MV_E_LOAD_LIBRARY] = "Failed to load dynamic library";
    m_errorMessages[MV_E_NOOUTBUF] = "No Available Buffer";
    m_errorMessages[MV_E_UNKNOW] = "Unknown error";
}

std::string CCameraController::GetLastErrorMessage(int errorCode) const
{
    auto it = m_errorMessages.find(errorCode);
    if (it != m_errorMessages.end()) {
        return it->second;
    }
    return "Unknown error code: " + std::to_string(errorCode);
}

void* CCameraController::OpenCameraBySN(const char* serialNumber)
{
    if (!serialNumber || strlen(serialNumber) == 0) {
        std::cerr << "Invalid serial number!" << std::endl;
        return nullptr;
    }

    void* existingHandle = FindCameraHandleBySN(serialNumber);
    if (existingHandle) {
        std::cout << "Camera with SN " << serialNumber << " is already opened!" << std::endl;
        return existingHandle;
    }

    MV_CC_DEVICE_INFO_LIST stDeviceList = {0};
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        std::cerr << "Enum devices failed! nRet = " << nRet << std::endl;
        return nullptr;
    }

    void* targetDevice = nullptr;
    MV_CC_DEVICE_INFO* pTargetDeviceInfo = nullptr;
    
    for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
        MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
        if (nullptr == pDeviceInfo) {
            continue;
        }

        std::string deviceSN;
        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            deviceSN = reinterpret_cast<const char*>(pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber);
        }
        else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
            deviceSN = reinterpret_cast<const char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        }

        if (deviceSN == serialNumber) {
            targetDevice = stDeviceList.pDeviceInfo[i];
            pTargetDeviceInfo = pDeviceInfo;
            break;
        }
    }

    if (!targetDevice || !pTargetDeviceInfo) {
        std::cerr << "Device with SN " << serialNumber << " not found!" << std::endl;
        return nullptr;
    }

    void* handle = nullptr;
    nRet = MV_CC_CreateHandle(&handle, pTargetDeviceInfo);
    if (MV_OK != nRet) {
        std::cerr << "Create handle failed! nRet = " << nRet << std::endl;
        return nullptr;
    }

    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
        std::cerr << "Open device failed! SN = " << serialNumber << ", nRet = " << nRet << std::endl;
        MV_CC_DestroyHandle(handle);
        return nullptr;
    }

    auto cameraInfo = std::make_shared<CameraInfo>();
    cameraInfo->serialNumber = serialNumber;
    cameraInfo->handle = handle;
    cameraInfo->isConnected = true;
    
    if (pTargetDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
        if (pTargetDeviceInfo->SpecialInfo.stGigEInfo.chModelName[0] != '\0') {
            cameraInfo->modelName = reinterpret_cast<const char*>(pTargetDeviceInfo->SpecialInfo.stGigEInfo.chModelName);
        }
        if (pTargetDeviceInfo->SpecialInfo.stGigEInfo.chManufacturerName[0] != '\0') {
            cameraInfo->vendorName = reinterpret_cast<const char*>(pTargetDeviceInfo->SpecialInfo.stGigEInfo.chManufacturerName);
        }
    }
    else if (pTargetDeviceInfo->nTLayerType == MV_USB_DEVICE) {
        if (pTargetDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName[0] != '\0') {
            cameraInfo->modelName = reinterpret_cast<const char*>(pTargetDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName);
        }
        if (pTargetDeviceInfo->SpecialInfo.stUsb3VInfo.chVendorName[0] != '\0') {
            cameraInfo->vendorName = reinterpret_cast<const char*>(pTargetDeviceInfo->SpecialInfo.stUsb3VInfo.chVendorName);
        }
    }

    m_cameraMap[handle] = cameraInfo;
    m_nCameraCounter++;

    std::cout << "Successfully opened camera: " << serialNumber << std::endl;
    return handle;
}

bool CCameraController::CloseCamera(void* cameraHandle)
{
    if (!cameraHandle) {
        return false;
    }

    auto it = m_cameraMap.find(cameraHandle);
    if (it == m_cameraMap.end()) {
        std::cerr << "Camera handle not found!" << std::endl;
        return false;
    }

    CleanupCameraResources(cameraHandle);
    
    int nRet = MV_CC_DestroyHandle(cameraHandle);
    if (MV_OK != nRet) {
        std::cerr << "Destroy handle failed! nRet = " << nRet << std::endl;
    }

    m_cameraMap.erase(it);
    std::cout << "Successfully closed camera" << std::endl;
    return true;
}

bool CCameraController::GetCameraInfo(void* cameraHandle, CameraInfo& info)
{
    if (!cameraHandle) {
        return false;
    }

    auto it = m_cameraMap.find(cameraHandle);
    if (it == m_cameraMap.end()) {
        return false;
    }

    info = *(it->second);
    return true;
}

std::vector<std::string> CCameraController::EnumerateCameras()
{
    std::vector<std::string> serialNumbers;
    
    MV_CC_DEVICE_INFO_LIST stDeviceList = {0};
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        std::cerr << "Enum devices failed! nRet = " << nRet << std::endl;
        return serialNumbers;
    }

    for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
        MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
        if (nullptr == pDeviceInfo) {
            continue;
        }

        std::string sn;
        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            sn = reinterpret_cast<const char*>(pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber);
        }
        else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
            sn = reinterpret_cast<const char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        }

        if (!sn.empty()) {
            serialNumbers.push_back(sn);
        }
    }

    return serialNumbers;
}

void* CCameraController::FindCameraHandleBySN(const std::string& serialNumber)
{
    for (const auto& pair : m_cameraMap) {
        if (pair.second && pair.second->serialNumber == serialNumber) {
            return pair.first;
        }
    }
    return nullptr;
}

bool CCameraController::IsCameraHandleValid(void* handle)
{
    return m_cameraMap.find(handle) != m_cameraMap.end();
}

void CCameraController::CleanupCameraResources(void* handle)
{
    if (!handle) return;
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
}

bool CCameraController::SetExposureTime(void* cameraHandle, float exposureTimeUs)
{
    if (!cameraHandle || !IsCameraHandleValid(cameraHandle)) {
        return false;
    }

    int nRet = MV_CC_SetFloatValue(cameraHandle, "ExposureTime", exposureTimeUs);
    if (MV_OK != nRet) {
        std::cerr << "Set exposure time failed! nRet = " << nRet << ", Message: " << GetLastErrorMessage(nRet) << std::endl;
        return false;
    }

    std::cout << "Successfully set exposure time to " << exposureTimeUs << " us" << std::endl;
    return true;
}

bool CCameraController::GetExposureTime(void* cameraHandle, float& exposureTimeUs)
{
    if (!cameraHandle || !IsCameraHandleValid(cameraHandle)) {
        return false;
    }

    MVCC_FLOATVALUE stFloatValue = {0};
    int nRet = MV_CC_GetFloatValue(cameraHandle, "ExposureTime", &stFloatValue);
    if (MV_OK != nRet) {
        std::cerr << "Get exposure time failed! nRet = " << nRet << std::endl;
        return false;
    }

    exposureTimeUs = stFloatValue.fCurValue;
    return true;
}

bool CCameraController::SetTriggerMode(void* cameraHandle, int triggerMode)
{
    if (!cameraHandle || !IsCameraHandleValid(cameraHandle)) {
        return false;
    }

    int nRet = MV_CC_SetEnumValue(cameraHandle, "TriggerMode", triggerMode);
    if (MV_OK != nRet) {
        std::cerr << "Set trigger mode failed! nRet = " << nRet << std::endl;
        return false;
    }

    std::string modeStr = (triggerMode == 0) ? "Continuous" : "Trigger";
    std::cout << "Successfully set trigger mode to " << modeStr << std::endl;
    return true;
}

bool CCameraController::GetTriggerMode(void* cameraHandle, int& triggerMode)
{
    if (!cameraHandle || !IsCameraHandleValid(cameraHandle)) {
        return false;
    }

    MVCC_ENUMVALUE stEnumValue = {0};
    int nRet = MV_CC_GetEnumValue(cameraHandle, "TriggerMode", &stEnumValue);
    if (MV_OK != nRet) {
        std::cerr << "Get trigger mode failed! nRet = " << nRet << std::endl;
        return false;
    }

    triggerMode = stEnumValue.nCurValue;
    return true;
}

bool CCameraController::SetTriggerSource(void* cameraHandle, const char* source)
{
    if (!cameraHandle || !source || !IsCameraHandleValid(cameraHandle)) {
        return false;
    }

    int nRet = MV_CC_SetEnumValueByString(cameraHandle, "TriggerSource", source);
    if (MV_OK != nRet) {
        std::cerr << "Set trigger source failed! Source = " << source << ", nRet = " << nRet << std::endl;
        return false;
    }

    std::cout << "Successfully set trigger source to " << source << std::endl;
    return true;
}

bool CCameraController::GetTriggerSource(void* cameraHandle, char* source, int bufferSize)
{
    if (!cameraHandle || !source || !IsCameraHandleValid(cameraHandle)) {
        return false;
    }

    MVCC_ENUMVALUE stEnumValue = {0};
    int nRet = MV_CC_GetEnumValue(cameraHandle, "TriggerSource", &stEnumValue);
    if (MV_OK != nRet) {
        std::cerr << "Get trigger source failed! nRet = " << nRet << std::endl;
        return false;
    }

    snprintf(source, bufferSize, "%u", stEnumValue.nCurValue);
    return true;
}

bool CCameraController::SetTriggerActivation(void* cameraHandle, unsigned int activation)
{
    if (!cameraHandle || !IsCameraHandleValid(cameraHandle) || activation > 4) {
        return false;
    }

    int nRet = MV_CC_SetEnumValue(cameraHandle, "TriggerActivation", activation);
    if (MV_OK != nRet) {
        std::cerr << "Set trigger activation failed! Activation = " << activation << ", nRet = " << nRet << std::endl;
        return false;
    }

    std::cout << "Successfully set trigger activation to " << activation << std::endl;
    return true;
}

bool CCameraController::GetTriggerActivation(void* cameraHandle, unsigned int& activation)
{
    if (!cameraHandle || !IsCameraHandleValid(cameraHandle)) {
        return false;
    }

    MVCC_ENUMVALUE stEnumValue = {0};
    int nRet = MV_CC_GetEnumValue(cameraHandle, "TriggerActivation", &stEnumValue);
    if (MV_OK != nRet) {
        std::cerr << "Get trigger activation failed! nRet = " << nRet << std::endl;
        return false;
    }

    activation = stEnumValue.nCurValue;
    return true;
}

bool CCameraController::SetGain(void* cameraHandle, float gain)
{
    if (!cameraHandle || !IsCameraHandleValid(cameraHandle)) {
        return false;
    }

    int nRet = MV_CC_SetFloatValue(cameraHandle, "Gain", gain);
    if (MV_OK != nRet) {
        std::cerr << "Set gain failed! Gain = " << gain << ", nRet = " << nRet << std::endl;
        return false;
    }

    std::cout << "Successfully set gain to " << gain << std::endl;
    return true;
}

bool CCameraController::GetGain(void* cameraHandle, float& gain)
{
    if (!cameraHandle || !IsCameraHandleValid(cameraHandle)) {
        return false;
    }

    MVCC_FLOATVALUE stFloatValue = {0};
    int nRet = MV_CC_GetFloatValue(cameraHandle, "Gain", &stFloatValue);
    if (MV_OK != nRet) {
        std::cerr << "Get gain failed! nRet = " << nRet << std::endl;
        return false;
    }

    gain = stFloatValue.fCurValue;
    return true;
}

bool CCameraController::StartImageCapture(void* cameraHandle, ImageCallbackFunc callback, void* userData)
{
    if (!cameraHandle || !IsCameraHandleValid(cameraHandle)) {
        return false;
    }

    auto it = m_cameraMap.find(cameraHandle);
    if (it != m_cameraMap.end() && it->second->isCapturing) {
        std::cout << "Camera is already capturing" << std::endl;
        return true;
    }
    
    int nRet;
    
    if (callback != nullptr) {
        nRet = MV_CC_RegisterImageCallBackEx(cameraHandle, 
            [](unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser) {
                ImageCallbackFunc userCallback = reinterpret_cast<ImageCallbackFunc>(pUser);
                if (userCallback) {
                    userCallback(pData, pFrameInfo, pUser);
                }
            }, callback);
        
        if (MV_OK != nRet) {
            std::cerr << "Register image callback failed! nRet = " << nRet << std::endl;
            return false;
        }
        
        std::cout << "Registered image callback mode" << std::endl;
    }
    
    nRet = MV_CC_StartGrabbing(cameraHandle);
    if (MV_OK != nRet) {
        std::cerr << "Start grabbing failed! nRet = " << nRet << std::endl;
        return false;
    }

    if (it != m_cameraMap.end()) {
        it->second.get()->isCapturing = true;
    }

    std::cout << "Successfully started image capture" << std::endl;
    return true;
}

bool CCameraController::StopImageCapture(void* cameraHandle)
{
    if (!cameraHandle || !IsCameraHandleValid(cameraHandle)) {
        return false;
    }

    auto it = m_cameraMap.find(cameraHandle);
    if (it != m_cameraMap.end() && !it->second->isCapturing) {
        std::cout << "Camera is not capturing" << std::endl;
        return true;
    }
    
    int nRet = MV_CC_StopGrabbing(cameraHandle);
    if (MV_OK != nRet) {
        std::cerr << "Stop grabbing failed! nRet = " << nRet << std::endl;
        return false;
    }

    if (it != m_cameraMap.end()) {
        it->second.get()->isCapturing = false;
    }

    std::cout << "Successfully stopped image capture" << std::endl;
    return true;
}

bool CCameraController::GetImage(void* cameraHandle, unsigned char* pBuffer, unsigned int bufferSize, MV_FRAME_OUT_INFO_EX* pFrameInfo)
{
    if (!cameraHandle || !pBuffer || !IsCameraHandleValid(cameraHandle)) {
        return false;
    }

    int nRet = MV_CC_GetOneFrameTimeout(cameraHandle, pBuffer, bufferSize, pFrameInfo, 1000);
    if (MV_OK != nRet) {
        if (nRet != MV_E_NODATA) {
            std::cerr << "Get one frame timeout failed! nRet = " << nRet << std::endl;
        }
        return false;
    }

    return true;
}

bool CCameraController::GetImage(void* cameraHandle, unsigned char* pBuffer, unsigned int bufferSize, MV_FRAME_OUT_INFO_EX* pFrameInfo, unsigned int timeoutMs)
{
    if (!cameraHandle || !pBuffer || !IsCameraHandleValid(cameraHandle)) {
        return false;
    }

    int nRet = MV_CC_GetOneFrameTimeout(cameraHandle, pBuffer, bufferSize, pFrameInfo, timeoutMs);
    if (MV_OK != nRet) {
        if (nRet != MV_E_NODATA) {
            std::cerr << "Get one frame timeout failed! nRet = " << nRet << ", timeoutMs = " << timeoutMs << std::endl;
        }
        return false;
    }

    return true;
}

bool CCameraController::IsCapturing(void* cameraHandle)
{
    if (!cameraHandle || !IsCameraHandleValid(cameraHandle)) {
        return false;
    }

    auto it = m_cameraMap.find(cameraHandle);
    if (it != m_cameraMap.end()) {
        return it->second.get()->isCapturing;
    }
    return false;
}

bool CCameraController::TriggerSoftware(void* cameraHandle)
{
    if (!cameraHandle || !IsCameraHandleValid(cameraHandle)) {
        return false;
    }

    int nRet = MV_CC_SetCommandValue(cameraHandle, "TriggerSoftware");
    if (MV_OK != nRet) {
        std::cerr << "Software trigger failed! nRet = " << nRet << std::endl;
        return false;
    }

    std::cout << "Software trigger sent successfully" << std::endl;
    return true;
}

bool CCameraController::IsCameraConnected(void* cameraHandle)
{
    if (!cameraHandle) {
        return false;
    }

    auto it = m_cameraMap.find(cameraHandle);
    if (it != m_cameraMap.end()) {
        return it->second.get()->isConnected;
    }
    return false;
}

int CCameraController::GetCameraCount() const
{
    return static_cast<int>(m_cameraMap.size());
}

std::vector<void*> CCameraController::GetAllCameraHandles() const
{
    std::vector<void*> handles;
    for (const auto& pair : m_cameraMap) {
        handles.push_back(pair.first);
    }
    return handles;
}