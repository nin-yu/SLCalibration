#pragma once
#include "MvCamera/MvCameraControl.h"
#include <string>
#include <vector>
#include <map>
#include <memory>

// Camera info structure
struct CameraInfo
{
    std::string serialNumber;
    std::string modelName;
    std::string vendorName;
    void* handle;
    bool isConnected;
    bool isCapturing;
    
    CameraInfo() : handle(nullptr), isConnected(false), isCapturing(false) {}
};

// Image callback function type
typedef void(*ImageCallbackFunc)(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);

class CCameraController
{
public:
    CCameraController();
    ~CCameraController();

    // Camera management interfaces
    void* OpenCameraBySN(const char* serialNumber);
    bool CloseCamera(void* cameraHandle);
    bool GetCameraInfo(void* cameraHandle, CameraInfo& info);
    std::vector<std::string> EnumerateCameras();

    // Parameter setting interfaces
    bool SetExposureTime(void* cameraHandle, float exposureTimeUs);
    bool GetExposureTime(void* cameraHandle, float& exposureTimeUs);
    bool SetTriggerMode(void* cameraHandle, int triggerMode);
    bool GetTriggerMode(void* cameraHandle, int& triggerMode);
    bool SetTriggerSource(void* cameraHandle, const char* source);
    bool GetTriggerSource(void* cameraHandle, char* source, int bufferSize);
    bool SetTriggerActivation(void* cameraHandle, unsigned int activation);
    bool GetTriggerActivation(void* cameraHandle, unsigned int& activation);
    bool SetGain(void* cameraHandle, float gain);
    bool GetGain(void* cameraHandle, float& gain);

    // Image capture control interfaces
    bool StartImageCapture(void* cameraHandle, ImageCallbackFunc callback = nullptr, void* userData = nullptr);
    bool StopImageCapture(void* cameraHandle);
    bool GetImage(void* cameraHandle, unsigned char* pBuffer, unsigned int bufferSize, MV_FRAME_OUT_INFO_EX* pFrameInfo);
    bool IsCapturing(void* cameraHandle);

    // Software trigger interface
    bool TriggerSoftware(void* cameraHandle);

    // Camera status query interfaces
    bool IsCameraConnected(void* cameraHandle);
    int GetCameraCount() const;
    std::vector<void*> GetAllCameraHandles() const;

private:
    void* FindCameraHandleBySN(const std::string& serialNumber);
    bool IsCameraHandleValid(void* handle);
    void CleanupCameraResources(void* handle);
    bool InitializeMVSdk();
    void UninitializeMVSdk();
    std::string GetLastErrorMessage(int errorCode) const;

private:
    std::map<void*, std::shared_ptr<CameraInfo>> m_cameraMap;
    bool m_bSdkInitialized;
    unsigned int m_nCameraCounter;
    std::map<int, std::string> m_errorMessages;
    void InitializeErrorMessages();
};