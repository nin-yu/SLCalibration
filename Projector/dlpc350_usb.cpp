#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <map>
#include <cstring>
#include "dlpc350_usb.h"
#include "hidapi.h"
#include "dlpc350_common.h" // 必须包含：API 依赖的类型定义
#include "dlpc350_api.h"    // 必须包含：用于调用 DLPC350_GetFirmwareTagInfo

// =========================================================================
// 全局变量定义
// =========================================================================
unsigned char g_InputBuffer[USB_MAX_PACKET_SIZE + 1];
unsigned char g_OutputBuffer[USB_MAX_PACKET_SIZE + 1];

// 序列号 -> 设备句柄 映射表
static std::map<std::string, hid_device*> g_DeviceMap;
// 序列号列表
static std::vector<std::string> g_SerialList;
// 当前选中的活跃句柄
static hid_device* CurrentDeviceHandle = NULL;

// 辅助函数：宽字符转 string
static std::string WCharToString(const wchar_t* wstr) {
    if (!wstr) return "";
    std::string str;
    size_t len = wcslen(wstr);
    for (size_t i = 0; i < len; i++) {
        str += (char)wstr[i];
    }
    return str;
}

int DLPC350_USB_Init(void) {
    return hid_init();
}

int DLPC350_USB_Exit(void) {
    for (auto& pair : g_DeviceMap) {
        if (pair.second) hid_close(pair.second);
    }
    g_DeviceMap.clear();
    g_SerialList.clear();
    CurrentDeviceHandle = NULL;
    return hid_exit();
}

// =========================================================================
// 核心：扫描并连接所有设备 (调用 DLPC350_GetFirmwareTagInfo 获取信息)
// =========================================================================
int DLPC350_USB_UpdateConnectionList() {
    // 1. 清理旧连接
    for (auto& pair : g_DeviceMap) {
        if (pair.second) hid_close(pair.second);
    }
    g_DeviceMap.clear();
    g_SerialList.clear();
    CurrentDeviceHandle = NULL;

    // 2. 枚举设备
    struct hid_device_info* devs, * cur_dev;
    devs = hid_enumerate(MY_VID, MY_PID);
    cur_dev = devs;

    while (cur_dev) {
        if (cur_dev->serial_number) {
            std::string original_sn = WCharToString(cur_dev->serial_number);

            // --- 打开设备 ---
            hid_device* handle = hid_open_path(cur_dev->path);

            if (handle) {
                // 【关键修改】临时劫持 CurrentDeviceHandle
                // 因为 DLPC350_GetFirmwareTagInfo 内部会调用 DLPC350_USB_Write
                // 而 DLPC350_USB_Write 依赖 CurrentDeviceHandle
                hid_device* tempBackup = CurrentDeviceHandle;
                CurrentDeviceHandle = handle;

                // --- 调用现有的 API ---
                unsigned char tagBuffer[33]; // 32 chars + null
                memset(tagBuffer, 0, 33);

                std::string finalTag = "Unknown";

                // 直接调用你提供的 API 函数
                if (DLPC350_GetFirmwareTagInfo(tagBuffer) == 0) {
                    // 成功获取，转换为 string
                    // 确保是合法的字符串 (过滤掉乱码)
                    finalTag = "";
                    for (int i = 0; i < 32; i++) {
                        if (tagBuffer[i] == '\0') break;
                        if (tagBuffer[i] >= 32 && tagBuffer[i] <= 126) {
                            finalTag += (char)tagBuffer[i];
                        }
                    }
                    if (finalTag.empty()) finalTag = "EmptyTag";
                }

                // --- 恢复现场 ---
                CurrentDeviceHandle = tempBackup; // (通常这里是NULL，为了严谨恢复一下)

                // --- 拼接结果 (LCR2@v3.0.0) ---
                // 使用 original_sn@tag 作为 key，保持 prefix 不被修改。
                std::string storedKey = original_sn + "@" + finalTag;

                // 处理极少数情况下完全相同的 storedKey 冲突（两台设备 serial+tag 都相同）
                // 为了不修改 prefix，我们在 tag 后追加 _1, _2 ...
                std::string uniqueKey = storedKey;
                int counter = 1;
                while (g_DeviceMap.find(uniqueKey) != g_DeviceMap.end()) {
                    uniqueKey = storedKey + "_" + std::to_string(counter++);
                }

                // 存入 Map，key 为 uniqueKey（通常等于 storedKey）
                g_DeviceMap[uniqueKey] = handle;
                g_SerialList.push_back(uniqueKey);
            }
        }
        cur_dev = cur_dev->next;
    }
    hid_free_enumeration(devs);

    // 确保 CurrentDeviceHandle 归零，避免指向未 Select 的设备
    CurrentDeviceHandle = NULL;

    return (int)g_SerialList.size();
}

int DLPC350_USB_GetSerialAtIndex(int index, char* outSerialBuffer, int maxLen) {
    if (index < 0 || index >= (int)g_SerialList.size()) return -1;

    // 这里返回的是带 Tag 的长字符串 "LCR2@v3.0.0"
    std::string sn = g_SerialList[index];
    if (sn.length() >= (size_t)maxLen) return -1;
    strcpy(outSerialBuffer, sn.c_str());
    return 0;
}

int DLPC350_USB_SelectBySerial(const char* serialNumber) {
    if (!serialNumber) return -1;
    std::string sn(serialNumber);

    // 兼容处理：如果传入的是完整的 "LCR2@v3.0.0"，直接查找
    size_t atPos = sn.find('@');
    if (atPos != std::string::npos) {
        if (g_DeviceMap.find(sn) != g_DeviceMap.end()) {
            CurrentDeviceHandle = g_DeviceMap[sn];
            return 0;
        }
    } else {
        // 如果只提供短名 "LCR2"，选择第一个以 "LCR2@" 前缀开头的条目
        std::string prefix = sn + "@";
        for (const auto& p : g_DeviceMap) {
            if (p.first.compare(0, prefix.size(), prefix) == 0) {
                CurrentDeviceHandle = p.second;
                return 0;
            }
        }
    }

    CurrentDeviceHandle = NULL;
    return -1;
}

int DLPC350_USB_IsConnected() {
    return (CurrentDeviceHandle != NULL) ? 1 : 0;
}

int DLPC350_USB_Write() {
    if (CurrentDeviceHandle == NULL) return -1;
    return hid_write(CurrentDeviceHandle, g_OutputBuffer, USB_MIN_PACKET_SIZE + 1);
}

int DLPC350_USB_Read() {
    if (CurrentDeviceHandle == NULL) return -1;
    memset(g_InputBuffer, 0, USB_MIN_PACKET_SIZE + 1);
    return hid_read_timeout(CurrentDeviceHandle, g_InputBuffer, USB_MIN_PACKET_SIZE + 1, 2000);
}

int DLPC350_USB_Close() {
    CurrentDeviceHandle = NULL;
    return 0;
}