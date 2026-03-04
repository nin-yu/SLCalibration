/*
 * dlpc350_usb.h
 *
 * This module has the wrapper functions to access USB driver functions.
 *
 * Copyright (C) {2015} Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/


#ifndef DLPC350_USB_H
#define DLPC350_USB_H

#ifdef __cplusplus
extern "C" {
#endif

#define USB_MIN_PACKET_SIZE 64
#define USB_MAX_PACKET_SIZE 64

#define MY_VID 0x0451
#define MY_PID 0x6401

	// 初始化与退出
	int DLPC350_USB_Init(void);
	int DLPC350_USB_Exit(void);

	// === 核心改造接口 ===

	// 1. 扫描所有连接的投影仪，返回数量
	// 这个函数会打开所有设备的句柄并缓存起来
	int DLPC350_USB_UpdateConnectionList();

	// 2. 获取第 index 个设备的序列号 (用于返回给你的上位机)
	// return: 0 成功, -1 失败
	int DLPC350_USB_GetSerialAtIndex(int index, char* outSerialBuffer, int maxLen);

	// 3. 根据序列号选中当前要操作的设备
	// 选中后，后续的 Write/Read 都会发给这台设备
	// return: 0 成功, -1 找不到
	int DLPC350_USB_SelectBySerial(const char* serialNumber);

	// === 标准读写接口 (操作当前选中的设备) ===
	int DLPC350_USB_IsConnected();
	int DLPC350_USB_Write();
	int DLPC350_USB_Read();
	int DLPC350_USB_Close(); // 只是逻辑关闭当前选中的

#ifdef __cplusplus
}
#endif

#endif // DLPC350_USB_H