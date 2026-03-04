#pragma once

#include "dlpc350_common.h"
#include "dlpc350_usb.h"
#include "dlpc350_api.h"
#include "dlpc350_firmware.h"
#include <windows.h>
#include <chrono>
#include <string>

class ProjectorControlDll
{
public :
	ProjectorControlDll();

	void Activate();
	void SetDLPC350InVideoMode();
	void SetDLPC350InPatternMode();
	bool SetProjectorUSB();
	bool SetProjectorHDMI();
	bool SendProjector(int srcTrigger, int srcProjTag, int srcExpose);
	bool ValidProjector();
	void PlayProjector();
	void PauseProjector();
	void CloseProjector();
	int  IsProjectorConnected();

	std::string m_serialNumber;
	int m_projIdx;
	int m_expose;
	int m_triggerTag;
	int m_projTag;
	bool m_playTag;
	bool m_openTag;
	bool m_lrTag;
	bool m_udTag;
};

