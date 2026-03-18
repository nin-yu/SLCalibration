#include "ProjectorControlDll.h"
#include <iostream>
#include <iomanip>
#include <map>

#define MAX_NUM_RETRIES 5

static std::map<std::string, ProjectorControlDll*> g_ProjObjects;

static ProjectorControlDll* GetInstance(const char* sn) {
    if (!sn) return nullptr;
    std::string s(sn);
    if (g_ProjObjects.find(s) == g_ProjObjects.end()) {
        ProjectorControlDll* p = new ProjectorControlDll();
        p->m_serialNumber = s;
        g_ProjObjects[s] = p;
    }
    return g_ProjObjects[s];
}

ProjectorControlDll::ProjectorControlDll()
{

}

void ProjectorControlDll::Activate()
{
    DLPC350_USB_SelectBySerial(m_serialNumber.c_str());
}

void ProjectorControlDll::SetDLPC350InVideoMode()
{
    Activate();
    int i = 0;
    bool mode;
    unsigned int patMode;

    //Check if it is in Pattern Mode
    DLPC350_GetMode(&mode);
    if (mode == true)
    {
        //First stop pattern sequence
        DLPC350_GetPatternDisplay(&patMode);
        //if it is in PAUSE or RUN mode
        if (patMode != 0)
        {
            PauseProjector();
        }

        //Switch to Video Mode
        DLPC350_SetMode(false);
        Sleep(100);
        while (1)
        {
            DLPC350_GetMode(&mode);
            if (!mode)
                break;
            Sleep(100);
            if (i++ > MAX_NUM_RETRIES)
                break;
        }
    }

    return;
}

void ProjectorControlDll::SetDLPC350InPatternMode()
{
    Activate();
    int i = 0;
    bool mode;
    unsigned int patMode;

    //Check if it is in Pattern Mode
    DLPC350_GetMode(&mode);
    if (mode == false)
    {
        //Switch to Pattern Mode
        DLPC350_SetMode(true);
        Sleep(100);
        while (1)
        {
            DLPC350_GetMode(&mode);
            if (mode)
                break;
            Sleep(100);
            if (i++ > MAX_NUM_RETRIES)
                break;
        }
    }
    else
    {
        //First stop pattern sequence
        DLPC350_GetPatternDisplay(&patMode);
        //if it is in PAUSE or RUN mode
        if (patMode != 0)
        {
            PauseProjector();
        }
    }

    return;
}

bool ProjectorControlDll::SetProjectorUSB()
{
    Activate();

    int trigMode = 0;
    bool isExtPatDisplayMode = false;

    SetDLPC350InPatternMode();

    //Update all the settings under the page
    if (DLPC350_GetPatternTriggerMode(&trigMode) == 0)
    {
        if (trigMode <= 2)
        {
            if (DLPC350_GetPatternDisplayMode(&isExtPatDisplayMode) == 0)
            {
                /*
                if(isExtPatDisplayMode) //if set to external DVI/FPD port
                {
                    ui->radioButton_PatSeqSrcFrmVideoPort->setChecked(true);
                    emit on_radioButton_PatSeqSrcFrmVideoPort_clicked();
                }
                else
                {
                    ui->radioButton_PatSeqSrcFrmFlash->setChecked(true);
                    emit on_radioButton_PatSeqSrcFrmFlash_clicked();
                }
                */
            }
        }
    }

    return true;
}

bool ProjectorControlDll::SetProjectorHDMI()
{
    Activate();

    SetDLPC350InVideoMode();
    m_playTag = false;
    return true;
}

void debugPrint()
{
    bool curInvert = false;
    unsigned int curRising = 0;
    unsigned int curFalling = 0;

    // Get Config for TRIG_OUT_1 (Camera Trigger)
    if (DLPC350_GetTrigOutConfig(1, &curInvert, &curRising, &curFalling) == 0)
    {
        // Calculate delay in microseconds (1 bit = 107.2 ns)
        double delayUs = (double)curRising * 107.2 / 1000.0;

        std::cout << "[Projector] Trigger:      TRIG_OUT_1" << std::endl;
        std::cout << "[Projector] Inverted:     "
            << (curInvert ? "TRUE (Active Low)" : "FALSE (Active High)")
            << std::endl;

        std::cout << "[Projector] Rising Edge:  0x" << std::hex << std::uppercase << curRising << std::dec
            << " (" << curRising << ")" << std::endl;

        std::cout << "[Projector]  -> Delay:   "
            << std::fixed << std::setprecision(3) << delayUs
            << " microseconds" << std::endl;

        std::cout << "[Projector] Falling Edge: 0x" << std::hex << std::uppercase << curFalling << std::dec
            << " (" << curFalling << ")" << std::endl;
    }
    else
    {
        std::cerr << "=== ERROR: Could not read TrigOutConfig ===" << std::endl;
    }
}

bool ProjectorControlDll::SendProjector(int srcTrigger, int srcProjTag, int srcExpose)
{
    Activate();
    PauseProjector();

    unsigned char splashLut[64];
    int frameNumb = 0;
    int totalNumb = 0;

	debugPrint();
	// 1. Force LEDs to turn on EARLY (-20 microseconds)
	//    0x00 = -20.05us (Start early)
	//    0xBB =  0.00us (Stop on time)
    DLPC350_SetRedLEDStrobeDelay(0xBB, 0xBB);
    DLPC350_SetGreenLEDStrobeDelay(0xBB, 0xBB);
    DLPC350_SetBlueLEDStrobeDelay(0xBB, 0xBB);

    // 2. Force Camera Trigger to fire LATE (+7.3 microseconds)
    //    0xBB =  0.00us (Current setting, too fast)
    //    0xFF = +7.29us (Max possible delay)
    //    This tells the camera: "Wait 7us after the pattern starts before opening shutter."
    DLPC350_SetTrigOutConfig(1, 0, 0xFF, 0xBB);

    // 3. Ensure LEDs are enabled
    DLPC350_SetLedEnables(true, true, true, true);
	debugPrint();

    m_expose = srcExpose;
    m_triggerTag = srcTrigger;
    m_projTag = srcProjTag;


    //翻转设置 default：false
    DLPC350_SetLongAxisImageFlip(m_lrTag); //左右
    DLPC350_SetShortAxisImageFlip(m_udTag); //上下

    DLPC350_ClearPatLut();
    switch (m_projTag)      //0:ruler 1:cross 2:chess 3:shift 4:white 5-8:calibration
    {
    case 0://ruler
    {
        frameNumb = 1;
        totalNumb = 1;
        splashLut[0] = 4;
        //trigtype=0/1/2（内触发、外上升、外下降）, patnum=0/1/2（G\R\B）,bitdepth=8,ledselect=7,invertpat=0,insertblack=1,bufswap=0/1(和上一张一样为0),trigoutprew=0
        if (DLPC350_AddToPatLut(m_triggerTag, 1, 8, 7, 0, 1, 1, 0) < 0)
            return false;
        break;
    }
    case 1://cross
    {
        frameNumb = 1;
        totalNumb = 1;
        splashLut[0] = 4;
        //trigtype=0/1/2（内触发、外上升、外下降）, patnum=0/1/2（G\R\B）,bitdepth=8,ledselect=7,invertpat=0,insertblack=1,bufswap=0/1(和上一张一样为0),trigoutprew=0
        if (DLPC350_AddToPatLut(m_triggerTag, 2, 8, 7, 0, 1, 1, 0) < 0)
            return false;
        break;
    }
    case 2://chess
    {
        frameNumb = 1;
        totalNumb = 1;
        splashLut[0] = 4;
        //trigtype=0/1/2（内触发、外上升、外下降）, patnum=0/1/2（G\R\B）,bitdepth=8,ledselect=7,invertpat=0,insertblack=1,bufswap=0/1(和上一张一样为0),trigoutprew=0
        if (DLPC350_AddToPatLut(m_triggerTag, 0, 8, 7, 0, 1, 1, 0) < 0)
            return false;
        break;
    }
    case 3://shift
    {
        frameNumb = 2;
        totalNumb = 6;
        splashLut[0] = 0;
        splashLut[1] = 1;
        //trigtype=0/1/2（内触发、外上升、外下降）, patnum=0/1/2（G\R\B）,bitdepth=8,ledselect=7,invertpat=0,insertblack=1,bufswap=0/1(和上一张一样为0),trigoutprew=0
        if (DLPC350_AddToPatLut(m_triggerTag, 0, 8, 7, 0, 1, 1, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 1, 8, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 2, 8, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 0, 8, 7, 0, 1, 1, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 1, 8, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 2, 8, 7, 0, 1, 0, 0) < 0)
            return false;
        break;
    }
    case 4://white
    {
        frameNumb = 1;
        totalNumb = 1;
        splashLut[0] = 5;
        if (DLPC350_AddToPatLut(m_triggerTag, 7, 1, 7, 0, 1, 1, 0) < 0)
            return false;
        break;
    }
    case 5://cali
    {
        frameNumb = 2;
        totalNumb = 9;
        splashLut[0] = 5;
        splashLut[1] = 7;
        //white
        if (DLPC350_AddToPatLut(m_triggerTag, 7, 1, 7, 0, 1, 1, 0) < 0)
            return false;
        //GC_H 0 to 4
        if (DLPC350_AddToPatLut(m_triggerTag, 0, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 1, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 2, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 3, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 4, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        //PS_H 0 to 2
        if (DLPC350_AddToPatLut(m_triggerTag, 0, 8, 7, 0, 1, 1, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 1, 8, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 2, 8, 7, 0, 1, 0, 0) < 0)
            return false;
        break;
    }
    case 6://cali full
    {
        frameNumb = 4;
        totalNumb = 19;
        splashLut[0] = 5;
        splashLut[1] = 6;
        splashLut[2] = 7;
        splashLut[3] = 8;
        //white
        if (DLPC350_AddToPatLut(m_triggerTag, 7, 1, 7, 0, 1, 1, 0) < 0)
            return false;
        //GC_H 0 to 4
        if (DLPC350_AddToPatLut(m_triggerTag, 0, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 1, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 2, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 3, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 4, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        //GC_V 0 to 4
        if (DLPC350_AddToPatLut(m_triggerTag, 8, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 9, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 10, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 11, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 12, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        //PS_V 0 to 3
        if (DLPC350_AddToPatLut(m_triggerTag, 2, 8, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 0, 8, 7, 0, 1, 1, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 1, 8, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 2, 8, 7, 0, 1, 0, 0) < 0)
            return false;
        //PS_H 0 to 3
        if (DLPC350_AddToPatLut(m_triggerTag, 0, 8, 7, 0, 1, 1, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 1, 8, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 2, 8, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 0, 8, 7, 0, 1, 1, 0) < 0)
            return false;
        break;
    }
    case 7://sl gc+ps
    {
        frameNumb = 2;
        totalNumb = 9;
        splashLut[0] = 5;
        splashLut[1] = 6;
        //GC_V
        if (DLPC350_AddToPatLut(m_triggerTag, 8, 1, 7, 0, 1, 1, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 9, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 10, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 11, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 12, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        //PS_V
        if (DLPC350_AddToPatLut(m_triggerTag, 2, 8, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 0, 8, 7, 0, 1, 1, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 1, 8, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 2, 8, 7, 0, 1, 0, 0) < 0)
            return false;
        break;
    }
    case 8://sl white+black+gc+ps
    {
        frameNumb = 2;
        totalNumb = 11;
        splashLut[0] = 5;
        splashLut[1] = 6;
        //white
        if (DLPC350_AddToPatLut(m_triggerTag, 7, 1, 7, 0, 1, 1, 0) < 0)
            return false;
        //black
        if (DLPC350_AddToPatLut(m_triggerTag, 7, 1, 7, 1, 1, 0, 0) < 0)
            return false;
        //GC_V
        if (DLPC350_AddToPatLut(m_triggerTag, 8, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 9, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 10, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 11, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 12, 1, 7, 0, 1, 0, 0) < 0)
            return false;
        //PS_V
        if (DLPC350_AddToPatLut(m_triggerTag, 2, 8, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 0, 8, 7, 0, 1, 1, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 1, 8, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 2, 8, 7, 0, 1, 0, 0) < 0)
            return false;
        break;
    }
    default://shift+chess
    {
        frameNumb = 2;
        totalNumb = 4;
        splashLut[0] = 4;
        splashLut[1] = 5;
        //trigtype=0/1/2（内触发、外上升、外下降）, patnum=0/1/2（G\R\B）,bitdepth=8,ledselect=7,invertpat=0,insertblack=1,bufswap=0/1(和上一张一样为0),trigoutprew=0
        if (DLPC350_AddToPatLut(m_triggerTag, 0, 8, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 1, 8, 7, 0, 1, 1, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 0, 8, 7, 0, 1, 0, 0) < 0)
            return false;
        if (DLPC350_AddToPatLut(m_triggerTag, 1, 8, 7, 0, 1, 1, 0) < 0)
            return false;
        break;
    }

    }

    DLPC350_SetPatternDisplayMode(0);

    if (DLPC350_SetPatternConfig(totalNumb, 1, 1, frameNumb) < 0)
        return false;

    if (DLPC350_SetExposure_FramePeriod(m_expose, m_expose) < 0)
        return false;

    if (DLPC350_SetPatternTriggerMode(1) < 0)
        return false;

    if (DLPC350_SendPatLut() < 0)
        return false;

    if (DLPC350_SendImageLut(&splashLut[0], frameNumb) < 0)
        return false;

    return true;
}

bool ProjectorControlDll::ValidProjector()
{
    Activate();
    PauseProjector();

    if (DLPC350_StartPatLutValidate())
        return false;

    int i = 0;
    unsigned int status;
    bool ready;
    while (true)
    {
        if (DLPC350_CheckPatLutValidate(&ready, &status) < 0)
            return false;

        if (ready)
        {
            break;
        }
        else
        {
            Sleep(100);
        }

        if (i++ > MAX_NUM_RETRIES)
            break;
    };
    return true;
}

void ProjectorControlDll::PlayProjector()
{
    Activate();
    int i = 0;
    unsigned int patMode;

    DLPC350_PatternDisplay(2);

    Sleep(100);

    while (1)
    {
        DLPC350_GetPatternDisplay(&patMode);
        if (patMode == 2)
            break;
        else
            DLPC350_PatternDisplay(2);
        Sleep(100);

        if (i++ > MAX_NUM_RETRIES)
            break;
    }
    m_playTag = true;
}

void ProjectorControlDll::PauseProjector()
{
    Activate();
    int i = 0;
    unsigned int patMode;

    DLPC350_PatternDisplay(0);
    Sleep(100);
    while (1)
    {
        DLPC350_GetPatternDisplay(&patMode);
        if (patMode == 0)
            break;
        else
            DLPC350_PatternDisplay(0);
        Sleep(100);
        if (i++ > MAX_NUM_RETRIES)
            break;
    }
}

void ProjectorControlDll::CloseProjector()
{
    Activate();
    if (m_openTag)
    {
        PauseProjector();
        DLPC350_USB_Close();
        m_openTag = false;
    }
}

int ProjectorControlDll::IsProjectorConnected()
{
    Activate();

    if (!DLPC350_USB_IsConnected())
    {
        return false;
    }

    bool mode;
    int result = DLPC350_GetMode(&mode);

    if (result == 0)
    {
        return true;
    }
    else
    {
        m_openTag = false;
        return false;
    }
}

bool ProjectorControlDll::SetLedBrightness(int brightnessPercent)
{
    Activate();

    // 将百分比 (0-100) 转换为 LED 电流值 (0-255)
    unsigned char currentValue = static_cast<unsigned char>(brightnessPercent * 255 / 100);

    // 设置三色 LED 电流值
    if (DLPC350_SetLedCurrents(currentValue, currentValue, currentValue) == 0)
    {
        return true;
    }
    return false;
}

int ProjectorControlDll::GetLedBrightness() const
{
    // 由于 Activate() 不是 const 方法，这里需要 const_cast
    ProjectorControlDll* nonConstThis = const_cast<ProjectorControlDll*>(this);
    nonConstThis->Activate();

    unsigned char redCurrent = 0;
    unsigned char greenCurrent = 0;
    unsigned char blueCurrent = 0;

    if (DLPC350_GetLedCurrents(&redCurrent, &greenCurrent, &blueCurrent) == 0)
    {
        // 返回平均值并转换为百分比
        unsigned char avgCurrent = (redCurrent + greenCurrent + blueCurrent) / 3;
        return (avgCurrent * 100) / 255;
    }
    return -1;
}


