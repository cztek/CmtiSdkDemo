#pragma once
#include <QString>
#include "CmtiSdk.h"

struct TSensorSetting
{
    QString SchemeName;
    QString VendorName;
    QString ChipName;
    int Mclk_kHz{ 0 };
    E_InterfaceType InterfaceType{ (E_InterfaceType)0 };
    int MipiFreq{ 0 };
    int Lanes{ 0 };
    int DataWidth{ 0 };
    int PclkPol{ 0 };
    int DataPol{ 0 };
    int HsyncPol{ 0 };
    int VsyncPol{ 0 };
    int Pwdn{ 0 };
    int Reset{ 0 };
    bool MipiClockMode{ false };

    E_ImageFormat ImageFormat{ (E_ImageFormat)0 };
    E_ImageMode ImageMode{ (E_ImageMode)0 };
    int PixelWidth{ 0 };
    int PixelHeight{ 0 };
    T_Rect CropParam{ 0 };

    T_I2CCommParam I2cParam;

    int FullModeParamCount{ 0 };
    T_RegConf* FullModeParams{ nullptr };
    int OtpInitParamCount{ 0 };
    T_RegConf* OtpInitParams{ nullptr };
    int SleepParamCount{ 0 };
    T_RegConf* SleepParams{ nullptr };
    int AfInitParamCount{ 0 };
    T_RegConf* AfInitParams{ nullptr };
    int AfAutoParamCount{ 0 };
    T_RegConf* AfAutoParams{ nullptr };
    int AfFarParamCount{ 0 };
    T_RegConf* AfFarParams{ nullptr };
    int AfNearParamCount{ 0 };
    T_RegConf* AfNearParams{ nullptr };
    int ExposureParamCount{ 0 };
    T_RegConf* ExposureParams{ nullptr };
    int GainParamCount{ 0 };
    T_RegConf* GainParams{ nullptr };
    int FlagRegisterCount{ 0 };
    T_RegConf* FlagRegisters{ nullptr };
    int PowerCount{ 0 };
    T_Power* Powers{ nullptr };

    TSensorSetting()
    {
        FullModeParamCount = 0;
        OtpInitParamCount = 0;
        SleepParamCount = 0;
        AfInitParamCount = 0;
        AfAutoParamCount = 0;
        AfFarParamCount = 0;
        AfNearParamCount = 0;
        ExposureParamCount = 0;
        GainParamCount = 0;
        FlagRegisterCount = 0;
        PowerCount = 0;
    }
    virtual ~TSensorSetting()
    {
        if (FullModeParamCount > 0)
            delete[] FullModeParams;
        if (OtpInitParamCount > 0)
            delete[] OtpInitParams;
        if (SleepParamCount > 0)
            delete[] SleepParams;
        if (AfInitParamCount > 0)
            delete[] AfInitParams;
        if (AfAutoParamCount > 0)
            delete[] AfAutoParams;
        if (AfFarParamCount > 0)
            delete[] AfFarParams;
        if (AfNearParamCount > 0)
            delete[] AfNearParams;
        if (ExposureParamCount > 0)
            delete[] ExposureParams;
        if (GainParamCount > 0)
            delete[] GainParams;
        if (FlagRegisterCount > 0)
            delete[] FlagRegisters;
        if (PowerCount > 0)
            delete[] Powers;
    }
};

/**
 * @brief 此类主要模拟从配置文件中取得的芯片必要参数，提供给Board类使用
*/
class IniFile;
class SensorSettingProvider
{
public:
    static SensorSettingProvider& Instance();
    SensorSettingProvider();
    virtual ~SensorSettingProvider();

    bool LoadSensorSettingFile(const QString& settingFileName);

public:
    TSensorSetting m_sensorSetting;

private:
    class Private;
    Private* const d;    
};

