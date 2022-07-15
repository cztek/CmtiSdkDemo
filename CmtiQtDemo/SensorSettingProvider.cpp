#include "SensorSettingProvider.h"
#include "IniFile.h"

class SensorSettingProvider::Private
{
public:
    static std::string& ltrim(std::string& s)
    {
        if (s.empty())
            return s;
        const char EMPTY_CHARS[] = { '\t', '\n', '\v', '\f', '\r','\x20' };
        bool bFind = true;
        while (bFind)
        {
            int pos = 0;
            for (unsigned int i = 0; i < sizeof(EMPTY_CHARS) / sizeof(char); i++)
            {
                int t = (int)s.find_first_not_of(EMPTY_CHARS[i]);
                if (t > pos)
                    pos = t;
            }
            if (pos == 0)
                bFind = false;
            s.erase(0, pos);
        }
        return s;
    }

    static std::string& rtrim(std::string& s)
    {
        if (s.empty())
            return s;
        const char EMPTY_CHARS[] = { '\t', '\n', '\v', '\f', '\r','\x20' };
        bool bFind = true;
        while (bFind)
        {
            size_t pos = s.size() - 1;
            for (size_t i = 0; i < sizeof(EMPTY_CHARS) / sizeof(char); i++)
            {
                int t = (int)s.find_last_not_of(EMPTY_CHARS[i]);
                if (t < pos)
                    pos = t;
            }
            if (pos == s.size() - 1)
                bFind = false;
            s.erase(pos + 1);
        }
        return s;
    }

    static std::string& trim(std::string& s)
    {
        s = rtrim(ltrim(s));
        return s;
    }

    static std::vector<std::string> split(const std::string& str, const std::string& spliter, bool skipEmptyParts)
    {
        std::vector<std::string> slTemp;
        std::string strTemp = str;
        std::string::size_type pos;
        do
        {
            pos = strTemp.find(spliter);
            std::string strSection = strTemp.substr(0, pos);
            strSection = trim(strSection);
            bool flag = !strSection.empty() || (strSection.empty() && !skipEmptyParts);
            if (flag)
                slTemp.push_back(strSection);
            if (pos != std::string::npos)
                strTemp = strTemp.substr(pos + 1);
        } while (pos != std::string::npos);
        return slTemp;
    }

    bool parseRegisterList(const std::string& paramString, std::vector<T_RegConf>& regList, bool isFlagRegister)
    {
        regList.clear();
        std::vector<std::string> lines = split(paramString, "\n", true);
        std::string strTemp;
        for (auto it = lines.begin(); it != lines.end(); ++it)
        {
            strTemp = *it;
            int idxSemicolon = (int)strTemp.find_last_of(';');
            if (idxSemicolon != std::string::npos)
                strTemp = trim(strTemp.erase(idxSemicolon));
            std::vector<std::string> paramList = split(strTemp, ",", true);
            if (paramList.size() < 2)
                continue;
            T_RegConf regConf;
            regConf.Addr = stoul(paramList[0], 0, 16);
            regConf.Value = stoul(paramList[1], 0, 16);
            regConf.Delay_ms = 0;
            if (isFlagRegister)
            {
                regConf.Mask = 0xff;
                if (paramList.size() > 2)
                    regConf.Mask = stoul(paramList[2], 0, 16);
            }
            else
            {
                if (paramList.size() > 2)
                    regConf.Delay_ms = stoi(paramList[2]);
                regConf.Mask = 0xff;
                if (paramList.size() > 3)
                    regConf.Mask = stoi(paramList[3], 0, 16);
            }
            regList.push_back(regConf);
        }
        return true;
    }

    bool parseVoltageList(const std::string& strVoltage, std::vector<T_Power>& powerList)
    {
        powerList.clear();
        std::vector<std::string> lines = split(strVoltage, "\n", true);
        std::string strTemp;
        for (auto it = lines.begin(); it != lines.end(); ++it)
        {
            strTemp = *it;
            int idxSemicolon = (int)strTemp.find_last_of(';');
            if (idxSemicolon != std::string::npos)
                strTemp = trim(strTemp.erase(idxSemicolon));
            std::vector<std::string> paramList = split(strTemp, ",", true);
            if (paramList.size() < 3)
                continue;
            T_Power power;
            std::string strVoltName = paramList[0];
            std::transform(strVoltName.begin(), strVoltName.end(), strVoltName.begin(), toupper);
            if (strVoltName == "DVDD")
                power.Id = (int)Power_DVDD;
            else if (strVoltName == "AVDD")
                power.Id = (int)Power_AVDD;
            else if (strVoltName == "DOVDD")
                power.Id = (int)Power_DOVDD;
            else if (strVoltName == "AFVCC")
                power.Id = (int)Power_AFVCC;
            else if (strVoltName == "VPP")
                power.Id = (int)Power_VPP;
            else if (strVoltName == "AVDD2")
                power.Id = (int)Power_AVDD2;
            else if (strVoltName == "VOIS")
                power.Id = (int)Power_VOIS;
            else
            {
                power.Id = 0;
                return false;
            }
            power.Value = stoi(paramList[1]); // mV
            power.Delay_ms = stoi(paramList[2]);
            powerList.push_back(power);
        }
        return true;
    }
private:
    friend class SensorSettingProvider;
    bool m_bIsLoaded{ false };
    IniFile m_iniFile;
};

/////////////////////////////////////////////// SensorSettingProvider ///////////////////////////////////////////////
SensorSettingProvider& SensorSettingProvider::Instance()
{
    static SensorSettingProvider s_instance;
    return s_instance;
}

SensorSettingProvider::SensorSettingProvider() :
    d(new Private())
{
    d->m_bIsLoaded = false;
}

SensorSettingProvider::~SensorSettingProvider()
{
    delete d;
}

bool SensorSettingProvider::LoadSensorSettingFile(const QString& settingFileName)
{
    if (d->m_bIsLoaded)
        return d->m_bIsLoaded;

    d->m_iniFile.LoadFile(settingFileName.toStdString());
    std::vector<IniFile::T_LineConf> vecSection;
    d->m_iniFile.ReadSection("default", vecSection);
    std::string version = "1.0";
    for (auto itLine = vecSection.begin(); itLine != vecSection.end(); ++itLine)
    {
        std::string::size_type pos = itLine->Comment.find("CZTEK", 0);
        if (pos != std::string::npos)
            version = itLine->Comment.substr(pos + std::string("CZTEK ").size(), 3);
    }

    d->m_iniFile.ReadSection("Sensor", vecSection);
    for (auto itLine = vecSection.begin(); itLine != vecSection.end(); ++itLine)
    {
        if (itLine->Key == "SchemeName") {
            m_sensorSetting.SchemeName = QString::fromStdString(itLine->Value);
        }
        if (itLine->Key == "VendorName") {
            m_sensorSetting.VendorName = QString::fromStdString(itLine->Value);
        }
        if (itLine->Key == "ChipName") {
            m_sensorSetting.ChipName = QString::fromStdString(itLine->Value);
        }
        if (itLine->Key == "InterfaceType") {
            m_sensorSetting.InterfaceType = (E_InterfaceType)stoi(itLine->Value);
        }
        if ((itLine->Key == "MipiLanes") || (itLine->Key == "Lanes")) {
            m_sensorSetting.Lanes = stoi(itLine->Value);
        }
        if (itLine->Key == "MipiFreq") {
            m_sensorSetting.MipiFreq = stoi(itLine->Value);
        }
        if (itLine->Key == "Mclk") {
            m_sensorSetting.Mclk_kHz = stoi(itLine->Value) * 1000;
        }
        if (itLine->Key == "DataWidth") {
            m_sensorSetting.DataWidth = stoi(itLine->Value);
        }
        if (itLine->Key == "ImageFormat") {
            m_sensorSetting.ImageFormat = (E_ImageFormat)stoi(itLine->Value);
        }
        if (itLine->Key == "ImageMode") {
            m_sensorSetting.ImageMode = (E_ImageMode)stoi(itLine->Value);
        }
        if (itLine->Key == "PixelWidth") {
            m_sensorSetting.PixelWidth = stoi(itLine->Value);
        }
        if (itLine->Key == "PixelHeight") {
            m_sensorSetting.PixelHeight = stoi(itLine->Value);
        }
        if (itLine->Key == "CropParam") {
            m_sensorSetting.CropParam.X = m_sensorSetting.CropParam.Y = 0;
            m_sensorSetting.CropParam.Width = m_sensorSetting.CropParam.Height = 0;
            std::vector<std::string> paramList = Private::split(itLine->Value, ",", false);
            if (paramList.size() > 3) {
                try
                {
                    m_sensorSetting.CropParam.X = stoi(paramList[0]);
                    m_sensorSetting.CropParam.Y = stoi(paramList[1]);
                    m_sensorSetting.CropParam.Width = stoi(paramList[2]);
                    m_sensorSetting.CropParam.Height = stoi(paramList[3]);
                }
                catch (...)
                {
                }
            }
        }
        if ((itLine->Key == "I2CMode") || (itLine->Key == "RegBitsMode")) {
            m_sensorSetting.I2cParam.RegBitsMode = (E_RegBitsMode)stoi(itLine->Value);
        }
        //if (itLine->Key == "CommIntfType") {
        //    m_sensorSetting.I2cParam->CommIntfType = (T_CommIntfConf::E_IntfType)stoi(itLine->Value);
        //    if (!ok) {
        //        qCritical() << QString("%1[%2] itLine->Value is invalid!").arg(itLine->Key).arg(itLine->Value);
        //        return false;
        //    }
        //}
        if ((itLine->Key == "I2CSpeed") || (itLine->Key == "CommSpeed")) {
            m_sensorSetting.I2cParam.Speed = stoi(itLine->Value);
            if (version == "1.0") {
                if (m_sensorSetting.I2cParam.Speed == 0)
                    m_sensorSetting.I2cParam.Speed = 1; // 100kHz
                else
                    m_sensorSetting.I2cParam.Speed = 4; // 400kHz
            }
        }
        if ((itLine->Key == "I2CAddress") || (itLine->Key == "CommAddr")) {
            m_sensorSetting.I2cParam.Addr = stoi(itLine->Value, 0, 16);
        }
        if (itLine->Key == "CommExtraParam") {
            //m_sensorSetting.CommExtraParam = itLine->Value;
        }
        if (itLine->Key == "PclkPol") {
            m_sensorSetting.PclkPol = stoi(itLine->Value);
        }
        if (itLine->Key == "DataPol") {
            m_sensorSetting.DataPol = stoi(itLine->Value);
        }
        if (itLine->Key == "HsyncPol") {
            m_sensorSetting.HsyncPol = stoi(itLine->Value);
        }
        if (itLine->Key == "VsyncPol") {
            m_sensorSetting.VsyncPol = stoi(itLine->Value);
        }
        if (itLine->Key == "Pwdn") {
            m_sensorSetting.Pwdn = stoi(itLine->Value);
        }
        if (itLine->Key == "PwdnParam") {
            //m_sensorSetting.PwdnParam = itLine->Value;
        }
        if (itLine->Key == "Reset") {
            m_sensorSetting.Reset = stoi(itLine->Value);
        }
        if (itLine->Key == "ResetParam") {
            //m_sensorSetting.ResetParam = itLine->Value;
        }
        if (itLine->Key == "FocusParam") {
            //m_sensorSetting.FocusParam = itLine->Value;
        }
    }

    std::vector<T_RegConf> vecRegConf;
    m_sensorSetting.FullModeParamCount = 0;
    std::string strTemp = d->m_iniFile.ReadSection("FullModeParams");
    if (!strTemp.empty()) {
        d->parseRegisterList(strTemp, vecRegConf, false);
        if (vecRegConf.size() > 0) {
            m_sensorSetting.FullModeParamCount = (int)vecRegConf.size();
            m_sensorSetting.FullModeParams = new T_RegConf[m_sensorSetting.FullModeParamCount];
            for (int i = 0; i < m_sensorSetting.FullModeParamCount; i++)
                m_sensorSetting.FullModeParams[i] = vecRegConf[i];
        }
    }

    m_sensorSetting.OtpInitParamCount = 0;
    strTemp = d->m_iniFile.ReadSection("OtpInitParams");
    if (!strTemp.empty()) {
        d->parseRegisterList(strTemp, vecRegConf, false);
        if (vecRegConf.size() > 0) {
            m_sensorSetting.OtpInitParamCount = (int)vecRegConf.size();
            m_sensorSetting.OtpInitParams = new T_RegConf[m_sensorSetting.OtpInitParamCount];
            for (int i = 0; i < m_sensorSetting.OtpInitParamCount; i++)
                m_sensorSetting.OtpInitParams[i] = vecRegConf[i];
        }
    }

    m_sensorSetting.SleepParamCount = 0;
    strTemp = d->m_iniFile.ReadSection("SleepParams");
    if (!strTemp.empty()) {
        d->parseRegisterList(strTemp, vecRegConf, false);
        if (vecRegConf.size() > 0) {
            m_sensorSetting.SleepParamCount = (int)vecRegConf.size();
            m_sensorSetting.SleepParams = new T_RegConf[m_sensorSetting.SleepParamCount];
            for (int i = 0; i < m_sensorSetting.SleepParamCount; i++)
                m_sensorSetting.SleepParams[i] = vecRegConf[i];
        }
    }

    m_sensorSetting.AfInitParamCount = 0;
    strTemp = d->m_iniFile.ReadSection("AfInitParams");
    if (!strTemp.empty()) {
        d->parseRegisterList(strTemp, vecRegConf, false);
        if (vecRegConf.size() > 0) {
            m_sensorSetting.AfInitParamCount = (int)vecRegConf.size();
            m_sensorSetting.AfInitParams = new T_RegConf[m_sensorSetting.AfInitParamCount];
            for (int i = 0; i < m_sensorSetting.AfInitParamCount; i++)
                m_sensorSetting.AfInitParams[i] = vecRegConf[i];
        }
    }

    m_sensorSetting.AfAutoParamCount = 0;
    strTemp = d->m_iniFile.ReadSection("AfAutoParams");
    if (!strTemp.empty()) {
        d->parseRegisterList(strTemp, vecRegConf, false);
        if (vecRegConf.size() > 0) {
            m_sensorSetting.AfAutoParamCount = (int)vecRegConf.size();
            m_sensorSetting.AfAutoParams = new T_RegConf[m_sensorSetting.AfAutoParamCount];
            for (int i = 0; i < m_sensorSetting.AfAutoParamCount; i++)
                m_sensorSetting.AfAutoParams[i] = vecRegConf[i];
        }
    }

    m_sensorSetting.AfFarParamCount = 0;
    strTemp = d->m_iniFile.ReadSection("AfFarParams");
    if (!strTemp.empty()) {
        d->parseRegisterList(strTemp, vecRegConf, false);
        if (vecRegConf.size() > 0) {
            m_sensorSetting.AfFarParamCount = (int)vecRegConf.size();
            m_sensorSetting.AfFarParams = new T_RegConf[m_sensorSetting.AfFarParamCount];
            for (int i = 0; i < m_sensorSetting.AfFarParamCount; i++)
                m_sensorSetting.AfFarParams[i] = vecRegConf[i];
        }
    }

    m_sensorSetting.AfNearParamCount = 0;
    strTemp = d->m_iniFile.ReadSection("AfNearParams");
    if (!strTemp.empty()) {
        d->parseRegisterList(strTemp, vecRegConf, false);
        if (vecRegConf.size() > 0) {
            m_sensorSetting.AfNearParamCount = (int)vecRegConf.size();
            m_sensorSetting.AfNearParams = new T_RegConf[m_sensorSetting.AfNearParamCount];
            for (int i = 0; i < m_sensorSetting.AfNearParamCount; i++)
                m_sensorSetting.AfNearParams[i] = vecRegConf[i];
        }
    }

    m_sensorSetting.ExposureParamCount = 0;
    strTemp = d->m_iniFile.ReadSection("ExposureParams");
    if (!strTemp.empty()) {
        d->parseRegisterList(strTemp, vecRegConf, false);
        if (vecRegConf.size() > 0) {
            m_sensorSetting.ExposureParamCount = (int)vecRegConf.size();
            m_sensorSetting.ExposureParams = new T_RegConf[m_sensorSetting.ExposureParamCount];
            for (int i = 0; i < m_sensorSetting.ExposureParamCount; i++)
                m_sensorSetting.ExposureParams[i] = vecRegConf[i];
        }
    }

    m_sensorSetting.GainParamCount = 0;
    strTemp = d->m_iniFile.ReadSection("GainParams");
    if (!strTemp.empty()) {
        d->parseRegisterList(strTemp, vecRegConf, false);
        if (vecRegConf.size() > 0) {
            m_sensorSetting.GainParamCount = (int)vecRegConf.size();
            m_sensorSetting.GainParams = new T_RegConf[m_sensorSetting.GainParamCount];
            for (int i = 0; i < m_sensorSetting.GainParamCount; i++)
                m_sensorSetting.GainParams[i] = vecRegConf[i];
        }
    }

    m_sensorSetting.PowerCount = 0;
    std::vector<T_Power> vecPowerConf;
    strTemp = d->m_iniFile.ReadSection("Voltages");
    if (!strTemp.empty()) {
        d->parseVoltageList(strTemp, vecPowerConf);
        if (vecPowerConf.size() > 0) {
            m_sensorSetting.PowerCount = (int)vecPowerConf.size();
            m_sensorSetting.Powers = new T_Power[m_sensorSetting.PowerCount];
            for (int i = 0; i < m_sensorSetting.PowerCount; i++)
                m_sensorSetting.Powers[i] = vecPowerConf[i];
        }
    }

    m_sensorSetting.FlagRegisterCount = 0;
    strTemp = d->m_iniFile.ReadSection("FlagRegisters");
    if (!strTemp.empty()) {
        d->parseRegisterList(strTemp, vecRegConf, true);
        if (vecRegConf.size() > 0) {
            m_sensorSetting.FlagRegisterCount = (int)vecRegConf.size();
            m_sensorSetting.FlagRegisters = new T_RegConf[m_sensorSetting.FlagRegisterCount];
            for (int i = 0; i < m_sensorSetting.FlagRegisterCount; i++)
                m_sensorSetting.FlagRegisters[i] = vecRegConf[i];
        }
    }

    d->m_bIsLoaded = true;
    return d->m_bIsLoaded;
}
