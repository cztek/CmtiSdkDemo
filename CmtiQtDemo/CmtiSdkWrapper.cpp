#include "CmtiSdkWrapper.h"
#pragma comment(lib, "CmtiSdk.lib")

extern "C" 
{
    namespace CmtiSdk 
    {
        /**
        * @brief 设置Test Pattern参数
        * @param patternMode 模式(默认0为关闭)
        * @param patternFps  模拟输出FPS
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetTestPattern(HDeviceClient hDevCli, uint16_t patternMode, uint16_t patternFps);

        /**
        * @brief 获取Test Pattern参数
        * @param [out]patternMode 模式(默认0为关闭)
        * @param [out]patternFps  模拟输出FPS
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetTestPattern(HDeviceClient hDevCli, uint16_t *patternMode, uint16_t *patternFps);

    }
}

DeviceController& DeviceController::Instance()
{
    static DeviceController s_instance;
    //CmtiSdk::Cmti_DisableNonCztekNic();
    return s_instance;
}

DeviceController::DeviceController()
{
    CmtiSdk::Cmti_GetDeviceController(&m_deviceController);
}

DeviceController::~DeviceController()
{
    CmtiSdk::Cmti_DestroyDeviceController(m_deviceController);
    m_deviceController = NULL;
}

DeviceClient *DeviceController::GetDeviceClient(const std::string & devName)
{
    CmtiSdk::HDeviceClient hDevCli = nullptr;
    CmtiSdk::Cmti_GetDeviceClient(m_deviceController, devName.c_str(), &hDevCli);
    if (m_mapHandle2DeviceClient.find(hDevCli) == m_mapHandle2DeviceClient.end()) {
        m_mapHandle2DeviceClient.insert(std::make_pair(hDevCli, new DeviceClient(hDevCli)));
    }
    return m_mapHandle2DeviceClient[hDevCli];
}

DeviceClient* DeviceController::operator[](const std::string& devName)
{
    return GetDeviceClient(devName);
}

DeviceClient* DeviceController::operator[](int nSockIndex)
{
    return GetDeviceClient(GetDevNameBySocketIndex(nSockIndex));
}

void DeviceController::ToggleDiscovery()
{
    CmtiSdk::Cmti_ToggleDiscovery(m_deviceController);
}

int DeviceController::EnumerateDevice(std::vector<std::string> &deviceNameList)
{
    const int MAX_SUPPORT_DEV_NUM = 8;
    char *pDeviceNameList[MAX_SUPPORT_DEV_NUM] = { NULL };
    for (int i = 0; i < MAX_SUPPORT_DEV_NUM; i++)
        pDeviceNameList[i] = new char[64];
    int devNum = MAX_SUPPORT_DEV_NUM;
    int ec = CmtiSdk::Cmti_EnumerateDevice(m_deviceController, pDeviceNameList, &devNum);
    if (ERR_NoError == ec)
    {
        for (int i = 0; i < devNum; i++)
            deviceNameList.push_back(std::string(pDeviceNameList[i]));
    }
    for (int i = 0; i < MAX_SUPPORT_DEV_NUM; i++)
        delete[] pDeviceNameList[i];
    return ec;
}

void DeviceController::BindSocket(int nSockIndex, const std::string& devName)
{
    m_mapSocketIdx2DeviceName[nSockIndex] = devName;
}

std::string DeviceController::GetDevNameBySocketIndex(int nSockIndex)
{
    if (m_mapSocketIdx2DeviceName.find(nSockIndex) != m_mapSocketIdx2DeviceName.end())
        return m_mapSocketIdx2DeviceName[nSockIndex];
    return "";
}

int DeviceController::GetDevicePropList(const std::string &devName, std::map<std::string, std::string> &propMap)
{
    const int MAX_SUPPORTED_PROP_NUM = 30;
    char *propNameList[MAX_SUPPORTED_PROP_NUM] = { NULL };
    char *propValList[MAX_SUPPORTED_PROP_NUM]  = { NULL };
    for (int i = 0; i < MAX_SUPPORTED_PROP_NUM; i++)
    {
        propNameList[i] = new char[32];
        propValList[i]  = new char[256];
    }
    int propNum = MAX_SUPPORTED_PROP_NUM;
    int ec = CmtiSdk::Cmti_GetDevicePropList(m_deviceController, devName.c_str(), propNameList, propValList, &propNum);
    if (ERR_NoError == ec)
    {
        for (int i = 0; i < propNum; i++)
            propMap.insert(std::make_pair(std::string(propNameList[i]), std::string(propValList[i])));
    }
    for (int i = 0; i < MAX_SUPPORTED_PROP_NUM; i++)
    {
        delete[] propNameList[i];
        delete[] propValList[i];
    }
    return ec;
}

int DeviceController::GetCmtiSdkVersion(std::string &strVersion)
{
    char szVersion[64];
    memset(szVersion, 0, sizeof(szVersion));
    int ec = CmtiSdk::Cmti_GetCmtiSdkVersion(szVersion, sizeof(szVersion));
    strVersion = std::string(szVersion);
    return ec;
}

DeviceClient::DeviceClient(CmtiSdk::HDeviceClient hDevClient) :
    m_hDevClient(hDevClient)
{}

DeviceClient::~DeviceClient()
{}

//int DeviceClient::GetDeviceProperty(const std::string& propName, std::string& propVal) const
//{
//    char szPropVal[64];
//    int ec = CmtiSdk::Cmti_GetDevicePropertyByHandle(m_hDevClient, propName.c_str(), szPropVal, sizeof(szPropVal));
//    propVal = std::string(szPropVal);
//    return ec;
//}

int DeviceClient::GetCaptureFps(float &fps)
{
    return CmtiSdk::Cmti_GetCaptureFps(m_hDevClient, &fps);
}

int DeviceClient::GetTransmitFps(float &fps)
{
    return CmtiSdk::Cmti_GetTransmitFps(m_hDevClient, &fps);
}

int DeviceClient::GetReceiveFps(float &fps)
{
    return CmtiSdk::Cmti_GetReceiveFps(m_hDevClient, &fps);
}

int DeviceClient::GetTransmitFer(float &fer)
{
    return CmtiSdk::Cmti_GetTransmitFer(m_hDevClient, &fer);
}

int DeviceClient::ProbeI2cAddress(uint8_t u8I2cAddr[], int& nCount)
{
    return CmtiSdk::Cmti_SearchI2cAddress(m_hDevClient, u8I2cAddr, &nCount);
}

int DeviceClient::GetBoardInfo(T_BoardInfo& boardInfo)
{
    return CmtiSdk::Cmti_GetBoardInfo(m_hDevClient, &boardInfo);
}

int DeviceClient::WriteSingleI2c(uint32_t i2cAddr, uint32_t speedkHz, uint32_t mode, uint32_t regAddr, uint32_t regData)
{
    return CmtiSdk::Cmti_WriteSingleI2c(m_hDevClient, i2cAddr, speedkHz, mode, regAddr, regData);
}

int DeviceClient::ReadSingleI2c(uint32_t i2cAddr, uint32_t speedkHz, uint32_t mode, uint32_t regAddr, uint32_t& regData)
{
    return CmtiSdk::Cmti_ReadSingleI2c(m_hDevClient, i2cAddr, speedkHz, mode, regAddr, &regData);
}

int DeviceClient::WriteDiscreteI2c(uint32_t i2cAddr, uint32_t speedkHz, uint32_t mode, const std::vector<T_RegConf> &regList)
{
    int count = static_cast<int>(regList.size());
    uint32_t *regAddr  = new uint32_t[count];
    uint32_t *regVal   = new uint32_t[count];
    uint32_t *regDelay = new uint32_t[count];
    for (int i = 0; i < count; i++)
    {
        regAddr[i]  = regList[i].Addr;
        regVal[i]   = regList[i].Value;
        regDelay[i] = regList[i].Delay_ms;
    }
    int ec = CmtiSdk::Cmti_WriteDiscreteI2c(m_hDevClient, i2cAddr, speedkHz, mode, regAddr, regVal, regDelay, count);
    delete[] regAddr;
    delete[] regVal;
    delete[] regDelay;

    return ec;
}

int DeviceClient::ReadDiscreteI2c(uint32_t i2cAddr, uint32_t speedkHz, uint32_t mode, std::vector<T_RegConf> &regList)
{
    size_t count = regList.size();
    uint32_t *regAddr = new uint32_t[count];
    uint32_t *regVal  = new uint32_t[count];
    for (size_t i = 0; i < count; i++)
    {
        regAddr[i] = regList[i].Addr;
        regVal[i]  = 0;
    }
    int ec = CmtiSdk::Cmti_ReadDiscreteI2c(m_hDevClient, i2cAddr, speedkHz, mode, regAddr, regVal, (int)count);
    for (size_t i = 0; i < count; i++)
    {
        regList[i].Value = regVal[i];
    }
    delete[] regAddr;
    delete[] regVal;

    return ec;
}

int DeviceClient::WriteDiscreteI2c_V2(uint32_t speedkHz, uint32_t mode[], const uint32_t i2cAddr[],
    const uint32_t regAddr[], const uint32_t regData[], const uint32_t delay_ms[], int regNum)
{
    int ec = CmtiSdk::Cmti_WriteDiscreteI2c_V2(m_hDevClient, speedkHz, mode, i2cAddr, regAddr, regData, delay_ms, regNum);
    return ec;
}

int DeviceClient::ReadDiscreteI2c_V2(uint32_t speedkHz, uint32_t mode, const uint32_t i2cAddr[],
    const uint32_t regAddr[], uint32_t regData[], int regNum)
{
    int ec = CmtiSdk::Cmti_ReadDiscreteI2c_V2(m_hDevClient, speedkHz, mode, i2cAddr, regAddr, regData, regNum);
    return ec;
}

int DeviceClient::WriteContinuousI2c(uint32_t i2cAddr, uint32_t speedkHz, uint32_t regAddr, uint32_t regAddrSize,
                                     const uint8_t *data, uint32_t dataSize)
{
    int ec = CmtiSdk::Cmti_WriteContinuousI2c(m_hDevClient, i2cAddr, speedkHz, regAddr, regAddrSize, data, dataSize);
    return ec;
}

int DeviceClient::ReadContinuousI2c(uint32_t i2cAddr, uint32_t speedkHz, uint32_t regAddr, uint32_t regAddrSize,
                                    uint8_t *data, uint32_t dataSize)
{
    int ec = CmtiSdk::Cmti_ReadContinuousI2c(m_hDevClient, i2cAddr, speedkHz, regAddr, regAddrSize, data, dataSize);
    return ec;
}

bool DeviceClient::IsOpen()
{
    return  CmtiSdk::Cmti_IsOpen(m_hDevClient);
}

bool DeviceClient::IsConnected(const std::string& devName)
{
    return CmtiSdk::Cmti_IsConnected(devName.c_str());
}

int DeviceClient::SetSensorClock(uint32_t clk100kHz)
{
    return CmtiSdk::Cmti_SetSensorClock(m_hDevClient, clk100kHz);
}

int DeviceClient::SetSensorInterface(uint32_t intf)
{
    return CmtiSdk::Cmti_SetSensorInterface(m_hDevClient, intf);
}

int DeviceClient::SetMipiParam(uint32_t laneNum, uint32_t freqMHz, uint32_t virtualChannel)
{
    return CmtiSdk::Cmti_SetMipiParam(m_hDevClient, laneNum, freqMHz, virtualChannel);
}

int DeviceClient::GetMipiParam(uint32_t &laneNum, uint32_t &freqMHz, uint32_t &virtualChannel)
{
    return CmtiSdk::Cmti_GetMipiParam(m_hDevClient, &laneNum, &freqMHz, &virtualChannel);
}

int DeviceClient::SetSensorGpioPinLevel(ushort pin, ushort level)
{
    return CmtiSdk::Cmti_SetSensorGpioPinLevel(m_hDevClient, pin, level);
}

int DeviceClient::GetSensorGpioPinLevel(ushort pin, ushort &level)
{
    return CmtiSdk::Cmti_GetSensorGpioPinLevel(m_hDevClient, pin, &level);
}

int DeviceClient::SetSensorGpioPinDir(ushort pin, ushort dir)
{
    return CmtiSdk::Cmti_SetSensorGpioPinDir(m_hDevClient, pin, dir);
}

int DeviceClient::GetSensorGpioPinDir(ushort pin, ushort &dir)
{
    return CmtiSdk::Cmti_GetSensorGpioPinDir(m_hDevClient, pin, &dir);
}

int DeviceClient::SetSensorPower(const std::vector<T_Power> &powerList)
{
    size_t count = powerList.size();
    uint32_t *powerId = new uint32_t[count];
    uint32_t *voltage = new uint32_t[count];
    uint32_t *delay = new uint32_t[count];
    for (size_t i = 0; i < count; i++)
    {
        powerId[i] = powerList[i].Id;
        voltage[i] = powerList[i].Value;
        delay[i] = powerList[i].Delay_ms;
    }
    int ec = CmtiSdk::Cmti_SetSensorPower(m_hDevClient, powerId, voltage, delay, (int)count);
    delete[] powerId;
    delete[] voltage;
    delete[] delay;
    return ec;
}

int DeviceClient::SetSensorPower(const T_Power& power)
{
    uint32_t powerId = power.Id;
    uint32_t voltage = power.Value;
    uint32_t delay = power.Delay_ms;
    int ec = CmtiSdk::Cmti_SetSensorPower(m_hDevClient, &powerId, &voltage, &delay, 1);
    return ec;
}

int DeviceClient::GetSensorPower(std::vector<T_Power> &powerList)
{
    size_t count = powerList.size();
    uint32_t *powerId = new uint32_t[count];
    uint32_t *voltage = new uint32_t[count];
    for (size_t i = 0; i < count; i++)
    {
        powerId[i] = powerList[i].Id;
        voltage[i] = 0;
    }
    int ec = CmtiSdk::Cmti_GetSensorPower(m_hDevClient, powerId, voltage, (int)count);
    for (size_t i = 0; i < count; i++)
    {
        powerList[i].Value = voltage[i];
    }
    delete[] powerId;
    delete[] voltage;
    return ec;
}

int DeviceClient::SetFeedBackVoltageSampleParam(int32_t nInterval_us, int32_t nPoints, int32_t nSampleValType)
{
    return CmtiSdk::Cmti_SetFeedBackVoltageSampleParam(m_hDevClient, nInterval_us, nPoints, nSampleValType);
}

int DeviceClient::GetFeedBackVoltageSampleParam(int32_t& nInterval_us, int32_t& nPoints, int32_t& nSampleValType)
{
    return CmtiSdk::Cmti_GetFeedBackVoltageSampleParam(m_hDevClient, &nInterval_us, &nPoints, &nSampleValType);
}

int DeviceClient::GetSensorFeedBackVoltage(std::vector<T_Power> &powerList)
{
    size_t count = powerList.size();
    int *powerId = new int[count];
    int *voltage = new int[count];
    for (size_t i = 0; i < count; i++)
    {
        powerId[i] = powerList[i].Id;
        voltage[i] = 0;
    }
    int ec = CmtiSdk::Cmti_GetSensorFeedBackVoltage(m_hDevClient, powerId, voltage, (int)count);
    for (size_t i = 0; i < count; i++)
    {
        powerList[i].Value = voltage[i];
    }
    delete[] voltage;
    delete[] powerId;
    return ec;
}

int DeviceClient::SetFrameParam(uint32_t imgFmt, uint32_t imgMode, uint32_t width, uint32_t height, uint32_t outImgFmt, uint32_t cropLeft, uint32_t cropTop, uint32_t cropWidth, uint32_t cropHeight)
{
    return CmtiSdk::Cmti_SetFrameParam2(m_hDevClient, imgFmt, imgMode, width, height, outImgFmt, cropLeft, cropTop, cropWidth, cropHeight);
}

int DeviceClient::GetFrameParam(uint32_t &imgFmt, uint32_t &imgMode, uint32_t &width, uint32_t &height, uint32_t &size)
{
    return CmtiSdk::Cmti_GetFrameParam(m_hDevClient, &imgFmt, &imgMode, &width, &height, &size);
}

int DeviceClient::SetEmbeddedLineSize(uint32_t size)
{
    return CmtiSdk::Cmti_SetEmbeddedLineSize(m_hDevClient, size);
} 

int DeviceClient::SetRoiParam(const T_Rect roiRect[], uint32_t roiCount)
{
    return CmtiSdk::Cmti_SetRoiParam(m_hDevClient, roiRect, roiCount);
}

int DeviceClient::SetSensorStreamOnRegister(uint32_t i2cAddr, uint32_t regAddr, uint32_t regDataOn, uint32_t regDataOff)
{
    return CmtiSdk::Cmti_SetSensorStreamOnRegister(m_hDevClient, i2cAddr, regAddr, regDataOn, regDataOff);
}

int DeviceClient::VideoControl(uint32_t ctrl)
{
    return CmtiSdk::Cmti_VideoControl(m_hDevClient, ctrl);
}

int DeviceClient::SetGrabTimeout(uint32_t grabTimeout)
{
    return CmtiSdk::Cmti_SetGrabTimeout(m_hDevClient, grabTimeout);
}

int DeviceClient::SetGrabTimeoutAndLoopTimes(uint32_t grabTimeout_ms, uint32_t loopTimes)
{
    return CmtiSdk::Cmti_SetGrabTimeoutAndLoopTimes(m_hDevClient, grabTimeout_ms, loopTimes);
}

int DeviceClient::GetSystemTimestamp(uint64_t &timestamp)
{
    return CmtiSdk::Cmti_GetSystemTimestamp(m_hDevClient, &timestamp);
}

int DeviceClient::SkipFrame(int count)
{
    return CmtiSdk::Cmti_SkipFrame(m_hDevClient, count);
}

int DeviceClient::GrabFrame(uint8_t *pbuffer, int bufferLen, uint64_t & timestamp)
{
    return CmtiSdk::Cmti_GrabFrame(m_hDevClient, pbuffer, bufferLen, &timestamp);
}

int DeviceClient::GrabLatestFrame(uint8_t *pbuffer, int bufferLen, uint64_t & timestamp)
{
    return CmtiSdk::Cmti_GrabLatestFrame(m_hDevClient, pbuffer, bufferLen, &timestamp);
}

int DeviceClient::GrabFrame2(uint8_t *pbuffer, int bufferLen, uint64_t &headTimestamp,
    uint64_t &tailTimestamp, uint32 &frameSequence)
{
    return CmtiSdk::Cmti_GrabFrame2(m_hDevClient, pbuffer, bufferLen, &headTimestamp, &tailTimestamp, &frameSequence);
}

int DeviceClient::GrabLatestFrame2(uint8_t *pbuffer, int bufferLen, uint64_t &headTimestamp,
    uint64_t &tailTimestamp, uint32 &frameSequence)
{
    return CmtiSdk::Cmti_GrabLatestFrame2(m_hDevClient, pbuffer, bufferLen, &headTimestamp, &tailTimestamp, &frameSequence);
}

int DeviceClient::DequeueFrameBuffer(int & bufIdx, uint8_t * & pbuffer, uint64_t & timestamp)
{
    return CmtiSdk::Cmti_DequeueFrameBuffer(m_hDevClient, &bufIdx, &pbuffer, &timestamp);
}

int DeviceClient::DequeueFrameBuffer2(int & bufIdx, uint8_t * & pbuffer, uint64_t &headTimestamp, uint64_t &tailTimestamp, uint32 &frameSequence)
{
    return CmtiSdk::Cmti_DequeueFrameBuffer2(m_hDevClient, &bufIdx, &pbuffer, &headTimestamp, &tailTimestamp, &frameSequence);
}

int DeviceClient::DequeueLatestFrameBuffer(int & bufIdx, uint8_t * & pbuffer, uint64_t & timestamp)
{
    return CmtiSdk::Cmti_DequeueLatestFrameBuffer(m_hDevClient, &bufIdx, &pbuffer, &timestamp);
}

int DeviceClient::DequeueLatestFrameBuffer2(int & bufIdx, uint8_t * & pbuffer, uint64_t &headTimestamp, uint64_t &tailTimestamp, uint32 &frameSequence)
{
    return CmtiSdk::Cmti_DequeueLatestFrameBuffer2(m_hDevClient, &bufIdx, &pbuffer, &headTimestamp, &tailTimestamp, &frameSequence);
}

int DeviceClient::EnqueueFrameBuffer(int bufIdx)
{
    return CmtiSdk::Cmti_EnqueueFrameBuffer(m_hDevClient, bufIdx);
}

int DeviceClient::SetOsTestConfig(uint32_t supplyVol, uint32_t supplyCurrent, const uint32_t pinId[],
                                  const uint32_t openStdVol[], const uint32_t shortStdVol[], uint32_t count)
{
    return CmtiSdk::Cmti_SetOsTestConfig(m_hDevClient, supplyVol, supplyCurrent, pinId, openStdVol, shortStdVol, count);
}

int DeviceClient::SetOsTestCurrentDirection(bool positive)
{
    return CmtiSdk::Cmti_SetOsTestCurrentDirection(m_hDevClient, positive);
}

int DeviceClient::ReadOsTestResult(const uint32_t pinId[], uint32_t openVol[], uint32_t shortVol[], uint32_t result[],
                                   uint32_t count)
{
    return CmtiSdk::Cmti_ReadOsTestResult(m_hDevClient, pinId, openVol, shortVol, result, count);
}

int DeviceClient::GetCurrent(const uint32_t powerId[], const uint32_t currentRange[], float current[], uint32_t count)
{
    return CmtiSdk::Cmti_GetCurrent(m_hDevClient, powerId, currentRange, current, count);
}

int DeviceClient::GetCurrentV2(const int powerIds[], const int upperLimit[], const uint16 atuoHighPrecision[], float current_nA[], int count)
{
    return CmtiSdk::Cmti_GetCurrentV2(m_hDevClient, powerIds, upperLimit, atuoHighPrecision, current_nA, count);
}

int DeviceClient::ReadCurrentCalibrationOffset(const uint32_t powerId[], const uint32_t voltage_mV[], const uint32_t delay_ms[], uint32_t nPowerCount,
            int nCurrent_nA[], int& nCurrentCount)
{
    return CmtiSdk::Cmti_ReadCurrentCalibrationOffset(m_hDevClient, powerId, voltage_mV, delay_ms, nPowerCount, nCurrent_nA, &nCurrentCount);
}

int DeviceClient::SetOvercurrentParam(const int powerId[], const int currentThrd_mA[], const int debounceInterval_ms[], int count)
{
    return CmtiSdk::Cmti_SetOvercurrentParam(m_hDevClient, powerId, currentThrd_mA, debounceInterval_ms, count);
}

int DeviceClient::GetOvercurrentParam(const int powerId[], int currentThrd_mA[], int debounceInterval_ms[], int count)
{
    return CmtiSdk::Cmti_GetOvercurrentParam(m_hDevClient, powerId, currentThrd_mA, debounceInterval_ms, count);
}

int DeviceClient::QueryOvercurrent(int powerId[], int &count)
{
    return CmtiSdk::Cmti_QueryOvercurrent(m_hDevClient, powerId, &count);
}

int DeviceClient::SetBeepOn(uint32_t delay)
{
    return CmtiSdk::Cmti_SetBeepOn(m_hDevClient, delay);
}

int DeviceClient::PowerPinLC(const int nPowerId[], const int nSupplyVoltage_mV[],
    const int nUpperLimitCurrent_nA[], const int bAutoHighPrecision[], int nLeakCurrent_nA[], int nCount)
{
    return CmtiSdk::Cmti_PowerPinLC(m_hDevClient, nPowerId, nSupplyVoltage_mV, nUpperLimitCurrent_nA, bAutoHighPrecision, nLeakCurrent_nA, nCount);
}

int DeviceClient::SignalPinLC(int nMipiPinSupplyVoltage_mV, int nIoPinSupplyVoltage_mV, int nDirection,
    const int nPinId[], int nLeakCurrent_nA[], int nCount)
{
    return CmtiSdk::Cmti_SignalPinLC(m_hDevClient, nMipiPinSupplyVoltage_mV, nIoPinSupplyVoltage_mV, nDirection, nPinId, nLeakCurrent_nA,nCount);
}

int DeviceClient::SetI2cPullupOutput(int nPullupVoltage, int bSclEnabled, int bSdaEnabled)
{
    return CmtiSdk::Cmti_SetI2cPullupOutput(m_hDevClient, nPullupVoltage, bSclEnabled, bSdaEnabled);
}

int DeviceClient::SetI2cPushPullOutput(bool enabled)
{
    return CmtiSdk::Cmti_SetI2cPushPullOutput(m_hDevClient, enabled);
}

int DeviceClient::GetI2cPushPullOutput(bool &enabled)
{
    return CmtiSdk::Cmti_GetI2cPushPullOutput(m_hDevClient, &enabled);
}

int DeviceClient::GetMipiPinVoltage(const int nMipiPin[], int nVoltage_uV[], int nCount)
{
    return CmtiSdk::Cmti_GetMipiPinVoltage(m_hDevClient, nMipiPin, nVoltage_uV, nCount);
}

int DeviceClient::GetPoPinVoltage(const int nPoPin[], int nVoltage_uV[], int nCount)
{
    return CmtiSdk::Cmti_GetPoPinVoltage(m_hDevClient, nPoPin, nVoltage_uV, nCount);
}

int DeviceClient::SpiTransfer(uint32_t spiId, const uint8_t txBuf[], uint8_t rxBuf[], int nLen)
{
    return CmtiSdk::Cmti_SpiTransfer(m_hDevClient, spiId, txBuf, rxBuf, nLen);
}

int DeviceClient::SetSpiParameter(uint32_t spiId, uint32_t mode, uint32_t speed_kHz, uint32_t bitsPerWord, uint32_t delay_us)
{
    return CmtiSdk::Cmti_SetSpiParameter(m_hDevClient, spiId, mode, speed_kHz, bitsPerWord, delay_us);
}

int DeviceClient::GetSpiParameter(uint32_t spiId, uint32_t &mode, uint32_t &speed_kHz, uint32_t &bitsPerWord, uint32_t &delay_us)
{
    return CmtiSdk::Cmti_GetSpiParameter(m_hDevClient, spiId, &mode, &speed_kHz, &bitsPerWord, &delay_us);
}

int DeviceClient::GetPoResistance(int endPoint, int& resistance_mOhm)
{
    return CmtiSdk::Cmti_GetPoResistance(m_hDevClient, endPoint, resistance_mOhm);
}

int DeviceClient::GetMipiStatus(int nStatusKey[], int nStatusVal[], int nCount)
{
    return CmtiSdk::Cmti_GetMipiStatus(m_hDevClient, nStatusKey, nStatusVal, nCount);
}

int DeviceClient::MeasureFrameSyncPeriodControl(int32_t nPin, int32_t nCtrl)
{
    return CmtiSdk::Cmti_MeasureFrameSyncPeriodControl(m_hDevClient, nPin, nCtrl);
}

int DeviceClient::GetFrameSyncPeriod(int32_t nPin, uint32_t &pPeriod_us)
{
    return CmtiSdk::Cmti_GetFrameSyncPeriod(m_hDevClient, nPin, &pPeriod_us);
}

int DeviceClient::SetPmicSwitchMode(int32_t mode)
{
    return CmtiSdk::Cmti_SetPmicSwitchMode(m_hDevClient, mode);
}

int DeviceClient::GetPmicSwitchMode(int32_t& pMode)
{
    return CmtiSdk::Cmti_GetPmicSwitchMode(m_hDevClient, &pMode);
}

int DeviceClient::GetMipiDPhyHSVoltage(int32_t nPositive, int32_t nSupplyCurrent_uA, const int32_t nPinsId[], int32_t nVoltage_uV[], int32_t nCount)
{
    return CmtiSdk::Cmti_GetMipiDPhyHSVoltage(m_hDevClient, nPositive, nSupplyCurrent_uA, nPinsId, nVoltage_uV, nCount);
}

int DeviceClient::SetMipiReceiverMode(int32_t mode)
{
    return CmtiSdk::Cmti_SetMipiReceiverMode(m_hDevClient, mode);
}

int DeviceClient::CalibrateDpsOutputVoltage(const int32_t nPowerIds[], const int32_t nVoltagemV[], int32_t nCount)
{
    return CmtiSdk::Cmti_CalibrateDpsOutputVoltage(m_hDevClient, nPowerIds, nVoltagemV, nCount);
}

int DeviceClient::GetDpsOutputVoltageCalibration(const int32_t nPowerIds[], int32_t nZeroOffsetmV[], int32_t nCount)
{
    return CmtiSdk::Cmti_GetDpsOutputVoltageCalibration(m_hDevClient, nPowerIds, nZeroOffsetmV, nCount);
}

int DeviceClient::SetDpsResistorStateBetweenForceAndSense(const int32_t nPowerIds[], const int32_t nEnable[], int32_t nCount)
{
    return CmtiSdk::Cmti_SetDpsResistorStateBetweenForceAndSense(m_hDevClient, nPowerIds, nEnable, nCount);
}


int DeviceClient::GetDpsResistorStateBetweenForceAndSense(const int32_t nPowerIds[], int32_t nEnable[], int32_t nCount)
{
    return CmtiSdk::Cmti_GetDpsResistorStateBetweenForceAndSense(m_hDevClient, nPowerIds, nEnable, nCount);
}

int DeviceClient::SetMeasureVoltageCurrentSourceParam(int32_t nMeasureTarget, int32_t nCurrent_uA, int32_t nLimitVoltage_mV)
{
    return CmtiSdk::Cmti_SetMeasureVoltageCurrentSourceParam(m_hDevClient, nMeasureTarget, nCurrent_uA, nLimitVoltage_mV);
}

int DeviceClient::SetDpsSlewRate(const int32_t nPowerIds[], const float fSlewRate[], int32_t nCount)
{
    return CmtiSdk::Cmti_SetDpsSlewRate(m_hDevClient, nPowerIds, fSlewRate, nCount);
}

int DeviceClient::GetDpsSlewRate(const int32_t nPowerIds[], float fSlewRate[], int32_t nCount)
{
    return CmtiSdk::Cmti_GetDpsSlewRate(m_hDevClient, nPowerIds, fSlewRate, nCount);
}

int DeviceClient::SetMipiDphyReceiverDataLaneHSSettleTime(int32_t settleTime_ns)
{
    return CmtiSdk::Cmti_SetMipiDphyReceiverDataLaneHSSettleTime(m_hDevClient, settleTime_ns);
}

int DeviceClient::GetMipiDphyReceiverDataLaneHSSettleTime(int32_t &settleTime_ns)
{
    return CmtiSdk::Cmti_GetMipiDphyReceiverDataLaneHSSettleTime(m_hDevClient, &settleTime_ns);
}

int DeviceClient::SetMipiDphyReceiverClockLaneHSSettleTime(int32_t settleTime_ns)
{
    return CmtiSdk::Cmti_SetMipiDphyReceiverClockLaneHSSettleTime(m_hDevClient, settleTime_ns);
}

int DeviceClient::GetMipiDphyReceiverClockLaneHSSettleTime(int32_t &settleTime_ns)
{
    return CmtiSdk::Cmti_GetMipiDphyReceiverClockLaneHSSettleTime(m_hDevClient, &settleTime_ns);
}

int DeviceClient::SetMipiCphyReceiverDataLaneHSSettleTime(int32_t settleTime_ns)
{
    return CmtiSdk::Cmti_SetMipiCphyReceiverDataLaneHSSettleTime(m_hDevClient, settleTime_ns);
}

int DeviceClient::GetMipiCphyReceiverDataLaneHSSettleTime(int32_t &settleTime_ns)
{
    return CmtiSdk::Cmti_GetMipiCphyReceiverDataLaneHSSettleTime(m_hDevClient, &settleTime_ns);
}

int DeviceClient::SetMipiCphyReceiverClockLaneHSSettleTime(int32_t settleTime_ns)
{
    return CmtiSdk::Cmti_SetMipiCphyReceiverClockLaneHSSettleTime(m_hDevClient, settleTime_ns);
}

int DeviceClient::GetMipiCphyReceiverClockLaneHSSettleTime(int32_t &settleTime_ns)
{
    return CmtiSdk::Cmti_GetMipiCphyReceiverClockLaneHSSettleTime(m_hDevClient, &settleTime_ns);
}

int DeviceClient::CreateAsyncGetDpsCurrentOperation(const int powerId[], const int currentRange[], int nCount,
    int nSampleInterval_us, int nSamplePoint, int eSampleType)
{
    return CmtiSdk::Cmti_CreateAsyncGetDpsCurrentOperation(m_hDevClient, powerId, currentRange, nCount, nSampleInterval_us, nSamplePoint, eSampleType);
}

int DeviceClient::AsyncGetDpsCurrentResult(int powerId[], double current_nA[], int nCount)
{
    return CmtiSdk::Cmti_AsyncGetDpsCurrentResult(m_hDevClient, powerId, current_nA, nCount);
}

int DeviceClient::GetDpsCurrent(const int powerId[], const int currentRange[], double current_nA[], int nCount, int nSampleInterval_us, int nSamplePoint, int eSampleType)
{
    return CmtiSdk::Cmti_GetDpsCurrent(m_hDevClient, powerId, currentRange, current_nA, nCount, nSampleInterval_us, nSamplePoint, eSampleType);
}

int DeviceClient::GetDpsAllSamplePointCurrent(const int powerId[], const int currentRange[], double current_nA[], int nCount, int nSampleInterval_us, int nSamplePoint)
{
    return CmtiSdk::Cmti_GetDpsAllSamplePointCurrent(m_hDevClient, powerId, currentRange, current_nA, nCount, nSampleInterval_us, nSamplePoint);
}

int DeviceClient::SetExtendGpioPinLevel(uint16_t pin, uint16_t level)
{
    return CmtiSdk::Cmti_SetExtendGpioPinLevel(m_hDevClient, pin, level);
}

int DeviceClient::GetExtendGpioPinLevel(uint16_t pin, uint16_t& level)
{
    return CmtiSdk::Cmti_GetExtendGpioPinLevel(m_hDevClient, pin, level);
}

int DeviceClient::SetExtendGpioPinDir(uint16_t pin, uint16_t dir)
{
    return CmtiSdk::Cmti_SetExtendGpioPinDir(m_hDevClient, pin, dir);
}

int DeviceClient::GetExtendGpioPinDir(uint16_t pin, uint16_t& dir)
{
    return CmtiSdk::Cmti_GetExtendGpioPinDir(m_hDevClient, pin, dir);
}

int DeviceClient::EnterDCTest(int32_t nPositiveVoltage)
{
    return CmtiSdk::Cmti_EnterDCTest(m_hDevClient, nPositiveVoltage);
}

int DeviceClient::LeaveDCTest()
{
    return CmtiSdk::Cmti_LeaveDCTest(m_hDevClient);
}

int DeviceClient::PMUForceCurrent(int32_t nSupplyCurrent_uA, int32_t nClampVol_mV, const int32_t nPins[], uint32_t nCount)
{
    return CmtiSdk::Cmti_PMUForceCurrent(m_hDevClient, nSupplyCurrent_uA, nClampVol_mV, nPins, nCount);
}

int DeviceClient::PMUMeasurePinVoltage(int32_t nDelayTime_ms, int32_t nSamplePoints, int32_t nSampleInterval_us, int32_t nSampleValType, const int32_t nPins[], int32_t nMeasVol_uV[], int32_t nCount)
{
    return CmtiSdk::Cmti_PMUMeasurePinVoltage(m_hDevClient, nDelayTime_ms, nSamplePoints, nSampleInterval_us, nSampleValType, nPins, nMeasVol_uV, nCount);
}

int DeviceClient::PMUForceVoltage(int32_t nSupplyVoltage_uV, int32_t nCurrentRange, int32_t nClampCurrent_uA, const int32_t nPins[], uint32_t nCount)
{
    return CmtiSdk::Cmti_PMUForceVoltage(m_hDevClient, nSupplyVoltage_uV, nCurrentRange, nClampCurrent_uA, nPins, nCount);
}

int DeviceClient::PMUMeasurePinCurrent(int32_t nDelayTime_ms, int32_t nSamplePoints, int32_t nSampleInterval_us, int32_t nSampleValType, const int32_t nPins[], double nMeasCur_nA[], int32_t nCount)
{
    return CmtiSdk::Cmti_PMUMeasurePinCurrent(m_hDevClient, nDelayTime_ms, nSamplePoints, nSampleInterval_us, nSampleValType, nPins, nMeasCur_nA, nCount);
}

int DeviceClient::SetPinVoltage(const int32_t nPins[], int32_t nCount, int32_t nSupplyVoltage_uV)
{
    return CmtiSdk::Cmti_SetPinVoltage(m_hDevClient, nPins, nCount, nSupplyVoltage_uV);
}

int DeviceClient::SetPinHighImpedanceState(const int32_t pinIds[], int32_t count)
{
    return CmtiSdk::Cmti_SetPinHighImpedanceState(m_hDevClient, pinIds, count);
}

int DeviceClient::SetTransmissionMode(int32_t transmitterMode)
{
    return CmtiSdk::Cmti_SetTransmissionMode(m_hDevClient, transmitterMode);
}

int DeviceClient::GetTransmissionMode(int32_t &pTransmitterMode)
{
    return CmtiSdk::Cmti_GetTransmissionMode(m_hDevClient, &pTransmitterMode);
}

int DeviceClient::GrabContinuousFrames(uint8_t *pImageBuf[], uint64_t pHeadTimestamp[], uint64_t pTailTimestamp[],
    uint32_t pFrameSequence[], int32_t imageCount, int32_t frameSize, int32_t timeout_ms)
{
    return CmtiSdk::Cmti_GrabContinuousFrames(m_hDevClient, pImageBuf, pHeadTimestamp, pTailTimestamp, pFrameSequence, imageCount, frameSize, timeout_ms);
}

int DeviceClient::CalcAverageImageAsync(int32_t nImageCount, T_CalcAverageImageAsyncContext* pAsyncContext)
{
    return CmtiSdk::Cmti_CalcAverageImageAsync(m_hDevClient, nImageCount, pAsyncContext);
}

int DeviceClient::WaitForAsyncResult(void * pAsyncContext, int32_t timeout_ms)
{
    return CmtiSdk::Cmti_WaitForAsyncResult(m_hDevClient, pAsyncContext, timeout_ms);
}

int DeviceClient::SetPinOutputPwmFrequency(int32_t pin, int32_t freq_Hz)
{
    return CmtiSdk::Cmti_SetPinOutputPwmFrequency(m_hDevClient, pin, freq_Hz);
}

int DeviceClient::SetPowerFixForce(const int32_t nPowerId[], const int32_t nFix[], int32_t nCount)
{
    return CmtiSdk::Cmti_SetPowerFixForce(m_hDevClient, nPowerId, nFix, nCount);
}

int DeviceClient::GetPowerFixForceState(const int32_t nPowerId[], int32_t nFix[], int32_t nCount)
{
    return CmtiSdk::Cmti_GetPowerFixForceState(m_hDevClient, nPowerId, nFix, nCount);
}

int DeviceClient::ImageProcessingControl(uint32_t ctrl, uint32_t pluginID)
{
    return ERR_NotImplemented;
}

int DeviceClient::SetImageProcessingParam(uint32_t pluginID, void *param, int size)
{
    return ERR_NotImplemented;
}

int DeviceClient::SendUpgradeFile(const char *filePathName)
{
    return CmtiSdk::Cmti_SendUpgradeFile(m_hDevClient, filePathName);
}

int DeviceClient::PowerPinOsTest(const int powerId[], const int supplyVoltage_mV[], const int upperLimitCurrent_nA[], const int autoHighPrecision[], int leakCurrent_nA[], int count)
{
    return CmtiSdk::Cmti_PowerPinOsTest(m_hDevClient, powerId, supplyVoltage_mV, upperLimitCurrent_nA, autoHighPrecision, leakCurrent_nA, count);
}

int DeviceClient::SetI2cPullupResistor(int sclPullupResistor, int sdaPullupResistor)
{
    return CmtiSdk::Cmti_SetI2cPullupResistor(m_hDevClient, sclPullupResistor, sdaPullupResistor);
}

int DeviceClient::GetI2cPullupResistor(int &sclPullupResistor, int &sdaPullupResistor)
{
    return CmtiSdk::Cmti_GetI2cPullupResistor(m_hDevClient, &sclPullupResistor, &sdaPullupResistor);
}

int DeviceClient::SetMipiClockMode(bool discontinuous)
{
    return CmtiSdk::Cmti_SetMipiClockMode(m_hDevClient, discontinuous);
}

int DeviceClient::GetMipiClockMode(bool &discontinuous)
{
    return CmtiSdk::Cmti_GetMipiClockMode(m_hDevClient, &discontinuous);
}

int DeviceClient::SetTestPattern(uint16_t mode, uint16_t fps)
{
    return CmtiSdk::Cmti_SetTestPattern(m_hDevClient, mode, fps);
}

int DeviceClient::GetTestPattern(uint16_t &mode, uint16_t &fps)
{
    return CmtiSdk::Cmti_GetTestPattern(m_hDevClient, &mode, &fps);
}

int DeviceClient::SetTransmitDelay(uint32_t delay)
{
    return CmtiSdk::Cmti_SetTransmitDelay(m_hDevClient, delay);
}

int DeviceClient::GetTransmitDelay(uint32_t &delay)
{
    return CmtiSdk::Cmti_GetTransmitDelay(m_hDevClient, &delay);
}

int DeviceClient::GetSensorFrameCountFromVideoOn(int &nFrameCount, int &nElapsed_ms)
{
    return CmtiSdk::Cmti_GetSensorFrameCountFromVideoOn(m_hDevClient, &nFrameCount, &nElapsed_ms);
}

int DeviceClient::GetReceivedFrameCountFromVideoOn(int &nFrameCount, int &nElapsed_ms)
{
    return CmtiSdk::Cmti_GetReceivedFrameCountFromVideoOn(m_hDevClient, &nFrameCount, &nElapsed_ms);
}
