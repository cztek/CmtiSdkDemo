#include "CZTEKBoard.h"
#include <memory>
#include "SensorSettingProvider.h"
#include "CmtiSdkWrapper.h"

CZTEKBoard::CZTEKBoard()
{
    m_pSensorSetting = nullptr;
}

CZTEKBoard::~CZTEKBoard()
{
    m_pSensorSetting = nullptr;
}

bool CZTEKBoard::LoadSensorSettings(const QString& qsSettingFileName)
{
    bool bFlag = SensorSettingProvider::Instance().LoadSensorSettingFile(qsSettingFileName); // FIXME：根据业务逻辑加载不同配置文件
    m_pSensorSetting = &SensorSettingProvider::Instance().m_sensorSetting;
    return bFlag;
}

void CZTEKBoard::EnumerateDevice(QList<QString> &deviceNameList)
{
    deviceNameList.clear();
    std::vector<std::string> stdDeviceNameList;
    DeviceController::Instance().EnumerateDevice(stdDeviceNameList);
    for (auto it = stdDeviceNameList.begin(); it != stdDeviceNameList.end(); ++it)
    {
        deviceNameList.append(QString::fromStdString(*it));
    }
}

void CZTEKBoard::BindSocket(int nSocIndex, const QString& qsDevName)
{
    DeviceController::Instance().BindSocket(nSocIndex, qsDevName.toStdString());
}

bool CZTEKBoard::fnDeviceInitialize(int nSocIndex, int& nDLLV)
{
    (void)nSocIndex;

    nDLLV = 0; // todo

    return true;
}

bool CZTEKBoard::fnPowerOnCanyouPinVoltSet(int nSocIndex)
{
    int ec = 0;
    DeviceClient* devCli = DeviceController::Instance()[nSocIndex];
    if (devCli == nullptr)
        return false;
    // 1. 上电
#if 0 // 推荐方式：所有电压一次下发，时序按前后顺序
    std::vector<T_Power> powerList;
    int count = m_pSensorSetting->PowerCount;
    for (int i = 0; i < count; i++)
    {
        T_Power power;
        power.Id = m_pSensorSetting->Powers[i].Id;
        power.Value = m_pSensorSetting->Powers[i].Value;
        power.Delay_ms = m_pSensorSetting->Powers[i].Delay_ms;
        powerList.push_back(power);
    }
    ec = devCli->SetSensorPower(powerList);
#else // 单路电源操作
    int count = m_pSensorSetting->PowerCount;
    for (int i = 0; i < count; i++)
    {
        T_Power power;
        power.Id = m_pSensorSetting->Powers[i].Id;
        power.Value = m_pSensorSetting->Powers[i].Value;
        power.Delay_ms = m_pSensorSetting->Powers[i].Delay_ms;
        ec = devCli->SetSensorPower(power);
        if (ec != ERR_NoError)
            break;
    }
#endif
    m_bIsPowerOn = (ERR_NoError == ec);
    if (!m_bIsPowerOn)
        return false;

    // 2. 设置Sensor Clock
    ec = devCli->SetSensorClock(m_pSensorSetting->Mclk_kHz / 100);
    if (ec < 0)
        return false;

    // 3. 设置GPIO
    uint16_t resetPwdnPin = IO_Reset | IO_Pwdn1;
    uint16_t resetPwdnLevel = 0;
    if (m_pSensorSetting->Reset)
        resetPwdnLevel |= IO_Reset;
    if (m_pSensorSetting->Pwdn)
        resetPwdnLevel |= IO_Pwdn1;
    ec = devCli->SetSensorGpioPinLevel(resetPwdnPin, resetPwdnLevel);
    if (ec < 0)
        return false;

    // 4. 设置采集超时
    ec = devCli->SetGrabTimeoutAndLoopTimes(500, 3);
    if (ec < 0)
        return false;

    // 5. 设置图像参数
    E_ImageFormat outImageFormat = m_pSensorSetting->ImageFormat; // todo: 可以修改为想要的格式
    ec = devCli->SetFrameParam(m_pSensorSetting->ImageFormat, m_pSensorSetting->ImageMode,
        m_pSensorSetting->PixelWidth, m_pSensorSetting->PixelHeight, outImageFormat, m_pSensorSetting->CropParam.X, m_pSensorSetting->CropParam.Y,
        m_pSensorSetting->CropParam.Width, m_pSensorSetting->CropParam.Height);
    if (ec < 0)
        return false;

    // 6. Set mipi param
    ec = devCli->SetMipiParam(m_pSensorSetting->Lanes, m_pSensorSetting->MipiFreq, 0);
    if (ec != 0)
        return false;

    // 7. Set interface type
    ec = devCli->SetSensorInterface((uint32_t)m_pSensorSetting->InterfaceType);
    if (ec != 0)
        return false;

    // 8. 写点亮寄存器
    T_I2CCommParam& i2cParam = m_pSensorSetting->I2cParam;
    count = m_pSensorSetting->FullModeParamCount;
    std::vector<T_RegConf> regList;
    for (int i = 0; i < count; i++)
    {
        T_RegConf regConf;
        regConf.Addr = m_pSensorSetting->FullModeParams[i].Addr;
        regConf.Value = m_pSensorSetting->FullModeParams[i].Value;
        regConf.Delay_ms = m_pSensorSetting->FullModeParams[i].Delay_ms;
        regList.push_back(regConf);
    }
    ec = devCli->WriteDiscreteI2c(i2cParam.Addr, i2cParam.Speed * 100, i2cParam.RegBitsMode, regList);
    if (ec < 0)
        return false;

    // 9. 打开arm接收端
    ec = devCli->VideoControl(1);
    if (ec < 0)
        return false;

    m_bIsVideoOn = (ERR_NoError == ec);

    return (ERR_NoError == ec);
}

bool CZTEKBoard::fnCanyouPowerOff(int nSocIndex)
{
    int ec = 0;
    DeviceClient* devCli = DeviceController::Instance()[nSocIndex];
    if (devCli == nullptr)
        return false;
    if (m_bIsVideoOn) // 先关图
    {
        // 关闭arm端接收
        devCli->VideoControl(0);
        // 关闭gpio
        devCli->SetSensorGpioPinLevel(IO_Pwdn1 | IO_Pwdn1, 0);
        // 关时钟
        devCli->SetSensorClock(0);
        m_bIsVideoOn = false;
    }

    if (m_bIsPowerOn) // 再下电
    {
        std::vector<T_Power> powerList;
        for (int i = m_pSensorSetting->PowerCount - 1; i >= 0; i--) // 逆序下电
        {
            T_Power power;
            power.Id = m_pSensorSetting->Powers[i].Id;
            power.Value = 0;
            power.Delay_ms = m_pSensorSetting->Powers[i].Delay_ms;
            powerList.push_back(power);
        }
        ec = devCli->SetSensorPower(powerList);
        m_bIsPowerOn = false;
    }

    return (ERR_NoError == ec);
}

void CZTEKBoard::Make10BitModeLSB(unsigned char* pImage, uint16_t* pDest, unsigned int nWidth, unsigned int nHeight)
{
}

void CZTEKBoard::Make10BitModeMSB(unsigned char* pImage, uint16_t* pDest, unsigned int nWidth, unsigned int nHeight)
{
}

bool CZTEKBoard::fnGetOneFrame(int nSocIndex, QString station, QString barcode, std::string FilePath, std::string fileName)
{
    DeviceClient* devCli = DeviceController::Instance()[nSocIndex];
    if (devCli == nullptr)
        return false;
    int nBufferLen = 120; // todo
    std::unique_ptr<uint8_t[]> pBuffer(new uint8_t[nBufferLen]());
    uint64_t headTimestamp, tailTimestamp;
    uint32_t frameSequence;
    int ec = devCli->GrabFrame2(pBuffer.get(), nBufferLen, headTimestamp, tailTimestamp, frameSequence);
    if (ERR_NoError == ec)
    {
        // save to file
    }
    return false;
}

bool CZTEKBoard::ReadI2CData(int nSocIndex, ushort nSlave, ushort nAddr, ushort nDataLen, uint8_t* pBuf, bool bAddr16)
{
    DeviceClient* devCli = DeviceController::Instance()[nSocIndex];
    if (devCli == nullptr)
        return false;

    T_I2CCommParam& i2cParam = m_pSensorSetting->I2cParam;
    int ec = devCli->ReadContinuousI2c(nSlave, i2cParam.Speed, nAddr, bAddr16 ? 2 : 1, pBuf, nDataLen);

    return (ERR_NoError == ec);
}

bool CZTEKBoard::WriteI2CData(int nSocIndex, ushort nSlave, ushort nAddr, ushort nDataLen, const uint8_t* pBuf, bool bAddr16)
{
    DeviceClient* devCli = DeviceController::Instance()[nSocIndex];
    if (devCli == nullptr)
        return false;
    
    T_I2CCommParam& i2cParam = m_pSensorSetting->I2cParam;
    int ec = devCli->WriteContinuousI2c(nSlave, i2cParam.Speed, nAddr, bAddr16 ? 2 : 1, pBuf, nDataLen);

    return (ERR_NoError == ec);
}

bool CZTEKBoard::fnOSTest(int nSocIndex, QList<float>& reading)
{
    DeviceClient* devCli = DeviceController::Instance()[nSocIndex];
    if (devCli == nullptr)
        return false;

    // 设置测试电流方向
    int ec = 0;
    ec = devCli->SetOsTestCurrentDirection(true); // true为正向电流，false为反向电流
    if (ec != ERR_NoError && ec != ERR_NotImplemented)
    {
        return false;
    }
    // 测试
    std::vector<uint> testPinVec;
    // todo: 以下PIN的选择根据实际模组要测试的PIN来选择
    if (m_pSensorSetting->InterfaceType == IT_MIPI_CPHY) // 如果是CPHY
    {
        testPinVec.push_back(OSM_PIN_AVDD);
        testPinVec.push_back(OSM_PIN_DOVDD);
        testPinVec.push_back(OSM_PIN_DVDD);
        testPinVec.push_back(OSM_PIN_AFVCC);
        testPinVec.push_back(OSM_PIN_VPP);
        testPinVec.push_back(OSM_PIN_MCLK);
        testPinVec.push_back(OSM_PIN_SCL);
        testPinVec.push_back(OSM_PIN_SDA);
        testPinVec.push_back(OSM_PIN_PWDN);
        testPinVec.push_back(OSM_PIN_RST);
        testPinVec.push_back(OSM_PIN_PO1);
        testPinVec.push_back(OSM_PIN_PO2);
        testPinVec.push_back(OSM_PIN_PO3);
        testPinVec.push_back(OSM_PIN_PO4);
        testPinVec.push_back(OSM_PIN_SGND1);
        testPinVec.push_back(OSM_PIN_AVDD2);
        testPinVec.push_back(OSM_PIN_C_PHY_TRIO_0A);
        testPinVec.push_back(OSM_PIN_C_PHY_TRIO_0B);
        testPinVec.push_back(OSM_PIN_C_PHY_TRIO_0C);
        testPinVec.push_back(OSM_PIN_C_PHY_TRIO_1A);
        testPinVec.push_back(OSM_PIN_C_PHY_TRIO_1B);
        testPinVec.push_back(OSM_PIN_C_PHY_TRIO_1C);
        testPinVec.push_back(OSM_PIN_C_PHY_TRIO_2A);
        testPinVec.push_back(OSM_PIN_C_PHY_TRIO_2B);
        testPinVec.push_back(OSM_PIN_C_PHY_TRIO_2C);
        testPinVec.push_back(OSM_PIN_SGND2);
    }
    else // 如果是DPHY
    {
        testPinVec.push_back(OSM_PIN_AVDD);
        testPinVec.push_back(OSM_PIN_DOVDD);
        testPinVec.push_back(OSM_PIN_DVDD);
        testPinVec.push_back(OSM_PIN_AFVCC);
        testPinVec.push_back(OSM_PIN_MCLK);
        testPinVec.push_back(OSM_PIN_SCL);
        testPinVec.push_back(OSM_PIN_SDA);
        testPinVec.push_back(OSM_PIN_PWDN);
        testPinVec.push_back(OSM_PIN_RST);
        testPinVec.push_back(OSM_PIN_MIPI_D3P);
        testPinVec.push_back(OSM_PIN_MIPI_D3N);
        testPinVec.push_back(OSM_PIN_MIPI_D2P);
        testPinVec.push_back(OSM_PIN_MIPI_D2N);
        testPinVec.push_back(OSM_PIN_MIPI_D1P);
        testPinVec.push_back(OSM_PIN_MIPI_D1N);
        testPinVec.push_back(OSM_PIN_MIPI_D0P);
        testPinVec.push_back(OSM_PIN_MIPI_D0N);
        testPinVec.push_back(OSM_PIN_MIPI_CLKP);
        testPinVec.push_back(OSM_PIN_MIPI_CLKN);
        testPinVec.push_back(OSM_PIN_SGND1);
#if 0 // 这部分接口只有部分测试盒支持，为了向下兼容，这里注释掉，真正的测试软件需要做界面还选择测试的引脚
        testPinVec.push_back(OSM_PIN_VPP);
        testPinVec.push_back(OSM_PIN_AVDD2);
        testPinVec.push_back(OSM_PIN_PO1);
        testPinVec.push_back(OSM_PIN_PO2);
        testPinVec.push_back(OSM_PIN_PO3);
        testPinVec.push_back(OSM_PIN_PO4);
#endif
    }
    size_t pinCnt = testPinVec.size();
    std::unique_ptr<uint[]> openStdVol_uV(new uint[pinCnt]());
    std::unique_ptr<uint[]> shortStdVol_uV(new uint[pinCnt]());
    for (uint i = 0; i < pinCnt; i++) // TODO: 这里填写每个Pin的开短路判断阈值
    {
        openStdVol_uV[i] = 1000 * 1000;  // 开路标准，单位uV，可以独立设置，推荐1V
        shortStdVol_uV[i] = 200 * 1000;  // 短路标准，单位uV，可以独立设置，推荐200mV
    }

    ec = devCli->SetOsTestConfig(
        1400 * 1000, // 测试供电电压，单位uV，推荐1.4V
        500,         // 测试供电电流，单位uA，推荐500uA
        testPinVec.data(),
        openStdVol_uV.get(),
        shortStdVol_uV.get(),
        (uint32_t)pinCnt);
    if (ec < 0)
        return false;

    struct T_OsResult // 详细Open/Short结果可以查看这个数据结构
    {
        int Pin;
        int State; // 0: OK 1: Open 2: Short
        int ShortPin;
        uint32_t m_nVoltage_mV;
    };
    std::unique_ptr<uint[]> readOpenVol_uV(new uint[pinCnt]());
    std::unique_ptr<uint[]> readShortVol_uV(new uint[pinCnt]());
    std::unique_ptr<uint[]> readResult(new uint[pinCnt]());
    ec = devCli->ReadOsTestResult(testPinVec.data(), readOpenVol_uV.get(), readShortVol_uV.get(), readResult.get(), (uint32_t)pinCnt);
    std::vector<T_OsResult> osResult;
    reading.clear();
    if (ERR_NoError == ec)
    {
        for (size_t i = 0; i < pinCnt; i++)
        {
            T_OsResult osState;
            osState.State = 0; // OK
            osState.m_nVoltage_mV = readOpenVol_uV[i] / 1000;
            osState.ShortPin = -1;
            if (readResult[i] & OS_Result_Open)
            {
                osState.State = 1; // Open
            }
            else if (readResult[i] & OS_Result_Short)
            {
                osState.State = 2; // Short
                osState.m_nVoltage_mV = readShortVol_uV[i] / 1000;
                osState.ShortPin = readResult[i] & OS_TEST_SHORT_PIN_MASK;
            }
            osResult.push_back(osState);
            reading.append(osState.m_nVoltage_mV);
        }
    }

    return (ERR_NoError == ec);
}

bool CZTEKBoard::fnDynamicReading(int nSocIndex, QList<float>& reading)
{
    DeviceClient* devCli = DeviceController::Instance()[nSocIndex];
    if (devCli == nullptr)
        return false;

    int count = m_pSensorSetting->PowerCount;
    std::unique_ptr<int[]> pPowerId(new int[count]());
    std::unique_ptr<int[]> pUpperLimitCurrent_nA(new int[count]());
    std::unique_ptr<uint16_t[]> pAutoHighPrecision(new uint16_t[count]());
    std::unique_ptr<float[]> pCurrent_nA(new float[count]());
    for (int i = 0; i < count; i++)
    {
        pPowerId[i] = m_pSensorSetting->Powers[i].Id;
        pUpperLimitCurrent_nA[i] = 1000 * 1000; // 电流上限值，对于上限以内的测量值会更精准，超出上限的值误差会较大。
        pAutoHighPrecision[i] = 1;              // 各类型电流自动匹配高精度标志，0：不匹配，1：向下匹配高精度
    }

    int ec = devCli->GetCurrentV2(pPowerId.get(), pUpperLimitCurrent_nA.get(), pAutoHighPrecision.get(), pCurrent_nA.get(), count);
    if (ec == ERR_NoError)
    {
        for (int i = 0; i < count; i++)
        {
            reading.append(pCurrent_nA[i]);
        }
    }

    return (ERR_NoError == ec);
}

bool CZTEKBoard::fnDynamicOffset(int nSocIndex, QList<float>& reading)
{
    DeviceClient* devCli = DeviceController::Instance()[nSocIndex];
    if (devCli == nullptr)
        return false;
    return false;
}

bool CZTEKBoard::fnStandbyReading(int nSocIndex, QList<float>& reading)
{
    DeviceClient* devCli = DeviceController::Instance()[nSocIndex];
    if (devCli == nullptr)
        return false;

    // todo: 静态电流状态

    // 读电流
    int count = m_pSensorSetting->PowerCount;
    std::unique_ptr<int[]> pPowerId(new int[count]());
    std::unique_ptr<int[]> pUpperLimitCurrent_nA(new int[count]());
    std::unique_ptr<uint16_t[]> pAutoHighPrecision(new uint16_t[count]());
    std::unique_ptr<float[]> pCurrent_nA(new float[count]());
    for (int i = 0; i < count; i++)
    {
        pPowerId[i] = m_pSensorSetting->Powers[i].Id;
        pUpperLimitCurrent_nA[i] = 1000 * 1000; // 电流上限值，对于上限以内的测量值会更精准，超出上限的值误差会较大。
        pAutoHighPrecision[i] = 1;              // 各类型电流自动匹配高精度标志，0：不匹配，1：向下匹配高精度
    }

    int ec = devCli->GetCurrentV2(pPowerId.get(), pUpperLimitCurrent_nA.get(), pAutoHighPrecision.get(), pCurrent_nA.get(), count);
    if (ec == ERR_NoError)
    {
        for (int i = 0; i < count; i++)
        {
            reading.append(pCurrent_nA[i]);
        }
    }

    return (ERR_NoError == ec);
}

bool CZTEKBoard::fnStandbyOffset(int nSocIndex, QList<float>& reading)
{
    DeviceClient* devCli = DeviceController::Instance()[nSocIndex];
    if (devCli == nullptr)
        return false;
    return false;
}
