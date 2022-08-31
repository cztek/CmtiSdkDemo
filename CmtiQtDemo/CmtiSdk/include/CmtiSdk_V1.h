#ifndef CMTISDK_V1_H
#define CMTISDK_V1_H 1

/**
 * V1�汾ΪAAʹ�ð汾��ֻ֧��MSVC++��������
 */

#include "TypeDefs.h"

#ifndef CMTISDK_API
#ifdef CMTISDK_EXPORTS
#ifdef __GNUC__
#define CMTISDK_API __attribute__((visibility("default")))
#else
#define CMTISDK_API __declspec(dllexport)
#endif
#else
#ifdef __GNUC__
#define CMTISDK_API __attribute__((visibility("default")))
#else
#define CMTISDK_API __declspec(dllimport)
#endif
#endif /* CMTISDK_EXPORTS */
#endif /* CMTISDK_API */

///////////////////////////////////////// C++ interface ////////////////////////////////////////////

/**
 * @brief SDK�¼�������
 *
 */
class ISdkEventHandler
{
public:
    /**
     * @brief �豸�仯�¼�
     *
     * @param devName �豸��
     * @param state (0 - offline 1 - online)
     */
    virtual void OnDeviceChanged(const char *devName, int state) = 0;
    /**
     * @brief �ɼ�֡��FPS�����¼�
     *
     * @param chnIdx ��Ƶͨ������
     * @param fps �ɼ�֡��FPS
     */
    virtual void OnCaptureFpsChanged(uint chnIdx, float fps) = 0;
    /**
     * @brief ����֡��FPS�����¼�
     *
     * @param chnIdx ��Ƶͨ������
     * @param fps ����֡��FPS
     */
    virtual void OnTransmitFpsChanged(uint chnIdx, float fps) = 0;
    /**
     * @brief ������֡�ʸ����¼�
     *
     * @param chnIdx ��Ƶͨ������
     * @param fer ������֡��
     */
    virtual void OnTransmitFerChanged(uint chnIdx, float fer) = 0;
};

/**
 * @brief IDeviceCli
 *
 * �����豸�ӿ�
 */
class IDeviceCli
{
public:
    /**
     * @brief ע��SDK�¼���������ע���SDK�������ϱ�����ע����¼�
     *
     * @param eventHandler SDK�¼��������ӿ�
     */
    virtual void RegisterEventHandler(ISdkEventHandler *eventHandler) = 0;
    /**
     * @brief ��ȡ�豸��
     *
     * @param devName �豸���������߸����ڴ����
     * @return int ������
     */
    virtual int GetDeviceName(char *devName) const = 0;

    /**
     * @brief ����дSensor�Ĵ�������
     *
     * @param chnIdx ��Ƶͨ������
     * @param slaveAddr I2C�ӵ�ַ
     * @param speedkHz I2C����
     * @param mode I2Cģʽ
     * @param pRegParam ָ��Ĵ���������ָ�룬��ʽΪ���Ĵ�����ַ+�Ĵ���ֵ+��ʱ�����Ĵ�����ַ���Ĵ���ֵ����ʱ��ʹ��uint��ʾ����˳����
     * @param length ����pRegParam�ĳ��ȣ������Ĵ�����ַ+�Ĵ���ֵ+��ʱ�����ܳ���
     * @return int ������
     */
    virtual int WriteSensorMultiRegs(uint chnIdx, uint slaveAddr, uint speedkHz, uint mode, const uint *pRegParam, int length) = 0;
    /**
     * @brief ����дSensor�Ĵ�������������ʱ���ܣ�
     *
     * @param chnIdx ��Ƶͨ������
     * @param slaveAddr I2C�ӵ�ַ
     * @param speedkHz I2C����
     * @param mode I2Cģʽ
     * @param regAddr[] �Ĵ�����ַ����
     * @param regData[] �Ĵ���ֵ����
     * @param regDelay[] �Ĵ�����ʱ����
     * @param regNum �Ĵ�����������ǰ���������Ԫ�ظ���
     * @return int ������
     */
    virtual int WriteSensorMultiRegsWithDelay(uint chnIdx, uint slaveAddr, uint speedkHz, uint mode,
        const ushort regAddr[], const ushort regData[], const ushort regDelay[], int regNum) = 0;

    /**
     * @brief д��ɢ��I2C����
     *
     * @param chnIdx ��Ƶͨ������
     * @param slaveAddr I2C�ӵ�ַ
     * @param speedkHz I2C����
     * @param mode I2Cģʽ
     * @param regs[] �Ĵ�����ַ
     * @param value[] �Ĵ���ֵ
     * @param regNum �Ĵ�������
     * @return int ������
     */
    virtual int WriteDiscreteI2c(uint chnIdx, uint slaveAddr, uint speedkHz, uint mode, const ushort regs[], const ushort value[], uint regNum) = 0;
    /**
     * @brief ����ɢ��I2C����
     *
     * @param chnIdx ��Ƶͨ������
     * @param slaveAddr I2C�ӵ�ַ
     * @param speedkHz I2C����
     * @param mode I2Cģʽ
     * @param regs[] �Ĵ�����ַ
     * @param value[] �Ĵ���ֵ
     * @param regNum �Ĵ�������
     * @return int ������
     */
    virtual int ReadDiscreteI2c(uint chnIdx, uint slaveAddr, uint speedkHz, uint mode, const ushort regs[], ushort value[], uint regNum) = 0;
    /**
     * @brief д������I2C����
     *
     * @param chnIdx ��Ƶͨ������
     * @param slaveAddr I2C�ӵ�ַ
     * @param speedkHz I2C����
     * @param regAddr �Ĵ�����ַ
     * @param regAddrSize �Ĵ�����ַ����
     * @param data ����
     * @param dataSize ���ݳ���
     * @return int ������
     */
    virtual int WriteContinuousI2c(uint chnIdx, uint slaveAddr, uint speedkHz, uint regAddr, uint regAddrSize, const uint8_t *data, uint dataSize) = 0;
    /**
     * @brief ��������I2C����
     *
     * @param chnIdx ��Ƶͨ������
     * @param slaveAddr I2C�ӵ�ַ
     * @param speedkHz I2C����
     * @param regAddr �Ĵ�����ַ
     * @param regAddrSize �Ĵ�����ַ����
     * @param data ����
     * @param dataSize ���ݳ���
     * @return int ������
     */
    virtual int ReadContinuousI2c(uint chnIdx, uint slaveAddr, uint speedkHz, uint regAddr, uint regAddrSize, uint8_t *data, uint dataSize) = 0;
    /**
     * @brief дSensorʱ��
     *
     * @param chnIdx ��Ƶͨ������
     * @param clk100kHz ʱ�ӣ�100kHzΪ��λ��
     * @return int ������
     */
    virtual int SetSensorClock(uint chnIdx, uint clk100kHz) = 0;
    /**
     * @brief ��Sensorʱ��
     *
     * @param chnIdx ��Ƶͨ������
     * @param clk100kHz ʱ�ӣ�100kHzΪ��λ��
     * @return int ������
     */
    virtual int GetSensorClock(uint chnIdx, uint &clk100kHz) = 0;
    /**
     * @brief дMipi����
     *
     * @param chnIdx ��Ƶͨ������
     * @param laneNum Mipi Lanes
     * @param freqMHz Mipiʱ�ӣ�Ĭ��800MHz
     * @param virtualChannel Mipi����ͨ����Ĭ��0
     * @return int ������
     */
    virtual int SetMipiParam(uint chnIdx, uint laneNum, uint freqMHz, uint virtualChannel) = 0;
    /**
     * @brief ��Mipi����
     *
     * @param chnIdx ��Ƶͨ������
     * @param laneNum Mipi Lanes
     * @param freqMHz Mipiʱ��
     * @param virtualChannel Mipi����ͨ��
     * @return int ������
     */
    virtual int GetMipiParam(uint chnIdx, uint &laneNum, uint &freqMHz, uint &virtualChannel) = 0;
    /**
     * @brief дSensor GPIOֵ
     *
     * @param chnIdx ��Ƶͨ������
     * @param pin Ҫ������Pin����
     * @param level Pin���ŵ�ƽMask
     * @return int ������
     */
    virtual int SetSensorGpioPinLevel(uint chnIdx, ushort pin, ushort level) = 0;
    /**
     * @brief ��Sensor GPIOֵ
     *
     * @param chnIdx
     * @param pin Ҫ������Pin����
     * @param level Pin���ŵ�ƽMask
     * @return int ������
     */
    virtual int GetSensorGpioPinLevel(uint chnIdx, ushort pin, ushort &level) = 0;
    /**
     * @brief дSensor GPIO����
     *
     * @param chnIdx ��Ƶͨ������
     * @param pin Ҫ������Pin����
     * @param dir Pin���ŷ���Mask
     * @return int ������
     */
    virtual int SetSensorGpioPinDir(uint chnIdx, ushort pin, ushort dir) = 0;
    /**
     * @brief дSensor GPIO����
     *
     * @param chnIdx ��Ƶͨ������
     * @param pin Ҫ������Pin����
     * @param dir Pin���ŷ���Mask
     * @return int ������
     */
    virtual int GetSensorGpioPinDir(uint chnIdx, ushort pin, ushort &dir) = 0;
    /**
     * @brief дSensor��ѹ
     *
     * @param chnIdx ��Ƶͨ������
     * @param powerIds[] ��ԴID���ο�E_PowerId����
     * @param voltagemV[] ��ѹֵ����λΪmV
     * @param delayms[] ��ʱ����λΪms
     * @param count ��ѹ����
     * @return int ������
     */
    virtual int SetSensorPower(uint chnIdx, const uint powerIds[], const uint voltagemV[], const uint delayms[], uint count) = 0;
    /**
     * @brief ��Sensor��ѹ
     *
     * @param chnIdx ��Ƶͨ������
     * @param powerIds[] ��ԴID���ο�E_PowerId����
     * @param voltagemV[] ��ѹֵ����λΪmV
     * @param delayms[] ��ʱ����λΪms
     * @param count ��ѹ����
     * @return int ������
     */
    virtual int GetSensorPower(uint chnIdx, const uint powerIds[], uint voltagemV[], uint delayms[], uint count) = 0;
    /**
     * @brief �豸ͼ��֡�������ڴ���Ƶǰ��Ҫ����ͼ��֡����
     *
     * @param chnIdx ��Ƶͨ������
     * @param imageFormat ͼ���ʽ���ο�E_ImageFormat����
     * @param imageMode ͼ��ģʽ���ο�E_ImageMode����
     * @param width ���
     * @param height �߶�
     * @param outImageFormat ���ͼ���ʽ���ο�E_ImageFormat����
     * @param cropLeft ��������x����
     * @param cropTop ��������y����
     * @param cropWidth ����������
     * @param cropHeight ��������߶�
     * @return int ������
     */
    virtual int SetFrameParam(uint chnIdx, uint imageFormat, uint imageMode, uint width, uint height, uint outImageFormat, 
        uint cropLeft, uint cropTop, uint cropWidth, uint cropHeight) = 0;
    /**
     * @brief ��ȡͼ��֡����
     *
     * @param chnIdx ��Ƶͨ������
     * @param imageFormat ͼ���ʽ���ο�E_ImageFormat����
     * @param imageMode ͼ��ģʽ���ο�E_ImageMode����
     * @param width ���
     * @param height �߶�
     * @param outImageFormat ���ͼ���ʽ
     * @param cropLeft ��������x����
     * @param cropTop ��������y����
     * @param cropWidth ����������
     * @param cropHeight ��������߶�
     * @return int ������
     */
    virtual int GetFrameParam(uint chnIdx, uint &imageFormat, uint &imageMode, uint &width, uint &height,
        uint &size, uint &cropLeft, uint &cropTop, uint &cropWidth, uint &cropHeight) = 0;
    /**
     * @brief ��Ƶ����
     *
     * @param chnIdx ��Ƶͨ������
     * @param ctrl �������0Ϊ�رգ�1Ϊ��
     * @return int ������
     */
    virtual int VideoControl(uint chnIdx, uint ctrl) = 0;
    /**
     * @brief �������
     *
     * @param chnIdx ��Ƶͨ������
     * @param ctrl ���������16λ��ʾ����ģʽ����16λΪ��������(0Ϊֹͣ���䣬1Ϊ��������)
     * @return int ������
     */
    virtual int TransmitControl(uint chnIdx, uint ctrl) = 0;
    /**
     * @brief ����Sensor�˿�
     *
     * @param chnIdx ��Ƶͨ������
     * @param port Sensor�˿ڣ��ο�E_InterfaceType����
     * @return int ������
     */
    virtual int SetSensorPort(uint chnIdx, uint port) = 0;
    /**
     * @brief ��ȡSensor�˿�
     *
     * @param chnIdx ��Ƶͨ������
     * @param port Sensor�˿ڣ��ο�E_InterfaceType����
     * @return int ������
     */
    virtual int GetSensorPort(uint chnIdx, uint &port) = 0;
    /**
     * @brief ���òɼ�֡��ʱ
     *
     * @param chnIdx ��Ƶͨ������
     * @param ��ʱʱ�䣬-1��ʾһֱ�ȴ�
     * @return int ������
     */
    virtual int SetGrabTimeout(uint chnIdx, uint grabTimeout) = 0;
    /**
     * @brief ��ȡ�ɼ�֡��ʱ
     *
     * @param chnIdx ��Ƶͨ������
     * @param ��ʱʱ�䣬-1��ʾһֱ�ȴ�
     * @return int ������
     */
    virtual int GetGrabTimeout(uint chnIdx, uint &grabTimeout) = 0;

    /**
     * @brief �ӻ���ؿ������ݵ�Ӧ�ò�
     *
     * @param chnIdx ��Ƶͨ������
     * @param pbuffer Ӧ�ò㻺����
     * @param bufferLen Ӧ�ò㻺��������
     * @param timestamp ʱ���
     * @return int ������
     */
    virtual int GrabFrame(uint chnIdx, uint8_t *pbuffer, int bufferLen, uint64_t &timestamp) = 0;
    virtual int GrabLatestFrame(uint chnIdx, uint8_t *pbuffer, int bufferLen, uint64_t &timestamp) = 0;
    /**
     * @brief �Ӷ�����ȡ��һ��Buffer(��EnqueueFrameBuffer���ʹ��)��ʹ�û�����еĻ�����������Ҫ�û������ڴ�
     *
     * @param chnIdx ��Ƶͨ������
     * @param bufIdx ������������ʹ����Ϻ���EnqueueFrameBuffer�й黹
     * @param pbuffer �������ͣ�������
     * @param timestamp ʱ���
     * @return int ������
     */
    virtual int DequeueFrameBuffer(uint chnIdx, int &bufIdx, uint8_t* &pbuffer, uint64_t &timestamp) = 0;
    virtual int DequeueLatestFrameBuffer(uint chnIdx, int &bufIdx, uint8_t* &pbuffer, uint64_t &timestamp) = 0;
    /**
     * @brief ���һ��Buffer(��DequeueFrameBuffer���ʹ��)
     *
     * @param chnIdx ��Ƶͨ������
     * @param bufIdx ����������
     * @return int ������
     */
    virtual int EnqueueFrameBuffer(uint chnIdx, int bufIdx) = 0;
    /**
     * @brief ���ÿ���·���Բ���
     *
     * @param chnIdx ��Ƶͨ������
     * @param supplyVol_uV ���Թ����ѹ���Ƽ�1.4V
     * @param supplyCurrent_uA ���Թ���������Ƽ�500uA
     * @param pinsId[] �����Ե�Pin���Ŷ��壬�ο�E_OSM_PIN_TYPE����
     * @param openStdVols_uV[] ��·��׼�����Զ������ã��Ƽ�1V
     * @param shortStdVols_uV[] ��·��׼�����Զ������ã��Ƽ�200mV
     * @param count ���Ÿ���
     * @return int ������
     */
    virtual int SetOsTestConfig(uint chnIdx, uint supplyVol_uV, uint supplyCurrent_uA, const uint pinsId[],
        const uint openStdVols_uV[], const uint shortStdVols_uV[], uint count) = 0;
    /**
     * @brief ������·���Խ��
     *
     * @param chnIdx ��Ƶͨ������
     * @param pinsId[] ���Ե�Pin���Ŷ��壬�ο�E_OSM_PIN_TYPE����
     * @param openVols_uV[] ��·��ѹ
     * @param shortVol_uV[] ��·��ѹ
     * @param results[] ���Խ��
     * @param pinCount ���Ÿ���
     * @return int ������
     */
    virtual int ReadOsTestResult(uint chnIdx, const uint pinsId[], uint openVols_uV[], uint shortVol_uV[], uint results[], uint pinCount) = 0;
    /**
     * @brief ��ȡ����
     *
     * @param chnIdx ��Ƶͨ������
     * @param powerIds[] ��ԴID���ο�E_PowerId����
     * @param currentRange[] �������̣��ο�E_CurrentRange����
     * @param current_nA[] ����ֵ����λΪnA
     * @param count ��Դ����
     * @return int ������
     */
    virtual int GetCurrent(uint chnIdx, const uint powerIds[], const uint currentRange[], float current_nA[], uint count) = 0;
    /**
     * @brief �򿪷�������
     *
     * @param chnIdx ��Ƶͨ������
     * @param ms ��ʱʱ�䣬��λms
     * @return int ������
     */
    virtual int SetBeepOn(uint chnIdx, uint ms) = 0;
    virtual int SendUpgradeFile(const char *filePathName) = 0;

    /**
    * @brief ����ROI������֧�����л�
    *
    * @param roiRect ROI���ο��壬�ο�T_Rect����
    * @param roiCount ROI���ο����
    * @return int ������
    */
    virtual int SetRoiParam(uint32_t chnIdx, const T_Rect roiRect[], uint32_t roiCount) = 0;
};

/**
 * @brief �豸������
 *
 * @class IDeviceController
 */
class IDeviceController
{
public:
    virtual ~IDeviceController() {}
    /**
     * @brief �������ƻ�ȡ�豸�ͻ���ʵ��
     *
     * @param devName �豸����
     * @return ����IDeviceCli*��ָ���豸�ͻ���ʵ��
     */
    virtual IDeviceCli *GetDeviceCliInstance(const char *devName) = 0;
    /**
     * @brief ע��SDK�¼�������
     *
     * @param eventHandler �¼�������
     */
    virtual void RegisterEventHandler(ISdkEventHandler *eventHandler) = 0;
    /**
     * @brief �豸���ֹ��ܿ��أ�ͨ����Ӧ�ó���ʼʱ���ÿ�������
     *
     * @param enabled ������ر�
     */
    virtual void SetDeviceDiscoveryEnabled(bool enabled) = 0;
    /**
     * @brief ִ��һ���豸����
     *
     */
    virtual void ToggleDiscovery() = 0;
    /**
     * @brief ö�ٵ�ʱ�����豸
     *
     * @param deviceNameList �����豸���б������߷����ڴ�
     * @param maxDeviceNum ���֧�ֵ��豸����
     * @param pDeviceNum ʵ�ʷ��ص��豸����
     * @return int ������
     */
    virtual int EnumerateDevice(char *deviceNameList[], int maxDeviceNum, int *pDeviceNum) = 0;

    /**
    * @brief ��ȡָ���豸�����б�
    *
    * @param devName �豸����
    * @param propNameList �������������б������߷����ڴ�
    * @param propValList ��������ֵ�б������߷����ڴ�
    * @param maxPropNum �������Ը���
    * @param pPropNum ʵ�ʷ��ص����Ը���
    * @return int ������
    */
    virtual int GetDevicePropList(const char *devName, char *propNameList[], char *propValList[], int maxPropNum, int *pPropNum) = 0;
};

extern "C" {
    CMTISDK_API void GetCztekLibVersion(char *libVer, int len);
    /**
    * @brief ����IDeviceController*��ָ���豸������
    *
    */
    CMTISDK_API IDeviceController *GetDeviceControllerInstance();
    /**
    * @brief �ͷ��豸������ʵ��
    *
    */
    CMTISDK_API void FreeDeviceControllerInstance();
}

// obsoleted API
#ifdef __cplusplus
extern "C" {
    namespace CmtiSdk {
#endif
        /**
        * @brief ���Ե�Դpin�ſ���·
        * @param nPowerId ��ԴID���ο�E_PowerId����
        * @param nSupplyVoltage_mV ���Թ����ѹ����λmV
        * @param nUpperLimitCurrent_nA  �������ޣ���λnA
        * @param bAutoHighPrecision[] �����͵����Զ�ƥ��߾��ȱ�־��0����ƥ�䣬1������ƥ��߾���
        * @param nLeakCurrent_nA ���������©��������λnA
        * @param nCount ��ԴID����
        * @return int ������
        */
        CMTISDK_API int Cmti_PowerPinOsTest(void* hDevCli, const int nPowerId[], const int nSupplyVoltage_mV[],
            const int nUpperLimitCurrent_nA[], const int bAutoHighPrecision[], int nLeakCurrent_nA[], int nCount);

        /**
        * @brief ���������ļ�
        */
        CMTISDK_API int Cmti_SendUpgradeFile(void* hDevCli, const char *filePathName);

        /**
        * @Deprecated
        * @brief ����MIPIʱ��ģʽ
        * @param discontinuous ʱ��ģʽ 0-continuous(default) 1-Discontinuous
        * @return int ������
        */
        CMTISDK_API int Cmti_SetMipiClockMode(void* hDevCli, bool discontinuous);
        /**
        * @Deprecated
        * @brief ��ȡMIPIʱ��ģʽ
        * @param [out]discontinuous ʱ��ģʽ 0-continuous(default) 1-Discontinuous
        * @return int ������
        */
        CMTISDK_API int Cmti_GetMipiClockMode(void* hDevCli, bool* discontinuous);

#ifdef __cplusplus
    } /* end of namespace CmtiSdk */
} /* end of extern "C" */
#endif /* end of __cplusplus */

#endif // CMTISDK_V1_H
