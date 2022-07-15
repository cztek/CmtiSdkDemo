/*
 * @brief CmtiSdk��ͷ�ļ�
 * ����ʹ�ñ�ͷ���е����½ӿڡ�
 */

#ifndef CMTISDK_H
#define CMTISDK_H 2

#include "TypeDefs.h"    // �������Ͷ���ͷ�ļ�
#include "CmtiDefs.h"    // ҵ�����Ͷ���ͷ�ļ�

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

// ���¼���ͷ�ļ��������ӿڻ��Ƽ�ʹ�ýӿڣ���Դ������ֻ����"CmtiSdk.h"����
#include "CmtiSdk_V1.h"
// ********************************************************************************************


#ifdef __cplusplus
extern "C" {
    namespace CmtiSdk {
#endif
        // �������
        typedef void * HDeviceController;
        typedef void * HDeviceClient;

        /**
        * @brief ��ȡSDK�汾��
        *
        * @param libVer ����SDK�汾���ַ����������߷����ڴ�
        * @param len SDK�汾���ַ����ڴ�ռ䳤��
        * @return int ������
        */
        CMTISDK_API int Cmti_GetCmtiSdkVersion(char *libVer, int len);

        /**************************************** DeviceController ****************************************/
        /**
        * @param hDevCtrl ����DeviceController���������һ���豸������
        * @brief ��ȡ�豸����������
        */
        CMTISDK_API int Cmti_GetDeviceController(HDeviceController *hDevCtrl);
        /**
        * @brief �ͷ��豸������ʵ��
        */
        CMTISDK_API int Cmti_DestroyDeviceController(HDeviceController hDevCtrl);

        /**
        * @brief ����һ���豸����
        *
        */
        CMTISDK_API int Cmti_ToggleDiscovery(HDeviceController hDevCtrl);

        /**
        * @brief �������ƻ�ȡ�豸�ͻ���ʵ����
        * ���豸���ƶ�Ӧ���豸�ͻ���ʵ��������ʱ���Զ�����ö��һ���豸��
        *
        * @param hDevCtrl DeviceController���
        * @param devName �豸����
        * @param hDevCli DeviceClient�����ָ���豸�ͻ���ʵ��
        * @return int ������
        */
        CMTISDK_API int Cmti_GetDeviceClient(HDeviceController hDevCtrl, const char *devName, HDeviceClient *hDevCli);

        /**
        * @brief ö�ٵ�ʱ�����豸
        *
        * @param deviceNameList �����豸���б������߷����ڴ�
        * @param deviceNum ������������������ʾ���֧�ֵ��豸��������������ڴ�ռ䣻�����ʾʵ�ʷ��ص��豸������
        * @return int ������
        */
        CMTISDK_API int Cmti_EnumerateDevice(HDeviceController hDevCtrl, char *deviceNameList[], int *deviceNum);

        /**
        * @brief ��ȡָ���豸�����б�propNameList��propValList�е�Ԫ�ذ��±�һһ��Ӧ����ʾһ����ֵ�ԡ�
        *
        * @param devName �豸����
        * @param propNameList �������������б������߷����ڴ�
        * @param propValList ��������ֵ�б������߷����ڴ�
        * @param propNum ������������������ʾ֧�ֵ��������Ը�������������ڴ�ռ䣻�����ʾʵ�ʷ��ص����Ը�����
        * @return int ������
        */
        CMTISDK_API int Cmti_GetDevicePropList(HDeviceController hDevCtrl, const char *devName, char *propNameList[], 
            char *propValList[], int *propNum);

        /******************************************************* DeviceClient *******************************************************/
        
        /**
        * @brief ��ȡ�豸��
        *
        * @param hDevCli �豸
        * @param devName �豸���������߸����ڴ����
        * @return int ������
        */
        CMTISDK_API int Cmti_GetDeviceName(HDeviceClient hDevCli, char *devName);

        /**
        * @brief ��ȡ�豸����
        *
        * @param hDevCli/devName �豸������豸����
        * @param propName ��������Ŀǰ֧�ֵ��������ο�SDK�ֲᣩ
        * @param propVal ����ֵ�������߸����ڴ����
        * @param maxPropValSize ����ֵ�ڴ���󳤶�
        * @return int ������
        */
        CMTISDK_API int Cmti_GetDevicePropertyByHandle(HDeviceClient hDevCli, const char *propName, char *propVal, int maxPropValSize);
        CMTISDK_API int Cmti_GetDevicePropertyByName(const char* devName, const char *propName, char *propVal, int maxPropValSize);
        /**
        * @brief ��ȡ�豸��Դ������Ϣ
        *
        * @param hDevCli �豸
        * @param pinName ������������Դ�������������߸����ڴ����pinName�ο�E_PowerId���塣
        * @param voltLowerLimit ������������ѹ���ޣ������߸����ڴ����
        * @param voltUpperLimit ������������ѹ���ޣ������߸����ڴ����
        * @param count ���Ÿ���������3������ĳ���
        * @return int ������
        */
        CMTISDK_API int Cmti_GetPowerPinInfo(HDeviceClient dev, int pinName[], int voltLowerLimit[], int voltUpperLimit[], int count);
        
        /**
        * @brief ��λ�豸����Ҫ�ǹرմ����Ӱ��
        *
        * @param hDevCli �豸���
        * @return int ������
        */
        CMTISDK_API int Cmti_ResetDevice(HDeviceClient hDevCli);

        /**
        * @brief ��ȡ�ɼ�֡��FPS���豸���¼��ϱ���
        *
        * @param hDevCli �豸
        * @param fps �ɼ�֡��FPS
        * @return int ������
        */
        CMTISDK_API int Cmti_GetCaptureFps(HDeviceClient hDevCli, float *fps);

        /**
        * @brief ��ȡ����֡��FPS���豸���¼��ϱ���
        *
        * @param hDevCli �豸
        * @param fps ����֡��FPS
        * @return int ������
        */
        CMTISDK_API int Cmti_GetTransmitFps(HDeviceClient hDevCli, float *fps);

        /**
        * @brief ��ȡ����֡��FPS�����ն�SDKͳ�ƣ�
        *
        * @param hDevCli �豸
        * @param fps ����֡��FPS
        * @return int ������
        */
        CMTISDK_API int Cmti_GetReceiveFps(HDeviceClient hDevCli, float *fps);

        /**
        * @brief ��ȡ������֡��FER�����ն�SDKͳ�ƣ�
        *
        * @param hDevCli �豸
        * @param fer ���ش�����֡��FER
        */
        CMTISDK_API int Cmti_GetTransmitFer(HDeviceClient hDevCli, float *fer);

        /**
        * @brief ����I2C��ַ
        * @param u8I2cAddr �����������������I2C��ַ���ɵ����߷���ռ�
        * @param nCount ������������������ʾ������ڴ�ռ�����������ʾʵ�ʷ��ص�I2c��ַ������
        * @return int ������
        */
        CMTISDK_API int Cmti_SearchI2cAddress(HDeviceClient hDevCli, uint8_t u8I2cAddr[], int *nCount);

        /**
        * @brief д����I2C�Ĵ���
        *
        * @param i2cAddr I2C�ӵ�ַ
        * @param speedkHz I2C����
        * @param mode I2Cģʽ
        * @param regAddr �Ĵ�����ַ
        * @param regData �Ĵ���ֵ
        * @return int ������
        */
        CMTISDK_API int Cmti_WriteSingleI2c(HDeviceClient hDevCli, uint32_t i2cAddr, uint32_t speedkHz, uint32_t mode,
            uint32_t regAddr, uint32_t regData);

        /**
        * @brief ������I2C�Ĵ���
        *
        * @param i2cAddr I2C�ӵ�ַ
        * @param speedkHz I2C����
        * @param mode I2Cģʽ
        * @param regAddr �Ĵ�����ַ
        * @param regData ����������Ĵ���ֵ
        * @return int ������
        */
        CMTISDK_API int Cmti_ReadSingleI2c(HDeviceClient hDevCli, uint32_t i2cAddr, uint32_t speedkHz, uint32_t mode,
            uint32_t regAddr, uint32_t* regData);

        /**
        * @brief ����дI2C�Ĵ���������֧��ת�����SDK�Զ��ְ�����
        * ��Ҫ���ڼĴ�����ַ�������ҼĴ������Ƚ϶�ĳ���
        *
        * @param i2cAddr I2C�ӵ�ַ
        * @param speedkHz I2C����
        * @param mode I2Cģʽ
        * @param regAddr[] �Ĵ�����ַ����
        * @param regData[] �Ĵ���ֵ����
        * @param delay_ms[] �Ĵ�����ʱ���飬��msΪ��λ
        * @param regNum �Ĵ�����������ǰ��������ĸ���Ԫ�ظ���
        * @return int ������
        */
        CMTISDK_API int Cmti_WriteDiscreteI2c(HDeviceClient hDevCli, uint32_t i2cAddr, uint32_t speedkHz, uint32_t mode,
            const uint32_t regAddr[], const uint32_t regData[], const uint32_t delay_ms[], int regNum);

        /**
        * @brief ������I2C�Ĵ���������֧��ת�����SDK�Զ��ְ�����
        * ��Ҫ���ڼĴ�����ַ�������ҼĴ������Ƚ϶�ĳ���
        *
        * @param i2cAddr I2C�ӵ�ַ
        * @param speedkHz I2C����
        * @param mode I2Cģʽ
        * @param regAddr[] �Ĵ�����ַ����
        * @param regData[] �Ĵ���ֵ����
        * @param regNum �Ĵ�����������ǰ��������ĸ���Ԫ�ظ���
        * @return int ������
        */
        CMTISDK_API int Cmti_ReadDiscreteI2c(HDeviceClient hDevCli, uint32_t i2cAddr, uint32_t speedkHz, uint32_t mode,
            const uint32_t regAddr[], uint32_t regData[], int regNum);

        /**
        * @brief ����дI2C�Ĵ���������֧�ֶ�IIC��ַд��֧��ת�����SDK�Զ��ְ�����
        * ��Ҫ���ڼĴ�����ַ�������ҼĴ������Ƚ϶�ĳ���
        *
        * @param speedkHz I2C����
        * @param mode[] I2Cģʽ
        * @param i2cAddr[] I2C�ӵ�ַ����
        * @param regAddr[] �Ĵ�����ַ����
        * @param regData[] �Ĵ���ֵ����
        * @param delay_ms[] �Ĵ�����ʱ���飬��msΪ��λ
        * @param regNum �Ĵ�����������ǰ�������ĸ���Ԫ�ظ���
        * @return int ������
        */
        CMTISDK_API int Cmti_WriteDiscreteI2c_V2(HDeviceClient hDevCli, uint32_t speedkHz, uint32_t mode[], const uint32_t i2cAddr[],
            const uint32_t regAddr[], const uint32_t regData[], const uint32_t delay_ms[], int regNum);

        /**
        * @brief ������I2C�Ĵ���������֧�ֶ�IIC��ַ����֧��ת�����SDK�Զ��ְ�����
        * ��Ҫ���ڼĴ�����ַ�������ҼĴ������Ƚ϶�ĳ���
        *
        * @param speedkHz I2C����
        * @param mode I2Cģʽ
        * @param i2cAddr[] I2C�ӵ�ַ����
        * @param regAddr[] �Ĵ�����ַ����
        * @param regData[] �Ĵ���ֵ����
        * @param regNum �Ĵ�����������ǰ��������ĸ���Ԫ�ظ���
        * @return int ������
        */
        CMTISDK_API int Cmti_ReadDiscreteI2c_V2(HDeviceClient hDevCli, uint32_t speedkHz, uint32_t mode, const uint32_t i2cAddr[],
            const uint32_t regAddr[], uint32_t regData[], int regNum);

        /**
        * @brief д������I2C������SDK�Զ��ְ�����
        *
        * @param i2cAddr I2C�ӵ�ַ
        * @param speedkHz I2C����
        * @param regAddr �Ĵ�����ַ
        * @param regAddrSize �Ĵ�����ַ���ȣ���λΪ�ֽڣ�8λ��ַΪ1��16λ��ַΪ2��
        * @param data ����
        * @param dataSize ���ݳ��ȣ����ֽڼ���ĳ���
        * @return int ������
        */
        CMTISDK_API int Cmti_WriteContinuousI2c(HDeviceClient hDevCli, uint32_t i2cAddr, uint32_t speedkHz, uint32_t regAddr,
            uint32_t regAddrSize, const uint8_t *data, uint32_t dataSize);

        /**
        * @brief ��������I2C������SDK�Զ��ְ�����
        *
        * @param i2cAddr I2C�ӵ�ַ
        * @param speedkHz I2C����
        * @param regAddr �Ĵ�����ַ
        * @param regAddrSize �Ĵ�����ַ���ȣ���λΪ�ֽڣ�8λ��ַΪ1��16λ��ַΪ2��
        * @param data ���ݣ������߷����ڴ�
        * @param dataSize ���ݳ���
        * @return int ������
        */
        CMTISDK_API int Cmti_ReadContinuousI2c(HDeviceClient hDevCli, uint32_t i2cAddr, uint32_t speedkHz, uint32_t regAddr,
            uint32_t regAddrSize, uint8_t *data, uint32_t dataSize);

        /**
        * @brief �ж��豸�Ƿ��Ѿ���
        *
        * @return bool true-�� false-�ر�
        */
        CMTISDK_API bool Cmti_IsOpen(HDeviceClient hDevCli);

        /**
        * @brief �ж��豸�Ƿ�������
        *
        * @param devName �豸��
        * @return bool true-������ false-�Ͽ�
        */
        CMTISDK_API bool Cmti_IsConnected(const char *devName);

        /**
        * @brief дSensorʱ��
        *
        * @param clk100kHz ʱ�ӣ�100kHzΪ��λ��
        * @return int ������
        */
        CMTISDK_API int Cmti_SetSensorClock(HDeviceClient hDevCli, uint32_t clk100kHz);

        /**
        * @brief ����Sensor�ӿ�
        *
        * @param intf Sensor�ӿڣ��ο�E_InterfaceType����
        * @return int ������
        */
        CMTISDK_API int Cmti_SetSensorInterface(HDeviceClient hDevCli, uint32_t intf);

        /**
        * @brief дMipi����
        *
        * @param laneNum Mipi Lanes
        * @param freqMHz Mipiʱ�ӣ�Ĭ��800MHz
        * @param virtualChannel Mipi����ͨ����Ĭ��0
        * @return int ������
        */
        CMTISDK_API int Cmti_SetMipiParam(HDeviceClient hDevCli, uint32_t laneNum, uint32_t freqMHz, uint32_t virtualChannel);
        /**
        * @brief ��Mipi����
        *
        * @param chnIdx ��Ƶͨ������
        * @param laneNum Mipi Lanes
        * @param freqMHz Mipiʱ��
        * @param virtualChannel Mipi����ͨ��
        * @return int ������
        */
        CMTISDK_API int Cmti_GetMipiParam(HDeviceClient hDevCli, uint32_t *laneNum, uint32_t *freqMHz, uint32_t *virtualChannel);

        /**
        * @brief дSensor GPIOֵ
        * 
        * @param pin Ҫ������Pin���ţ�ʹ��E_Gpioö��
        * @param level Pin���ŵ�ƽMask
        * @return int ������
        */
        CMTISDK_API int Cmti_SetSensorGpioPinLevel(HDeviceClient hDevCli, ushort pin, ushort level);
        /**
        * @brief ��Sensor GPIOֵ
        *
        * @param pin Ҫ������Pin���ţ�ʹ��E_Gpioö��
        * @param level Pin���ŵ�ƽMask
        * @return int ������
        */
        CMTISDK_API int Cmti_GetSensorGpioPinLevel(HDeviceClient hDevCli, ushort pin, ushort *level);

        /**
        * @brief дSensor GPIO����
        *
        * @param pin Ҫ������Pin���ţ�ʹ��E_Gpioö��
        * @param dir Pin���ŷ���Mask
        * @return int ������
        */
        CMTISDK_API int Cmti_SetSensorGpioPinDir(HDeviceClient hDevCli, ushort pin, ushort dir);
        /**
        * @brief дSensor GPIO����
        *
        * @param pin Ҫ������Pin���ţ�ʹ��E_Gpioö��
        * @param dir Pin���ŷ���Mask
        * @return int ������
        */
        CMTISDK_API int Cmti_GetSensorGpioPinDir(HDeviceClient hDevCli, ushort pin, ushort *dir);

        /**
        * @brief дSensor��ѹ
        *
        * @param powerId[] ��ԴID���ο�E_PowerId����
        * @param voltage_mV[] ��ѹֵ����λΪmV
        * @param delay_ms[] ��ʱ����λΪms
        * @param count ��ѹ����
        * @return int ������
        */
        CMTISDK_API int Cmti_SetSensorPower(HDeviceClient hDevCli, const uint32_t powerId[], const uint32_t voltage_mV[], const uint32_t delay_ms[], uint32_t count);

        /**
        * @brief ��Sensor��ѹ
        *
        * @param powerId[] ��ԴID���ο�E_PowerId����
        * @param voltage_mV[] �����ĵ�ѹֵ����λΪmV
        * @param count ��ѹ����
        * @return int ������
        */
        CMTISDK_API int Cmti_GetSensorPower(HDeviceClient hDevCli, const uint32_t powerId[], uint32_t voltage_mV[], uint32_t count);

        /**
        * @brief ���òɼ�������ѹ�������·����������ں���Cmti_GetSensorFeedBackVoltage���˶�ȡ�����ĺ�ʱȡ���ڲ�������Ͳ��������ĳ˻���
        *        ������ö�ȡ������ѹ�ӿ�ʱÿ��ֻ��ȡһ����ע�⣺�ں���ʵ���ڿ��� (������� * �������� <= 2s)
        * @param nInterval_us �����������λ΢��
        * @param nPoints ��������
        * @param nSampleValType �������ݴ������ͣ�ʹ��E_SampleValueTypeö��
        * @return int ������
        */
        CMTISDK_API int Cmti_SetFeedBackVoltageSampleParam(HDeviceClient hDevCli, int32_t nInterval_us, int32_t nPoints, int32_t nSampleValType);

        /**
        * @brief ���òɼ�������ѹ�������·����������ں���Cmti_GetSensorFeedBackVoltage���˶�ȡ�����ĺ�ʱȡ���ڲ�������Ͳ��������ĳ˻���
        *        ������ö�ȡ������ѹ�ӿ�ʱÿ��ֻ��ȡһ����ע�⣺�ں���ʵ���ڿ��� (������� * �������� <= 2s)
        * @param nInterval_us �����������λ΢��
        * @param nPoints ��������
        * @param nSampleValType �������ݴ������ͣ�ʹ��E_SampleValueTypeö��
        * @return int ������
        */
        CMTISDK_API int Cmti_GetFeedBackVoltageSampleParam(HDeviceClient hDevCli, int32_t* nInterval_us, int32_t* nPoints, int32_t* nSampleValType);

        /**
        * @brief ��Senso������ѹ��������ö�ȡ������ѹ�ӿ�ʱÿ��ֻ��ȡһ������ϸ�����ο�Cmti_SetFeedBackVoltageSampleParam������
        *
        * @param fbPowerId[] ��ԴID���ο�E_SensorFeedBackPowerId����
        * @param voltage[] �����ĵ�ѹֵ����λΪmV
        * @param count ��ѹ����
        * @return int ������
        */
        CMTISDK_API int Cmti_GetSensorFeedBackVoltage(HDeviceClient hDevCli, const int fbPowerId[], int voltage_mV[], int count);

        /**
        * @brief ����ͼ��֡�������ڴ���Ƶǰ��Ҫ����ͼ��֡����
        *
        * @param imgFmt ͼ���ʽ���ο�E_ImageFormat����
        * @param imgMode ͼ��ģʽ���ο�E_ImageMode����
        * @param width ���
        * @param height �߶�
        * @param outImgFmt ���ͼ���ʽ���ο�E_ImageFormat���壨�ɶ��Ʋɼ��������ʽ����ImgFmt_RAW8, ImgFmt_PackedRaw10...��
        * @return int ������
        */
        CMTISDK_API int Cmti_SetFrameParam(HDeviceClient hDevCli, uint32_t imgFmt, uint32_t imgMode, uint32_t width, uint32_t height, uint32_t outImgFmt);
        /**
        * @brief �豸ͼ��֡�������ڴ���Ƶǰ��Ҫ����ͼ��֡��������Crop����
        *
        * @param imgFmt ͼ���ʽ���ο�E_ImageFormat����
        * @param imgMode ͼ��ģʽ���ο�E_ImageMode����
        * @param width ���
        * @param height �߶�
        * @param outImgFmt ���ͼ���ʽ���ο�E_ImageFormat���壨�ɶ��Ʋɼ��������ʽ����ImgFmt_RAW8, ImgFmt_PackedRaw10...��
        * @param cropLeft ��������x����
        * @param cropTop ��������y����
        * @param cropWidth ����������
        * @param cropHeight ��������߶�
        * @return int ������
        */
        CMTISDK_API int Cmti_SetFrameParam2(HDeviceClient hDevCli, uint32_t imgFmt, uint32_t imgMode, uint32_t width, uint32_t height, uint32_t outImgFmt,
            uint32_t cropLeft, uint32_t cropTop, uint32_t cropWidth, uint32_t cropHeight);

        /**
        * @brief ��ȡͼ��֡����
        *
        * @param imgFmt ͼ���ʽ���ο�E_ImageFormat����
        * @param imgMode ͼ��ģʽ���ο�E_ImageMode����
        * @param width ���
        * @param height �߶�
        * @param size ���ͼ���С
        * @return int ������
        */
        CMTISDK_API int Cmti_GetFrameParam(HDeviceClient hDevCli, uint32_t *imgFmt, uint32_t *imgMode, uint32_t *width, uint32_t *height, uint32_t *size);

        /**
        * @brief ����MIPI Embedded Line Size
        *
        * @param hDevCli �豸������
        * @param size Embedded line size
        * @return int ������
        */
        CMTISDK_API int Cmti_SetEmbeddedLineSize(HDeviceClient hDevCli, uint32_t size);

        /**
        * @brief ����ROI������֧�����л�
        *
        * @param roiRect ROI���ο��壬�ο�T_Rect����
        * @param roiCount ROI���ο����
        * @return int ������
        */
        CMTISDK_API int Cmti_SetRoiParam(HDeviceClient hDevCli, const T_Rect roiRect[], uint32_t roiCount);

        /**
        * @brief ����Sensor�����StreamOn�Ĵ���
        * ������ i2cAddr��regAddr��regDataOn��regDataOffȫ��������Ϊ0ʱ����ʾ�����ǰ��������
        *
        * @param hDevCli �豸������
        * @param i2cAddr оƬI2c��ַ
        * @param regAddr Stream On�Ĵ�����ַ
        * @param regDataOn  Stream Onʱ�Ĵ���ֵ
        * @param regDataOff Stream Offʱ�Ĵ���ֵ
        * @return int ������
        */
        CMTISDK_API int Cmti_SetSensorStreamOnRegister(HDeviceClient hDevCli, uint32_t i2cAddr, uint32_t regAddr, uint32_t regDataOn, uint32_t regDataOff);
        
        /**
        * @brief ��Ƶ����
        *
        * @param ctrl �������0Ϊ�رգ�1Ϊ��
        * @return int ������
        */
        CMTISDK_API int Cmti_VideoControl(HDeviceClient hDevCli, uint32_t ctrl);

        /**
        * @brief ������ʱ����
        *
        * @param ctrl delay_ms ��ʱ����λms
        * @return int ������
        */
        CMTISDK_API int Cmti_SetTransmitDelay(HDeviceClient hDevCli, uint32_t delay_ms);
        CMTISDK_API int Cmti_GetTransmitDelay(HDeviceClient hDevCli, uint32_t *delay_ms);

        /**
        * @brief ���ù�װ�ɼ���ѯ��ʱʱ�䣬Ĭ��SDK��ѯ�ɼ�����3��
        *
        * @param grabTimeout��ʱʱ�䣬��λms
        * @return int ������
        */
        CMTISDK_API int Cmti_SetGrabTimeout(HDeviceClient hDevCli, uint32_t grabTimeout_ms);

        /**
        * @brief ���ù�װ�ɼ���ѯ��ʱʱ���SDK�ɼ���ѯ�������ýṹ�ܵķ���ʱ��Ϊ grabTimeout_ms * loopTimes
        *
        * @param grabTimeout��װ��ѯ��ʱʱ�䣬��λms
        * @param loopTimes SDK�ɼ���ѯ����
        * @return int ������
        */
        CMTISDK_API int Cmti_SetGrabTimeoutAndLoopTimes(HDeviceClient hDevCli, uint32_t grabTimeout_ms, uint32_t loopTimes);

        /**
        * @brief ��ȡ�����ǵ�ǰϵͳʱ��
        *
        * @param timestamp ʱ���
        * @return int ������
        */
        CMTISDK_API int Cmti_GetSystemTimestamp(HDeviceClient hDevCli, uint64 *timestamp);

        /**
        * @brief ������ǰ֡���ָ��֡�������������������ڶ���N֡������֡����ʱ�ŷ��ء���ʱʱ�䰴���õ�grabTimeout*N���㡣
        *
        * @param hDevCli �豸����
        * @param count Ҫ������֡��
        * @return int ������
        */
        CMTISDK_API int Cmti_SkipFrame(HDeviceClient hDevCli, int count);

        /**
        * @brief �ӻ���ؿ������ݵ�Ӧ�ò�
        *
        * @param pbuffer Ӧ�ò㻺����
        * @param bufferLen Ӧ�ò㻺��������
        * @param timestamp ʱ�����Ĭ��Ϊ֡��ʱ�����
        * @param headTimestamp ֡��ʱ���
        * @param tailTimestamp ֡βʱ���
        * @param frameSequence ֡���
        * @return int ������
        */
        CMTISDK_API int Cmti_GrabFrame(HDeviceClient hDevCli, uint8_t *pbuffer, int bufferLen, uint64 *timestamp);
        CMTISDK_API int Cmti_GrabFrame2(HDeviceClient hDevCli, uint8_t *pbuffer, int bufferLen, uint64 *headTimestamp,
            uint64 *tailTimestamp, uint32 *frameSequence);
        CMTISDK_API int Cmti_GrabLatestFrame(HDeviceClient hDevCli, uint8_t *pbuffer, int bufferLen, uint64 *timestamp);
        CMTISDK_API int Cmti_GrabLatestFrame2(HDeviceClient hDevCli, uint8_t *pbuffer, int bufferLen, uint64 *headTimestamp,
            uint64 *tailTimestamp, uint32 *frameSequence);

        /**
        * @brief �Ӷ�����ȡ��һ��Buffer(��EnqueueFrameBuffer���ʹ��)��ʹ�û�����еĻ�����������Ҫ�û������ڴ�
        *
        * @param bufIdx ������������ʹ����Ϻ���EnqueueFrameBuffer�й黹
        * @param pbuffer �������ͣ�������
        * @param timestamp ʱ�����Ĭ��Ϊ֡��ʱ�����
        * @param headTimestamp ֡��ʱ���
        * @param tailTimestamp ֡βʱ���
        * @param frameSequence ֡���
        * @return int ������
        */
        CMTISDK_API int Cmti_DequeueFrameBuffer(HDeviceClient hDevCli, int *bufIdx, uint8_t **pbuffer, uint64 *timestamp);
        CMTISDK_API int Cmti_DequeueFrameBuffer2(HDeviceClient hDevCli, int *bufIdx, uint8_t **pbuffer, uint64 *headTimestamp, 
            uint64 *tailTimestamp, uint32 *frameSequence);
        CMTISDK_API int Cmti_DequeueLatestFrameBuffer(HDeviceClient hDevCli, int *bufIdx, uint8_t **pbuffer, uint64 *timestamp);
        CMTISDK_API int Cmti_DequeueLatestFrameBuffer2(HDeviceClient hDevCli, int *bufIdx, uint8_t **pbuffer, uint64 *headTimestamp,
            uint64 *tailTimestamp, uint32 *frameSequence);

        /**
        * @brief ���һ��Buffer(��DequeueFrameBuffer���ʹ��)
        *
        * @param bufIdx ����������
        * @return int ������
        */
        CMTISDK_API int Cmti_EnqueueFrameBuffer(HDeviceClient hDevCli, int bufIdx);

        /**
        * @brief ���ÿ���·���Բ���
        *
        * @param supplyVol_uV ���Թ����ѹ����λuV���Ƽ�1.4V
        * @param supplyCurrent_uA ���Թ����������λuA���Ƽ�500uA
        * @param pinId[] �����Ե�Pin���Ŷ��壬�ο�E_OSM_PIN_TYPE����
        * @param openStdVol_uV[] ��·��׼����λuV�����Զ������ã��Ƽ�1V
        * @param shortStdVol_uV[] ��·��׼����λuV�����Զ������ã��Ƽ�200mV
        * @param count ���Ÿ���
        * @return int ������
        */
        CMTISDK_API int Cmti_SetOsTestConfig(HDeviceClient hDevCli, uint32_t supplyVol_uV, uint32_t supplyCurrent_uA, const uint32_t pinId[],
            const uint32_t openStdVol_uV[], const uint32_t shortStdVol_uV[], uint32_t count);

        /**
        * @brief ���ÿ���·���Ե�������
        *
        * @param bool ��������trueΪ���������falseΪ�������
        * @return int ������
        */
        CMTISDK_API int Cmti_SetOsTestCurrentDirection(HDeviceClient hDevCli, bool positive);

        /**
        * @brief ������·���Խ��
        *
        * @param pinId[] ���Ե�Pin���Ŷ��壬�ο�E_OSM_PIN_TYPE����
        * @param openVol_uV[] ��·��ѹ����λuV
        * @param shortVol_uV[] ��·��ѹ����λuV
        * @param result[] ���Խ��
        * @param count ���Ÿ���
        * @return int ������
        */
        CMTISDK_API int Cmti_ReadOsTestResult(HDeviceClient hDevCli, const uint32_t pinId[], uint32_t openVol_uV[], uint32_t shortVol_uV[],
            uint32_t result[], uint32_t count);

        /**
        * @brief ��ȡ����
        *
        * @param powerId[] ��ԴID���ο�E_PowerId����
        * @param currentRange[] �������̣��ο�E_CurrentRange����
        * @param current_nA[] ����ֵ����λΪnA
        * @param count ��Դ����
        * @return int ������
        */
        CMTISDK_API int Cmti_GetCurrent(HDeviceClient hDevCli, const uint32_t powerId[], const uint32_t currentRange[], float current_nA[], uint32_t count);

        /**
        * @brief ��ȡ����
        *
        * @param powerId[] ��ԴID���ο�E_PowerId����
        * @param upperLimit_nA[] ��������ֵ�������������ڵĲ���ֵ�����׼���������޵�ֵ����ϴ�
        * @param autoHighPrecision[] �����͵����Զ�ƥ��߾��ȱ�־��0����ƥ�䣬1������ƥ��߾���
        * @param current_nA[] ����ֵ����λΪnA
        * @param count ��Դ����
        * @return int ������
        */
        CMTISDK_API int Cmti_GetCurrentV2(HDeviceClient hDevCli, const int powerId[], const int upperLimit_nA[], const uint16_t autoHighPrecision[],
            float current_nA[], int count);

        /**
        * @brief ���ù�������
        *
        * @param powerId[] ��ԴID���ο�E_PowerId����
        * @param currentThrd_mA[] ������ֵ����λΪmA��
        * @param debounceInterval_ms[] ����ʱ�䣬��λΪms��
        * @param count ��Դ����
        * @return int ������
        */
        CMTISDK_API int Cmti_SetOvercurrentParam(HDeviceClient hDevCli, const int powerId[], const int currentThrd_mA[],
            const int debounceInterval_ms[], int count);

        /**
        * @brief ��ȡ��������
        *
        * @param powerId[] ��ԴID���ο�E_PowerId����
        * @param currentThrd_mA[] ���������������ֵ����λΪmA��
        * @param debounceInterval_ms[] �������������ʱ�䣬��λΪms
        * @param count ��Դ����
        * @return int ������
        */
        CMTISDK_API int Cmti_GetOvercurrentParam(HDeviceClient hDevCli, const int powerId[], int currentThrd_mA[],
            int debounceInterval_ms[], int count);
        
        /**
        * @brief ��ѯ����
        *
        * @param powerId[] ������������ع����ĵ�ԴID���ο�E_PowerId���壬�����߷����ڴ档
        * @param count ������������������ʾ�����PowerId����ռ�����������ʾʵ�ʷ��ص�PowerId������
        * @return int ������
        */
        CMTISDK_API int Cmti_QueryOvercurrent(HDeviceClient hDevCli, int powerId[], int *count);

        /**
        * @brief �򿪷�������
        *
        * @param delay_ms ��ʱʱ�䣬��λms
        * @return int ������
        */
        CMTISDK_API int Cmti_SetBeepOn(HDeviceClient hDevCli, uint32_t delay_ms);

        /**
        * @brief ���Ե�Դpin��©��������
        * @param nPowerId ��ԴID���ο�E_PowerId����
        * @param nSupplyVoltage_mV ���Թ����ѹ����λmV
        * @param nUpperLimitCurrent_nA  �������ޣ���λnA
        * @param bAutoHighPrecision[] �����͵����Զ�ƥ��߾��ȱ�־��0����ƥ�䣬1������ƥ��߾���
        * @param nLeakCurrent_nA ���������©��������λnA
        * @param nCount ��ԴID����
        * @return int ������
        */
        CMTISDK_API int Cmti_PowerPinLC(HDeviceClient hDevCli, const int nPowerId[], const int nSupplyVoltage_mV[],
            const int nUpperLimitCurrent_nA[], const int bAutoHighPrecision[], int nLeakCurrent_nA[], int nCount);

        /**
        * @brief �ź�pin��©��������
        * @param nMipiPinSupplyVoltage_mV MIPI�Ź����ѹ����λmV
        * @param nIoPinSupplyVoltage_mV IO�Ź����ѹ����λmV
        * @param nDirection ��������Ϊ1������Ϊ0
        * @param nPinId Pin ID���ο�E_OSM_PIN_TYPE����
        * @param nLeakCurrent_nA ���������©��������λnA
        * @param nCount pin�Ÿ���
        * @return int ������
        */
        CMTISDK_API int Cmti_SignalPinLC(HDeviceClient hDevCli, int nMipiPinSupplyVoltage_mV, int nIoPinSupplyVoltage_mV, int nDirection,
            const int nPinId[], int nLeakCurrent_nA[], int nCount);

        /**
        * @brief ����I2C��������
        * @param nSclPullupResistor sclֵ����λ��ŷķ
        * @param nSdaPullupResistor sdaֵ����λ��ŷķ
        * @return int ������
        */
        CMTISDK_API int Cmti_SetI2cPullupResistor(HDeviceClient hDevCli, int nSclPullupResistor, int nSdaPullupResistor);
        /**
        * @brief ��ȡI2C��������
        * @param nSclPullupResistor ���������sclֵ����λ��ŷķ
        * @param nSdaPullupResistor ���������sdaֵ����λ��ŷķ
        * @return int ������
        */
        CMTISDK_API int Cmti_GetI2cPullupResistor(HDeviceClient hDevCli, int *nSclPullupResistor, int *nSdaPullupResistor);

        /**
        * @brief ����I2C���������ѹ
        * @param nPullupVoltage ���������ѹ����λ��mV
        * @param bSclEnabled �Ƿ�ʹ��SCL���
        * @param bSdaEnabled �Ƿ�ʹ��SDA���
        * @return int ������
        */
        CMTISDK_API int Cmti_SetI2cPullupOutput(HDeviceClient hDevCli, int nPullupVoltage, int bSclEnabled, int bSdaEnabled);

        /**
        * @brief ����I2C�������
        *
        * @param enabled �����������ʹ�ܣ�trueΪʹ�ܣ�falseΪ��ֹ
        * @return int ������
        */
        CMTISDK_API int Cmti_SetI2cPushPullOutput(HDeviceClient hDevCli, bool enabled);

        /**
        * @brief ��ȡI2C�������״̬
        *
        * @param enabled ��������������������ʹ�ܣ�trueΪʹ�ܣ�falseΪ��ֹ
        * @return int ������
        */
        CMTISDK_API int Cmti_GetI2cPushPullOutput(HDeviceClient hDevCli, bool *enabled);

        /**
        * @brief ����mipi ���ŵ�ѹ,���оƬ����LPģʽ��
        * @param nMipiPin ����mipi���ţ��ο�E_OSM_PIN_TYPE����
        * @param nVoltage_uV ���ŵ�ѹ����λuV
        * @param nCount ���Ÿ���
        * @return int ������
        */
        CMTISDK_API int Cmti_GetMipiPinVoltage(HDeviceClient hDevCli, const int nMipiPin[], int nVoltage_uV[], int nCount);

        /**
        * @brief ����PO ���ŵ�ѹ
        * @param nPoPin ����PO���ţ��ο�E_OSM_PIN_TYPE����
        * @param nVoltage_uV ���ŵ�ѹ����λuV
        * @param nCount ���Ÿ���
        * @return int ������
        */
        CMTISDK_API int Cmti_GetPoPinVoltage(HDeviceClient hDevCli, const int nPoPin[], int nVoltage_uV[], int nCount);

        /**
        * @brief SPI����
        * @param spiId SPI��ţ��ο�E_SpiId
        * @param txBuf �������ݻ�����
        * @param rxBuf �������ݻ���������һ��ͨ�����е�MISO���ݱ��浽���ջ�������
        * @param nLen ���պͷ��ͻ��������ȣ��ó��ȱ�ʾ��SPI�����Ϸ��͵�ʱ�������ֽ�����
        * @return int ������
        */
        CMTISDK_API int Cmti_SpiTransfer(HDeviceClient hDevCli, uint spiId, const uchar txBuf[], uchar rxBuf[], int nLen);

        /**
        * @brief ����SPIͨ�Ų���
        * @param spiId SPI��ţ��ο�E_SpiId
        * @param mode SPIͨ��ģʽ���ο�E_SpiMode
        * @param speed_kHz SPIͨ�����ʣ���λkHz
        * @param bitsPerWord SPIͨ����bit���ȣ�Ĭ��8bit
        * @param delay_us SPIͨ����ʱ����ǰSPIͨ�Ž�������һ��SPIͨ�ŵ���ʱʱ��
        * @return int ������
        */
        CMTISDK_API int Cmti_SetSpiParameter(HDeviceClient hDevCli, uint spiId, uint mode, uint speed_kHz, uint bitsPerWord, uint delay_us);

        /**
       * @brief ��ȡSPIͨ�Ų���
       * @param spiId SPI��ţ��ο�E_SpiId
       * @param mode SPIͨ��ģʽ���ο�E_SpiMode
       * @param speed_kHz SPIͨ�����ʣ���λkHz
       * @param bitsPerWord SPIͨ����bit���ȣ�Ĭ��8bit
       * @param delay_us SPIͨ����ʱ����ǰSPIͨ�Ž�������һ��SPIͨ�ŵ���ʱʱ��
       * @return int ������
       */
        CMTISDK_API int Cmti_GetSpiParameter(HDeviceClient hDevCli, uint spiId, uint *mode, uint *speed_kHz, uint *bitsPerWord, uint *delay_us);

        /**
        * @brief ��ȡPO ���ż����
        * @param endPoint ����˵㣬�ο�E_PoResEndPoint����
        * @param [out]resistance_mOhm ����ֵ����λ��ŷķ
        * @return int ������
        */
        CMTISDK_API int Cmti_GetPoResistance(HDeviceClient hDevCli, int endPoint, int& resistance_mOhm);

        /**
        * @brief ��ȡMIPI״̬
        * @param nStatusKey ״̬���������ο�E_MipiStatus���塣
        * @param nStatusVal ״̬����ֵ
        * @param nCount ���ԶԸ�������ǰ�������С
        * @return int ������
        */
        CMTISDK_API int Cmti_GetMipiStatus(HDeviceClient hDevCli, int nStatusKey[], int nStatusVal[], int nCount);


        /**
        * @brief ����ָ�����ŵ���������ʹ�ܽӿ�
        * @param nPin ����Ĳ������ű�ţ�ʹ��E_OSM_PIN_TYPE���ͣ�����CS821֧��OSM_PIN_PO1���ţ�����CP881֧��OSM_PIN_PO1��OSM_PIN_PO2����
        * @param nCtrl ���Ʊ�־��1��ʾ����������0��ʾֹͣ����
        * @return int ������
        */
        CMTISDK_API int Cmti_MeasureFrameSyncPeriodControl(HDeviceClient hDevCli, int32_t nPin, int32_t nCtrl);

        /**
        * @brief ��ȡָ����������ķ������ڲ������
        * @param nPin ����Ĳ������ű�ţ�ʹ��E_OSM_PIN_TYPE���ͣ�����CS821֧��OSM_PIN_PO1���ţ�����CP881֧��OSM_PIN_PO1��OSM_PIN_PO2����
        * @param [out]pPeriod_us �������ڣ���λ΢��
        * @return int ������
        */
        CMTISDK_API int Cmti_GetFrameSyncPeriod(HDeviceClient hDevCli, int32_t nPin, uint32_t* pPeriod_us);

        /**
        * @brief ����PMIC����ģʽ
        * @param mode ���õ�ģʽ��ʹ��E_PmicSwitchMode���������
        * @return int ������
        */
        CMTISDK_API int Cmti_SetPmicSwitchMode(HDeviceClient hDevCli, int32_t mode);

        /**
        * @brief ��ȡPMIC����ģʽ
        * @param [out]pMode ��ǰPMICʹ�õĿ���ģʽ
        * @return int ������
        */
        CMTISDK_API int Cmti_GetPmicSwitchMode(HDeviceClient hDevCli, int32_t* pMode);

        /**
        * @brief ��ȡMIPI DPHY HS��ѹ
        * @param nPositive ��������Ա�־��1��ʾ����0��ʾ����
        * @param nSupplyCurrent_uA ����ʱ�ṩ�ĵ���������ֻ��ʾ��ֵ�޷��򣬵�λ΢��
        * @param nPinsId ����Ĳ������ű�ţ�ʹ��E_OSM_PIN_TYPE����
        * @param [out]nVoltage_uV ��ȡ���ĵ�ѹֵ����λ΢��
        * @param nCount ��������������
        * @return int ������
        */
        CMTISDK_API int Cmti_GetMipiDPhyHSVoltage(HDeviceClient hDevCli, int32_t nPositive, int32_t nSupplyCurrent_uA, const int32_t nPinsId[], int32_t nVoltage_uV[], int32_t nCount);

        /**
        * @brief ����MipiReceiverModeģʽ
        * @mode �ο�ö��E_MipiReceiverMode
        * @return int ������
        */
        CMTISDK_API int Cmti_SetMipiReceiverMode(HDeviceClient hDevCli, int32_t mode);

        /**
        * @brief У��DPS�����ѹ
        * @param powerIds ��У���ĵ�Դ��ʹ��E_PowerId����
        * @param voltagemV ��Դ����ĵ�ѹ����λmV
        * @param count У����Դ��������Ҫһ����У��������֧�ֵ����е�Դ
        * @return int ������
        */
        CMTISDK_API int Cmti_CalibrateDpsOutputVoltage(HDeviceClient hDevCli, const int32_t nPowerIds[], const int32_t nVoltagemV[], int32_t nCount);

        /**
        * @brief ��ȡDPS�����ѹУ������
        * @param powerIds ����ȡ�ĵ�Դ��ʹ��E_PowerId����
        * @param zeroOffsetmV DPS��ƫУ��ֵ����λmV
        * @param count ��Դ����
        * @return int ������
        */
        CMTISDK_API int Cmti_GetDpsOutputVoltageCalibration(HDeviceClient hDevCli, const int32_t nPowerIds[], int32_t nZeroOffsetmV[], int32_t nCount);

        /**
        * @brief ����DPS�ڲ�Force��Sense�����̽�/�Ͽ�״̬
        * @param powerIds �����õĵ�Դ��ʹ��E_PowerId����
        * @param enable ʹ�ܱ�־��1��ʾʹ�ܣ�0��ʾ�Ͽ�
        * @param count ��Դ����
        * @return int ������
        */
        CMTISDK_API int Cmti_SetDpsResistorStateBetweenForceAndSense(HDeviceClient hDevCli, const int32_t nPowerIds[], const int32_t nEnable[], int32_t nCount);

        /**
        * @brief ��ȡDPS�ڲ�Force��Sense�����̽�/�Ͽ�״̬
        * @param powerIds ����ȡ�ĵ�Դ��ʹ��E_PowerId����
        * @param enable ��ȡ����״̬��1��ʾʹ�ܣ�0��ʾ�Ͽ�
        * @param count ��Դ����
        * @return int ������
        */
        CMTISDK_API int Cmti_GetDpsResistorStateBetweenForceAndSense(HDeviceClient hDevCli, const int32_t nPowerIds[], int32_t nEnable[], int32_t nCount);

        /**
        * @brief ���õ�ѹ����ʹ�õĸ��ص���Դ����
        * @param nMeasureTarget ��ѹ��������ʹ��E_VolSampleTargetö��
        * @param nCurrent_uA �����������λuA
        * @param nLimitVoltage_mV ����Դǯλ��ѹ����λmV
        * @return int32_t ������
        * */
        CMTISDK_API int Cmti_SetMeasureVoltageCurrentSourceParam(HDeviceClient hDevCli, int32_t nMeasureTarget, int32_t nCurrent_uA, int32_t nLimitVoltage_mV);

        /**
        * @brief ���õ�Դѹ����
        * @param nPowerIds ��Դ��ţ�ʹ��E_PowerId���ͣ�(DOVDD��VOIS)�˲���Ϊֻ��״̬��
        * @param fSlewRate ѹ���ʣ���λmV/us�� 0��ʾ���������ò�����DPSʹ���ڲ�Ĭ��ֵ
        * @param nCount ���õĵ�Դ����
        * @return int32_t ������
        * */
        CMTISDK_API int Cmti_SetDpsSlewRate(HDeviceClient hDevCli, const int32_t nPowerIds[], const float fSlewRate[], int32_t nCount);

        /**
        * @brief ��ȡ��ǰ��Դѹ����
        * @param nPowerIds ��Դ��ţ�ʹ��E_PowerId����
        * @param fSlewRate ѹ���ʣ���λmV/us��0��ʾDPSʹ���ڲ�Ĭ��ֵ
        * @param nCount ���õĵ�Դ����
        * @return int32_t ������
        * */
        CMTISDK_API int Cmti_GetDpsSlewRate(HDeviceClient hDevCli, const int32_t nPowerIds[], float fSlewRate[], int32_t nCount);

        /**
        * @brief ��ȡSensor��ͼ���֡���ͺ�ʱ
        * @param [out]nFrameCount ֡��
        * @param [out]nElapsed_ms ��ʱ����λms
        * @return int ������
        */
        CMTISDK_API int Cmti_GetSensorFrameCountFromVideoOn(HDeviceClient hDevCli, int *nFrameCount, int *nElapsed_ms);

        /**
        * @brief ��ȡSDK���յ���֡���ͺ�ʱ
        * @param [out]nFrameCount ֡��
        * @param [out]nElapsed_ms ��ʱ����λms
        * @return int ������
        */
        CMTISDK_API int Cmti_GetReceivedFrameCountFromVideoOn(HDeviceClient hDevCli, int *nFrameCount, int *nElapsed_ms);

        /**
         * @brief ����MIPI D-Phy������ DataLane �ȶ�ʱ��
         * @param settleTime_ns ʱ��ֵ,��λ��ns��ֵ��Χ[70,400]
         * @return int ������
         */
        CMTISDK_API int Cmti_SetMipiDphyReceiverDataLaneHSSettleTime(HDeviceClient hDevCli, int32_t settleTime_ns);

        /**
          * @brief ��ȡMIPI D-Phy������ DataLane �ȶ�ʱ��
          * @param settleTime_ns ʱ��ֵ,��λ��ns��ֵ��Χ[70,400]
          * @return int ������
          */
        CMTISDK_API int Cmti_GetMipiDphyReceiverDataLaneHSSettleTime(HDeviceClient hDevCli, int32_t* settleTime_ns);

        /**
          * @brief ����MIPI D-Phy������ ClockLane �ȶ�ʱ��
          * @param settleTime_ns ʱ��ֵ,��λ��ns��ֵ��Χ[0,320]
          * @return int ������
          */
        CMTISDK_API int Cmti_SetMipiDphyReceiverClockLaneHSSettleTime(HDeviceClient hDevCli, int32_t settleTime_ns);

        /**
         * @brief ��ȡMIPI D-Phy������ ClockLane �ȶ�ʱ��
         * @param settleTime_ns ʱ��ֵ,��λ��ns��ֵ��Χ[0,320]
         * @return int ������
         */
        CMTISDK_API int Cmti_GetMipiDphyReceiverClockLaneHSSettleTime(HDeviceClient hDevCli, int32_t* settleTime_ns);

        /**
         * @brief ����MIPI C-Phy������ DataLane �ȶ�ʱ��
         * @param settleTime_ns ʱ��ֵ,��λ��ns��ֵ��Χ[70,400]
         * @return int ������
         */
        CMTISDK_API int Cmti_SetMipiCphyReceiverDataLaneHSSettleTime(HDeviceClient hDevCli, int32_t settleTime_ns);

        /**
          * @brief ��ȡMIPI C-Phy������ DataLane �ȶ�ʱ��
          * @param settleTime_ns ʱ��ֵ,��λ��ns��ֵ��Χ[70,400]
          * @return int ������
          */
        CMTISDK_API int Cmti_GetMipiCphyReceiverDataLaneHSSettleTime(HDeviceClient hDevCli, int32_t* settleTime_ns);

        /**
          * @brief ����MIPI C-Phy������ ClockLane �ȶ�ʱ��
          * @param settleTime_ns ʱ��ֵ,��λ��ns��ֵ��Χ[0,320]
          * @return int ������
          */
        CMTISDK_API int Cmti_SetMipiCphyReceiverClockLaneHSSettleTime(HDeviceClient hDevCli, int32_t settleTime_ns);

        /**
         * @brief ��ȡMIPI C-Phy������ ClockLane �ȶ�ʱ��
         * @param settleTime_ns ʱ��ֵ,��λ��ns��ֵ��Χ[0,320]
         * @return int ������
         */
        CMTISDK_API int Cmti_GetMipiCphyReceiverClockLaneHSSettleTime(HDeviceClient hDevCli, int32_t* settleTime_ns);

        /**
        * @brief �첽��ȡDPS������֧��ȡn���е����ֵ����Сֵ��ƽ��ֵ, ��Cmti_AsyncGetDpsCurrentResult��ɶԵ���
        *
        * @param powerId[] ��ԴID���ο�E_PowerId����
        * @param nCurrentRange ����������λ
        * @param nCount ��Դ����
        * @param nSampleInterval_us �������ʱ�䣬��ʱ�����ٴλ�ȡ��������λ(΢��)
        * @param nSamplePoint �첽�����л�ȡ�����Ĳ�������,Ҫ�� nSamplePoint * nSampleInterval_us < 2s
        * @param eSampleType ���ջ�ȡ��������Ĳ���ģʽ��ʽ���ο�ö�� E_SampleValueType
        * @return int ������
        */
        CMTISDK_API int Cmti_CreateAsyncGetDpsCurrentOperation(HDeviceClient hDevCli, const int powerId[], const int currentRange[], int nCount,
            int nSampleInterval_us, int nSamplePoint, int eSampleType);

        /**
        * @brief ��ȡ�첽�����Ľ����������Cmti_CreateAsyncGetDpsCurrentOperation��ɶԵ���
        *
        * @param powerId[] ��ȡ���������ţ���Ӧcurrent_nA�ĵ���ֵ
        * @param current_nA[] ����ֵ����λΪnA
        * @param nCount ��Դ����
        * @return int ������
        */
        CMTISDK_API int Cmti_AsyncGetDpsCurrentResult(HDeviceClient hDevCli, int powerId[], double current_nA[], int nCount);

        /**
        * @brief ͬ����ȡDPS������֧�ֻ�ȡ�����е����ֵ����Сֵ��ƽ��ֵ
        *
        * @param powerId[] ��ԴID���ο�E_PowerId����
        * @param nCurrentRange ����������λ
        * @param current_nA ��ȡ�ĵ���ֵ
        * @param nCount ��Դ����
        * @param nSampleInterval_us �������ʱ�䣬��ʱ�����ٴλ�ȡ��������λ(΢��)
        * @param nSamplePoint ͬ�������л�ȡ�����Ĳ�������
        * @param eSampleType ���ջ�ȡ��������Ĳ���ģʽ��ʽ���ο�ö�� E_SampleValueType
        * @return int ������
        */
        CMTISDK_API int32_t Cmti_GetDpsCurrent(HDeviceClient hDevCli, const int powerId[], const int currentRange[], double current_nA[], int nCount, int nSampleInterval_us, int nSamplePoint, int eSampleType);

        /**
        * @brief ��ȡDPS���в��������
        *
        * @param powerId[] ��ԴID���ο�E_PowerId����
        * @param nCurrentRange ����������λ
        * @param current_nA ��ȡ�ĵ���ֵ�����ظ�ʽ��[Dvdd0,Dvdd1,...Dvddn,Avdd0,Avdd1,...Avddn]������ n = nSamplePoiint - 1��
        * @param nCount ��Դ����
        * @param nSampleInterval_us �������ʱ�䣬��ʱ�����ٴλ�ȡ��������λ(΢��)
        * @param nSamplePoint ͬ�������л�ȡ�����Ĳ���������Ҫ��nSamplePoint * nCount * 8 < 1446
        * @return int ������
        */
        CMTISDK_API int32_t Cmti_GetDpsAllSamplePointCurrent(HDeviceClient hDevCli, const int powerId[], const int currentRange[], double current_nA[], int nCount, int nSampleInterval_us, int nSamplePoint);

        /**
        * @brief ����ָ����չIO���ŵĵ�ƽ
        *
        * @param pin ��չIO ID���ο� ����
        * @param level ���õĵ�ƽ
        * @return int ������
        */
        CMTISDK_API int32_t Cmti_SetExtendGpioPinLevel(HDeviceClient hDevCli, uint16_t pin, uint16_t level);

        /**
        * @brief ��ȡָ����չIO���ŵĵ�ƽ
        *
        * @param pin ��չIO ID���ο� ����
        * @param level ��ȡ�ĵ�ƽ
        * @return int ������
        */
        CMTISDK_API int32_t Cmti_GetExtendGpioPinLevel(HDeviceClient hDevCli, uint16_t pin, uint16_t& level);

        /**
        * @brief ����ָ����չIO�ķ���
        *
        * @param pin ��չIO ID���ο� ����
        * @param dir ������չIO ����
        * @return int ������
        */
        CMTISDK_API int32_t Cmti_SetExtendGpioPinDir(HDeviceClient hDevCli, uint16_t pin, uint16_t dir);

        /**
        * @brief ��ȡָ����չIO�ķ���
        *
        * @param pin ��չIO ID���ο� ����
        * @param dir ��ȡ����չ IO ����
        * @return int ������
        */
        CMTISDK_API int32_t Cmti_GetExtendGpioPinDir(HDeviceClient hDevCli, uint16_t pin, uint16_t& dir);

        /**
        * @brief ����DC����
        * @param nPositiveVoltage ������������ѹ��־��1��ʾ����ѹ��0��ʾ����ѹ��
        * @return int32_t ������
        * */
        CMTISDK_API int Cmti_EnterDCTest(HDeviceClient hDevCli, int32_t nPositiveVoltage);

        /**
        * @brief �˳�DC����
        * @return int32_t ������
        * */
        CMTISDK_API int Cmti_LeaveDCTest(HDeviceClient hDevCli);

        /**
        * @brief ����PMU�������
        * @param nSupplyCurrent_uA PMU����ĵ�������λuA
        * @param nClamp_mV ǯλ��ѹ����λmV���������������ײ�ʵ���������ѹǯλ
        * @param nPins ��������ű�ţ�ʹ��E_OSM_PIN_TYPE����
        * @param nCount ��������
        * @return int32_t ������
        * */
        CMTISDK_API int Cmti_PMUForceCurrent(HDeviceClient hDevCli, int32_t nSupplyCurrent_uA, int32_t nClampVol_mV, const int32_t nPins[], uint32_t nCount);

        /**
        * @brief ����ָ�����ŵ�ѹ
        * @param nDelayTime_ms ����ǰ��ʱʱ�䣬��λms
        * @param nSamplePoints ��������
        * @param nSampleInterval_us �����������λus
        * @param nSampleValType  ��ȡ���Ĳ������ݴ���ʽ��ʹ��E_SampleValueTypeö��
        * @param nPins ����Ĳ������ű�ţ�ʹ��E_OSM_PIN_TYPE����
        * @param nMeasVol_mV ����ĵ�ѹֵ����λuV
        * @param nCount ��������
        * @return int32_t ������
        * */
        CMTISDK_API int Cmti_PMUMeasurePinVoltage(HDeviceClient hDevCli, int32_t nDelayTime_ms, int32_t nSamplePoints, int32_t nSampleInterval_us, int32_t nSampleValType, const int32_t nPins[], int32_t nMeasVol_uV[], int32_t nCount);

        /**
        * @brief ����PMU�����ѹ
        * @param nSupplyVoltage_uV PMU����ĵ�ѹ����λuV
        * @param nCurrentRange ������λ
        * @param nClampCurrent_uA ǯλ��������λuA���������������ײ�ʵ���������ѹǯλ
        * @param nPins ��������ű�ţ�ʹ��E_OSM_PIN_TYPE����
        * @param nCount ��������
        * @return int32_t ������
        * */
        CMTISDK_API int Cmti_PMUForceVoltage(HDeviceClient hDevCli, int32_t nSupplyVoltage_uV, int32_t nCurrentRange, int32_t nClampCurrent_uA, const int32_t nPins[], uint32_t nCount);

        /**
        * @brief ����ָ�����ŵ���
        * @param nDelayTime_ms ����ǰ��ʱʱ�䣬��λms
        * @param nSamplePoints ��������
        * @param nSampleInterval_us �����������λus
        * @param nSampleValType  ��ȡ���Ĳ������ݴ���ʽ��ʹ��E_SampleValueTypeö��
        * @param nPins ����Ĳ������ű�ţ�ʹ��E_OSM_PIN_TYPE����
        * @param fMeasCur_nA �����õ��ĵ���ֵ����λnA
        * @param nCount ��������
        * @return int32_t ������
        * */
        CMTISDK_API int Cmti_PMUMeasurePinCurrent(HDeviceClient hDevCli, int32_t nDelayTime_ms, int32_t nSamplePoints, int32_t nSampleInterval_us, int32_t nSampleValType, const int32_t nPins[], double nMeasCur_nA[], int32_t nCount);

        /**
        * @brief ����ָ�����ŵĵ�ѹ
        * @param nPins ����Ĳ������ű�ţ�ʹ��E_OSM_PIN_TYPE����
        * @param nCount ��������
        * @param nSupplyVoltage_uV ֧�ֵ�ѹ��
        * @return int32_t ������
        * */
        CMTISDK_API int Cmti_SetPinVoltage(HDeviceClient hDevCli, const int32_t nPins[], int32_t nCount, int32_t nSupplyVoltage_uV);

        /**
        * @brief ����ָ��������Ϊ����̬
        * @param pinIds ����Ĳ������ű�ţ�ʹ��E_OSM_PIN_TYPE����
        * @param count ��������
        * @return int32_t ������
        * */
        CMTISDK_API int Cmti_SetPinHighImpedanceState(HDeviceClient hDevCli, const int32_t pinIds[], int32_t count);

        /**
        * @brief ����ͼ����ģʽ��ģʽ�ᱣ��ֱ�����̽������ٴε��øýӿڸ���ģʽ�����°󶨹�װ�����Ըýӿ��ڲ���Ҫ���Ĵ���ģʽ������£�����һ�μ��ɣ�����Ҫ�ظ����á�
        * @param transmitterMode ����ģʽ���ͣ�ʹ��E_TransmitterMode����
        * @return int32_t ������
        * */
        CMTISDK_API int Cmti_SetTransmissionMode(HDeviceClient hDevCli, int32_t transmitterMode);

        /**
        * @brief ��ȡͼ����ģʽ
        * @param pTransmitterMode ����ģʽ���ͣ�ʹ��E_TransmitterMode����
        * @return int32_t ������
        * */
        CMTISDK_API int Cmti_GetTransmissionMode(HDeviceClient hDevCli, int32_t *pTransmitterMode);

        /**
        * @brief �ɼ�ָ����������֡���ýӿ��Զ��ж�ץȡ����ͼ��֡
        *
        * @param pImageBuf ���洫��ͼ���ڴ�ռ��ɵ����߹���
        * @param pHeadTimestamp ֡ͷʱ���buf
        * @param pTailTimestamp ֡βʱ���buf
        * @param pFrameSequence ֡��buf
        * @param imageCount ����ͼ������
        * @param timeout �ӿڵ��ó�ʱʱ�䣬����0Ϊ����ʽ�ȴ���timeout > imageCount * һ֡�ɼ�ʱ��
        * @return int32_t ������
        * */
        CMTISDK_API int Cmti_GrabContinuousFrames(HDeviceClient hDevCli, uint8_t *pImageBuf[], uint64_t pHeadTimestamp[], uint64_t pTailTimestamp[],
                                                  uint32_t pFrameSequence[], int32_t imageCount, int32_t frameSize, int32_t timeout_ms);

        /**
        * @brief �����֡ͼ��ƽ��ֵ�첽���ã���Cmti_WaitForAsyncResult����ʹ�û�ȡ�����,
        *        ��֧��Raw8 ~ Raw14���Ҳ�֧��Roi���ܣ�����֡ͼ���䣬�ýӿ�����������֡ģʽ��ʹ�ã�ģʽ���ü�Cmti_SetTransmissionMode�ӿ�
        * @param nImageCount �������ƽ��ֵ��֡����
        * @param [in/out]pAsyncContext �첽����������ָ��
        * @return int32_t ������
        * */
        CMTISDK_API int Cmti_CalcAverageImageAsync(HDeviceClient hDevCli, int32_t nImageCount, T_CalcAverageImageAsyncContext* pAsyncContext);

        /**
        * @brief �ȴ��첽������ɣ�����timeout_ms����Ϊ0ʱ������ѯ�첽������ǰ״̬����ѯ����״̬��
        *        ���״̬ΪΪAsyncState_WaitTransmitter����Ӧ�Ľ���洢�ڷ���ֵ�ṹ����
        * @param [in/out]pAsyncContext �첽����������ָ��
        * @param timeout_ms ����0ʱ����ѯ��ǰ�첽����״̬�����״̬ΪΪAsyncState_WaitTransmitter����Ӧ�Ľ���洢�ڷ���ֵ�ṹ���У��������0ʱ��Ϊ��ʱʱ�䣬
        *                   �������ÿһ֡�ɼ�ʱ�� * imageCount��
        * @return int32_t ������
        * */
        CMTISDK_API int Cmti_WaitForAsyncResult(HDeviceClient hDevCli, void * pAsyncContext, int32_t timeout_ms);

        /**
        * @brief �趨ָ���������PWMƵ���ź�
        * @param pin ���PMW�ź����ű�ţ�ʹ��E_OSM_PIN_TYPE����, ��CP881֧��OSM_PIN_PO3����
        * @param freq_Hz PWMƵ�ʣ���λHz
        * @return int32_t ������
        * */
        CMTISDK_API int32_t Cmti_SetPinOutputPwmFrequency(HDeviceClient hDevCli, int32_t pin, int32_t freq_Hz);

        /**
        * @brief ���õ�Դ�̶�Force/Sense,��CP881֧��DVDD��Դ
        * @param nPowerId[] ��ԴID���ο�E_PowerId����
        * @param nFix �̶�Force��ʶ��1��ʶ�̶�Force��0��ʾ�̶�Sense
        * @return int32_t ������
        * */
        CMTISDK_API int32_t Cmti_SetPowerFixForce(HDeviceClient hDevCli, const int32_t nPowerId[], const int32_t nFix[], int32_t nCount);

        /**
        * @brief ��ȡ��Դ�̶�Force/Sense״̬,��CP881֧��DVDD��Դ
        * @param nPowerId[] ��ԴID���ο�E_PowerId����
        * @param nFix �̶�Force��ʶ��1��ʶ�̶�Force��0��ʾ�̶�Sense
        * @return int32_t ������
        * */
        CMTISDK_API int32_t Cmti_GetPowerFixForceState(HDeviceClient hDevCli, const int32_t nPowerId[], int32_t nFix[], int32_t nCount);


        /**
        * @brief дI2C���ȡDPS���в��������
        *
        * @param i2cAddr I2C�ӵ�ַ
        * @param speedkHz I2C����
        * @param mode I2Cģʽ
        * @param regAddr[] �Ĵ�����ַ����
        * @param regData[] �Ĵ���ֵ����
        * @param delay_ms[] �Ĵ�����ʱ���飬��msΪ��λ
        * @param regNum �Ĵ�����������ǰ��������ĸ���Ԫ�ظ���,Ҫ�󣺣�1446 - 20�� / 8
        * @param powerId ��ԴID���ο�E_PowerId����
        * @param nCurrentRange ����������λ���ο�E_DPSCurrentRange����
        * @param current_nA ��ȡ�ĵ���ֵ�����ظ�ʽ��[Dvdd0,Dvdd1,...Dvddn,Avdd0,Avdd1,...Avddn]������ n = nSamplePoiint - 1��
        * @param nSampleInterval_us �������ʱ�䣬��ʱ�����ٴλ�ȡ��������λ(΢��)
        * @param nSamplePoint ͬ�������л�ȡ�����Ĳ���������Ҫ��nSamplePoint * 8 < 1446
        * @return int ������
        */
        CMTISDK_API int32_t Cmti_WriteICAndGetDpsAllSamplePointCurrent(HDeviceClient hDevCli, uint32_t i2cAddr, uint32_t speedkHz, uint32_t mode,const uint32_t regAddr[], const uint32_t regData[], 
            const uint32_t delay_ms[], int32_t regNum, int32_t powerId, int32_t currentRange, double current_nA[], int32_t nSampleInterval_us, int32_t nSamplePoint);

        /**
        * @brief ����MIPI����©����У��״̬
        * @param nState ����У��ģʽ��־��1��ʾ����У��״̬��0��ʾ�˳�У��״̬
        * @return int32_t ������
        * */
        CMTISDK_API int32_t Cmti_SetMipiPinLeakCurrentCalibrateState(HDeviceClient hDevCli, int32_t nState);

        /**
        * @brief ����MIPI����©����У������
        * @param nGroupIdx,У�������������ţ���0��ʼ
        * @param nPositive�������־��1��ʾ����У��������0��ʾ����У������
        * @param nPinId ���ű�ţ��ο�E_OSM_PIN_TYPE����
        * @param nGain ���õ�У������ֵ������1000��
        * @param nOffset_nA ���õ�У��ƫ��ֵ����λnA
        * @param nCount ��������
        * @return int32_t ������
        * */
        CMTISDK_API int32_t Cmti_SetMipiPinLeakCurrentCalibration(HDeviceClient hDevCli, int32_t nGroupIdx, int32_t nPositive, const int32_t nPinId[],
            const int32_t nGain[], const int32_t nOffset_nA[], int32_t nCount);

        /**
        * @brief ��ȡMIPI����©����У������
        * @param nGroupIdx,У�������������ţ���0��ʼ
        * @param nPositive�������־��1��ʾ����У��������0��ʾ����У������
        * @param nPinId ���ű�ţ��ο�E_OSM_PIN_TYPE����
        * @param nGain ��ȡ��У������ֵ������1000��
        * @param nOffset_nA ��ȡ��У��ƫ��ֵ����λnA
        * @param nCount ��������
        * @return int32_t ������
        * */
        CMTISDK_API int32_t Cmti_ReadMipiPinLeakCurrentCalibration(HDeviceClient hDevCli, int32_t nGroupIdx, int32_t nPositive, const int32_t nPinId[],
            int32_t nGain[], int32_t nOffset_nA[], int32_t nCount);

        /**
        * @brief ��Sensor���ͼ������в���ָ��MIPI���ŵ�һ֡�ڵ�LPʱ��
        * @param nMeasureTime_ms����������ʱ�䣬��λms
        * @param nHighThreshold_mV���ߵ�ƽ��ֵ����λmV���Ƽ�����800mV
        * @param nPins ����Ĳ������ű�ţ�ʹ��E_OSM_PIN_TYPE����
        * @param nPeriod_us �����õ���LPʱ�䣬��λus��
        * @param nCount ����������nMeasureTime * nCount < TimeOut(3000ms)
        * @return int ������
        * */
        CMTISDK_API int32_t Cmti_MeasureMipiLPPeriod(HDeviceClient hDevCli, int32_t nMeasureTime_ms, int32_t nHighThreshold_mV, const int32_t nPins[],int32_t nPeriod_us[], int32_t nCount);

#ifdef __cplusplus
    } /* end of namespace CmtiSdk */
} /* end of extern "C" */
#endif /* end of __cplusplus */

/******************************** ISP��غ��� ********************************/
#ifdef __cplusplus
extern "C" {
    namespace CmtiSdk {
#endif
        /**
        * @brief ��ͼ��תΪRGB24
        * @param pOutRgb24Image ���������RGB24ͼ��
        * @param pInImage �����ת����ͼ��
        * @param frameParam ����ͼ������ṹ�塣
        * @param algo ͼ��ת���㷨��
        * @return int ������
        */
        CMTISDK_API int ISP_DataToRgb24(uint8_t *pOutRgb24Image, const uint8_t *pInImage, const T_FrameParam& frameParam, int32_t algo);
        /**
        * @brief ��ͼ��תΪ�Ҷ�ͼ
        * @param pOutGray ����������Ҷ�ͼ��
        * @param pInImage �����ת����ͼ��
        * @param frameParam ����ͼ������ṹ�塣
        * @param algo ͼ��ת���㷨��Ĭ��Ϊ0��
        * @return int ������
        */
        CMTISDK_API int ISP_DataToGray(uint8_t *pOutGrayImage, const uint8_t *pInImage, const T_FrameParam& frameParam, int32_t algo);

#ifdef __cplusplus
    } /* end of namespace CmtiSdk */
} /* end of extern "C" */
#endif /* end of __cplusplus */

#endif // CMTISDK_H
