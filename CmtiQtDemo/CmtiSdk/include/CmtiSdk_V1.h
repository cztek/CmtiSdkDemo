#ifndef CMTISDK_V1_H
#define CMTISDK_V1_H 1

/**
 * V1版本为AA使用版本，只支持MSVC++编译器。
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
 * @brief SDK事件处理器
 *
 */
class ISdkEventHandler
{
public:
    /**
     * @brief 设备变化事件
     *
     * @param devName 设备名
     * @param state (0 - offline 1 - online)
     */
    virtual void OnDeviceChanged(const char *devName, int state) = 0;
    /**
     * @brief 采集帧率FPS更新事件
     *
     * @param chnIdx 视频通道索引
     * @param fps 采集帧率FPS
     */
    virtual void OnCaptureFpsChanged(uint chnIdx, float fps) = 0;
    /**
     * @brief 传输帧率FPS更新事件
     *
     * @param chnIdx 视频通道索引
     * @param fps 传输帧率FPS
     */
    virtual void OnTransmitFpsChanged(uint chnIdx, float fps) = 0;
    /**
     * @brief 传输误帧率更新事件
     *
     * @param chnIdx 视频通道索引
     * @param fer 传输误帧率
     */
    virtual void OnTransmitFerChanged(uint chnIdx, float fer) = 0;
};

/**
 * @brief IDeviceCli
 *
 * 操作设备接口
 */
class IDeviceCli
{
public:
    /**
     * @brief 注册SDK事件处理器，注册后SDK会主动上报各类注册的事件
     *
     * @param eventHandler SDK事件处理器接口
     */
    virtual void RegisterEventHandler(ISdkEventHandler *eventHandler) = 0;
    /**
     * @brief 获取设备名
     *
     * @param devName 设备名，调用者负责内存管理
     * @return int 错误码
     */
    virtual int GetDeviceName(char *devName) const = 0;

    /**
     * @brief 批量写Sensor寄存器参数
     *
     * @param chnIdx 视频通道索引
     * @param slaveAddr I2C从地址
     * @param speedkHz I2C速率
     * @param mode I2C模式
     * @param pRegParam 指向寄存器参数的指针，格式为“寄存器地址+寄存器值+延时”，寄存器地址、寄存器值和延时均使用uint表示，按顺序存放
     * @param length 参数pRegParam的长度，即“寄存器地址+寄存器值+延时”的总长度
     * @return int 错误码
     */
    virtual int WriteSensorMultiRegs(uint chnIdx, uint slaveAddr, uint speedkHz, uint mode, const uint *pRegParam, int length) = 0;
    /**
     * @brief 批量写Sensor寄存器参数（带延时功能）
     *
     * @param chnIdx 视频通道索引
     * @param slaveAddr I2C从地址
     * @param speedkHz I2C速率
     * @param mode I2C模式
     * @param regAddr[] 寄存器地址数组
     * @param regData[] 寄存器值数组
     * @param regDelay[] 寄存器延时数组
     * @param regNum 寄存器个数，即前三个数组的元素个数
     * @return int 错误码
     */
    virtual int WriteSensorMultiRegsWithDelay(uint chnIdx, uint slaveAddr, uint speedkHz, uint mode,
        const ushort regAddr[], const ushort regData[], const ushort regDelay[], int regNum) = 0;

    /**
     * @brief 写离散的I2C参数
     *
     * @param chnIdx 视频通道索引
     * @param slaveAddr I2C从地址
     * @param speedkHz I2C速率
     * @param mode I2C模式
     * @param regs[] 寄存器地址
     * @param value[] 寄存器值
     * @param regNum 寄存器个数
     * @return int 错误码
     */
    virtual int WriteDiscreteI2c(uint chnIdx, uint slaveAddr, uint speedkHz, uint mode, const ushort regs[], const ushort value[], uint regNum) = 0;
    /**
     * @brief 读离散的I2C参数
     *
     * @param chnIdx 视频通道索引
     * @param slaveAddr I2C从地址
     * @param speedkHz I2C速率
     * @param mode I2C模式
     * @param regs[] 寄存器地址
     * @param value[] 寄存器值
     * @param regNum 寄存器个数
     * @return int 错误码
     */
    virtual int ReadDiscreteI2c(uint chnIdx, uint slaveAddr, uint speedkHz, uint mode, const ushort regs[], ushort value[], uint regNum) = 0;
    /**
     * @brief 写连续的I2C参数
     *
     * @param chnIdx 视频通道索引
     * @param slaveAddr I2C从地址
     * @param speedkHz I2C速率
     * @param regAddr 寄存器地址
     * @param regAddrSize 寄存器地址长度
     * @param data 数据
     * @param dataSize 数据长度
     * @return int 错误码
     */
    virtual int WriteContinuousI2c(uint chnIdx, uint slaveAddr, uint speedkHz, uint regAddr, uint regAddrSize, const uint8_t *data, uint dataSize) = 0;
    /**
     * @brief 读连续的I2C参数
     *
     * @param chnIdx 视频通道索引
     * @param slaveAddr I2C从地址
     * @param speedkHz I2C速率
     * @param regAddr 寄存器地址
     * @param regAddrSize 寄存器地址长度
     * @param data 数据
     * @param dataSize 数据长度
     * @return int 错误码
     */
    virtual int ReadContinuousI2c(uint chnIdx, uint slaveAddr, uint speedkHz, uint regAddr, uint regAddrSize, uint8_t *data, uint dataSize) = 0;
    /**
     * @brief 写Sensor时钟
     *
     * @param chnIdx 视频通道索引
     * @param clk100kHz 时钟（100kHz为单位）
     * @return int 错误码
     */
    virtual int SetSensorClock(uint chnIdx, uint clk100kHz) = 0;
    /**
     * @brief 读Sensor时钟
     *
     * @param chnIdx 视频通道索引
     * @param clk100kHz 时钟（100kHz为单位）
     * @return int 错误码
     */
    virtual int GetSensorClock(uint chnIdx, uint &clk100kHz) = 0;
    /**
     * @brief 写Mipi参数
     *
     * @param chnIdx 视频通道索引
     * @param laneNum Mipi Lanes
     * @param freqMHz Mipi时钟，默认800MHz
     * @param virtualChannel Mipi虚拟通道，默认0
     * @return int 错误码
     */
    virtual int SetMipiParam(uint chnIdx, uint laneNum, uint freqMHz, uint virtualChannel) = 0;
    /**
     * @brief 读Mipi参数
     *
     * @param chnIdx 视频通道索引
     * @param laneNum Mipi Lanes
     * @param freqMHz Mipi时钟
     * @param virtualChannel Mipi虚拟通道
     * @return int 错误码
     */
    virtual int GetMipiParam(uint chnIdx, uint &laneNum, uint &freqMHz, uint &virtualChannel) = 0;
    /**
     * @brief 写Sensor GPIO值
     *
     * @param chnIdx 视频通道索引
     * @param pin 要操作的Pin引脚
     * @param level Pin引脚电平Mask
     * @return int 错误码
     */
    virtual int SetSensorGpioPinLevel(uint chnIdx, ushort pin, ushort level) = 0;
    /**
     * @brief 读Sensor GPIO值
     *
     * @param chnIdx
     * @param pin 要操作的Pin引脚
     * @param level Pin引脚电平Mask
     * @return int 错误码
     */
    virtual int GetSensorGpioPinLevel(uint chnIdx, ushort pin, ushort &level) = 0;
    /**
     * @brief 写Sensor GPIO方向
     *
     * @param chnIdx 视频通道索引
     * @param pin 要操作的Pin引脚
     * @param dir Pin引脚方向Mask
     * @return int 错误码
     */
    virtual int SetSensorGpioPinDir(uint chnIdx, ushort pin, ushort dir) = 0;
    /**
     * @brief 写Sensor GPIO方向
     *
     * @param chnIdx 视频通道索引
     * @param pin 要操作的Pin引脚
     * @param dir Pin引脚方向Mask
     * @return int 错误码
     */
    virtual int GetSensorGpioPinDir(uint chnIdx, ushort pin, ushort &dir) = 0;
    /**
     * @brief 写Sensor电压
     *
     * @param chnIdx 视频通道索引
     * @param powerIds[] 电源ID，参考E_PowerId定义
     * @param voltagemV[] 电压值，单位为mV
     * @param delayms[] 延时，单位为ms
     * @param count 电压个数
     * @return int 错误码
     */
    virtual int SetSensorPower(uint chnIdx, const uint powerIds[], const uint voltagemV[], const uint delayms[], uint count) = 0;
    /**
     * @brief 读Sensor电压
     *
     * @param chnIdx 视频通道索引
     * @param powerIds[] 电源ID，参考E_PowerId定义
     * @param voltagemV[] 电压值，单位为mV
     * @param delayms[] 延时，单位为ms
     * @param count 电压个数
     * @return int 错误码
     */
    virtual int GetSensorPower(uint chnIdx, const uint powerIds[], uint voltagemV[], uint delayms[], uint count) = 0;
    /**
     * @brief 设备图像帧参数，在打开视频前需要设置图像帧参数
     *
     * @param chnIdx 视频通道索引
     * @param imageFormat 图像格式，参考E_ImageFormat定义
     * @param imageMode 图像模式，参考E_ImageMode定义
     * @param width 宽度
     * @param height 高度
     * @param outImageFormat 输出图像格式，参考E_ImageFormat定义
     * @param cropLeft 剪裁区域x坐标
     * @param cropTop 剪裁区域y坐标
     * @param cropWidth 剪裁区域宽度
     * @param cropHeight 剪裁区域高度
     * @return int 错误码
     */
    virtual int SetFrameParam(uint chnIdx, uint imageFormat, uint imageMode, uint width, uint height, uint outImageFormat, 
        uint cropLeft, uint cropTop, uint cropWidth, uint cropHeight) = 0;
    /**
     * @brief 获取图像帧参数
     *
     * @param chnIdx 视频通道索引
     * @param imageFormat 图像格式，参考E_ImageFormat定义
     * @param imageMode 图像模式，参考E_ImageMode定义
     * @param width 宽度
     * @param height 高度
     * @param outImageFormat 输出图像格式
     * @param cropLeft 剪裁区域x坐标
     * @param cropTop 剪裁区域y坐标
     * @param cropWidth 剪裁区域宽度
     * @param cropHeight 剪裁区域高度
     * @return int 错误码
     */
    virtual int GetFrameParam(uint chnIdx, uint &imageFormat, uint &imageMode, uint &width, uint &height,
        uint &size, uint &cropLeft, uint &cropTop, uint &cropWidth, uint &cropHeight) = 0;
    /**
     * @brief 视频控制
     *
     * @param chnIdx 视频通道索引
     * @param ctrl 控制命令，0为关闭，1为打开
     * @return int 错误码
     */
    virtual int VideoControl(uint chnIdx, uint ctrl) = 0;
    /**
     * @brief 传输控制
     *
     * @param chnIdx 视频通道索引
     * @param ctrl 控制命令，高16位表示传输模式，低16位为控制命令(0为停止传输，1为开启传输)
     * @return int 错误码
     */
    virtual int TransmitControl(uint chnIdx, uint ctrl) = 0;
    /**
     * @brief 设置Sensor端口
     *
     * @param chnIdx 视频通道索引
     * @param port Sensor端口，参考E_InterfaceType定义
     * @return int 错误码
     */
    virtual int SetSensorPort(uint chnIdx, uint port) = 0;
    /**
     * @brief 获取Sensor端口
     *
     * @param chnIdx 视频通道索引
     * @param port Sensor端口，参考E_InterfaceType定义
     * @return int 错误码
     */
    virtual int GetSensorPort(uint chnIdx, uint &port) = 0;
    /**
     * @brief 设置采集帧超时
     *
     * @param chnIdx 视频通道索引
     * @param 超时时间，-1表示一直等待
     * @return int 错误码
     */
    virtual int SetGrabTimeout(uint chnIdx, uint grabTimeout) = 0;
    /**
     * @brief 获取采集帧超时
     *
     * @param chnIdx 视频通道索引
     * @param 超时时间，-1表示一直等待
     * @return int 错误码
     */
    virtual int GetGrabTimeout(uint chnIdx, uint &grabTimeout) = 0;

    /**
     * @brief 从缓冲池拷贝数据到应用层
     *
     * @param chnIdx 视频通道索引
     * @param pbuffer 应用层缓冲区
     * @param bufferLen 应用层缓冲区长度
     * @param timestamp 时间戳
     * @return int 错误码
     */
    virtual int GrabFrame(uint chnIdx, uint8_t *pbuffer, int bufferLen, uint64_t &timestamp) = 0;
    virtual int GrabLatestFrame(uint chnIdx, uint8_t *pbuffer, int bufferLen, uint64_t &timestamp) = 0;
    /**
     * @brief 从队列中取出一个Buffer(与EnqueueFrameBuffer配对使用)，使用缓冲池中的缓冲区，无需要用户分配内存
     *
     * @param chnIdx 视频通道索引
     * @param bufIdx 缓冲区索引，使用完毕后在EnqueueFrameBuffer中归还
     * @param pbuffer 引用类型，缓冲区
     * @param timestamp 时间戳
     * @return int 错误码
     */
    virtual int DequeueFrameBuffer(uint chnIdx, int &bufIdx, uint8_t* &pbuffer, uint64_t &timestamp) = 0;
    virtual int DequeueLatestFrameBuffer(uint chnIdx, int &bufIdx, uint8_t* &pbuffer, uint64_t &timestamp) = 0;
    /**
     * @brief 入队一个Buffer(与DequeueFrameBuffer配对使用)
     *
     * @param chnIdx 视频通道索引
     * @param bufIdx 缓冲区索引
     * @return int 错误码
     */
    virtual int EnqueueFrameBuffer(uint chnIdx, int bufIdx) = 0;
    /**
     * @brief 设置开短路测试参数
     *
     * @param chnIdx 视频通道索引
     * @param supplyVol_uV 测试供电电压，推荐1.4V
     * @param supplyCurrent_uA 测试供电电流，推荐500uA
     * @param pinsId[] 待测试的Pin引脚定义，参考E_OSM_PIN_TYPE定义
     * @param openStdVols_uV[] 开路标准，可以独立设置，推荐1V
     * @param shortStdVols_uV[] 短路标准，可以独立设置，推荐200mV
     * @param count 引脚个数
     * @return int 错误码
     */
    virtual int SetOsTestConfig(uint chnIdx, uint supplyVol_uV, uint supplyCurrent_uA, const uint pinsId[],
        const uint openStdVols_uV[], const uint shortStdVols_uV[], uint count) = 0;
    /**
     * @brief 读开短路测试结果
     *
     * @param chnIdx 视频通道索引
     * @param pinsId[] 测试的Pin引脚定义，参考E_OSM_PIN_TYPE定义
     * @param openVols_uV[] 开路电压
     * @param shortVol_uV[] 短路电压
     * @param results[] 测试结果
     * @param pinCount 引脚个数
     * @return int 错误码
     */
    virtual int ReadOsTestResult(uint chnIdx, const uint pinsId[], uint openVols_uV[], uint shortVol_uV[], uint results[], uint pinCount) = 0;
    /**
     * @brief 获取电流
     *
     * @param chnIdx 视频通道索引
     * @param powerIds[] 电源ID，参考E_PowerId定义
     * @param currentRange[] 电流量程，参考E_CurrentRange定义
     * @param current_nA[] 电流值，单位为nA
     * @param count 电源个数
     * @return int 错误码
     */
    virtual int GetCurrent(uint chnIdx, const uint powerIds[], const uint currentRange[], float current_nA[], uint count) = 0;
    /**
     * @brief 打开蜂鸣器开
     *
     * @param chnIdx 视频通道索引
     * @param ms 延时时间，单位ms
     * @return int 错误码
     */
    virtual int SetBeepOn(uint chnIdx, uint ms) = 0;
    virtual int SendUpgradeFile(const char *filePathName) = 0;

    /**
    * @brief 设置ROI参数，支持热切换
    *
    * @param roiRect ROI矩形框定义，参考T_Rect定义
    * @param roiCount ROI矩形框个数
    * @return int 错误码
    */
    virtual int SetRoiParam(uint32_t chnIdx, const T_Rect roiRect[], uint32_t roiCount) = 0;
};

/**
 * @brief 设备管理类
 *
 * @class IDeviceController
 */
class IDeviceController
{
public:
    virtual ~IDeviceController() {}
    /**
     * @brief 根据名称获取设备客户端实例
     *
     * @param devName 设备名称
     * @return 返回IDeviceCli*，指向设备客户端实例
     */
    virtual IDeviceCli *GetDeviceCliInstance(const char *devName) = 0;
    /**
     * @brief 注册SDK事件处理器
     *
     * @param eventHandler 事件处理器
     */
    virtual void RegisterEventHandler(ISdkEventHandler *eventHandler) = 0;
    /**
     * @brief 设备发现功能开关，通常在应用程序开始时调用开启功能
     *
     * @param enabled 开启或关闭
     */
    virtual void SetDeviceDiscoveryEnabled(bool enabled) = 0;
    /**
     * @brief 执行一次设备发现
     *
     */
    virtual void ToggleDiscovery() = 0;
    /**
     * @brief 枚举当时在线设备
     *
     * @param deviceNameList 返回设备名列表，调用者分配内存
     * @param maxDeviceNum 最大支持的设备个数
     * @param pDeviceNum 实际返回的设备个数
     * @return int 错误码
     */
    virtual int EnumerateDevice(char *deviceNameList[], int maxDeviceNum, int *pDeviceNum) = 0;

    /**
    * @brief 获取指定设备属性列表
    *
    * @param devName 设备名称
    * @param propNameList 返回属性名称列表，调用者分配内存
    * @param propValList 返回属性值列表，调用者分配内存
    * @param maxPropNum 最大的属性个数
    * @param pPropNum 实际返回的属性个数
    * @return int 错误码
    */
    virtual int GetDevicePropList(const char *devName, char *propNameList[], char *propValList[], int maxPropNum, int *pPropNum) = 0;
};

extern "C" {
    CMTISDK_API void GetCztekLibVersion(char *libVer, int len);
    /**
    * @brief 返回IDeviceController*，指向设备管理类
    *
    */
    CMTISDK_API IDeviceController *GetDeviceControllerInstance();
    /**
    * @brief 释放设备管理类实例
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
        * @brief 测试电源pin脚开短路
        * @param nPowerId 电源ID，参考E_PowerId定义
        * @param nSupplyVoltage_mV 测试供电电压，单位mV
        * @param nUpperLimitCurrent_nA  电流上限，单位nA
        * @param bAutoHighPrecision[] 各类型电流自动匹配高精度标志，0：不匹配，1：向下匹配高精度
        * @param nLeakCurrent_nA 输出参数，漏电流，单位nA
        * @param nCount 电源ID个数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_PowerPinOsTest(void* hDevCli, const int nPowerId[], const int nSupplyVoltage_mV[],
            const int nUpperLimitCurrent_nA[], const int bAutoHighPrecision[], int nLeakCurrent_nA[], int nCount);

        /**
        * @brief 发送升级文件
        */
        CMTISDK_API int Cmti_SendUpgradeFile(void* hDevCli, const char *filePathName);

        /**
        * @Deprecated
        * @brief 设置MIPI时钟模式
        * @param discontinuous 时钟模式 0-continuous(default) 1-Discontinuous
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetMipiClockMode(void* hDevCli, bool discontinuous);
        /**
        * @Deprecated
        * @brief 获取MIPI时钟模式
        * @param [out]discontinuous 时钟模式 0-continuous(default) 1-Discontinuous
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetMipiClockMode(void* hDevCli, bool* discontinuous);

#ifdef __cplusplus
    } /* end of namespace CmtiSdk */
} /* end of extern "C" */
#endif /* end of __cplusplus */

#endif // CMTISDK_V1_H
