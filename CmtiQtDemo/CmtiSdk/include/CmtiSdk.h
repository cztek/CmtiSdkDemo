/*
 * @brief CmtiSdk总头文件
 * 建议使用本头件中的最新接口。
 */

#ifndef CMTISDK_H
#define CMTISDK_H 2

#include "TypeDefs.h"    // 基本类型定义头文件
#include "CmtiDefs.h"    // 业务类型定义头文件

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

// 向下兼容头文件（废弃接口或不推荐使用接口），源代码中只引用"CmtiSdk.h"即可
#include "CmtiSdk_V1.h"
// ********************************************************************************************


#ifdef __cplusplus
extern "C" {
    namespace CmtiSdk {
#endif
        // 句柄定义
        typedef void * HDeviceController;
        typedef void * HDeviceClient;

        /**
        * @brief 获取SDK版本号
        *
        * @param libVer 返回SDK版本号字符串，调用者分配内存
        * @param len SDK版本号字符串内存空间长度
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetCmtiSdkVersion(char *libVer, int len);

        /**************************************** DeviceController ****************************************/
        /**
        * @param hDevCtrl 返回DeviceController句柄，代表一个设备控制器
        * @brief 获取设备控制器对象。
        */
        CMTISDK_API int Cmti_GetDeviceController(HDeviceController *hDevCtrl);
        /**
        * @brief 释放设备管理类实例
        */
        CMTISDK_API int Cmti_DestroyDeviceController(HDeviceController hDevCtrl);

        /**
        * @brief 触发一次设备发现
        *
        */
        CMTISDK_API int Cmti_ToggleDiscovery(HDeviceController hDevCtrl);

        /**
        * @brief 根据名称获取设备客户端实例。
        * 当设备名称对应的设备客户端实例不存在时，自动重新枚举一次设备。
        *
        * @param hDevCtrl DeviceController句柄
        * @param devName 设备名称
        * @param hDevCli DeviceClient句柄，指向设备客户端实例
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetDeviceClient(HDeviceController hDevCtrl, const char *devName, HDeviceClient *hDevCli);

        /**
        * @brief 枚举当时在线设备
        *
        * @param deviceNameList 返回设备名列表，调用者分配内存
        * @param deviceNum 输入输出参数，输入表示最大支持的设备个数，即分配的内存空间；输出表示实际返回的设备个数。
        * @return int 错误码
        */
        CMTISDK_API int Cmti_EnumerateDevice(HDeviceController hDevCtrl, char *deviceNameList[], int *deviceNum);

        /**
        * @brief 获取指定设备属性列表。propNameList和propValList中的元素按下标一一对应，表示一个键值对。
        *
        * @param devName 设备名称
        * @param propNameList 返回属性名称列表，调用者分配内存
        * @param propValList 返回属性值列表，调用者分配内存
        * @param propNum 输入输出参数，输入表示支持的最大的属性个数，即分配的内存空间；输出表示实际返回的属性个数。
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetDevicePropList(HDeviceController hDevCtrl, const char *devName, char *propNameList[], 
            char *propValList[], int *propNum);

        /******************************************************* DeviceClient *******************************************************/
        
        /**
        * @brief 获取设备名
        *
        * @param hDevCli 设备
        * @param devName 设备名，调用者负责内存管理
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetDeviceName(HDeviceClient hDevCli, char *devName);

        /**
        * @brief 获取设备属性
        *
        * @param hDevCli/devName 设备句柄或设备名称
        * @param propName 属性名（目前支持的属性名参考SDK手册）
        * @param propVal 属性值，调用者负责内存管理
        * @param maxPropValSize 属性值内存最大长度
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetDevicePropertyByHandle(HDeviceClient hDevCli, const char *propName, char *propVal, int maxPropValSize);
        CMTISDK_API int Cmti_GetDevicePropertyByName(const char* devName, const char *propName, char *propVal, int maxPropValSize);
        /**
        * @brief 获取设备电源引脚信息
        *
        * @param hDevCli 设备
        * @param pinName 传出参数，电源引脚名，调用者负责内存管理。pinName参考E_PowerId定义。
        * @param voltLowerLimit 传出参数，电压下限，调用者负责内存管理
        * @param voltUpperLimit 传出参数，电压上限，调用者负责内存管理
        * @param count 引脚个数，以上3个数组的长度
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetPowerPinInfo(HDeviceClient dev, int pinName[], int voltLowerLimit[], int voltUpperLimit[], int count);
        
        /**
        * @brief 复位设备（主要是关闭传输和影像）
        *
        * @param hDevCli 设备句柄
        * @return int 错误码
        */
        CMTISDK_API int Cmti_ResetDevice(HDeviceClient hDevCli);

        /**
        * @brief 获取采集帧率FPS（设备端事件上报）
        *
        * @param hDevCli 设备
        * @param fps 采集帧率FPS
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetCaptureFps(HDeviceClient hDevCli, float *fps);

        /**
        * @brief 获取传输帧率FPS（设备端事件上报）
        *
        * @param hDevCli 设备
        * @param fps 传输帧率FPS
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetTransmitFps(HDeviceClient hDevCli, float *fps);

        /**
        * @brief 获取接收帧率FPS（接收端SDK统计）
        *
        * @param hDevCli 设备
        * @param fps 传输帧率FPS
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetReceiveFps(HDeviceClient hDevCli, float *fps);

        /**
        * @brief 获取传输误帧率FER（接收端SDK统计）
        *
        * @param hDevCli 设备
        * @param fer 返回传输误帧率FER
        */
        CMTISDK_API int Cmti_GetTransmitFer(HDeviceClient hDevCli, float *fer);

        /**
        * @brief 搜索I2C地址
        * @param u8I2cAddr 输出参数，搜索到的I2C地址，由调用者分配空间
        * @param nCount 输入输出参数，输入表示分配的内存空间个数；输出表示实际返回的I2c地址个数。
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SearchI2cAddress(HDeviceClient hDevCli, uint8_t u8I2cAddr[], int *nCount);

        /**
        * @brief 写单个I2C寄存器
        *
        * @param i2cAddr I2C从地址
        * @param speedkHz I2C速率
        * @param mode I2C模式
        * @param regAddr 寄存器地址
        * @param regData 寄存器值
        * @return int 错误码
        */
        CMTISDK_API int Cmti_WriteSingleI2c(HDeviceClient hDevCli, uint32_t i2cAddr, uint32_t speedkHz, uint32_t mode,
            uint32_t regAddr, uint32_t regData);

        /**
        * @brief 读单个I2C寄存器
        *
        * @param i2cAddr I2C从地址
        * @param speedkHz I2C速率
        * @param mode I2C模式
        * @param regAddr 寄存器地址
        * @param regData 输出参数，寄存器值
        * @return int 错误码
        */
        CMTISDK_API int Cmti_ReadSingleI2c(HDeviceClient hDevCli, uint32_t i2cAddr, uint32_t speedkHz, uint32_t mode,
            uint32_t regAddr, uint32_t* regData);

        /**
        * @brief 批量写I2C寄存器参数（支持转义符，SDK自动分包处理）
        * 主要用于寄存器地址不连续且寄存器数比较多的场景
        *
        * @param i2cAddr I2C从地址
        * @param speedkHz I2C速率
        * @param mode I2C模式
        * @param regAddr[] 寄存器地址数组
        * @param regData[] 寄存器值数组
        * @param delay_ms[] 寄存器延时数组，以ms为单位
        * @param regNum 寄存器个数，即前三个数组的各自元素个数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_WriteDiscreteI2c(HDeviceClient hDevCli, uint32_t i2cAddr, uint32_t speedkHz, uint32_t mode,
            const uint32_t regAddr[], const uint32_t regData[], const uint32_t delay_ms[], int regNum);

        /**
        * @brief 批量读I2C寄存器参数（支持转义符，SDK自动分包处理）
        * 主要用于寄存器地址不连续且寄存器数比较多的场景
        *
        * @param i2cAddr I2C从地址
        * @param speedkHz I2C速率
        * @param mode I2C模式
        * @param regAddr[] 寄存器地址数组
        * @param regData[] 寄存器值数组
        * @param regNum 寄存器个数，即前两个数组的各自元素个数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_ReadDiscreteI2c(HDeviceClient hDevCli, uint32_t i2cAddr, uint32_t speedkHz, uint32_t mode,
            const uint32_t regAddr[], uint32_t regData[], int regNum);

        /**
        * @brief 批量写I2C寄存器参数，支持多IIC地址写（支持转义符，SDK自动分包处理）
        * 主要用于寄存器地址不连续且寄存器数比较多的场景
        *
        * @param speedkHz I2C速率
        * @param mode[] I2C模式
        * @param i2cAddr[] I2C从地址数组
        * @param regAddr[] 寄存器地址数组
        * @param regData[] 寄存器值数组
        * @param delay_ms[] 寄存器延时数组，以ms为单位
        * @param regNum 寄存器个数，即前五个数组的各自元素个数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_WriteDiscreteI2c_V2(HDeviceClient hDevCli, uint32_t speedkHz, uint32_t mode[], const uint32_t i2cAddr[],
            const uint32_t regAddr[], const uint32_t regData[], const uint32_t delay_ms[], int regNum);

        /**
        * @brief 批量读I2C寄存器参数，支持多IIC地址读（支持转义符，SDK自动分包处理）
        * 主要用于寄存器地址不连续且寄存器数比较多的场景
        *
        * @param speedkHz I2C速率
        * @param mode I2C模式
        * @param i2cAddr[] I2C从地址数组
        * @param regAddr[] 寄存器地址数组
        * @param regData[] 寄存器值数组
        * @param regNum 寄存器个数，即前三个数组的各自元素个数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_ReadDiscreteI2c_V2(HDeviceClient hDevCli, uint32_t speedkHz, uint32_t mode, const uint32_t i2cAddr[],
            const uint32_t regAddr[], uint32_t regData[], int regNum);

        /**
        * @brief 写连续的I2C参数（SDK自动分包处理）
        *
        * @param i2cAddr I2C从地址
        * @param speedkHz I2C速率
        * @param regAddr 寄存器地址
        * @param regAddrSize 寄存器地址长度，单位为字节（8位地址为1，16位地址为2）
        * @param data 数据
        * @param dataSize 数据长度，按字节计算的长度
        * @return int 错误码
        */
        CMTISDK_API int Cmti_WriteContinuousI2c(HDeviceClient hDevCli, uint32_t i2cAddr, uint32_t speedkHz, uint32_t regAddr,
            uint32_t regAddrSize, const uint8_t *data, uint32_t dataSize);

        /**
        * @brief 读连续的I2C参数（SDK自动分包处理）
        *
        * @param i2cAddr I2C从地址
        * @param speedkHz I2C速率
        * @param regAddr 寄存器地址
        * @param regAddrSize 寄存器地址长度，单位为字节（8位地址为1，16位地址为2）
        * @param data 数据，调用者分配内存
        * @param dataSize 数据长度
        * @return int 错误码
        */
        CMTISDK_API int Cmti_ReadContinuousI2c(HDeviceClient hDevCli, uint32_t i2cAddr, uint32_t speedkHz, uint32_t regAddr,
            uint32_t regAddrSize, uint8_t *data, uint32_t dataSize);

        /**
        * @brief 判断设备是否已经打开
        *
        * @return bool true-打开 false-关闭
        */
        CMTISDK_API bool Cmti_IsOpen(HDeviceClient hDevCli);

        /**
        * @brief 判断设备是否已连接
        *
        * @param devName 设备名
        * @return bool true-已连接 false-断开
        */
        CMTISDK_API bool Cmti_IsConnected(const char *devName);

        /**
        * @brief 写Sensor时钟
        *
        * @param clk100kHz 时钟（100kHz为单位）
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetSensorClock(HDeviceClient hDevCli, uint32_t clk100kHz);

        /**
        * @brief 设置Sensor接口
        *
        * @param intf Sensor接口，参考E_InterfaceType定义
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetSensorInterface(HDeviceClient hDevCli, uint32_t intf);

        /**
        * @brief 写Mipi参数
        *
        * @param laneNum Mipi Lanes
        * @param freqMHz Mipi时钟，默认800MHz
        * @param virtualChannel Mipi虚拟通道，默认0
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetMipiParam(HDeviceClient hDevCli, uint32_t laneNum, uint32_t freqMHz, uint32_t virtualChannel);
        /**
        * @brief 读Mipi参数
        *
        * @param chnIdx 视频通道索引
        * @param laneNum Mipi Lanes
        * @param freqMHz Mipi时钟
        * @param virtualChannel Mipi虚拟通道
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetMipiParam(HDeviceClient hDevCli, uint32_t *laneNum, uint32_t *freqMHz, uint32_t *virtualChannel);

        /**
        * @brief 写Sensor GPIO值
        * 
        * @param pin 要操作的Pin引脚，使用E_Gpio枚举
        * @param level Pin引脚电平Mask
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetSensorGpioPinLevel(HDeviceClient hDevCli, ushort pin, ushort level);
        /**
        * @brief 读Sensor GPIO值
        *
        * @param pin 要操作的Pin引脚，使用E_Gpio枚举
        * @param level Pin引脚电平Mask
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetSensorGpioPinLevel(HDeviceClient hDevCli, ushort pin, ushort *level);

        /**
        * @brief 写Sensor GPIO方向
        *
        * @param pin 要操作的Pin引脚，使用E_Gpio枚举
        * @param dir Pin引脚方向Mask
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetSensorGpioPinDir(HDeviceClient hDevCli, ushort pin, ushort dir);
        /**
        * @brief 写Sensor GPIO方向
        *
        * @param pin 要操作的Pin引脚，使用E_Gpio枚举
        * @param dir Pin引脚方向Mask
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetSensorGpioPinDir(HDeviceClient hDevCli, ushort pin, ushort *dir);

        /**
        * @brief 写Sensor电压
        *
        * @param powerId[] 电源ID，参考E_PowerId定义
        * @param voltage_mV[] 电压值，单位为mV
        * @param delay_ms[] 延时，单位为ms
        * @param count 电压个数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetSensorPower(HDeviceClient hDevCli, const uint32_t powerId[], const uint32_t voltage_mV[], const uint32_t delay_ms[], uint32_t count);

        /**
        * @brief 读Sensor电压
        *
        * @param powerId[] 电源ID，参考E_PowerId定义
        * @param voltage_mV[] 读到的电压值，单位为mV
        * @param count 电压个数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetSensorPower(HDeviceClient hDevCli, const uint32_t powerId[], uint32_t voltage_mV[], uint32_t count);

        /**
        * @brief 设置采集反馈电压参数，下发参数作用于函数Cmti_GetSensorFeedBackVoltage，此读取函数的耗时取决于采样间隔和采样数量的乘积。
        *        建议调用读取反馈电压接口时每次只读取一个。注意：在函数实现内控制 (采样间隔 * 采样数量 <= 2s)
        * @param nInterval_us 采样间隔，单位微秒
        * @param nPoints 采样数量
        * @param nSampleValType 采样数据处理类型，使用E_SampleValueType枚举
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetFeedBackVoltageSampleParam(HDeviceClient hDevCli, int32_t nInterval_us, int32_t nPoints, int32_t nSampleValType);

        /**
        * @brief 设置采集反馈电压参数，下发参数作用于函数Cmti_GetSensorFeedBackVoltage，此读取函数的耗时取决于采样间隔和采样数量的乘积。
        *        建议调用读取反馈电压接口时每次只读取一个。注意：在函数实现内控制 (采样间隔 * 采样数量 <= 2s)
        * @param nInterval_us 采样间隔，单位微秒
        * @param nPoints 采样数量
        * @param nSampleValType 采样数据处理类型，使用E_SampleValueType枚举
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetFeedBackVoltageSampleParam(HDeviceClient hDevCli, int32_t* nInterval_us, int32_t* nPoints, int32_t* nSampleValType);

        /**
        * @brief 读Senso反馈电压，建议调用读取反馈电压接口时每次只读取一个。详细描述参考Cmti_SetFeedBackVoltageSampleParam函数。
        *
        * @param fbPowerId[] 电源ID，参考E_SensorFeedBackPowerId定义
        * @param voltage[] 读到的电压值，单位为mV
        * @param count 电压个数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetSensorFeedBackVoltage(HDeviceClient hDevCli, const int fbPowerId[], int voltage_mV[], int count);

        /**
        * @brief 设置图像帧参数，在打开视频前需要设置图像帧参数
        *
        * @param imgFmt 图像格式，参考E_ImageFormat定义
        * @param imgMode 图像模式，参考E_ImageMode定义
        * @param width 宽度
        * @param height 高度
        * @param outImgFmt 输出图像格式，参考E_ImageFormat定义（可定制采集的输出格式，如ImgFmt_RAW8, ImgFmt_PackedRaw10...）
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetFrameParam(HDeviceClient hDevCli, uint32_t imgFmt, uint32_t imgMode, uint32_t width, uint32_t height, uint32_t outImgFmt);
        /**
        * @brief 设备图像帧参数，在打开视频前需要设置图像帧参数，带Crop功能
        *
        * @param imgFmt 图像格式，参考E_ImageFormat定义
        * @param imgMode 图像模式，参考E_ImageMode定义
        * @param width 宽度
        * @param height 高度
        * @param outImgFmt 输出图像格式，参考E_ImageFormat定义（可定制采集的输出格式，如ImgFmt_RAW8, ImgFmt_PackedRaw10...）
        * @param cropLeft 剪裁区域x坐标
        * @param cropTop 剪裁区域y坐标
        * @param cropWidth 剪裁区域宽度
        * @param cropHeight 剪裁区域高度
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetFrameParam2(HDeviceClient hDevCli, uint32_t imgFmt, uint32_t imgMode, uint32_t width, uint32_t height, uint32_t outImgFmt,
            uint32_t cropLeft, uint32_t cropTop, uint32_t cropWidth, uint32_t cropHeight);

        /**
        * @brief 获取图像帧参数
        *
        * @param imgFmt 图像格式，参考E_ImageFormat定义
        * @param imgMode 图像模式，参考E_ImageMode定义
        * @param width 宽度
        * @param height 高度
        * @param size 输出图像大小
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetFrameParam(HDeviceClient hDevCli, uint32_t *imgFmt, uint32_t *imgMode, uint32_t *width, uint32_t *height, uint32_t *size);

        /**
        * @brief 设置MIPI Embedded Line Size
        *
        * @param hDevCli 设备对象句柄
        * @param size Embedded line size
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetEmbeddedLineSize(HDeviceClient hDevCli, uint32_t size);

        /**
        * @brief 设置ROI参数，支持热切换
        *
        * @param roiRect ROI矩形框定义，参考T_Rect定义
        * @param roiCount ROI矩形框个数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetRoiParam(HDeviceClient hDevCli, const T_Rect roiRect[], uint32_t roiCount);

        /**
        * @brief 设置Sensor自身的StreamOn寄存器
        * 当器件 i2cAddr、regAddr、regDataOn、regDataOff全都被设置为0时，表示清除此前设置数据
        *
        * @param hDevCli 设备对象句柄
        * @param i2cAddr 芯片I2c地址
        * @param regAddr Stream On寄存器地址
        * @param regDataOn  Stream On时寄存器值
        * @param regDataOff Stream Off时寄存器值
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetSensorStreamOnRegister(HDeviceClient hDevCli, uint32_t i2cAddr, uint32_t regAddr, uint32_t regDataOn, uint32_t regDataOff);
        
        /**
        * @brief 视频控制
        *
        * @param ctrl 控制命令，0为关闭，1为打开
        * @return int 错误码
        */
        CMTISDK_API int Cmti_VideoControl(HDeviceClient hDevCli, uint32_t ctrl);

        /**
        * @brief 传输延时控制
        *
        * @param ctrl delay_ms 延时，单位ms
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetTransmitDelay(HDeviceClient hDevCli, uint32_t delay_ms);
        CMTISDK_API int Cmti_GetTransmitDelay(HDeviceClient hDevCli, uint32_t *delay_ms);

        /**
        * @brief 设置工装采集轮询超时时间，默认SDK轮询采集次数3次
        *
        * @param grabTimeout超时时间，单位ms
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetGrabTimeout(HDeviceClient hDevCli, uint32_t grabTimeout_ms);

        /**
        * @brief 设置工装采集轮询超时时间和SDK采集轮询次数，该结构总的返回时间为 grabTimeout_ms * loopTimes
        *
        * @param grabTimeout工装轮询超时时间，单位ms
        * @param loopTimes SDK采集轮询次数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetGrabTimeoutAndLoopTimes(HDeviceClient hDevCli, uint32_t grabTimeout_ms, uint32_t loopTimes);

        /**
        * @brief 获取测试仪当前系统时间
        *
        * @param timestamp 时间戳
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetSystemTimestamp(HDeviceClient hDevCli, uint64 *timestamp);

        /**
        * @brief 丢弃当前帧后的指定帧数。阻塞操作，函数在丢弃N帧后，有新帧到达时才返回。超时时间按设置的grabTimeout*N计算。
        *
        * @param hDevCli 设备对象
        * @param count 要丢弃的帧数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SkipFrame(HDeviceClient hDevCli, int count);

        /**
        * @brief 从缓冲池拷贝数据到应用层
        *
        * @param pbuffer 应用层缓冲区
        * @param bufferLen 应用层缓冲区长度
        * @param timestamp 时间戳（默认为帧首时间戳）
        * @param headTimestamp 帧首时间戳
        * @param tailTimestamp 帧尾时间戳
        * @param frameSequence 帧序号
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GrabFrame(HDeviceClient hDevCli, uint8_t *pbuffer, int bufferLen, uint64 *timestamp);
        CMTISDK_API int Cmti_GrabFrame2(HDeviceClient hDevCli, uint8_t *pbuffer, int bufferLen, uint64 *headTimestamp,
            uint64 *tailTimestamp, uint32 *frameSequence);
        CMTISDK_API int Cmti_GrabLatestFrame(HDeviceClient hDevCli, uint8_t *pbuffer, int bufferLen, uint64 *timestamp);
        CMTISDK_API int Cmti_GrabLatestFrame2(HDeviceClient hDevCli, uint8_t *pbuffer, int bufferLen, uint64 *headTimestamp,
            uint64 *tailTimestamp, uint32 *frameSequence);

        /**
        * @brief 从队列中取出一个Buffer(与EnqueueFrameBuffer配对使用)，使用缓冲池中的缓冲区，无需要用户分配内存
        *
        * @param bufIdx 缓冲区索引，使用完毕后在EnqueueFrameBuffer中归还
        * @param pbuffer 引用类型，缓冲区
        * @param timestamp 时间戳（默认为帧首时间戳）
        * @param headTimestamp 帧首时间戳
        * @param tailTimestamp 帧尾时间戳
        * @param frameSequence 帧序号
        * @return int 错误码
        */
        CMTISDK_API int Cmti_DequeueFrameBuffer(HDeviceClient hDevCli, int *bufIdx, uint8_t **pbuffer, uint64 *timestamp);
        CMTISDK_API int Cmti_DequeueFrameBuffer2(HDeviceClient hDevCli, int *bufIdx, uint8_t **pbuffer, uint64 *headTimestamp, 
            uint64 *tailTimestamp, uint32 *frameSequence);
        CMTISDK_API int Cmti_DequeueLatestFrameBuffer(HDeviceClient hDevCli, int *bufIdx, uint8_t **pbuffer, uint64 *timestamp);
        CMTISDK_API int Cmti_DequeueLatestFrameBuffer2(HDeviceClient hDevCli, int *bufIdx, uint8_t **pbuffer, uint64 *headTimestamp,
            uint64 *tailTimestamp, uint32 *frameSequence);

        /**
        * @brief 入队一个Buffer(与DequeueFrameBuffer配对使用)
        *
        * @param bufIdx 缓冲区索引
        * @return int 错误码
        */
        CMTISDK_API int Cmti_EnqueueFrameBuffer(HDeviceClient hDevCli, int bufIdx);

        /**
        * @brief 设置开短路测试参数
        *
        * @param supplyVol_uV 测试供电电压，单位uV，推荐1.4V
        * @param supplyCurrent_uA 测试供电电流，单位uA，推荐500uA
        * @param pinId[] 待测试的Pin引脚定义，参考E_OSM_PIN_TYPE定义
        * @param openStdVol_uV[] 开路标准，单位uV，可以独立设置，推荐1V
        * @param shortStdVol_uV[] 短路标准，单位uV，可以独立设置，推荐200mV
        * @param count 引脚个数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetOsTestConfig(HDeviceClient hDevCli, uint32_t supplyVol_uV, uint32_t supplyCurrent_uA, const uint32_t pinId[],
            const uint32_t openStdVol_uV[], const uint32_t shortStdVol_uV[], uint32_t count);

        /**
        * @brief 设置开短路测试电流方向
        *
        * @param bool 电流方向，true为正向电流，false为反向电流
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetOsTestCurrentDirection(HDeviceClient hDevCli, bool positive);

        /**
        * @brief 读开短路测试结果
        *
        * @param pinId[] 测试的Pin引脚定义，参考E_OSM_PIN_TYPE定义
        * @param openVol_uV[] 开路电压，单位uV
        * @param shortVol_uV[] 短路电压，单位uV
        * @param result[] 测试结果
        * @param count 引脚个数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_ReadOsTestResult(HDeviceClient hDevCli, const uint32_t pinId[], uint32_t openVol_uV[], uint32_t shortVol_uV[],
            uint32_t result[], uint32_t count);

        /**
        * @brief 获取电流
        *
        * @param powerId[] 电源ID，参考E_PowerId定义
        * @param currentRange[] 电流量程，参考E_CurrentRange定义
        * @param current_nA[] 电流值，单位为nA
        * @param count 电源个数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetCurrent(HDeviceClient hDevCli, const uint32_t powerId[], const uint32_t currentRange[], float current_nA[], uint32_t count);

        /**
        * @brief 获取电流
        *
        * @param powerId[] 电源ID，参考E_PowerId定义
        * @param upperLimit_nA[] 电流上限值，对于上限以内的测量值会更精准，超出上限的值误差会较大。
        * @param autoHighPrecision[] 各类型电流自动匹配高精度标志，0：不匹配，1：向下匹配高精度
        * @param current_nA[] 电流值，单位为nA
        * @param count 电源个数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetCurrentV2(HDeviceClient hDevCli, const int powerId[], const int upperLimit_nA[], const uint16_t autoHighPrecision[],
            float current_nA[], int count);

        /**
        * @brief 设置过流参数
        *
        * @param powerId[] 电源ID，参考E_PowerId定义
        * @param currentThrd_mA[] 过流阈值，单位为mA。
        * @param debounceInterval_ms[] 防抖时间，单位为ms。
        * @param count 电源个数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetOvercurrentParam(HDeviceClient hDevCli, const int powerId[], const int currentThrd_mA[],
            const int debounceInterval_ms[], int count);

        /**
        * @brief 获取过流参数
        *
        * @param powerId[] 电源ID，参考E_PowerId定义
        * @param currentThrd_mA[] 输出参数，过流阈值，单位为mA。
        * @param debounceInterval_ms[] 输出参数，防抖时间，单位为ms
        * @param count 电源个数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetOvercurrentParam(HDeviceClient hDevCli, const int powerId[], int currentThrd_mA[],
            int debounceInterval_ms[], int count);
        
        /**
        * @brief 查询过流
        *
        * @param powerId[] 输出参数，返回过流的电源ID，参考E_PowerId定义，调用者分配内存。
        * @param count 输入输出参数，输入表示分配的PowerId数组空间个数；输出表示实际返回的PowerId个数。
        * @return int 错误码
        */
        CMTISDK_API int Cmti_QueryOvercurrent(HDeviceClient hDevCli, int powerId[], int *count);

        /**
        * @brief 打开蜂鸣器开
        *
        * @param delay_ms 延时时间，单位ms
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetBeepOn(HDeviceClient hDevCli, uint32_t delay_ms);

        /**
        * @brief 测试电源pin脚漏电流测试
        * @param nPowerId 电源ID，参考E_PowerId定义
        * @param nSupplyVoltage_mV 测试供电电压，单位mV
        * @param nUpperLimitCurrent_nA  电流上限，单位nA
        * @param bAutoHighPrecision[] 各类型电流自动匹配高精度标志，0：不匹配，1：向下匹配高精度
        * @param nLeakCurrent_nA 输出参数，漏电流，单位nA
        * @param nCount 电源ID个数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_PowerPinLC(HDeviceClient hDevCli, const int nPowerId[], const int nSupplyVoltage_mV[],
            const int nUpperLimitCurrent_nA[], const int bAutoHighPrecision[], int nLeakCurrent_nA[], int nCount);

        /**
        * @brief 信号pin脚漏电流测试
        * @param nMipiPinSupplyVoltage_mV MIPI脚供电电压，单位mV
        * @param nIoPinSupplyVoltage_mV IO脚供电电压，单位mV
        * @param nDirection 方向，正向为1，反向为0
        * @param nPinId Pin ID，参考E_OSM_PIN_TYPE定义
        * @param nLeakCurrent_nA 输出参数，漏电流，单位nA
        * @param nCount pin脚个数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SignalPinLC(HDeviceClient hDevCli, int nMipiPinSupplyVoltage_mV, int nIoPinSupplyVoltage_mV, int nDirection,
            const int nPinId[], int nLeakCurrent_nA[], int nCount);

        /**
        * @brief 设置I2C上拉电阻
        * @param nSclPullupResistor scl值，单位：欧姆
        * @param nSdaPullupResistor sda值，单位：欧姆
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetI2cPullupResistor(HDeviceClient hDevCli, int nSclPullupResistor, int nSdaPullupResistor);
        /**
        * @brief 读取I2C上拉电阻
        * @param nSclPullupResistor 输出参数，scl值，单位：欧姆
        * @param nSdaPullupResistor 输出参数，sda值，单位：欧姆
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetI2cPullupResistor(HDeviceClient hDevCli, int *nSclPullupResistor, int *nSdaPullupResistor);

        /**
        * @brief 设置I2C上拉输出电压
        * @param nPullupVoltage 上拉输出电压，单位：mV
        * @param bSclEnabled 是否使能SCL输出
        * @param bSdaEnabled 是否使能SDA输出
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetI2cPullupOutput(HDeviceClient hDevCli, int nPullupVoltage, int bSclEnabled, int bSdaEnabled);

        /**
        * @brief 设置I2C推挽输出
        *
        * @param enabled 推挽输出功能使能，true为使能，false为禁止
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetI2cPushPullOutput(HDeviceClient hDevCli, bool enabled);

        /**
        * @brief 读取I2C推挽输出状态
        *
        * @param enabled 输出参数，推挽输出功能使能，true为使能，false为禁止
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetI2cPushPullOutput(HDeviceClient hDevCli, bool *enabled);

        /**
        * @brief 测量mipi 引脚电压,针对芯片处于LP模式下
        * @param nMipiPin 测量mipi引脚，参考E_OSM_PIN_TYPE定义
        * @param nVoltage_uV 引脚电压，单位uV
        * @param nCount 引脚个数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetMipiPinVoltage(HDeviceClient hDevCli, const int nMipiPin[], int nVoltage_uV[], int nCount);

        /**
        * @brief 测量PO 引脚电压
        * @param nPoPin 测量PO引脚，参考E_OSM_PIN_TYPE定义
        * @param nVoltage_uV 引脚电压，单位uV
        * @param nCount 引脚个数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetPoPinVoltage(HDeviceClient hDevCli, const int nPoPin[], int nVoltage_uV[], int nCount);

        /**
        * @brief SPI传输
        * @param spiId SPI编号，参考E_SpiId
        * @param txBuf 发送数据缓存区
        * @param rxBuf 接收数据缓存区，将一次通信所有的MISO数据保存到接收缓存区中
        * @param nLen 接收和发送缓存区长度，该长度表示在SPI总线上发送的时钟脉冲字节数据
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SpiTransfer(HDeviceClient hDevCli, uint spiId, const uchar txBuf[], uchar rxBuf[], int nLen);

        /**
        * @brief 设置SPI通信参数
        * @param spiId SPI编号，参考E_SpiId
        * @param mode SPI通信模式，参考E_SpiMode
        * @param speed_kHz SPI通信速率，单位kHz
        * @param bitsPerWord SPI通信字bit长度，默认8bit
        * @param delay_us SPI通信延时，当前SPI通信结束后到下一次SPI通信的延时时间
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetSpiParameter(HDeviceClient hDevCli, uint spiId, uint mode, uint speed_kHz, uint bitsPerWord, uint delay_us);

        /**
       * @brief 获取SPI通信参数
       * @param spiId SPI编号，参考E_SpiId
       * @param mode SPI通信模式，参考E_SpiMode
       * @param speed_kHz SPI通信速率，单位kHz
       * @param bitsPerWord SPI通信字bit长度，默认8bit
       * @param delay_us SPI通信延时，当前SPI通信结束后到下一次SPI通信的延时时间
       * @return int 错误码
       */
        CMTISDK_API int Cmti_GetSpiParameter(HDeviceClient hDevCli, uint spiId, uint *mode, uint *speed_kHz, uint *bitsPerWord, uint *delay_us);

        /**
        * @brief 获取PO 引脚间电阻
        * @param endPoint 电阻端点，参考E_PoResEndPoint定义
        * @param [out]resistance_mOhm 电阻值，单位毫欧姆
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetPoResistance(HDeviceClient hDevCli, int endPoint, int& resistance_mOhm);

        /**
        * @brief 获取MIPI状态
        * @param nStatusKey 状态属性名。参考E_MipiStatus定义。
        * @param nStatusVal 状态属性值
        * @param nCount 属性对个数，即前两数组大小
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetMipiStatus(HDeviceClient hDevCli, int nStatusKey[], int nStatusVal[], int nCount);


        /**
        * @brief 测量指定引脚的输入周期使能接口
        * @param nPin 输入的测量引脚编号，使用E_OSM_PIN_TYPE类型，对于CS821支持OSM_PIN_PO1引脚，对于CP881支持OSM_PIN_PO1和OSM_PIN_PO2引脚
        * @param nCtrl 控制标志，1表示启动测量；0表示停止测量
        * @return int 错误码
        */
        CMTISDK_API int Cmti_MeasureFrameSyncPeriodControl(HDeviceClient hDevCli, int32_t nPin, int32_t nCtrl);

        /**
        * @brief 获取指定引脚输出的方波周期测量结果
        * @param nPin 输入的测量引脚编号，使用E_OSM_PIN_TYPE类型，对于CS821支持OSM_PIN_PO1引脚，对于CP881支持OSM_PIN_PO1和OSM_PIN_PO2引脚
        * @param [out]pPeriod_us 方波周期，单位微秒
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetFrameSyncPeriod(HDeviceClient hDevCli, int32_t nPin, uint32_t* pPeriod_us);

        /**
        * @brief 设置PMIC开关模式
        * @param mode 设置的模式，使用E_PmicSwitchMode定义的类型
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetPmicSwitchMode(HDeviceClient hDevCli, int32_t mode);

        /**
        * @brief 获取PMIC开关模式
        * @param [out]pMode 当前PMIC使用的开关模式
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetPmicSwitchMode(HDeviceClient hDevCli, int32_t* pMode);

        /**
        * @brief 获取MIPI DPHY HS电压
        * @param nPositive 正反向测试标志，1表示正向，0表示反向
        * @param nSupplyCurrent_uA 测试时提供的电流，标量只表示数值无方向，单位微安
        * @param nPinsId 输入的测量引脚编号，使用E_OSM_PIN_TYPE类型
        * @param [out]nVoltage_uV 获取到的电压值，单位微伏
        * @param nCount 测量的引脚数量
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetMipiDPhyHSVoltage(HDeviceClient hDevCli, int32_t nPositive, int32_t nSupplyCurrent_uA, const int32_t nPinsId[], int32_t nVoltage_uV[], int32_t nCount);

        /**
        * @brief 设置MipiReceiverMode模式
        * @mode 参考枚举E_MipiReceiverMode
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetMipiReceiverMode(HDeviceClient hDevCli, int32_t mode);

        /**
        * @brief 校正DPS输出电压
        * @param powerIds 待校正的电源，使用E_PowerId类型
        * @param voltagemV 电源输出的电压，单位mV
        * @param count 校正电源数量，需要一次性校正测试仪支持的所有电源
        * @return int 错误码
        */
        CMTISDK_API int Cmti_CalibrateDpsOutputVoltage(HDeviceClient hDevCli, const int32_t nPowerIds[], const int32_t nVoltagemV[], int32_t nCount);

        /**
        * @brief 读取DPS输出电压校正参数
        * @param powerIds 待读取的电源，使用E_PowerId类型
        * @param zeroOffsetmV DPS零偏校正值，单位mV
        * @param count 电源数量
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetDpsOutputVoltageCalibration(HDeviceClient hDevCli, const int32_t nPowerIds[], int32_t nZeroOffsetmV[], int32_t nCount);

        /**
        * @brief 设置DPS内部Force和Sense间电阻短接/断开状态
        * @param powerIds 待设置的电源，使用E_PowerId类型
        * @param enable 使能标志，1表示使能，0表示断开
        * @param count 电源数量
        * @return int 错误码
        */
        CMTISDK_API int Cmti_SetDpsResistorStateBetweenForceAndSense(HDeviceClient hDevCli, const int32_t nPowerIds[], const int32_t nEnable[], int32_t nCount);

        /**
        * @brief 读取DPS内部Force和Sense间电阻短接/断开状态
        * @param powerIds 待读取的电源，使用E_PowerId类型
        * @param enable 读取到的状态，1表示使能，0表示断开
        * @param count 电源数量
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetDpsResistorStateBetweenForceAndSense(HDeviceClient hDevCli, const int32_t nPowerIds[], int32_t nEnable[], int32_t nCount);

        /**
        * @brief 设置电压测量使用的负载电流源参数
        * @param nMeasureTarget 电压测量对象，使用E_VolSampleTarget枚举
        * @param nCurrent_uA 供电电流，单位uA
        * @param nLimitVoltage_mV 电流源钳位电压，单位mV
        * @return int32_t 错误码
        * */
        CMTISDK_API int Cmti_SetMeasureVoltageCurrentSourceParam(HDeviceClient hDevCli, int32_t nMeasureTarget, int32_t nCurrent_uA, int32_t nLimitVoltage_mV);

        /**
        * @brief 设置电源压摆率
        * @param nPowerIds 电源编号，使用E_PowerId类型，(DOVDD、VOIS)此参数为只读状态。
        * @param fSlewRate 压摆率，单位mV/us， 0表示不启用设置参数，DPS使用内部默认值
        * @param nCount 设置的电源数量
        * @return int32_t 错误码
        * */
        CMTISDK_API int Cmti_SetDpsSlewRate(HDeviceClient hDevCli, const int32_t nPowerIds[], const float fSlewRate[], int32_t nCount);

        /**
        * @brief 读取当前电源压摆率
        * @param nPowerIds 电源编号，使用E_PowerId类型
        * @param fSlewRate 压摆率，单位mV/us，0表示DPS使用内部默认值
        * @param nCount 设置的电源数量
        * @return int32_t 错误码
        * */
        CMTISDK_API int Cmti_GetDpsSlewRate(HDeviceClient hDevCli, const int32_t nPowerIds[], float fSlewRate[], int32_t nCount);

        /**
        * @brief 获取Sensor出图后的帧数和耗时
        * @param [out]nFrameCount 帧数
        * @param [out]nElapsed_ms 耗时，单位ms
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetSensorFrameCountFromVideoOn(HDeviceClient hDevCli, int *nFrameCount, int *nElapsed_ms);

        /**
        * @brief 获取SDK接收到的帧数和耗时
        * @param [out]nFrameCount 帧数
        * @param [out]nElapsed_ms 耗时，单位ms
        * @return int 错误码
        */
        CMTISDK_API int Cmti_GetReceivedFrameCountFromVideoOn(HDeviceClient hDevCli, int *nFrameCount, int *nElapsed_ms);

        /**
         * @brief 设置MIPI D-Phy接收器 DataLane 稳定时间
         * @param settleTime_ns 时间值,单位：ns，值范围[70,400]
         * @return int 错误码
         */
        CMTISDK_API int Cmti_SetMipiDphyReceiverDataLaneHSSettleTime(HDeviceClient hDevCli, int32_t settleTime_ns);

        /**
          * @brief 获取MIPI D-Phy接收器 DataLane 稳定时间
          * @param settleTime_ns 时间值,单位：ns，值范围[70,400]
          * @return int 错误码
          */
        CMTISDK_API int Cmti_GetMipiDphyReceiverDataLaneHSSettleTime(HDeviceClient hDevCli, int32_t* settleTime_ns);

        /**
          * @brief 设置MIPI D-Phy接收器 ClockLane 稳定时间
          * @param settleTime_ns 时间值,单位：ns，值范围[0,320]
          * @return int 错误码
          */
        CMTISDK_API int Cmti_SetMipiDphyReceiverClockLaneHSSettleTime(HDeviceClient hDevCli, int32_t settleTime_ns);

        /**
         * @brief 获取MIPI D-Phy接收器 ClockLane 稳定时间
         * @param settleTime_ns 时间值,单位：ns，值范围[0,320]
         * @return int 错误码
         */
        CMTISDK_API int Cmti_GetMipiDphyReceiverClockLaneHSSettleTime(HDeviceClient hDevCli, int32_t* settleTime_ns);

        /**
         * @brief 设置MIPI C-Phy接收器 DataLane 稳定时间
         * @param settleTime_ns 时间值,单位：ns，值范围[70,400]
         * @return int 错误码
         */
        CMTISDK_API int Cmti_SetMipiCphyReceiverDataLaneHSSettleTime(HDeviceClient hDevCli, int32_t settleTime_ns);

        /**
          * @brief 获取MIPI C-Phy接收器 DataLane 稳定时间
          * @param settleTime_ns 时间值,单位：ns，值范围[70,400]
          * @return int 错误码
          */
        CMTISDK_API int Cmti_GetMipiCphyReceiverDataLaneHSSettleTime(HDeviceClient hDevCli, int32_t* settleTime_ns);

        /**
          * @brief 设置MIPI C-Phy接收器 ClockLane 稳定时间
          * @param settleTime_ns 时间值,单位：ns，值范围[0,320]
          * @return int 错误码
          */
        CMTISDK_API int Cmti_SetMipiCphyReceiverClockLaneHSSettleTime(HDeviceClient hDevCli, int32_t settleTime_ns);

        /**
         * @brief 获取MIPI C-Phy接收器 ClockLane 稳定时间
         * @param settleTime_ns 时间值,单位：ns，值范围[0,320]
         * @return int 错误码
         */
        CMTISDK_API int Cmti_GetMipiCphyReceiverClockLaneHSSettleTime(HDeviceClient hDevCli, int32_t* settleTime_ns);

        /**
        * @brief 异步获取DPS电流，支持取n次中的最大值、最小值、平均值, 与Cmti_AsyncGetDpsCurrentResult需成对调用
        *
        * @param powerId[] 电源ID，参考E_PowerId定义
        * @param nCurrentRange 电流测量档位
        * @param nCount 电源个数
        * @param nSampleInterval_us 采样间隔时间，延时过后，再次获取电流，单位(微秒)
        * @param nSamplePoint 异步操作中获取电流的采样次数,要求： nSamplePoint * nSampleInterval_us < 2s
        * @param eSampleType 最终获取结果电流的采样模式方式，参考枚举 E_SampleValueType
        * @return int 错误码
        */
        CMTISDK_API int Cmti_CreateAsyncGetDpsCurrentOperation(HDeviceClient hDevCli, const int powerId[], const int currentRange[], int nCount,
            int nSampleInterval_us, int nSamplePoint, int eSampleType);

        /**
        * @brief 获取异步操作的结果电流，与Cmti_CreateAsyncGetDpsCurrentOperation需成对调用
        *
        * @param powerId[] 获取电流的引脚，对应current_nA的电流值
        * @param current_nA[] 电流值，单位为nA
        * @param nCount 电源个数
        * @return int 错误码
        */
        CMTISDK_API int Cmti_AsyncGetDpsCurrentResult(HDeviceClient hDevCli, int powerId[], double current_nA[], int nCount);

        /**
        * @brief 同步获取DPS电流，支持或取采样中的最大值、最小值、平均值
        *
        * @param powerId[] 电源ID，参考E_PowerId定义
        * @param nCurrentRange 电流测量档位
        * @param current_nA 获取的电流值
        * @param nCount 电源个数
        * @param nSampleInterval_us 采样间隔时间，延时过后，再次获取电流，单位(微秒)
        * @param nSamplePoint 同步操作中获取电流的采样次数
        * @param eSampleType 最终获取结果电流的采样模式方式，参考枚举 E_SampleValueType
        * @return int 错误码
        */
        CMTISDK_API int32_t Cmti_GetDpsCurrent(HDeviceClient hDevCli, const int powerId[], const int currentRange[], double current_nA[], int nCount, int nSampleInterval_us, int nSamplePoint, int eSampleType);

        /**
        * @brief 获取DPS所有采样点电流
        *
        * @param powerId[] 电源ID，参考E_PowerId定义
        * @param nCurrentRange 电流测量档位
        * @param current_nA 获取的电流值，返回格式：[Dvdd0,Dvdd1,...Dvddn,Avdd0,Avdd1,...Avddn]（其中 n = nSamplePoiint - 1）
        * @param nCount 电源个数
        * @param nSampleInterval_us 采样间隔时间，延时过后，再次获取电流，单位(微秒)
        * @param nSamplePoint 同步操作中获取电流的采样次数，要求：nSamplePoint * nCount * 8 < 1446
        * @return int 错误码
        */
        CMTISDK_API int32_t Cmti_GetDpsAllSamplePointCurrent(HDeviceClient hDevCli, const int powerId[], const int currentRange[], double current_nA[], int nCount, int nSampleInterval_us, int nSamplePoint);

        /**
        * @brief 设置指定扩展IO引脚的电平
        *
        * @param pin 扩展IO ID，参考 定义
        * @param level 设置的电平
        * @return int 错误码
        */
        CMTISDK_API int32_t Cmti_SetExtendGpioPinLevel(HDeviceClient hDevCli, uint16_t pin, uint16_t level);

        /**
        * @brief 获取指定扩展IO引脚的电平
        *
        * @param pin 扩展IO ID，参考 定义
        * @param level 获取的电平
        * @return int 错误码
        */
        CMTISDK_API int32_t Cmti_GetExtendGpioPinLevel(HDeviceClient hDevCli, uint16_t pin, uint16_t& level);

        /**
        * @brief 设置指定扩展IO的方向
        *
        * @param pin 扩展IO ID，参考 定义
        * @param dir 设置扩展IO 方向
        * @return int 错误码
        */
        CMTISDK_API int32_t Cmti_SetExtendGpioPinDir(HDeviceClient hDevCli, uint16_t pin, uint16_t dir);

        /**
        * @brief 获取指定扩展IO的方向
        *
        * @param pin 扩展IO ID，参考 定义
        * @param dir 获取的扩展 IO 方向
        * @return int 错误码
        */
        CMTISDK_API int32_t Cmti_GetExtendGpioPinDir(HDeviceClient hDevCli, uint16_t pin, uint16_t& dir);

        /**
        * @brief 进入DC测试
        * @param nPositiveVoltage 待测引脚正电压标志，1表示正电压，0表示负电压。
        * @return int32_t 错误码
        * */
        CMTISDK_API int Cmti_EnterDCTest(HDeviceClient hDevCli, int32_t nPositiveVoltage);

        /**
        * @brief 退出DC测试
        * @return int32_t 错误码
        * */
        CMTISDK_API int Cmti_LeaveDCTest(HDeviceClient hDevCli);

        /**
        * @brief 设置PMU输出电流
        * @param nSupplyCurrent_uA PMU输出的电流，单位uA
        * @param nClamp_mV 钳位电压，单位mV，仅输入正数，底层实现正反向电压钳位
        * @param nPins 输入的引脚编号，使用E_OSM_PIN_TYPE类型
        * @param nCount 引脚数量
        * @return int32_t 错误码
        * */
        CMTISDK_API int Cmti_PMUForceCurrent(HDeviceClient hDevCli, int32_t nSupplyCurrent_uA, int32_t nClampVol_mV, const int32_t nPins[], uint32_t nCount);

        /**
        * @brief 测量指定引脚电压
        * @param nDelayTime_ms 测量前延时时间，单位ms
        * @param nSamplePoints 采样点数
        * @param nSampleInterval_us 采样间隔，单位us
        * @param nSampleValType  读取到的采样数据处理方式，使用E_SampleValueType枚举
        * @param nPins 输入的测量引脚编号，使用E_OSM_PIN_TYPE类型
        * @param nMeasVol_mV 输出的电压值，单位uV
        * @param nCount 引脚数量
        * @return int32_t 错误码
        * */
        CMTISDK_API int Cmti_PMUMeasurePinVoltage(HDeviceClient hDevCli, int32_t nDelayTime_ms, int32_t nSamplePoints, int32_t nSampleInterval_us, int32_t nSampleValType, const int32_t nPins[], int32_t nMeasVol_uV[], int32_t nCount);

        /**
        * @brief 设置PMU输出电压
        * @param nSupplyVoltage_uV PMU输出的电压，单位uV
        * @param nCurrentRange 电流档位
        * @param nClampCurrent_uA 钳位电流，单位uA，仅输入正数，底层实现正反向电压钳位
        * @param nPins 输入的引脚编号，使用E_OSM_PIN_TYPE类型
        * @param nCount 引脚数量
        * @return int32_t 错误码
        * */
        CMTISDK_API int Cmti_PMUForceVoltage(HDeviceClient hDevCli, int32_t nSupplyVoltage_uV, int32_t nCurrentRange, int32_t nClampCurrent_uA, const int32_t nPins[], uint32_t nCount);

        /**
        * @brief 测量指定引脚电流
        * @param nDelayTime_ms 测量前延时时间，单位ms
        * @param nSamplePoints 采样点数
        * @param nSampleInterval_us 采样间隔，单位us
        * @param nSampleValType  读取到的采样数据处理方式，使用E_SampleValueType枚举
        * @param nPins 输入的测量引脚编号，使用E_OSM_PIN_TYPE类型
        * @param fMeasCur_nA 测量得到的电流值，单位nA
        * @param nCount 引脚数量
        * @return int32_t 错误码
        * */
        CMTISDK_API int Cmti_PMUMeasurePinCurrent(HDeviceClient hDevCli, int32_t nDelayTime_ms, int32_t nSamplePoints, int32_t nSampleInterval_us, int32_t nSampleValType, const int32_t nPins[], double nMeasCur_nA[], int32_t nCount);

        /**
        * @brief 设置指定引脚的电压
        * @param nPins 输入的测量引脚编号，使用E_OSM_PIN_TYPE类型
        * @param nCount 引脚数量
        * @param nSupplyVoltage_uV 支持电压。
        * @return int32_t 错误码
        * */
        CMTISDK_API int Cmti_SetPinVoltage(HDeviceClient hDevCli, const int32_t nPins[], int32_t nCount, int32_t nSupplyVoltage_uV);

        /**
        * @brief 设置指定的引脚为高阻态
        * @param pinIds 输入的测量引脚编号，使用E_OSM_PIN_TYPE类型
        * @param count 引脚数量
        * @return int32_t 错误码
        * */
        CMTISDK_API int Cmti_SetPinHighImpedanceState(HDeviceClient hDevCli, const int32_t pinIds[], int32_t count);

        /**
        * @brief 设置图像传输模式，模式会保存直至进程结束或再次调用该接口更改模式或重新绑定工装，所以该接口在不需要更改传输模式的情况下，调用一次即可，不需要重复调用。
        * @param transmitterMode 传输模式类型，使用E_TransmitterMode类型
        * @return int32_t 错误码
        * */
        CMTISDK_API int Cmti_SetTransmissionMode(HDeviceClient hDevCli, int32_t transmitterMode);

        /**
        * @brief 获取图像传输模式
        * @param pTransmitterMode 传输模式类型，使用E_TransmitterMode类型
        * @return int32_t 错误码
        * */
        CMTISDK_API int Cmti_GetTransmissionMode(HDeviceClient hDevCli, int32_t *pTransmitterMode);

        /**
        * @brief 采集指定数量连续帧，该接口自动判定抓取连续图像帧
        *
        * @param pImageBuf 保存传输图像，内存空间由调用者管理
        * @param pHeadTimestamp 帧头时间戳buf
        * @param pTailTimestamp 帧尾时间戳buf
        * @param pFrameSequence 帧序buf
        * @param imageCount 接收图像数量
        * @param timeout 接口调用超时时间，传入0为阻塞式等待，timeout > imageCount * 一帧采集时间
        * @return int32_t 错误码
        * */
        CMTISDK_API int Cmti_GrabContinuousFrames(HDeviceClient hDevCli, uint8_t *pImageBuf[], uint64_t pHeadTimestamp[], uint64_t pTailTimestamp[],
                                                  uint32_t pFrameSequence[], int32_t imageCount, int32_t frameSize, int32_t timeout_ms);

        /**
        * @brief 计算多帧图像平均值异步调用（与Cmti_WaitForAsyncResult配套使用获取结果）,
        *        仅支持Raw8 ~ Raw14，且不支持Roi功能，需整帧图像传输，该接口需在请求传输帧模式下使用，模式设置见Cmti_SetTransmissionMode接口
        * @param nImageCount 参与计算平均值的帧数量
        * @param [in/out]pAsyncContext 异步调用上下文指针
        * @return int32_t 错误码
        * */
        CMTISDK_API int Cmti_CalcAverageImageAsync(HDeviceClient hDevCli, int32_t nImageCount, T_CalcAverageImageAsyncContext* pAsyncContext);

        /**
        * @brief 等待异步调用完成，或者timeout_ms传入为0时当作查询异步操作当前状态，查询更新状态，
        *        如果状态为为AsyncState_WaitTransmitter，对应的结果存储在返回值结构体中
        * @param [in/out]pAsyncContext 异步调用上下文指针
        * @param timeout_ms 传入0时，查询当前异步操作状态，如果状态为为AsyncState_WaitTransmitter，对应的结果存储在返回值结构体中，传入大于0时，为超时时间，
        *                   必需大于每一帧采集时间 * imageCount。
        * @return int32_t 错误码
        * */
        CMTISDK_API int Cmti_WaitForAsyncResult(HDeviceClient hDevCli, void * pAsyncContext, int32_t timeout_ms);

        /**
        * @brief 设定指定引脚输出PWM频率信号
        * @param pin 输出PMW信号引脚编号，使用E_OSM_PIN_TYPE类型, 仅CP881支持OSM_PIN_PO3引脚
        * @param freq_Hz PWM频率，单位Hz
        * @return int32_t 错误码
        * */
        CMTISDK_API int32_t Cmti_SetPinOutputPwmFrequency(HDeviceClient hDevCli, int32_t pin, int32_t freq_Hz);

        /**
        * @brief 设置电源固定Force/Sense,仅CP881支持DVDD电源
        * @param nPowerId[] 电源ID，参考E_PowerId定义
        * @param nFix 固定Force标识，1标识固定Force，0表示固定Sense
        * @return int32_t 错误码
        * */
        CMTISDK_API int32_t Cmti_SetPowerFixForce(HDeviceClient hDevCli, const int32_t nPowerId[], const int32_t nFix[], int32_t nCount);

        /**
        * @brief 读取电源固定Force/Sense状态,仅CP881支持DVDD电源
        * @param nPowerId[] 电源ID，参考E_PowerId定义
        * @param nFix 固定Force标识，1标识固定Force，0表示固定Sense
        * @return int32_t 错误码
        * */
        CMTISDK_API int32_t Cmti_GetPowerFixForceState(HDeviceClient hDevCli, const int32_t nPowerId[], int32_t nFix[], int32_t nCount);


        /**
        * @brief 写I2C后获取DPS所有采样点电流
        *
        * @param i2cAddr I2C从地址
        * @param speedkHz I2C速率
        * @param mode I2C模式
        * @param regAddr[] 寄存器地址数组
        * @param regData[] 寄存器值数组
        * @param delay_ms[] 寄存器延时数组，以ms为单位
        * @param regNum 寄存器个数，即前三个数组的各自元素个数,要求：（1446 - 20） / 8
        * @param powerId 电源ID，参考E_PowerId定义
        * @param nCurrentRange 电流测量档位，参考E_DPSCurrentRange定义
        * @param current_nA 获取的电流值，返回格式：[Dvdd0,Dvdd1,...Dvddn,Avdd0,Avdd1,...Avddn]（其中 n = nSamplePoiint - 1）
        * @param nSampleInterval_us 采样间隔时间，延时过后，再次获取电流，单位(微秒)
        * @param nSamplePoint 同步操作中获取电流的采样次数，要求：nSamplePoint * 8 < 1446
        * @return int 错误码
        */
        CMTISDK_API int32_t Cmti_WriteICAndGetDpsAllSamplePointCurrent(HDeviceClient hDevCli, uint32_t i2cAddr, uint32_t speedkHz, uint32_t mode,const uint32_t regAddr[], const uint32_t regData[], 
            const uint32_t delay_ms[], int32_t regNum, int32_t powerId, int32_t currentRange, double current_nA[], int32_t nSampleInterval_us, int32_t nSamplePoint);

        /**
        * @brief 设置MIPI引脚漏电流校正状态
        * @param nState 进入校正模式标志，1表示进入校正状态，0表示退出校正状态
        * @return int32_t 错误码
        * */
        CMTISDK_API int32_t Cmti_SetMipiPinLeakCurrentCalibrateState(HDeviceClient hDevCli, int32_t nState);

        /**
        * @brief 设置MIPI引脚漏电流校正参数
        * @param nGroupIdx,校正参数组索引号，从0开始
        * @param nPositive，正向标志，1表示正向校正参数，0表示反向校正参数
        * @param nPinId 引脚编号，参考E_OSM_PIN_TYPE定义
        * @param nGain 设置的校正增益值，扩大1000倍
        * @param nOffset_nA 设置的校正偏差值，单位nA
        * @param nCount 引脚数量
        * @return int32_t 错误码
        * */
        CMTISDK_API int32_t Cmti_SetMipiPinLeakCurrentCalibration(HDeviceClient hDevCli, int32_t nGroupIdx, int32_t nPositive, const int32_t nPinId[],
            const int32_t nGain[], const int32_t nOffset_nA[], int32_t nCount);

        /**
        * @brief 获取MIPI引脚漏电流校正参数
        * @param nGroupIdx,校正参数组索引号，从0开始
        * @param nPositive，正向标志，1表示正向校正参数，0表示反向校正参数
        * @param nPinId 引脚编号，参考E_OSM_PIN_TYPE定义
        * @param nGain 读取的校正增益值，扩大1000倍
        * @param nOffset_nA 读取的校正偏差值，单位nA
        * @param nCount 引脚数量
        * @return int32_t 错误码
        * */
        CMTISDK_API int32_t Cmti_ReadMipiPinLeakCurrentCalibration(HDeviceClient hDevCli, int32_t nGroupIdx, int32_t nPositive, const int32_t nPinId[],
            int32_t nGain[], int32_t nOffset_nA[], int32_t nCount);

        /**
        * @brief 在Sensor输出图像过程中测量指定MIPI引脚的一帧内的LP时间
        * @param nMeasureTime_ms，测量采样时间，单位ms
        * @param nHighThreshold_mV，高电平阈值，单位mV，推荐配置800mV
        * @param nPins 输入的测量引脚编号，使用E_OSM_PIN_TYPE类型
        * @param nPeriod_us 测量得到的LP时间，单位us。
        * @param nCount 引脚数量，nMeasureTime * nCount < TimeOut(3000ms)
        * @return int 错误码
        * */
        CMTISDK_API int32_t Cmti_MeasureMipiLPPeriod(HDeviceClient hDevCli, int32_t nMeasureTime_ms, int32_t nHighThreshold_mV, const int32_t nPins[],int32_t nPeriod_us[], int32_t nCount);

#ifdef __cplusplus
    } /* end of namespace CmtiSdk */
} /* end of extern "C" */
#endif /* end of __cplusplus */

/******************************** ISP相关函数 ********************************/
#ifdef __cplusplus
extern "C" {
    namespace CmtiSdk {
#endif
        /**
        * @brief 将图像转为RGB24
        * @param pOutRgb24Image 输出参数。RGB24图像。
        * @param pInImage 输入待转换的图像。
        * @param frameParam 输入图像参数结构体。
        * @param algo 图像转换算法。
        * @return int 错误码
        */
        CMTISDK_API int ISP_DataToRgb24(uint8_t *pOutRgb24Image, const uint8_t *pInImage, const T_FrameParam& frameParam, int32_t algo);
        /**
        * @brief 将图像转为灰度图
        * @param pOutGray 输出参数。灰度图像。
        * @param pInImage 输入待转换的图像。
        * @param frameParam 输入图像参数结构体。
        * @param algo 图像转换算法，默认为0。
        * @return int 错误码
        */
        CMTISDK_API int ISP_DataToGray(uint8_t *pOutGrayImage, const uint8_t *pInImage, const T_FrameParam& frameParam, int32_t algo);

#ifdef __cplusplus
    } /* end of namespace CmtiSdk */
} /* end of extern "C" */
#endif /* end of __cplusplus */

#endif // CMTISDK_H
