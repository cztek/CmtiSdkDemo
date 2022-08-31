#ifndef __CMTIDEFS_H__
#define __CMTIDEFS_H__

#include "TypeDefs.h"

/**
 * @ingroup Sensor控制接口
 * @brief 电源Pin的ID
*/
enum E_PowerId
{
    // old name style
    PI_DVDD = 0,    /**< Deprecated */
    PI_AVDD = 1,    /**< Deprecated */
    PI_DOVDD = 2,   /**< Deprecated */
    PI_AFVCC = 3,   /**< Deprecated */
    PI_VPP = 4,     /**< Deprecated */
    PI_AVDD2 = 5,   /**< Deprecated */
    PowerId_MaxPowerCount,
	
    // new name style
    Power_DVDD = 0,  /**< DVDD */
    Power_AVDD = 1,  /**< AVDD */
    Power_DOVDD = 2, /**< DOVDD */
    Power_AFVCC = 3, /**< AFVCC */
    Power_VPP = 4,   /**< VPP */
    Power_AVDD2 = 5, /**< AVDD2 */
    Power_VOIS = 6,  /**< VOIS */
    Power_VAUX = 7,  /**< VAUX */
    Power_MaxPowerCount,
};

struct T_Power
{
    int Id;  // E_PowerId
    int Value;
    int Delay_ms;
    T_Power() {
        Id = Value = Delay_ms = 0;
    }
    T_Power(int id) {
        Id = id;
        Value = Delay_ms = 0;
    }
};

enum E_SensorFeedBackPowerId
{
    FB_VPIX = 0,
    FB_VREF,
    FB_TXLOW,
    FB_BACK,
	FB_TXLOW2,
};

#define GPIO_DIR_INPUT      1
#define GPIO_DIR_OUTPUT     0

enum E_Gpio
{
    IO_Reset = 1 << 0,
    IO_Pwdn1 = 1 << 1,
    IO_Pwdn2 = 1 << 2,
    IO_PO1 = 1 << 3,
    IO_PO2 = 1 << 4,
    IO_PO3 = 1 << 5,
    IO_PO4 = 1 << 6,
    IO_MCLK = 1 << 7,
};

enum E_ExtGpio
{
    ExtGpio1 = 1 << 0,
    ExtGpio2 = 1 << 1,
    ExtGpio3 = 1 << 2,
};

enum E_CsiExtGpio
{
    CSI_ExtGpio1 = 0xA1,
    CSI_ExtGpio2 = 0xA2,
    CSI_ExtGpio3 = 0xA3,
    CSI_ExtGpio4 = 0xA4,
};

enum E_InterfaceType
{
    IT_MIPI = 0,
    IT_DVP,
    IT_MTK,
    IT_SPI,
    IT_HIPI,
    IT_SPREADTRUM,
    IT_TV,
    IT_UVC,
    IT_LVDS,
    IT_MIPI_CPHY,
    IT_MAX_VAL = IT_MIPI_CPHY,
    IT_MIPI_DVP = (IT_DVP << 8) | IT_MIPI, // NOTICE: non-db value, only for ui
};

#define ESC_MODE 0xBADA0001 // Escape character for IIC mode
#define ESC_ADDR 0xBADA0002 // Escape character for IIC address

#define I2C_READ_NO_STOP    0x80

/**
 * @ingroup Sensor控制接口
 * @brief 寄存器读写模式
*/
enum E_RegBitsMode
{
    // old name
    RB_NORMAL = 0,            /**< Deprecated */
    RB_ADDR8_DATA8 = 1,       /**< Deprecated */
    RB_ADDR8_DATA16 = 2,      /**< Deprecated */
    RB_ADDR16_DATA8 = 3,      /**< Deprecated */
    RB_ADDR16_DATA16 = 4,     /**< Deprecated */
    // new name

    RegMode_Normal = 0,          /**<  8位地址， 8位值 */
    RegMode_ADDR8_DATA8 = 1,     /**<  8位地址， 8位值 */
    RegMode_ADDR8_DATA16 = 2,    /**<  8位地址，16位值 */
    RegMode_ADDR16_DATA8 = 3,    /**< 16位地址， 8位值 */
    RegMode_ADDR16_DATA16 = 4,   /**< 16位地址，16位值 */
    RegMode_ADDR0_DATA8 = 5,     /**<  0位地址， 8位值 */
    RegMode_ADDR0_DATA16 = 6,    /**<  0位地址，16位值  */
    RegMode_ADDR0_DATA32 = 7,    /**<  0位地址，32位值 */
    RegMode_ADDR8_DATA32 = 8,    /**<  8位地址，32位值 */
    RegMode_ADDR16_DATA32 = 9,   /**< 16位地址，32位值 */
};

struct T_I2CCommParam
{
    E_RegBitsMode RegBitsMode;
    uint Addr;
    uint Speed; // unit:100kHz, 0 stands for max(400k)
};

struct T_RegConf
{
    uint Addr;
    uint Value;
    uint Delay_ms;
    uint Mask;
    T_RegConf() {
        Addr = 0;
        Value = 0;
        Delay_ms = 0;
        Mask = 0xff;
    }
    T_RegConf(uint addr, uint value) {
        Addr = addr;
        Value = value;
        Delay_ms = 0;
        Mask = 0xff;
    }
    T_RegConf(uint addr, uint value, uint delayMs) {
        Addr = addr;
        Value = value;
        Delay_ms = delayMs;
        Mask = 0xff;
    }
    T_RegConf(uint addr, uint value, uint delayMs, uint mask) {
        Addr = addr;
        Value = value;
        Delay_ms = delayMs;
        Mask = mask;
    }
};

/**
 * @brief SPI
*/
enum E_SpiId
{
    E_Spi_Id_Camera = 0x00,
    E_Spi_Id_Extend1 = 0x100,
};

enum E_SpiMode
{
    SPI_CPHA = 0x01,
    SPI_CPOL = 0x02,
    SPI_MODE_0 = 0,
    SPI_MODE_1 = (0 | SPI_CPHA),
    SPI_MODE_2 = (SPI_CPOL | 0),
    SPI_MODE_3 = (SPI_CPOL | SPI_CPHA),

    SPI_CS_HIGH = 0x04,
    SPI_LSB_FIRST = 0x08,
};

enum E_PoResEndPoint
{
    PoResEndPoint_PO1ReferencePO4 = 0,
    PoResEndPoint_PO2ReferencePO4,
    PoResEndPoint_PO3ReferencePO4
};

enum E_ImageFormat
{
    // old name style	
    IMAGE_FMT_INVALID = -1,
    IMAGE_FMT_RAW8 = 0,
    IMAGE_FMT_RAW10 = 1, // unpacked, 2 bytes
    IMAGE_FMT_RAW12 = 2, // unpacked, 2 bytes
    IMAGE_FMT_RAW14 = 3, // unpacked, 2 bytes
    IMAGE_FMT_RAW16 = 4,
    IMAGE_FMT_RGB16 = 5,
    IMAGE_FMT_RGB24 = 6,
    IMAGE_FMT_RGB32 = 7,
    IMAGE_FMT_YUV422 = 8,
    IMAGE_FMT_YUV420 = 9,
    IMAGE_FMT_LUMINANCE = 10,
    IMAGE_FMT_PackedRaw10 = 11, // packed
    IMAGE_FMT_PackedRaw12 = 12, // packed
    IMAGE_FMT_PackedRaw14 = 13, // packed
    // new name style
    ImgFmt_Invalid = -1,
    ImgFmt_RAW8 = 0,
    ImgFmt_RAW10 = 1, // unpacked, 2 bytes
    ImgFmt_RAW12 = 2, // unpacked, 2 bytes
    ImgFmt_RAW14 = 3, // unpacked, 2 bytes
    ImgFmt_RAW16 = 4,
    ImgFmt_RGB16 = 5,
    ImgFmt_RGB24 = 6,
    ImgFmt_RGB32 = 7,
    ImgFmt_YUV422 = 8,
    ImgFmt_YUV420 = 9,
    ImgFmt_LUMINANCE = 10,
    ImgFmt_PackedRaw10 = 11,  // packed(obsoleted)
    ImgFmt_PackedRaw12 = 12,  // packed
    ImgFmt_PackedRaw14 = 13,  // packed
    ImgFmt_Raw10Std4P5B = 14, // MIPI standard 4 pixel 5 bytes
};

enum E_ImageMode
{
    // old name style
    IMAGE_MODE_INVALID = -1,
    IMAGE_MODE_YCbYCr_RG_GB = 0,
    IMAGE_MODE_YCrYCb_GR_BG = 1,
    IMAGE_MODE_CbYCrY_GB_RG = 2,
    IMAGE_MODE_CrYCbY_BG_GR = 3,
    // new name style
    ImgMode_Invalid = -1,
    ImgMode_YCbYCr_RG_GB = 0,
    ImgMode_YCrYCb_GR_BG = 1,
    ImgMode_CbYCrY_GB_RG = 2,
    ImgMode_CrYCbY_BG_GR = 3,
};

struct T_FrameParam
{
    E_ImageFormat ImageFormat;
    E_ImageMode   ImageMode;
    uint Width;
    uint Height;
    uint Size;      // bytes
    T_FrameParam() {
        Reset();
    }
    T_FrameParam(E_ImageFormat imgFormat, E_ImageMode imgMode, int width, int height) {
        ImageFormat = imgFormat;
        ImageMode = imgMode;
        Width = width;
        Height = height;
        Size = 0;
    }
    T_FrameParam(E_ImageFormat imgFormat, E_ImageMode imgMode, int width, int height, uint size) {
        ImageFormat = imgFormat;
        ImageMode = imgMode;
        Width = width;
        Height = height;
        Size = size;
    }
    void Reset() {
        ImageFormat = ImgFmt_Invalid;
        ImageMode = ImgMode_Invalid;
        Width = Height = Size = 0;
    }
};

enum E_OSM_PIN_TYPE
{
    OSM_PIN_INVALID = -1,
    OSM_PIN_AVDD = 0,
    OSM_PIN_DOVDD,
    OSM_PIN_DVDD,
    OSM_PIN_AFVCC,
    OSM_PIN_VPP,
    OSM_PIN_MCLK,
    OSM_PIN_SCL,
    OSM_PIN_SDA,
    OSM_PIN_PWDN,
    OSM_PIN_RST,
    OSM_PIN_PO1,
    OSM_PIN_PO2,
    OSM_PIN_PO3,
    OSM_PIN_PO4,
    OSM_PIN_MIPI_D3P,
    OSM_PIN_MIPI_D3N,
    OSM_PIN_MIPI_D2P,
    OSM_PIN_MIPI_D2N,
    OSM_PIN_MIPI_D1P,
    OSM_PIN_MIPI_D1N,
    OSM_PIN_MIPI_D0P,
    OSM_PIN_MIPI_D0N,
    OSM_PIN_MIPI_CLKP,
    OSM_PIN_MIPI_CLKN,
    OSM_PIN_SGND1,
    OSM_PIN_SGND2,
    OSM_PIN_SGND3,
    OSM_PIN_SGND4,

    OSM_PIN_C_PHY_TRIO_0A = 0x100,
    OSM_PIN_C_PHY_TRIO_0B,
    OSM_PIN_C_PHY_TRIO_0C,
    OSM_PIN_C_PHY_TRIO_1A,
    OSM_PIN_C_PHY_TRIO_1B,
    OSM_PIN_C_PHY_TRIO_1C,
    OSM_PIN_C_PHY_TRIO_2A,
    OSM_PIN_C_PHY_TRIO_2B,
    OSM_PIN_C_PHY_TRIO_2C,
    OSM_PIN_AVDD2,
    OSM_PIN_SPI_CLK,
    OSM_PIN_SPI_CS,
    OSM_PIN_SPI_MOSI,
    OSM_PIN_SPI_MISO,
    OSM_PIN_VOIS,
    OSM_PIN_VAUX,
    OSM_PIN_PO5,
    OSM_PIN_PO6,
    OSM_PIN_PO7,
    OSM_PIN_PO8,
    OSM_PIN_PO9,
    OSM_PIN_PO10,
    OSM_PIN_PO11,
    OSM_PIN_PO12,
    OSM_PIN_PO13,
    OSM_PIN_PO14,
    OSM_PIN_PO15,
    OSM_PIN_PO16,
    OSM_PIN_PO17,
    OSM_PIN_PO18,
    OSM_PIN_PO19,
};
#define OS_TEST_SHORT_PIN_MASK  0x3FF

enum E_OsTest_Result
{
    OS_Result_Pass = 0,
    OS_Result_Open = (1 << 15),
    OS_Result_Short = (1 << 14),
};

enum E_CurrentRange
{
    CurrentRange_mA = 0,
    CurrentRange_uA,
    CurrentRange_nA
};

#define CURRENT_RESULT_UPOVERFLOW   (-1.0)

enum E_MipiStatus
{
    MIPI_WordCount = 0,
    MIPI_DataType,
    MIPI_CPHY_PacketHeader_CRC_Error,
    MIPI_DPHY_PacketHeader_ECC_SingleBitError,
    MIPI_DPHY_PacketHeader_ECC_MultiBitError,
    MIPI_PacketFooter_CRC_Error,
    MIPI_VirtualChannel,
};

enum E_PmicSwitchMode {
    PmicSwitchMode_PWM = 1,
    PmicSwitchMode_APS = 2,
};

struct T_BoardInfo
{
    uint32_t Version;       // struct version
    uint32_t MainBoardId;
    uint32_t CurrentBoardId;
    uint32_t PowerDomainsNum; // PowerDomainsNum表示最大电源编号，如统计电源数量时需要加1
    uint32_t CameraSwap;
    uint32_t MainboardHardWareVer;
    uint32_t CurrentBoardHardWareVer;
    char DtbModel[32];

    // 从PlatformId成员开始要保证占用32个int型数据保证向下兼容,并且新增加的成员有效值不为0
    uint32_t PlatformId;                    // 该字段在应用层填充使用E_PlatFormId
    uint32_t CurrentConvertMode;            // V2 版本添加 使用E_CurrentConvertMode
    uint32_t SupportAutoSelectCurrentRange; // V2 版本添加 板子是否支持根据输入电流上限自动选择量程
    uint32_t SupportNegativeOsTest;         // V2 版本添加 板子是否支持反向测试OS
    uint32_t SupportMeasureSensorPower;     // V2 版本添加 板子是否支持测量sensor供电电压
    uint32_t SupportSetFrameInterVal;       // 驱动是否支持设置帧间隔
    uint32_t Feature;                       // 硬件支持的特性，为了节省空间按bit设置
    uint32_t PowerIdBit;                    // 硬件支持的电源Id bit位
    uint32_t Reserved[24];                  // filled with 0
};

/**
 * @ingroup 设备控制接口
 * @brief 错误码
*/
enum E_ErrorCode
{
    ERR_NoError = 0,            /**< 操作成功 */
    ERR_Continue = 1,           /**< 操作继续 */
    // General Error Code
    ERR_Failed = -1,            /**< 通用错误 */
    ERR_NotImplemented = -2,    /**< 功能未实现 */
    ERR_InvalidParameter = -3,  /**< 参数非法 */
    ERR_NoMemory = -4,          /**< 内存不足 */
    ERR_FileNotFound = -5,      /**< 文件不存在 */
    ERR_LoadLibrary = -6,       /**< 加载动态库失败 */
    ERR_NotSupported = -7,      /**< 功能不支持 */
    ERR_Unkown = -9999,         /**< 未知错误 */
    // SDK Error Code(from -20 to -100)
    ERR_SetMclk = -20,          /**< 设置时钟失败 */
    ERR_GetMclk = -21,          /**< 获取时钟失败 */
    ERR_SetGpio = -22,          /**< 设置GPIO电平失败 */
    ERR_GetGpio = -23,          /**< 读取GPIO电平失败 */
    ERR_SetGpioDir = -24,       /**< 设置GPIO方向失败 */
    ERR_GetGpioDir = -25,       /**< 读取GPIO方向失败 */
    ERR_SetMipiParam = -26,     /**< 设置MIPI参数失败 */
    ERR_GetMipiParam = -27,     /**< 读取MIPI参数失败 */
    ERR_SetFramParam = -28,     /**< 设置图像参数失败 */
    ERR_GetFramParam = -29,     /**< 读取图像参数失败 */
    ERR_SetSensorPower = -30,   /**< 设置引脚电源参数失败 */
    ERR_GetSensorPower = -31,   /**< 读取引脚电源参数失败 */
    ERR_I2cWrite = -32,         /**< I2C写失败 */
    ERR_I2cRead = -33,          /**< I2C读失败 */
    ERR_I2cWriteBlock = -34,    /**< 连续I2C写失败 */
    ERR_I2cReadBlock = -35,     /**< 连续I2C读失败 */
    ERR_StartVideo = -36,       /**< 开图失败 */
    ERR_StopVideo = -37,        /**< 关图失败 */
    ERR_OpenDriver = -38,       /**< 性能驱动失败 */
    ERR_WriteControlChannelParam = -39, /**< 设置控制接口参数失败 */ //no use
    ERR_ReadControlChannelParam = -40,  /**< 读取控制接口参数失败 */  //no use
    ERR_WriteMessageChannelParam = -41, /**< 设置通信接口参数失败 */
    ERR_ReadMessageChannelParam = -42,  /**< 读取通信接口参数失败 */  //no use
    ERR_WriteStreamChannelParam = -43,  /**< 设置传输接口参数失败 */
    ERR_ReadStreamChannelParam = -44,   /**< 读取传输接口参数失败 */
    ERR_StartTransmit = -45,            /**< 开启传输失败 */
    ERR_StopTransmit = -46,             /**< 停止传输失败 */
    ERR_OpenShortTest = -47,            /**< 开短路测试失败 */
    ERR_ReadCurrent = -48,              /**< 读电流失败 */
    ERR_Overcurrent = -49,              /**< 过流 */
    ERR_Beep = -50,                     /**< 设置蜂鸣失败 */
    ERR_VideoDequeuTimeout = -51,       /**< 视频帧获取超时 */
    ERR_EmptyBufferPool = -52,          /**< 视频帧为空 */
    ERR_DeviceOffline = -53,            /**< 设备离线 */
    ERR_SetSpiParameter = -54,          /**< SPI参数失败 */
    ERR_SpiTransfer = -55,              /**< SPI传输失败 */
    ERR_ConnectFailed = -56,            /**< 连接错误 */
    ERR_SocketIO = -57,                 /**< 通信套件字错误 */
    ERR_InvalidProtPacket = -58,        /**< 无效协议包 */
    ERR_SetExtendPower = -59,           /**< 设置拓展引脚电源失败 */ //no use
    ERR_VideoNoDevice = -60,            /**< 开图时无设备 *///no use
    ERR_VideoCritical = -61,            /**< 开图无效 *///no use
    ERR_SetVideoParam = -62,            /**< 设置开图参数错误 */ //no use
    ERR_Hardware = -63,                 /**< 硬件错误 */ //no use
    ERR_CreatAsyncThreadFailed = -64,   /**< 创建异步线程失败 */ //no use
    ERR_AsyncStateError = -65,          /**< 设置异步状态异常 */  //no use
    ERR_Firmware = -100,                /**< 固件错误 */
};


enum E_VolSampleTarget /* 仅供SDK内部使用 */
{
    VolSample_Sensor_FeedBack = 0,
    VolSample_MIPI,
};


enum E_SampleValueType
{
    SampleValue_Average = 0,
    SampleValue_Min,
    SampleValue_Max,
};

enum E_MipiReceiverMode
{
    MipiReceiverMode0 = 0,
    MipiReceiverMode1,
};

enum E_DCMeasureTarget {
    MeasureTarget_Sensor_FeedBack = (1 << 0),
    MeasureTarget_MIPI = (1 << 1),
    MeasureTarget_SensorIO = (1 << 2), /* PWDN,RESET,MCLK,SDA,SCL pin */
    MeasureTarget_PO = (1 << 3),
    MeasureTarget_SensorPower = (1 << 4),
};

enum E_PMUCurrentRange {
    PMU_CurrentRange_5uA = 0,
    PMU_CurrentRange_20uA,
    PMU_CurrentRange_200uA,
    PMU_CurrentRange_2mA,
    PMU_CurrentRange_80mA,
};

enum E_DC_CalibrateMode {
    DC_CalibrateMode_FI = 0,
    DC_CalibrateMode_FV,
    DC_CalibrateMode_MI,
    DC_CalibrateMode_MV,
};

enum E_DPSCurrentRange {
    DPS_CurrentRange_5uA = 0,
    DPS_CurrentRange_25uA,
    DPS_CurrentRange_250uA,
    DPS_CurrentRange_2500uA,
    DPS_CurrentRange_25mA,
    DPS_CurrentRange_500mA,
    DPS_CurrentRange_1200mA,
};

enum E_TransmitterMode
{
    TransmitterMode_ContinueTransmitter = 0,    /* 持续传输模式，从Video On持续不间断的传输图像到SDK */
    TransmitterMode_RequestTransfer             /* 用户请求传输模式，配合Cmti_GrabContinuousFrames调用，用户不需要图像时，不会有图像传输 */
};

enum E_AsyncState
{
    AsyncState_Idle = 0,
    AsyncState_Running,
    AsyncState_WaitTransmitter,
    AsyncState_Finished,
    AsyncState_Error
};

struct T_CalcAverageImageAsyncContext
{
    uint32_t m_u32Length;   /* 输入参数，整个结构体内存大小 */
    uint32_t m_u32Version;  /* 输入参数，结构体版本，兼容性设计 */
    int32_t m_nAsyncID;     /* SDK内部维护，标识异步操作ID */
    int32_t m_nAsyncState;  /* 输出函数，用于标识当前异步操作状态，见E_AsyncState的定义 */
    struct
    {
        uint32_t m_u32ImageSize;    /* 输入参数，计算得到的均值图像大小 */
        uint8_t* m_pAvgImageBuf;    /* 输入输出参数，计算得到的均值图像缓冲区，调用者管理空间 */
        uint64_t m_nHeadTimestamp;  /* 帧头时间戳 */
        uint64_t m_nTailTimestamp;  /* 帧尾时间戳 */
        uint32_t m_nFrameSequence;  /* 帧序列号 */
    }m_tAverageImageFrame;
};

enum E_VirtualChannelFlag {
    VC_0 = (1 << 0),
    VC_1 = (1 << 1),
    VC_2 = (1 << 2),
    VC_3 = (1 << 3),
};

#endif // __CMTIDEFS_H__
