#ifndef __CMTIDEFS_H__
#define __CMTIDEFS_H__

#include "TypeDefs.h"

enum E_PowerId
{
    // old name style
    PI_DVDD = 0,
    PI_AVDD = 1,
    PI_DOVDD = 2,
    PI_AFVCC = 3,
    PI_VPP = 4,
    PI_AVDD2 = 5,
    PowerId_MaxPowerCount,
	
    // new name style
    Power_DVDD = 0,
    Power_AVDD = 1,
    Power_DOVDD = 2,
    Power_AFVCC = 3,
    Power_VPP = 4,
    Power_AVDD2 = 5,
    Power_VOIS = 6,
    Power_VAUX = 7,
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

enum E_RegBitsMode
{
    // old name
    RB_NORMAL = 0,            // 8Addr,8Data
    RB_ADDR8_DATA8 = 1,       // 8Addr,8Data
    RB_ADDR8_DATA16 = 2,      // 8Addr,16Data
    RB_ADDR16_DATA8 = 3,      // 16Addr,8Data
    RB_ADDR16_DATA16 = 4,     // 16Addr,16Data
    // new name
    RegMode_Normal = 0,          // 8Addr,8Data
    RegMode_ADDR8_DATA8 = 1,     // 8Addr,8Data
    RegMode_ADDR8_DATA16 = 2,    // 8Addr,16Data
    RegMode_ADDR16_DATA8 = 3,    // 16Addr,8Data
    RegMode_ADDR16_DATA16 = 4,   // 16Addr,16Data
    RegMode_ADDR0_DATA8 = 5,     // 0Addr,8Data
    RegMode_ADDR0_DATA16 = 6,    // 0Addr,16Data 
    RegMode_ADDR0_DATA32 = 7,    // 0Addr,32Data
    RegMode_ADDR8_DATA32 = 8,    // 8Addr,32Data
    RegMode_ADDR16_DATA32 = 9,   // 16Addr,32Data
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

// SPI
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
    ImgFmt_PackedRaw10 = 11, // packed
    ImgFmt_PackedRaw12 = 12, // packed
    ImgFmt_PackedRaw14 = 13, // packed
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

enum E_ErrorCode
{
    ERR_NoError = 0,
    ERR_Continue = 1,
    // General Error Code
    ERR_Failed = -1,           /* General error */
    ERR_NotImplemented = -2,   /* Not implemented function */
    ERR_InvalidParameter = -3, /* Invalid parameter */
    ERR_NoMemory = -4,
    ERR_FileNotFound = -5,
    ERR_LoadLibrary = -6,
    ERR_NotSupported = -7,
    ERR_Unkown = -9999,        /* Unkown error */
    // SDK Error Code(from -20 to -100)
    ERR_SetMclk = -20,
    ERR_GetMclk = -21,
    ERR_SetGpio = -22,
    ERR_GetGpio = -23,
    ERR_SetGpioDir = -24,
    ERR_GetGpioDir = -25,
    ERR_SetMipiParam = -26,
    ERR_GetMipiParam = -27,
    ERR_SetFramParam = -28,
    ERR_GetFramParam = -29,
    ERR_SetSensorPower = -30,
    ERR_GetSensorPower = -31,
    ERR_I2cWrite = -32,
    ERR_I2cRead = -33,
    ERR_I2cWriteBlock = -34,
    ERR_I2cReadBlock = -35,
    ERR_StartVideo = -36,
    ERR_StopVideo = -37,
    ERR_OpenDriver = -38,
    ERR_WriteControlChannelParam = -39, /* write control channel parameter fail */
    ERR_ReadControlChannelParam = -40,  /* read control channel parameter fail */
    ERR_WriteMessageChannelParam = -41, /* write message channel parameter fail */
    ERR_ReadMessageChannelParam = -42,  /* read message channel parameter fail */
    ERR_WriteStreamChannelParam = -43,  /* write stream channel parameter fail */
    ERR_ReadStreamChannelParam = -44,   /* read stream channel parameter fail */
    ERR_StartTransmit = -45,
    ERR_StopTransmit = -46,
    ERR_OpenShortTest = -47,
    ERR_ReadCurrent = -48,
    ERR_Overcurrent = -49,
    ERR_Beep = -50,
    ERR_VideoDequeuTimeout = -51,
    ERR_EmptyBufferPool = -52,
    ERR_DeviceOffline = -53,
    ERR_SetSpiParameter = -54,
    ERR_SpiTransfer = -55,
    ERR_ConnectFailed = -56,
    ERR_SocketIO = -57,          /* Socket error, socket write/read */
    ERR_InvalidProtPacket = -58, /* Invalid protocol packet */
    ERR_SetExtendPower = -59,
    ERR_VideoNoDevice = -60,
    ERR_VideoCritical = -61,
    ERR_SetVideoParam = -62,
    ERR_Hardware = -63,
    ERR_CreatAsyncThreadFailed = -64,
    ERR_AsyncStateError = -65,
    ERR_Firmware = -100,         /* Firmware error */
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


#endif // __CMTIDEFS_H__
