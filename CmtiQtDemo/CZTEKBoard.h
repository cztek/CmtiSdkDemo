#ifndef __CZTEKBOARD_H__
#define __CZTEKBOARD_H__

#include <QList>

struct TSensorSetting;
class CZTEKBoard
{
public:
    CZTEKBoard();
    virtual ~CZTEKBoard();

    bool LoadSensorSettings(const QString& qsSettingFileName);

    /**
     * @brief 枚举设备，枚举后执行绑定操作
     * @param deviceNameList 
    */
    void EnumerateDevice(QList<QString>& deviceNameList);

    /**
     * @brief 绑定Socket，将SocketID和设备名建立绑定关系，后续操作可以直接使用SocketID来操作
     * @param nSocIndex Socket ID
     * @param qsDevName 枚举到的设备名
    */
    void BindSocket(int nSocIndex, const QString& qsDevName);

    /**
    * @brief 盒子初始化函数
    *
    * @param nSocIndex[] 设备ID号码，提前通过device 映射
    * @param nDLLV[] DLL版本号
    * @return int 错误码
    */
    bool fnDeviceInitialize(int nSocIndex, int& nDLLV);


    /**
    * @brief 模组供电函数,使用此函数，模组供电完成
    *
    * @param nSocIndex[] 设备ID号码，提前通过device 映射
    * @return int 错误码
    */
    bool fnPowerOnCanyouPinVoltSet(int nSocIndex);


    /**
    * @brief 模组断电函数
    *
    * @param nSocIndex[] 设备ID号码，提前通过device 映射
    * @return int 错误码
    */
    bool fnCanyouPowerOff(int nSocIndex);


    /**
    * @brief 将模组 mipi 输出的10bit 图片，转换为16bit 大端
    *
    * @param pImage[] 10bit原始图数据输入
    * @param pDest[] 转换后结果
    * @param nWidth 图片宽度
    * @param nHeight 图片高度
    * @return int 错误码
    */
    void Make10BitModeLSB(unsigned char* pImage, uint16_t* pDest, unsigned int nWidth, unsigned int nHeight);

    /**
    * @brief 将模组 mipi 输出的10bit 图片，转换为16bit 小端
    *
    * @param pImage[] 10bit原始图数据输入
    * @param pDest[] 转换后结果
    * @param nWidth 图片宽度
    * @param nHeight 图片高度
    * @return int 错误码
    */
    void Make10BitModeMSB(unsigned char* pImage, uint16_t* pDest, unsigned int nWidth, unsigned int nHeight);


    /**
    * @brief DUT拍照函数，一次获取一张图片
    *
    * @param nSocIndex[] 设备ID号码，提前通过device 映射
    * @param currentRange[] 电流量程，参考E_CurrentRange定义
    * @param current_nA[] 电流值，单位为nA
    * @param count 电源个数
    * @return int 错误码
    */
    bool fnGetOneFrame(int nSocIndex, QString station, QString barcode, std::string FilePath, std::string fileName);

    /**
    * @brief 模组8bit 地址读取函数
    *
    * @param nSocIndex[] 设备ID号码，提前通过device 映射
    * @param nSlave[] 模组偏移地址
    * @param nAddr[] 模组地址
    * @param nDataLen 数据长度
    * @param pBuf 待写入数据
    * @return int 错误码
    */
    bool ReadI2CData(int nSocIndex, ushort nSlave, ushort nAddr, ushort nDataLen, uint8_t* pBuf, bool bAddr16 = false);


    /**
    * @brief 模组8bit 地址写入函数
    *
    * @param nSocIndex[] 设备ID号码，提前通过device 映射
    * @param nSlave[] 模组偏移地址
    * @param nAddr[] 模组地址
    * @param nDataLen 数据长度
    * @param pBuf 待写入数据
    * @return int 错误码
    */
    bool WriteI2CData(int nSocIndex, ushort nSlave, ushort nAddr, ushort nDataLen, const uint8_t* pBuf, bool bAddr16 = false);

    /**
    * @brief OS 读取函数
    *
    * @param nSocIndex[] 设备ID号码，提前通过device 映射
    * @param reading[] OS测试结果
    * @return int 错误码
    */
    bool fnOSTest(int nSocIndex, QList<float>& reading);


    /**
    * @brief 动态电流测试函数
    *
    * @param nSocIndex 设备ID号码，提前通过device 映射
    * @param reading[] 动态电流测试结果
    * @return int 错误码
    */
    bool fnDynamicReading(int nSocIndex, QList<float>& reading);


    /**
    * @brief 动态电流offset计算函数
    *
    * @param nSocIndex[] 设备ID号码，提前通过device 映射
    * @param reading[] offset读取结果
    * @return int 错误码
    */
    bool fnDynamicOffset(int nSocIndex, QList<float>& reading);


    /**
    * @brief 静态电流测试函数
    *
    * @param nSocIndex[] 设备ID号码，提前通过device 映射
    * @param reading[] 静态电流测量结果
    * @return int 错误码
    */
    bool fnStandbyReading(int nSocIndex, QList<float>& reading);


    /**
    * @brief 静态电流offset计算函数
    *
    * @param nSocIndex[] 设备ID号码，提前通过device 映射
    * @param reading[] offset 读取结果
    * @return int 错误码
    */
    bool fnStandbyOffset(int nSocIndex, QList<float>& reading);

private:
     TSensorSetting* m_pSensorSetting;
     bool m_bIsPowerOn{ false };
     bool m_bIsVideoOn{ false };
};

#endif /* __CZTEKBOARD_H__ */

