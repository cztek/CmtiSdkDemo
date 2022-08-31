#include "CmtiQtDemoDialog.h"
#include <QDateTime>
#include <QMessageBox>
#include <QFileDialog>
#include "GrabThread.h"

CmtiQtDemoDialog::CmtiQtDemoDialog(QWidget *parent)
    : QDialog(parent)
{
    ui.setupUi(this);
    ui.tableWidgetDevice->setAlternatingRowColors(true);

    m_pCztekBoard = new CZTEKBoard();
    m_pGrabThread = new GrabThread();
    connect(m_pGrabThread, &GrabThread::frameGrabbed, this, &CmtiQtDemoDialog::pGrabThread_frameGrabbed, Qt::QueuedConnection);

    connect(ui.btnEnum, &QPushButton::clicked, this, &CmtiQtDemoDialog::btnEnum_clicked);
    connect(ui.btnBind, &QPushButton::clicked, this, &CmtiQtDemoDialog::btnBind_clicked);

    connect(ui.btnLoadSettingFile, &QPushButton::clicked, this, &CmtiQtDemoDialog::btnLoadSettingFile_clicked);
    connect(ui.btnOpen, &QPushButton::clicked, this, &CmtiQtDemoDialog::btnOpen_clicked);
    connect(ui.btnClose, &QPushButton::clicked, this, &CmtiQtDemoDialog::btnClose_clicked);
    connect(ui.btnReadCurrentCaliOffset, &QPushButton::clicked, this, &CmtiQtDemoDialog::btnReadCurrentCaliOffset_clicked);
    connect(ui.btnTest, &QPushButton::clicked, this, &CmtiQtDemoDialog::btnTest_clicked);
}

CmtiQtDemoDialog::~CmtiQtDemoDialog()
{
    m_pGrabThread->StopRunning();
    delete m_pCztekBoard;
}

void CmtiQtDemoDialog::btnEnum_clicked()
{
    QList<QString> deviceNameList;
    m_pCztekBoard->EnumerateDevice(deviceNameList);

    ui.tableWidgetDevice->setRowCount(deviceNameList.count());
    for (uint32_t rowIdx = 0; rowIdx < deviceNameList.count(); rowIdx++)
    {
        ui.tableWidgetDevice->setItem(rowIdx, 0, new QTableWidgetItem(QString::number(rowIdx + 1)));
        ui.tableWidgetDevice->setItem(rowIdx, 1, new QTableWidgetItem(deviceNameList[rowIdx]));
    }
}

void CmtiQtDemoDialog::btnBind_clicked()
{
    // todo: 绑定动作，即绑定每一个socketIndex和设备名的关系。下面模拟按顺序编号绑定，实际使用时需要根据UI上的配置或实际情况绑定
    for (int rowIdx = 0; rowIdx < ui.tableWidgetDevice->rowCount(); rowIdx++)
    {
        int nSocketIndex = ui.tableWidgetDevice->item(rowIdx, 0)->data(Qt::EditRole).toInt();
        QString qsDevName = ui.tableWidgetDevice->item(rowIdx, 1)->data(Qt::EditRole).toString();
        writeLog(QString("Bind...(%1, %2)").arg(nSocketIndex).arg(qsDevName), LogLevel_Info);
        m_pCztekBoard->BindSocket(nSocketIndex, qsDevName);
    }
}

void CmtiQtDemoDialog::btnLoadSettingFile_clicked()
{
    static QString qsLastPath;
    QString qsFileName = QFileDialog::getOpenFileName(this, tr("Select sensor setting file..."), qsLastPath, tr("Sensor setting files(*.ini);;All files(*.*)"));
    if (qsFileName.isEmpty())
    {
        QMessageBox::critical(this, tr("Error"), tr("Invalid file!"));
        return;
    }
    if (m_pCztekBoard->LoadSensorSettings(qsFileName))
    {
        writeLog(QString("Load setting file %1 success!").arg(qsFileName), LogLevel_Info);
        m_bSensorSettingLoaded = true;
    }
    else
    {
        writeLog(QString("Load setting file %1 failed!").arg(qsFileName), LogLevel_Error);
    }
}

/**
 * @brief 出图函数，具体流程包括上电、设置各项参数、写I2C、打开视频传输
*/
void CmtiQtDemoDialog::btnOpen_clicked()
{
    if (!m_bSensorSettingLoaded)
    {
        QMessageBox::critical(this, tr("Error"), tr("Please load sensor setting file first!"));
        return;
    }
    if (ui.tableWidgetDevice->selectedItems().size() < 1)
    {
        writeLog("Select device first!", LogLevel_Error);
        return;
    }

    QList<QTableWidgetItem*> items = ui.tableWidgetDevice->selectedItems();
    if (items.size() == ui.tableWidgetDevice->columnCount()) // select a row
    {
        int nSocketIndex = items[0]->data(Qt::EditRole).toInt();
        QString qsDevName = items[1]->text();
        writeLog(QString("Bind...(%1, %2)").arg(nSocketIndex).arg(qsDevName), LogLevel_Info);
        m_pCztekBoard->BindSocket(nSocketIndex, qsDevName);

        writeLog(QString("Start video...(%1, %2)").arg(nSocketIndex).arg(qsDevName), LogLevel_Info);
        // 上电&开图
        if (!m_pCztekBoard->fnPowerOnCanyouPinVoltSet(nSocketIndex))
        {
            writeLog("Power on & video on failed!", LogLevel_Error);
            return;
        }
        m_pGrabThread->StartRunning(nSocketIndex);
    }
}

void CmtiQtDemoDialog::btnClose_clicked()
{
    QList<QTableWidgetItem*> items = ui.tableWidgetDevice->selectedItems();
    if (items.size() == ui.tableWidgetDevice->columnCount()) // select a row
    {
        int nSocketIndex = items[0]->data(Qt::EditRole).toInt();
        QString qsDevName = items[1]->text();
        writeLog(QString("Bind...(%1, %2)").arg(nSocketIndex).arg(qsDevName), LogLevel_Info);
        m_pCztekBoard->BindSocket(nSocketIndex, qsDevName);

        writeLog(QString("Stop video...(%1)").arg(nSocketIndex), LogLevel_Info);
        // 停止采集线程
        m_pGrabThread->StopRunning();
        // 关图&下电
        if (!m_pCztekBoard->fnCanyouPowerOff(nSocketIndex))
        {
            writeLog("Video off & power off failed!", LogLevel_Error);
            return;
        }
        writeLog(QString("Closed(%1).").arg(nSocketIndex), LogLevel_Info);
    }
}

void CmtiQtDemoDialog::btnReadCurrentCaliOffset_clicked()
{
    QList<QTableWidgetItem*> items = ui.tableWidgetDevice->selectedItems();
    if (items.size() == ui.tableWidgetDevice->columnCount()) // select a row
    {
        int nSocketIndex = items[0]->data(Qt::EditRole).toInt();
        QString qsDevName = items[1]->text();
        writeLog(QString("Bind...(%1, %2)").arg(nSocketIndex).arg(qsDevName), LogLevel_Info);
        m_pCztekBoard->BindSocket(nSocketIndex, qsDevName);

        writeLog(QString("Read Calibration I Offset...(%1)").arg(nSocketIndex), LogLevel_Info);
        QList<float> currentVal;
        if (m_pCztekBoard->fnDynamicOffset(nSocketIndex, currentVal))
        {
            int offset = 0;
            enum {
                CURRENT_BOARD_ID_83X = 0x8310,
                CURRENT_BOARD_ID_81X = 0x8110,
                CURRENT_BOARD_ID_ATE_811S = 0x8010,
                CURRENT_BOARD_ID_7DPS = 0x8320,
                CURRENT_BOARD_ID_500_TOF = 0x5210,
                CURRENT_BOARD_ID_CZCS821 = 0x8020,
            };
            T_BoardInfo boardInfo;
            m_pCztekBoard->fnGetBoardInfo(nSocketIndex, boardInfo);
            if ((CURRENT_BOARD_ID_ATE_811S == boardInfo.CurrentBoardId) || (CURRENT_BOARD_ID_CZCS821 == boardInfo.CurrentBoardId))
            {
                QVector<QString> vecPowerName;
                int32_t powerIdBit = boardInfo.PowerIdBit;
                if (powerIdBit & 1)
                {
                    vecPowerName.append("DVDD_5uA");
                    vecPowerName.append("DVDD_25uA");
                    vecPowerName.append("DVDD_250uA");
                    vecPowerName.append("DVDD_2.5mA");
                    vecPowerName.append("DVDD_25mA");
                    vecPowerName.append("DVDD_500mA");
                    vecPowerName.append("DVDD_1.2A");
                }
                if (powerIdBit & 2)
                {
                    vecPowerName.append("AVDD_5uA");
                    vecPowerName.append("AVDD_25uA");
                    vecPowerName.append("AVDD_250uA");
                    vecPowerName.append("AVDD_2.5mA");
                    vecPowerName.append("AVDD_25mA");
                    vecPowerName.append("AVDD_500mA");
                    vecPowerName.append("AVDD_1.2A");
                }
                if (powerIdBit & 4)
                {
                    vecPowerName.append("DOVDD_5uA");
                    vecPowerName.append("DOVDD_20uA");
                    vecPowerName.append("DOVDD_200uA");
                    vecPowerName.append("DOVDD_2mA");
                    vecPowerName.append("DOVDD_80mA");
                    vecPowerName.append("DOVDD_NC1");
                    vecPowerName.append("DOVDD_NC2");
                }
                if (powerIdBit & 8)
                {
                    vecPowerName.append("AFVCC_5uA");
                    vecPowerName.append("AFVCC_25uA");
                    vecPowerName.append("AFVCC_250uA");
                    vecPowerName.append("AFVCC_2.5mA");
                    vecPowerName.append("AFVCC_25mA");
                    vecPowerName.append("AFVCC_500mA");
                    vecPowerName.append("AFVCC_1.2A");
                }
                if (powerIdBit & 16)
                {
                    vecPowerName.append("VPP_5uA");
                    vecPowerName.append("VPP_25uA");
                    vecPowerName.append("VPP_250uA");
                    vecPowerName.append("VPP_2.5mA");
                    vecPowerName.append("VPP_25mA");
                    vecPowerName.append("VPP_500mA");
                    vecPowerName.append("VPP_1.2A");
                }
                if (powerIdBit & 32)
                {
                    vecPowerName.append("AVDD2_5uA");
                    vecPowerName.append("AVDD2_25uA");
                    vecPowerName.append("AVDD2_250uA");
                    vecPowerName.append("AVDD2_2.5mA");
                    vecPowerName.append("AVDD2_25mA");
                    vecPowerName.append("AVDD2_500mA");
                    vecPowerName.append("AVDD2_1.2A");
                }
                if (powerIdBit & 64)
                {
                    vecPowerName.append("VOIS_5uA");
                    vecPowerName.append("VOIS_20uA");
                    vecPowerName.append("VOIS_200uA");
                    vecPowerName.append("VOIS_2mA");
                    vecPowerName.append("VOIS_80mA");
                    vecPowerName.append("VOIS_NC1");
                    vecPowerName.append("VOIS_NC2");
                }
                if (powerIdBit & 128)
                {
                    vecPowerName.append("VAUX_5uA");
                    vecPowerName.append("VAUX_20uA");
                    vecPowerName.append("VAUX_200uA");
                    vecPowerName.append("VAUX_2mA");
                    vecPowerName.append("VAUX_80mA");
                    vecPowerName.append("VAUX_NC1");
                    vecPowerName.append("VAUX_NC2");
                }

                for (int i = 0; i < vecPowerName.size(); i++)
                {
                    writeLog(QString("%1: %2 nA").arg(vecPowerName[i]).arg(currentVal[i]), LogLevel_Info);
                }
            }
            else
            {
                QString names[] =
                {
                    "DVDD_mA", "DOVDD_mA", "AVDD_mA", "AFVCC_mA", "VPP_mA",
                    "DVDD_uA", "DOVDD_uA", "AVDD_uA", "AFVCC_uA", "VPP_uA",
                    "DVDD_nA", "DOVDD_nA", "AVDD_nA", "AFVCC_nA", "VPP_nA",
                    "AVDD2_mA", "AVDD2_uA", "AVDD2_nA", "VOIS", "VOIS_uA", "VOIS_nA",
                    "VAUX", "VAUX_uA", "VAUX_nA"
                }; // accroding to sequence of driver
                for (uint32_t i = 0; i < (boardInfo.PowerDomainsNum + 1) * 3; i++)
                {
                    writeLog(QString("%1: %2 nA").arg(names[i]).arg(currentVal[i]), LogLevel_Info);
                }
            }
        }
    }
}

void CmtiQtDemoDialog::btnTest_clicked()
{
    QList<QTableWidgetItem*> items = ui.tableWidgetDevice->selectedItems();
    if (items.size() == ui.tableWidgetDevice->columnCount()) // select a row
    {
        int nSocketIndex = items[0]->data(Qt::EditRole).toInt();
        QString qsDevName = items[1]->text();
        writeLog(QString("Bind...(%1, %2)").arg(nSocketIndex).arg(qsDevName), LogLevel_Info);
        m_pCztekBoard->BindSocket(nSocketIndex, qsDevName);
        uint8_t pData[2];
        bool bFlag = m_pCztekBoard->ReadI2CData(nSocketIndex, 0x20, 0x0016, 2, pData, true); // 尝试读imx214的flag寄存器
        if (bFlag)
        {
            writeLog(QString("RegAddr: 0x0016, RegData(2B): 0x%2 0x%3").arg(pData[0], 2, 16, QChar('0')).arg(pData[1], 2, 16, QChar('0')));
        }
    }
}

void CmtiQtDemoDialog::pGrabThread_frameGrabbed(const uint8_t* pBuffer, int nBufferLen, uint64_t headTimestamp, uint64_t tailTimestamp, uint32_t frameSequence)
{
    // 正式逻辑可以在此将数据显示出来或做其他处理
    writeLog(QString("SEQ: %1, HT: %2, TT: %3").arg(frameSequence).arg(headTimestamp).arg(tailTimestamp));
}

void CmtiQtDemoDialog::writeLog(const QString& text, ELogLevel logLevel)
{
    QTextCharFormat fmt;
    if (LogLevel_Error == logLevel)
        fmt.setForeground(QBrush(Qt::red));
    else if (LogLevel_Warning == logLevel)
        fmt.setForeground(QBrush(Qt::darkYellow));
    else
        fmt.setForeground(QBrush(Qt::black));
    QTextCursor cursor = ui.textEditLogViewer->textCursor();
    cursor.mergeCharFormat(fmt);
    ui.textEditLogViewer->mergeCurrentCharFormat(fmt);

    QString text2 = QString("[%1] %2").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz")).arg(text);
    ui.textEditLogViewer->append(text2);
}
