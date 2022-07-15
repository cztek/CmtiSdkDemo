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
