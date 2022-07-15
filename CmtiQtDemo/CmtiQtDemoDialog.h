#pragma once

#include <QtWidgets/QDialog>
#include "ui_CmtiQtDemoDialog.h"
#include "CZTEKBoard.h"

class GrabThread;
class CmtiQtDemoDialog : public QDialog
{
    Q_OBJECT

public:
    CmtiQtDemoDialog(QWidget *parent = Q_NULLPTR);
    ~CmtiQtDemoDialog() override;

private:
    void btnEnum_clicked();
    void btnBind_clicked();
    void btnLoadSettingFile_clicked();
    void btnOpen_clicked();
    void btnClose_clicked();
    void pGrabThread_frameGrabbed(const uint8_t* pBuffer, int nBufferLen, uint64_t headTimestamp, uint64_t tailTimestamp, uint32_t frameSequence);

private:
    enum ELogLevel
    {
        LogLevel_Info,
        LogLevel_Warning,
        LogLevel_Error
    };
    void writeLog(const QString& text, ELogLevel logLevel = LogLevel_Info);

private:
    Ui::CmtiQtDemoDialog ui;
    CZTEKBoard* m_pCztekBoard;
    GrabThread* m_pGrabThread;
    bool m_bSensorSettingLoaded{ false };
};
