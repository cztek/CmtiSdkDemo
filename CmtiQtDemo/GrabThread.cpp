#include "GrabThread.h"
#include "CmtiSdkWrapper.h"

GrabThread::GrabThread() :
    m_nSocketIndex(0)
{
    qRegisterMetaType<uint8_t>("uint8_t");
    qRegisterMetaType<uint32_t>("uint32_t");
    qRegisterMetaType<uint64_t>("uint64_t");
}

GrabThread::~GrabThread()
{
}

void GrabThread::StartRunning(int nSocketIndex)
{
    m_nSocketIndex = nSocketIndex;
    m_bIsRunning = true;
    start();
}

void GrabThread::StopRunning()
{
    m_bIsRunning = false;
    wait();
}

void GrabThread::run()
{
    DeviceClient* devCli = DeviceController::Instance()[m_nSocketIndex];
    if (devCli == nullptr)
    {
        m_bIsRunning = false;
        return;
    }

    uint32_t imgFmt = 0, imgMode = 0, width = 0, height = 0, size = 0;
    devCli->GetFrameParam(imgFmt, imgMode, width, height, size);
    int nBufferLen = size;
    std::unique_ptr<uint8_t[]> pBuffer(new uint8_t[nBufferLen]());
    uint64_t headTimestamp = 0, tailTimestamp = 0;
    uint32_t frameSequence = 0;
    while (m_bIsRunning)
    {
        int ec = devCli->GrabFrame2(pBuffer.get(), nBufferLen, headTimestamp, tailTimestamp, frameSequence);
        if (ERR_NoError == ec)
        {
            emit frameGrabbed(pBuffer.get(), nBufferLen, headTimestamp, tailTimestamp, frameSequence);
            QThread::msleep(10);
        }
    }
}
