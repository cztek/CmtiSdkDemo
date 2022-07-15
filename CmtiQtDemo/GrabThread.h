#pragma once
#include <QThread>
#include <atomic>

class GrabThread : public QThread
{
    Q_OBJECT

public:
    GrabThread();
    ~GrabThread() override;

    void StartRunning(int nSocketIndex);
    void StopRunning();

signals:
    void frameGrabbed(const uint8_t* pBuffer, int nBufferLen, uint64_t headTimestamp, uint64_t tailTimestamp, uint32_t frameSequence);

private:
    void run() override;

private:
    std::atomic<bool> m_bIsRunning;
    int m_nSocketIndex;
};

