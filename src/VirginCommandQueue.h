#pragma once

#include <list>
#include "cectypes.h"
#include <p8-platform/threads/threads.h>

#define SOCKET int

class VirginKeyCodes;

class VirginCommandQueue : P8PLATFORM::CThread
{
public:
    VirginCommandQueue(const char *ipaddress, short port);
    ~VirginCommandQueue();

    void PowerOn();
    void PowerOff();

    void KeyPress(CEC::cec_user_control_code key);

    void Shutdown();

protected:
    virtual void *Process();

private:

    bool Connect();
    void SendTivoMessage(const char *message);

    volatile bool _retry = true;
    volatile bool _isConnected = false;
    VirginKeyCodes *_keyCodes = NULL;

    std::list<const char *> _commandQueue;
    P8PLATFORM::CMutex _socketLock;

    const char *_tivoIpAddresss = NULL;
    short _port = -1;

    SOCKET _tivoSocket = -1;
};
