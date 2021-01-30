#include "VirginCommandQueue.h"
#include "VirginKeyCodes.h"
#include <p8-platform/sockets/tcp.h>

#include "helpers.h"

using namespace P8PLATFORM;

VirginCommandQueue::VirginCommandQueue(const char *ipaddress, short port)
    :_tivoIpAddresss(ipaddress),
    _port(port)
{
    _keyCodes = new VirginKeyCodes();
}


VirginCommandQueue::~VirginCommandQueue()
{
    delete _keyCodes;
}

void VirginCommandQueue::PowerOn()
{
    SendTivoMessage("IRCODE GUIDE\r\n");
}

void VirginCommandQueue::PowerOff()
{
    SendTivoMessage("IRCODE STANDBY\r\n");
}

void VirginCommandQueue::KeyPress(CEC::cec_user_control_code key)
{
    auto irCode = _keyCodes->GetIRCode(key);
    if (irCode != NULL)
    {
        SendTivoMessage(irCode);
    }
}

void VirginCommandQueue::Shutdown()
{
    {
        CLockObject lock(_socketLock);
        if (_isConnected)
        {
            ::shutdown(_tivoSocket, SHUT_WR);
        }
    }

    StopThread();
}

bool VirginCommandQueue::Connect()
{
    {
        CLockObject lock(_socketLock);
        _retry = true;
        if (_isConnected)
            return true;
    }

    this->StopThread(10000);

    _tivoSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (_tivoSocket < 0)
    {
        return false;
    }

    PrintToStdOut("Connecting to TIVO....");
    struct sockaddr_in serv_addr;
    memset(&serv_addr, '0', sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    inet_pton(AF_INET, _tivoIpAddresss, &serv_addr.sin_addr);
    serv_addr.sin_port = htons(_port);
    
    if (connect(_tivoSocket, (struct sockaddr*) &serv_addr, sizeof(struct sockaddr_in)) < 0)
    {
        return false;
    }

    _isConnected = true;
    PrintToStdOut(" Connected!\n");

    CreateThread();

    return true;
}

void VirginCommandQueue::SendTivoMessage(const char * message)
{
    if (!Connect())
        return;

    ::send(_tivoSocket, message, strlen(message), 0);
}

void * VirginCommandQueue::Process()
{
    while (true)
    {
        timeval timeout;
        timeout.tv_sec = 15;
        timeout.tv_usec = 0;

        fd_set readFds;
        FD_ZERO(&readFds);
        FD_SET(_tivoSocket, &readFds);
        int rc = ::select(_tivoSocket + 1, &readFds, NULL, NULL, &timeout);

        if (rc == -1)
        {
            PrintToStdOut("Tivo connection lost unexpectedly\n");
            CLockObject lock(_socketLock);
            // Error shutdown
            _isConnected = false;
            ::close(_tivoSocket);
            _tivoSocket = -1;
            break;
        }
        else if (rc == 0)
        {
            {
                CLockObject lock(_socketLock);
                if (_retry)
                {
                    _retry = false;
                    continue;
                }

                PrintToStdOut("Closing Tivo connection due to inactivity\n");
                _isConnected = false;
                // Timeout -- Initial graceful shutdown
                ::shutdown(_tivoSocket, SHUT_WR);
            }
            timeout.tv_sec = 5;
            timeout.tv_usec = 0;
            int rc = ::select(_tivoSocket + 1, &readFds, NULL, NULL, &timeout);

            ::close(_tivoSocket);
            break;
        }
        else
        {
            char msg[1025];
            memset(msg, 0, sizeof(msg));
            auto nBytes = ::recv(_tivoSocket, msg, sizeof(msg), 0);

            if (nBytes == 0)
            {
                PrintToStdOut("Tivo connection closed by Tivo box\n");
                CLockObject lock(_socketLock);
                // Graceful shutdown from Tivo box
                _isConnected = false;
                ::close(_tivoSocket);
                break;
            }

            PrintToStdOut("##TIVO Message: %s\n", msg);
        }

    }
    return nullptr;
}
