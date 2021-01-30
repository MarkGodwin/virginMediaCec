/*
 * This file is part of the libCEC(R) library.
 *
 * libCEC(R) is Copyright (C) 2011-2015 Pulse-Eight Limited.  All rights reserved.
 * libCEC(R) is an original work, containing original code.
 *
 * libCEC(R) is a trademark of Pulse-Eight Limited.
 *
 * This program is dual-licensed; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301  USA
 *
 *
 * Alternatively, you can license this library under a commercial license,
 * please contact Pulse-Eight Licensing for more information.
 *
 * For more information contact:
 * Pulse-Eight Licensing       <license@pulse-eight.com>
 *     http://www.pulse-eight.com/
 *     http://www.pulse-eight.net/
 */

#include "env.h"
#include "cec.h"

#include <cstdio>
#include <fcntl.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <signal.h>
#include <stdlib.h>
#include <p8-platform/os.h>
#include <p8-platform/util/StringUtils.h>
#include <p8-platform/threads/threads.h>
#include <sys/wait.h>
#if defined(HAVE_CURSES_API)
#include "curses/CursesControl.h"
#endif
#include "VirginCommandQueue.h"
#include "helpers.h"

using namespace CEC;
using namespace P8PLATFORM;

#include "cecloader.h"

const char *g_virginIpAddress = "192.168.1.31";
const short g_virginPort = 31339;
const uint16_t g_virginCecAddress = 0x3200; // HDMI 2 on Reciever attached to HDMI 3 on TV

ICECCallbacks        g_callbacks;
libcec_configuration g_config;
int                  g_cecLogLevel(-1);
int                  g_cecDefaultLogLevel(CEC_LOG_TRAFFIC | CEC_LOG_WARNING | CEC_LOG_ERROR | CEC_LOG_NOTICE);
std::ofstream        g_logOutput;
bool                 g_bShortLog(false);
std::string          g_strPort;
bool                 g_bSingleCommand(false);
bool                 g_bExit(false);
bool                 g_bHardExit(false);
CMutex               g_outputMutex;
CEvent               g_quitEvent;
ICECAdapter*         g_parser;

VirginCommandQueue   *g_virginMediaCommand;

#if defined(HAVE_CURSES_API)
bool                 g_cursesEnable(false);
CCursesControl        g_cursesControl("1", "0");
#endif

class CReconnect : public P8PLATFORM::CThread
{
public:
    static CReconnect& Get(void)
    {
        static CReconnect _instance;
        return _instance;
    }

    virtual ~CReconnect(void) {}

    void* Process(void)
    {
        if (g_parser)
        {
            g_parser->Close();
            if (!g_parser->Open(g_strPort.c_str()))
            {
                PrintToStdOut("Failed to reconnect\n");
                g_bExit = true;
                g_quitEvent.Signal();
            }
        }
        return NULL;
    }

private:
    CReconnect(void) {}
};

class CSwitchToVMThread : public P8PLATFORM::CThread
{
public:
    static CSwitchToVMThread& Get(void)
    {
        static CSwitchToVMThread _instance;
        return _instance;
    }

    virtual ~CSwitchToVMThread(void) {}

    void* Process(void)
    {
        if (g_parser)
        {
            // May as well try and power on the VM box
            g_virginMediaCommand->PowerOn();

            // Activate the Raspberry Pi, so it gets the Remote Control keypresses
            // and to prevent the Apple TV from taking over
            g_parser->SetActiveSource();

            for (auto a = 0; a < 5; a++)
            {
                ::usleep(1000 * 1000);
                if (g_parser->IsLibCECActiveSource())
                {
                    PrintToStdOut("Attempting to switch to Virgin input on Reciever");
                    cec_command cmd;
                    cec_command::Format(cmd, g_parser->GetLogicalAddresses().primary, CECDEVICE_BROADCAST, CEC_OPCODE_SET_STREAM_PATH);
                    cmd.parameters.PushBack(g_virginCecAddress >> 8);
                    cmd.parameters.PushBack(g_virginCecAddress & 0xFF);
                    g_parser->Transmit(cmd);

                    return NULL;
                }
            }
        }
        PrintToStdOut("Pi didn't become active. Giving up.");
        return NULL;
    }

private:
    CSwitchToVMThread(void) {}
};

int CecLogMessage(void *UNUSED(cbParam), const cec_log_message message)
{
    if ((message.level & g_cecLogLevel) == message.level)
    {
        std::string strLevel;
        switch (message.level)
        {
        case CEC_LOG_ERROR:
            strLevel = "ERROR:   ";
            break;
        case CEC_LOG_WARNING:
            strLevel = "WARNING: ";
            break;
        case CEC_LOG_NOTICE:
            strLevel = "NOTICE:  ";
            break;
        case CEC_LOG_TRAFFIC:
            strLevel = "TRAFFIC: ";
            break;
        case CEC_LOG_DEBUG:
            strLevel = "DEBUG:   ";
            break;
        default:
            break;
        }

        std::string strFullLog;
        strFullLog = StringUtils::Format("%s[%16lld]\t%s", strLevel.c_str(), message.time, message.message);
        PrintToStdOut(strFullLog.c_str());

        if (g_logOutput.is_open())
        {
            if (g_bShortLog)
                g_logOutput << message.message << std::endl;
            else
                g_logOutput << strFullLog.c_str() << std::endl;
        }
    }

    return 0;
}

int CecKeyPress(void *UNUSED(cbParam), const cec_keypress key)
{
    PrintToStdOut("Keypress: %x", key.keycode);

    if (key.duration == 0)
        g_virginMediaCommand->KeyPress(key.keycode);

    return 0;

}

int CecCommand(void *UNUSED(cbParam), const cec_command command)
{
    PrintToStdOut(StringUtils::Format("----  %2x = FROM %d to %d ----\n", command.opcode, command.initiator, command.destination).c_str());

    if (command.opcode == CEC_OPCODE_SET_STREAM_PATH)
    {
        if (command.initiator == CECDEVICE_TV)
        {
            PrintToStdOut("Looks like a stream path is being set by the TV\n");

            auto targetAddress = (uint16_t)command.parameters[0] << 8 | command.parameters[1];
            PrintToStdOut(StringUtils::Format("--- Tgt path: %x", (int)targetAddress).c_str());
            if (targetAddress == g_virginCecAddress)
            {
                PrintToStdOut("Looks like the TV is activating the VM box...\n");
                // The TV is asking to switch to Virgin Media. Better grab control now
                if (!CSwitchToVMThread::Get().IsRunning())
                    CSwitchToVMThread::Get().CreateThread();
            }
        }
    }
    else if (command.opcode == CEC_OPCODE_ROUTING_CHANGE)
    {
        PrintToStdOut("Looks like a routing change initiated by");
        if (command.initiator == CECDEVICE_TV)
        {
            PrintToStdOut(" the TV\n");
        }
        else if (command.initiator == CECDEVICE_AUDIOSYSTEM)
        {
            PrintToStdOut(" the Amplifier\n");
            auto targetAddress = (uint16_t)command.parameters[2] << 8 | command.parameters[3];
            if (targetAddress == g_virginCecAddress)
            {
                PrintToStdOut("-- Switching to Pi, and then back again, we hope.\n");
                // Someone has cranked the knob to Virgin Media. Activate the Pi instead, and then switch back
                if (!CSwitchToVMThread::Get().IsRunning())
                    CSwitchToVMThread::Get().CreateThread();
            }
        }
        else
            PrintToStdOut(" another device\n");
    }
    else if (command.opcode == CEC_OPCODE_STANDBY)
    {
        PrintToStdOut("Being asked to switch off. Turning off VM");
        g_virginMediaCommand->PowerOff();
        g_virginMediaCommand->PowerOff();
    }

    return 0;
}

int CecAlert(void *UNUSED(cbParam), const libcec_alert type, const libcec_parameter UNUSED(param))
{
    switch (type)
    {
    case CEC_ALERT_CONNECTION_LOST:
        if (!CReconnect::Get().IsRunning())
        {
            PrintToStdOut("Connection lost - trying to reconnect\n");
            CReconnect::Get().CreateThread(false);
        }
        break;
    default:
        break;
    }
    return 0;
}

void CecSourceActivated(void *UNUSED(cbParam), const cec_logical_address address, const uint8_t bActivated)
{
    if (bActivated)
    {
        PrintToStdOut("!!I've been activated!!\n");
    }
    else
        PrintToStdOut("--I've been de-activated--\n");
}

void ListDevices(ICECAdapter *parser)
{
    cec_adapter_descriptor devices[10];
    std::string strMessage = StringUtils::Format("libCEC version: %s, %s",
        parser->VersionToString(g_config.serverVersion).c_str(),
        parser->GetLibInfo());
    PrintToStdOut(strMessage.c_str());

    int8_t iDevicesFound = parser->DetectAdapters(devices, 10, NULL);
    if (iDevicesFound <= 0)
    {
        PrintToStdOut("Found devices: NONE");
    }
    else
    {
        PrintToStdOut("Found devices: %d\n", iDevicesFound);

        for (int8_t iDevicePtr = 0; iDevicePtr < iDevicesFound; iDevicePtr++)
        {
            PrintToStdOut("device:              %d", iDevicePtr + 1);
            PrintToStdOut("com port:            %s", devices[iDevicePtr].strComName);
            PrintToStdOut("vendor id:           %04x", devices[iDevicePtr].iVendorId);
            PrintToStdOut("product id:          %04x", devices[iDevicePtr].iProductId);
            PrintToStdOut("firmware version:    %d", devices[iDevicePtr].iFirmwareVersion);

            if (devices[iDevicePtr].iFirmwareBuildDate != CEC_FW_BUILD_UNKNOWN)
            {
                time_t buildTime = (time_t)devices[iDevicePtr].iFirmwareBuildDate;
                std::string strDeviceInfo;
                strDeviceInfo = StringUtils::Format("firmware build date: %s", asctime(gmtime(&buildTime)));
                strDeviceInfo = StringUtils::Left(strDeviceInfo, strDeviceInfo.length() > 1 ? (unsigned)(strDeviceInfo.length() - 1) : 0); // strip \n added by asctime
                strDeviceInfo.append(" +0000");
                PrintToStdOut(strDeviceInfo.c_str());
            }

            if (devices[iDevicePtr].adapterType != ADAPTERTYPE_UNKNOWN)
            {
                PrintToStdOut("type:                %s", parser->ToString(devices[iDevicePtr].adapterType));
            }

            PrintToStdOut("");
        }
    }
}

void ShowHelpConsole(void)
{
    CLockObject lock(g_outputMutex);
    std::cout << std::endl <<
        "================================================================================" << std::endl <<
        "Available commands:" << std::endl <<
        std::endl <<
        "[tx] {bytes}              transfer bytes over the CEC line." << std::endl <<
        "[txn] {bytes}             transfer bytes but don't wait for transmission ACK." << std::endl <<
        "[on] {address}            power on the device with the given logical address." << std::endl <<
        "[standby] {address}       put the device with the given address in standby mode." << std::endl <<
        "[la] {logical address}    change the logical address of the CEC adapter." << std::endl <<
        "[p] {device} {port}       change the HDMI port number of the CEC adapter." << std::endl <<
        "[pa] {physical address}   change the physical address of the CEC adapter." << std::endl <<
        "[as]                      make the CEC adapter the active source." << std::endl <<
        "[is]                      mark the CEC adapter as inactive source." << std::endl <<
        "[osd] {addr} {string}     set OSD message on the specified device." << std::endl <<
        "[ver] {addr}              get the CEC version of the specified device." << std::endl <<
        "[ven] {addr}              get the vendor ID of the specified device." << std::endl <<
        "[lang] {addr}             get the menu language of the specified device." << std::endl <<
        "[pow] {addr}              get the power status of the specified device." << std::endl <<
        "[name] {addr}             get the OSD name of the specified device." << std::endl <<
        "[poll] {addr}             poll the specified device." << std::endl <<
        "[lad]                     lists active devices on the bus" << std::endl <<
        "[ad] {addr}               checks whether the specified device is active." << std::endl <<
        "[at] {type}               checks whether the specified device type is active." << std::endl <<
        "[sp] {addr}               makes the specified physical address active." << std::endl <<
        "[spl] {addr}              makes the specified logical address active." << std::endl <<
        "[volup]                   send a volume up command to the amp if present" << std::endl <<
        "[voldown]                 send a volume down command to the amp if present" << std::endl <<
        "[mute]                    send a mute/unmute command to the amp if present" << std::endl <<
        "[self]                    show the list of addresses controlled by libCEC" << std::endl <<
        "[scan]                    scan the CEC bus and display device info" << std::endl <<
        "[mon] {1|0}               enable or disable CEC bus monitoring." << std::endl <<
        "[log] {1 - 31}            change the log level. see cectypes.h for values." << std::endl <<
        "[ping]                    send a ping command to the CEC adapter." << std::endl <<
        "[bl]                      to let the adapter enter the bootloader, to upgrade" << std::endl <<
        "                          the flash rom." << std::endl <<
        "[r]                       reconnect to the CEC adapter." << std::endl <<
        "[h] or [help]             show this help." << std::endl <<
        "[q] or [quit]             to quit the CEC test client and switch off all" << std::endl <<
        "                          connected CEC devices." << std::endl <<
        "================================================================================" << std::endl;
}

bool ProcessCommandSELF(ICECAdapter *parser, const std::string &command, std::string & UNUSED(arguments))
{
    if (command == "self")
    {
        cec_logical_addresses addr = parser->GetLogicalAddresses();
        std::string strOut = "Addresses controlled by libCEC: ";
        bool bFirst(true);
        for (uint8_t iPtr = 0; iPtr <= 15; iPtr++)
        {
            if (addr[iPtr])
            {
                strOut += StringUtils::Format((bFirst ? "%d%s" : ", %d%s"), iPtr, parser->IsActiveSource((cec_logical_address)iPtr) ? "*" : "");
                bFirst = false;
            }
        }
        PrintToStdOut(strOut.c_str());
        return true;
    }

    return false;
}

bool ProcessCommandSP(ICECAdapter *parser, const std::string &command, std::string &arguments)
{
    if (command == "sp")
    {
        std::string strAddress;
        int iAddress;
        if (GetWord(arguments, strAddress))
        {
            sscanf(strAddress.c_str(), "%x", &iAddress);
            if (iAddress >= 0 && iAddress <= CEC_INVALID_PHYSICAL_ADDRESS)
                parser->SetStreamPath((uint16_t)iAddress);
            return true;
        }
    }

    return false;
}

bool ProcessCommandSPL(ICECAdapter *parser, const std::string &command, std::string &arguments)
{
    if (command == "spl")
    {
        std::string strAddress;
        cec_logical_address iAddress;
        if (GetWord(arguments, strAddress))
        {
            iAddress = (cec_logical_address)atoi(strAddress.c_str());
            if (iAddress >= CECDEVICE_TV && iAddress < CECDEVICE_BROADCAST)
                parser->SetStreamPath(iAddress);
            return true;
        }
    }

    return false;
}

bool ProcessCommandTX(ICECAdapter *parser, const std::string &command, std::string &arguments)
{
    if (command == "tx" || command == "txn")
    {
        std::string strvalue;
        cec_command bytes = parser->CommandFromString(arguments.c_str());

        if (command == "txn")
            bytes.transmit_timeout = 0;

        parser->Transmit(bytes);

        return true;
    }

    return false;
}

bool ProcessCommandON(ICECAdapter *parser, const std::string &command, std::string &arguments)
{
    if (command == "on")
    {
        std::string strValue;
        uint8_t iValue = 0;
        if (GetWord(arguments, strValue) && HexStrToInt(strValue, iValue) && iValue <= 0xF)
        {
            parser->PowerOnDevices((cec_logical_address)iValue);
            return true;
        }
        else
        {
            PrintToStdOut("invalid destination");
        }
    }

    return false;
}

bool ProcessCommandSTANDBY(ICECAdapter *parser, const std::string &command, std::string &arguments)
{
    if (command == "standby")
    {
        std::string strValue;
        uint8_t iValue = 0;
        if (GetWord(arguments, strValue) && HexStrToInt(strValue, iValue) && iValue <= 0xF)
        {
            parser->StandbyDevices((cec_logical_address)iValue);
            return true;
        }
        else
        {
            PrintToStdOut("invalid destination");
        }
    }

    return false;
}

bool ProcessCommandPOLL(ICECAdapter *parser, const std::string &command, std::string &arguments)
{
    if (command == "poll")
    {
        std::string strValue;
        uint8_t iValue = 0;
        if (GetWord(arguments, strValue) && HexStrToInt(strValue, iValue) && iValue <= 0xF)
        {
            if (parser->PollDevice((cec_logical_address)iValue))
                PrintToStdOut("POLL message sent");
            else
                PrintToStdOut("POLL message not sent");
            return true;
        }
        else
        {
            PrintToStdOut("invalid destination");
        }
    }

    return false;
}

bool ProcessCommandLA(ICECAdapter *parser, const std::string &command, std::string &arguments)
{
    if (command == "la")
    {
        std::string strvalue;
        if (GetWord(arguments, strvalue))
        {
            parser->SetLogicalAddress((cec_logical_address)atoi(strvalue.c_str()));
            return true;
        }
    }

    return false;
}

bool ProcessCommandP(ICECAdapter *parser, const std::string &command, std::string &arguments)
{
    if (command == "p")
    {
        std::string strPort, strDevice;
        if (GetWord(arguments, strDevice) && GetWord(arguments, strPort))
        {
            parser->SetHDMIPort((cec_logical_address)atoi(strDevice.c_str()), (uint8_t)atoi(strPort.c_str()));
            return true;
        }
    }

    return false;
}

bool ProcessCommandPA(ICECAdapter *parser, const std::string &command, std::string &arguments)
{
    if (command == "pa")
    {
        std::string strB1, strB2;
        uint8_t iB1, iB2;
        if (GetWord(arguments, strB1) && HexStrToInt(strB1, iB1) &&
            GetWord(arguments, strB2) && HexStrToInt(strB2, iB2))
        {
            uint16_t iPhysicalAddress = ((uint16_t)iB1 << 8) + iB2;
            parser->SetPhysicalAddress(iPhysicalAddress);
            return true;
        }
    }

    return false;
}

bool ProcessCommandOSD(ICECAdapter *parser, const std::string &command, std::string &arguments)
{
    if (command == "osd")
    {
        bool bFirstWord(false);
        std::string strAddr, strMessage, strWord;
        uint8_t iAddr;
        if (GetWord(arguments, strAddr) && HexStrToInt(strAddr, iAddr) && iAddr < 0xF)
        {
            while (GetWord(arguments, strWord))
            {
                if (bFirstWord)
                {
                    bFirstWord = false;
                    strMessage.append(" ");
                }
                strMessage.append(strWord);
            }
            parser->SetOSDString((cec_logical_address)iAddr, CEC_DISPLAY_CONTROL_DISPLAY_FOR_DEFAULT_TIME, strMessage.c_str());
            return true;
        }
    }

    return false;
}

bool ProcessCommandAS(ICECAdapter *parser, const std::string &command, std::string & UNUSED(arguments))
{
    if (command == "as")
    {
        parser->SetActiveSource();
        // wait for the source switch to finish for 15 seconds tops
        if (g_bSingleCommand)
        {
            CTimeout timeout(15000);
            bool bActiveSource(false);
            while (timeout.TimeLeft() > 0 && !bActiveSource)
            {
                bActiveSource = parser->IsLibCECActiveSource();
                if (!bActiveSource)
                    CEvent::Sleep(100);
            }
        }
        return true;
    }

    return false;
}

bool ProcessCommandIS(ICECAdapter *parser, const std::string &command, std::string & UNUSED(arguments))
{
    if (command == "is")
        return parser->SetInactiveView();

    return false;
}

bool ProcessCommandPING(ICECAdapter *parser, const std::string &command, std::string & UNUSED(arguments))
{
    if (command == "ping")
    {
        parser->PingAdapter();
        return true;
    }

    return false;
}

bool ProcessCommandVOLUP(ICECAdapter *parser, const std::string &command, std::string & UNUSED(arguments))
{
    if (command == "volup")
    {
        PrintToStdOut("volume up: %2X", parser->VolumeUp());
        return true;
    }

    return false;
}

bool ProcessCommandVOLDOWN(ICECAdapter *parser, const std::string &command, std::string & UNUSED(arguments))
{
    if (command == "voldown")
    {
        PrintToStdOut("volume down: %2X", parser->VolumeDown());
        return true;
    }

    return false;
}

bool ProcessCommandMUTE(ICECAdapter *parser, const std::string &command, std::string & UNUSED(arguments))
{
    if (command == "mute")
    {
        PrintToStdOut("mute: %2X", parser->MuteAudio());
        return true;
    }

    return false;
}

bool ProcessCommandMON(ICECAdapter *parser, const std::string &command, std::string &arguments)
{
    if (command == "mon")
    {
        std::string strEnable;
        if (GetWord(arguments, strEnable) && (strEnable == "0" || strEnable == "1"))
        {
            parser->SwitchMonitoring(strEnable == "1");
            return true;
        }
    }

    return false;
}


bool ProcessCommandLANG(ICECAdapter *parser, const std::string &command, std::string &arguments)
{
    if (command == "lang")
    {
        std::string strDev;
        if (GetWord(arguments, strDev))
        {
            int iDev = atoi(strDev.c_str());
            if (iDev >= 0 && iDev < 15)
            {
                std::string strLog;
                cec_menu_language language;
                if (parser->GetDeviceMenuLanguage((cec_logical_address)iDev, &language))
                    strLog = StringUtils::Format("menu language '%s'", language.language);
                else
                    strLog = "failed!";
                PrintToStdOut(strLog.c_str());
                return true;
            }
        }
    }

    return false;
}

bool ProcessCommandVEN(ICECAdapter *parser, const std::string &command, std::string &arguments)
{
    if (command == "ven")
    {
        std::string strDev;
        if (GetWord(arguments, strDev))
        {
            int iDev = atoi(strDev.c_str());
            if (iDev >= 0 && iDev < 15)
            {
                uint64_t iVendor = parser->GetDeviceVendorId((cec_logical_address)iDev);
                PrintToStdOut("vendor id: %06llx", iVendor);
                return true;
            }
        }
    }

    return false;
}

bool ProcessCommandVER(ICECAdapter *parser, const std::string &command, std::string &arguments)
{
    if (command == "ver")
    {
        std::string strDev;
        if (GetWord(arguments, strDev))
        {
            int iDev = atoi(strDev.c_str());
            if (iDev >= 0 && iDev < 15)
            {
                cec_version iVersion = parser->GetDeviceCecVersion((cec_logical_address)iDev);
                PrintToStdOut("CEC version %s", parser->ToString(iVersion));
                return true;
            }
        }
    }

    return false;
}

bool ProcessCommandPOW(ICECAdapter *parser, const std::string &command, std::string &arguments)
{
    if (command == "pow")
    {
        std::string strDev;
        if (GetWord(arguments, strDev))
        {
            int iDev = atoi(strDev.c_str());
            if (iDev >= 0 && iDev < 15)
            {
                cec_power_status iPower = parser->GetDevicePowerStatus((cec_logical_address)iDev);
                PrintToStdOut("power status: %s", parser->ToString(iPower));
                return true;
            }
        }
    }

    return false;
}

bool ProcessCommandNAME(ICECAdapter *parser, const std::string &command, std::string &arguments)
{
    if (command == "name")
    {
        std::string strDev;
        if (GetWord(arguments, strDev))
        {
            int iDev = atoi(strDev.c_str());
            if (iDev >= 0 && iDev < 15)
            {
                cec_osd_name name = parser->GetDeviceOSDName((cec_logical_address)iDev);
                PrintToStdOut("OSD name of device %d is '%s'", iDev, name.name);
            }
            return true;
        }
    }

    return false;
}

bool ProcessCommandLAD(ICECAdapter *parser, const std::string &command, std::string & UNUSED(arguments))
{
    if (command == "lad")
    {
        PrintToStdOut("listing active devices:");
        cec_logical_addresses addresses = parser->GetActiveDevices();
        for (uint8_t iPtr = 0; iPtr <= 11; iPtr++)
            if (addresses[iPtr])
            {
                PrintToStdOut("logical address %X", (int)iPtr);
            }
        return true;
    }

    return false;
}

bool ProcessCommandAD(ICECAdapter *parser, const std::string &command, std::string &arguments)
{
    if (command == "ad")
    {
        std::string strDev;
        if (GetWord(arguments, strDev))
        {
            int iDev = atoi(strDev.c_str());
            if (iDev >= 0 && iDev < 15)
                PrintToStdOut("logical address %X is %s", iDev, (parser->IsActiveDevice((cec_logical_address)iDev) ? "active" : "not active"));
        }
    }

    return false;
}

bool ProcessCommandAT(ICECAdapter *parser, const std::string &command, std::string &arguments)
{
    if (command == "at")
    {
        std::string strType;
        if (GetWord(arguments, strType))
        {
            cec_device_type type = CEC_DEVICE_TYPE_TV;
            if (strType == "a")
                type = CEC_DEVICE_TYPE_AUDIO_SYSTEM;
            else if (strType == "p")
                type = CEC_DEVICE_TYPE_PLAYBACK_DEVICE;
            else if (strType == "r")
                type = CEC_DEVICE_TYPE_RECORDING_DEVICE;
            else if (strType == "t")
                type = CEC_DEVICE_TYPE_TUNER;

            PrintToStdOut("device %d is %s", type, (parser->IsActiveDeviceType(type) ? "active" : "not active"));
            return true;
        }
    }

    return false;
}

bool ProcessCommandR(ICECAdapter *parser, const std::string &command, std::string & UNUSED(arguments))
{
    if (command == "r")
    {
        bool bReactivate = parser->IsLibCECActiveSource();

        PrintToStdOut("closing the connection");
        parser->Close();

        PrintToStdOut("opening a new connection");
        parser->Open(g_strPort.c_str());

        if (bReactivate)
        {
            PrintToStdOut("setting active source");
            parser->SetActiveSource();
        }
        return true;
    }

    return false;
}

bool ProcessCommandH(ICECAdapter * UNUSED(parser), const std::string &command, std::string & UNUSED(arguments))
{
    if (command == "h" || command == "help")
    {
        ShowHelpConsole();
        return true;
    }

    return false;
}

bool ProcessCommandLOG(ICECAdapter * UNUSED(parser), const std::string &command, std::string &arguments)
{
    if (command == "log")
    {
        std::string strLevel;
        if (GetWord(arguments, strLevel))
        {
            int iNewLevel = atoi(strLevel.c_str());
            if (iNewLevel >= CEC_LOG_ERROR && iNewLevel <= CEC_LOG_ALL)
            {
                g_cecLogLevel = iNewLevel;

                PrintToStdOut("log level changed to %s", strLevel.c_str());
                return true;
            }
        }
    }

    return false;
}

bool ProcessCommandSCAN(ICECAdapter *parser, const std::string &command, std::string & UNUSED(arguments))
{
    if (command == "scan")
    {
        std::string strLog;
        PrintToStdOut("requesting CEC bus information ...");

        strLog.append("CEC bus information\n===================\n");
        cec_logical_addresses addresses = parser->GetActiveDevices();
        cec_logical_address activeSource = parser->GetActiveSource();
        for (uint8_t iPtr = 0; iPtr < 16; iPtr++)
        {
            if (addresses[iPtr])
            {
                uint64_t iVendorId = parser->GetDeviceVendorId((cec_logical_address)iPtr);
                uint16_t iPhysicalAddress = parser->GetDevicePhysicalAddress((cec_logical_address)iPtr);
                bool     bActive = parser->IsActiveSource((cec_logical_address)iPtr);
                cec_version iCecVersion = parser->GetDeviceCecVersion((cec_logical_address)iPtr);
                cec_power_status power = parser->GetDevicePowerStatus((cec_logical_address)iPtr);
                cec_osd_name osdName = parser->GetDeviceOSDName((cec_logical_address)iPtr);
                std::string strAddr;
                strAddr = StringUtils::Format("%x.%x.%x.%x", (iPhysicalAddress >> 12) & 0xF, (iPhysicalAddress >> 8) & 0xF, (iPhysicalAddress >> 4) & 0xF, iPhysicalAddress & 0xF);
                cec_menu_language lang;
                lang.device = CECDEVICE_UNKNOWN;
                parser->GetDeviceMenuLanguage((cec_logical_address)iPtr, &lang);

                strLog += StringUtils::Format("device #%X: %s\n", (int)iPtr, parser->ToString((cec_logical_address)iPtr));
                strLog += StringUtils::Format("address:       %s\n", strAddr.c_str());
                strLog += StringUtils::Format("active source: %s\n", (bActive ? "yes" : "no"));
                strLog += StringUtils::Format("vendor:        %s\n", parser->ToString((cec_vendor_id)iVendorId));
                strLog += StringUtils::Format("osd string:    %s\n", osdName.name);
                strLog += StringUtils::Format("CEC version:   %s\n", parser->ToString(iCecVersion));
                strLog += StringUtils::Format("power status:  %s\n", parser->ToString(power));
                if ((uint8_t)lang.device == iPtr)
                    strLog += StringUtils::Format("language:      %s\n", lang.language);
                strLog.append("\n\n");
            }
        }

        activeSource = parser->GetActiveSource();
        strLog += StringUtils::Format("currently active source: %s (%d)", parser->ToString(activeSource), (int)activeSource);

        PrintToStdOut(strLog.c_str());
        return true;
    }

    return false;
}

bool ProcessConsoleCommand(ICECAdapter *parser, std::string &input)
{
    if (!input.empty())
    {
        std::string command;
        if (GetWord(input, command))
        {
            if (command == "q" || command == "quit")
                return false;

            ProcessCommandTX(parser, command, input) ||
                ProcessCommandON(parser, command, input) ||
                ProcessCommandSTANDBY(parser, command, input) ||
                ProcessCommandPOLL(parser, command, input) ||
                ProcessCommandLA(parser, command, input) ||
                ProcessCommandP(parser, command, input) ||
                ProcessCommandPA(parser, command, input) ||
                ProcessCommandAS(parser, command, input) ||
                ProcessCommandIS(parser, command, input) ||
                ProcessCommandOSD(parser, command, input) ||
                ProcessCommandPING(parser, command, input) ||
                ProcessCommandVOLUP(parser, command, input) ||
                ProcessCommandVOLDOWN(parser, command, input) ||
                ProcessCommandMUTE(parser, command, input) ||
                ProcessCommandMON(parser, command, input) ||
                ProcessCommandLANG(parser, command, input) ||
                ProcessCommandVEN(parser, command, input) ||
                ProcessCommandVER(parser, command, input) ||
                ProcessCommandPOW(parser, command, input) ||
                ProcessCommandNAME(parser, command, input) ||
                ProcessCommandLAD(parser, command, input) ||
                ProcessCommandAD(parser, command, input) ||
                ProcessCommandAT(parser, command, input) ||
                ProcessCommandR(parser, command, input) ||
                ProcessCommandH(parser, command, input) ||
                ProcessCommandLOG(parser, command, input) ||
                ProcessCommandSCAN(parser, command, input) ||
                ProcessCommandSP(parser, command, input) ||
                ProcessCommandSPL(parser, command, input) ||
                ProcessCommandSELF(parser, command, input);
        }
    }
    return true;
}

void sighandler(int iSignal)
{
    PrintToStdOut("signal caught: %d - exiting", iSignal);
    g_bExit = true;
    g_quitEvent.Signal();
#if defined(HAVE_CURSES_API)
    if (g_cursesEnable)
        g_cursesControl.End();
#endif
}

int main(int argc, char *argv[])
{

    bool isDaemon = false;

    if (argc > 1 && !strcmp(argv[1], "daemonise"))
    {
#ifndef __WINDOWS__

        /* Fork off the parent process */
        auto pid = ::fork();

        /* An error occurred */
        if (pid < 0)
            exit(EXIT_FAILURE);

        /* Success: Let the parent terminate */
        if (pid > 0)
        {
            int status;
            ::waitpid(pid, &status, 0);
            ::exit(status);
        }

        /* On success: The child process becomes session leader */
        if (::setsid() < 0)
            exit(EXIT_FAILURE);

        /* Catch, ignore and handle signals */
        //TODO: Implement a working signal handler */
        signal(SIGCHLD, SIG_IGN);
        signal(SIGHUP, SIG_IGN);

        /* Fork off for the second time*/
        pid = ::fork();

        /* An error occurred */
        if (pid < 0)
            exit(EXIT_FAILURE);

        /* Success: Let the parent terminate */
        if (pid > 0)
            exit(EXIT_SUCCESS);

        /* Set new file permissions */
        ::umask(0);

        /* Change the working directory to the root directory */
        /* or another appropriated directory */
        ::chdir("/");

        /* Close all open file descriptors */
        int x;
        for (x = 3; x>0; x--)
        {
            close(x);
        }
#endif
        isDaemon = true;
    }

    if (signal(SIGINT, sighandler) == SIG_ERR ||
        signal(SIGTERM, sighandler) == SIG_ERR)
    {
        PrintToStdOut("can't register sighandler");
        return -1;
    }

    g_config.Clear();
    g_callbacks.Clear();
    snprintf(g_config.strDeviceName, 13, "virginMedia");
    g_config.clientVersion = LIBCEC_VERSION_CURRENT;
    g_config.bActivateSource = 0;
    g_callbacks.CBCecLogMessage = &CecLogMessage;
    g_callbacks.CBCecKeyPress = &CecKeyPress;
    g_callbacks.CBCecCommand = &CecCommand;
    g_callbacks.CBCecAlert = &CecAlert;
    g_callbacks.CBCecSourceActivated = &CecSourceActivated;

    g_config.callbacks = &g_callbacks;

    if (g_cecLogLevel == -1)
        g_cecLogLevel = g_cecDefaultLogLevel;

    g_config.deviceTypes.Add(CEC_DEVICE_TYPE_RECORDING_DEVICE);

    // Command channel to VM box, for forwarding remote commands
    g_virginMediaCommand = new VirginCommandQueue(g_virginIpAddress, g_virginPort);

    g_parser = LibCecInitialise(&g_config);
    if (!g_parser)
    {
#ifdef __WINDOWS__
        std::cout << "Cannot load cec.dll" << std::endl;
#else
        std::cout << "Cannot load libcec.so" << std::endl;
#endif

        if (g_parser)
            UnloadLibCec(g_parser);

        return 1;
    }

    // init video on targets that need this
    g_parser->InitVideoStandalone();

    std::string strLog;
    strLog = StringUtils::Format("CEC Parser created - libCEC version %s", g_parser->VersionToString(g_config.serverVersion).c_str());
    std::cout << strLog.c_str() << std::endl;

    //make stdin non-blocking
#ifndef __WINDOWS__
    int flags = fcntl(0, F_GETFL, 0);
    flags |= O_NONBLOCK;
    fcntl(0, F_SETFL, flags);
#endif

    if (g_strPort.empty())
    {
        if (!g_bSingleCommand)
            std::cout << "no serial port given. trying autodetect: ";
        cec_adapter devices[10];
        uint8_t iDevicesFound = g_parser->FindAdapters(devices, 10, NULL);
        if (iDevicesFound <= 0)
        {
            if (g_bSingleCommand)
                std::cout << "autodetect ";
            std::cout << "FAILED" << std::endl;
            UnloadLibCec(g_parser);
            return 1;
        }
        else
        {
            g_strPort = devices[0].comm;
        }
    }

    PrintToStdOut("opening a connection to the CEC adapter...");

    if (!g_parser->Open(g_strPort.c_str()))
    {
        PrintToStdOut("unable to open the device on port %s", g_strPort.c_str());
        UnloadLibCec(g_parser);
        return 1;
    }

#if defined(HAVE_CURSES_API)
    if (g_cursesEnable)
        g_cursesControl.Init();
#endif

    if (isDaemon)
        g_quitEvent.Wait();
    else
    {

        while (!g_bExit && !g_bHardExit)
        {
            std::string input;
#if defined(HAVE_CURSES_API)
            if (!g_cursesEnable) {
                getline(std::cin, input);
                std::cin.clear();
            }
            else
            {
                input = g_cursesControl.ParseCursesKey();
            }
#else
            getline(std::cin, input);
            std::cin.clear();
#endif

            if (ProcessConsoleCommand(g_parser, input) && !g_bSingleCommand && !g_bExit && !g_bHardExit)
            {
                if (!input.empty())
                    PrintToStdOut("waiting for input");
            }
            else
            {
#if defined(HAVE_CURSES_API)
                if (g_cursesEnable)
                    g_cursesControl.End();
#endif
                g_bExit = true;
            }

            if (!g_bExit && !g_bHardExit)
                CEvent::Sleep(50);
        }
    }

    g_parser->Close();
    UnloadLibCec(g_parser);

    if (g_logOutput.is_open())
        g_logOutput.close();

    return 0;
}

