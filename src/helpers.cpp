
#include "env.h"
#include <string>
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

#include "helpers.h"

using namespace P8PLATFORM;
extern CMutex               g_outputMutex;

void PrintToStdOut(const char *strFormat, ...)
{
    std::string strLog;

    va_list argList;
    va_start(argList, strFormat);
    strLog = StringUtils::FormatV(strFormat, argList);
    va_end(argList);

    CLockObject lock(g_outputMutex);
    std::cout << strLog << std::endl;
}

bool HexStrToInt(const std::string& data, uint8_t& value)
{
    int iTmp(0);
    if (sscanf(data.c_str(), "%x", &iTmp) == 1)
    {
        if (iTmp > 256)
            value = 255;
        else if (iTmp < 0)
            value = 0;
        else
            value = (uint8_t)iTmp;

        return true;
    }

    return false;
}

//get the first word (separated by whitespace) from string data and place that in word
//then remove that word from string data
bool GetWord(std::string& data, std::string& word)
{
    std::stringstream datastream(data);
    std::string end;

    datastream >> word;
    if (datastream.fail())
    {
        data.clear();
        return false;
    }

    size_t pos = data.find(word) + word.length();

    if (pos >= data.length())
    {
        data.clear();
        return true;
    }

    data = data.substr(pos);

    datastream.clear();
    datastream.str(data);

    datastream >> end;
    if (datastream.fail())
        data.clear();

    return true;
}
