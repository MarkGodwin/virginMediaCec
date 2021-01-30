#pragma once

#ifndef HELPERS_H__
#define HELPERS_H__

void PrintToStdOut(const char *strFormat, ...);
bool HexStrToInt(const std::string& data, uint8_t& value);
bool GetWord(std::string& data, std::string& word);

#endif
