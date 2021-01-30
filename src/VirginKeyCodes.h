#pragma once

#ifndef VIRGINKEYCODES_H__
#define VIRGINKEYCODES_H__

#include <map>
#include "cectypes.h"

class VirginKeyCodes
{
public:
    VirginKeyCodes();
    ~VirginKeyCodes();

    const char *GetIRCode(CEC::cec_user_control_code cecKey);

private:
    std::map<CEC::cec_user_control_code, const char *> _keyMap;

};

#endif