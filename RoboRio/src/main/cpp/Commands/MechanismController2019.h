//==============================================================================
// MechanismController2019.h
//==============================================================================

#pragma once

#include "RobotPath/IMechanismController.h"

// MechanismController2019 implements mechanism actions on autonomous paths for
// the 2019 robot
class MechanismController2019 : public IMechanismController {
public:
    //==========================================================================
    // Overrides from IMechanismController

    virtual void DoAction(const std::string& action);
};
