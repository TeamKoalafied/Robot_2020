//==============================================================================
// MechanismController2020.cpp
//==============================================================================

#include "MechanismController2020.h"

#include "../Subsystems/Manipulator.h"
#include <iostream>


//==========================================================================
// Overrides from IMechanismController

void MechanismController2020::DoAction(const std::string& action) {

    Manipulator& manipulator = Manipulator::GetInstance();

	if (action == "ExtendIntake") manipulator.ExtendIntake();
    else if (action == "RetractIntake") manipulator.RetractIntake();
	else if (action == "RunIndexForward") manipulator.RunIndexForward();
	else if (action == "RunIndexBack") manipulator.RunIndexBack();
//	else if (action == "Shoot") manipulator.Shoot();
	else if (action == "StartIntaking") manipulator.StartIntaking();
	else if (action == "StopIntaking") manipulator.StopIntaking();
    else {
         std::cout << "ERROR: Unrecognised mechanism command " << action << "\n";
    }

}

