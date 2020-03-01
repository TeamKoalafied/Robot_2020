//==============================================================================
// DistanceSensor.cpp
//
// From: https://github.com/REVrobotics/2m-Distance-Sensor/
//       blob/master/Examples/C%2B%2B/Read%20Range/src/main/cpp/Robot.cpp
//==============================================================================

#include "DistanceSensor.h"

#include "../../RobotConfiguration.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include "rev/Rev2mDistanceSensor.h"



//==============================================================================
// Construction

DistanceSensor::DistanceSensor()  {
}

DistanceSensor::~DistanceSensor() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void DistanceSensor::Setup() {
    std::cout << "DistanceSensor::Setup()\n";
    RevSensor = new rev::Rev2mDistanceSensor{rev::Rev2mDistanceSensor::Port::kOnboard, 
                                                rev::Rev2mDistanceSensor::DistanceUnit::kInches};
    RevSensor->SetAutomaticMode(true);
    RevSensor->SetEnabled(true);
}

void DistanceSensor::Shutdown() {
    std::cout << "DistanceSensor::Shutdown()\n";
}

void DistanceSensor::Periodic(bool show_dashboard)
{   
    // if (!RevSensor->IsRangeValid()) {
    //     std::cout << "splippity boop" << std::endl;

    //     delete RevSensor;
    //     RevSensor = new rev::Rev2mDistanceSensor{rev::Rev2mDistanceSensor::Port::kOnboard, 
    //                                             rev::Rev2mDistanceSensor::DistanceUnit::kInches};
    //     std::cout << "boppity splip" << std::endl;
    //     RevSensor->SetAutomaticMode(true);
    //     RevSensor->SetEnabled(true);
    // }

    
    if (show_dashboard) {
        bool isValid = RevSensor->IsRangeValid();

        frc::SmartDashboard::PutBoolean("Distance Sensor Valid", isValid);

        if (isValid) {
            frc::SmartDashboard::PutNumber("RevDistance (in)", RevSensor->GetRange());

            frc::SmartDashboard::PutNumber("Distance Timestamp", RevSensor->GetTimestamp());
        } else {
            frc::SmartDashboard::PutNumber("RevDistance (in)", -1);
        }
    }
}

double DistanceSensor::GetDistance() {
    return RevSensor->GetRange();
}