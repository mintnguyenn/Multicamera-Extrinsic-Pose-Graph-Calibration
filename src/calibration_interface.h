#ifndef CALIBRATION_INTERFACE_H
#define CALIBRATION_INTERFACE_H

#include "camera.h"

class ExtrinsicCalibrationInterface{
public:
    ExtrinsicCalibrationInterface(){}; // Default constructor

    virtual void threads() = 0;

};

#endif // CALIBRATION_INTERFACE_H