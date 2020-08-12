/////////////////////////////////////////////////////////////////////////////////
// MIT License
//
// Copyright (c) 2020 RoboLab19
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <iterator>
#include <algorithm>

#include "LegV2.h"
#include "IK.h"
#include "Utils.h"

const double kWheelRatio = 2.0;

LegV2::LegV2(std::shared_ptr<LX16a> lx16a, int servoIDs[3], bool inverseX) :
    Leg(lx16a, inverseX)
{
    memcpy(m_servoIDs, servoIDs, sizeof(m_servoIDs));
}

bool LegV2::set_servo_offsets(int offsets[3])
{
    for (int i = 0; i < 3; ++i) {
        if (!m_lx16a->set_offset(m_servoIDs[i], offsets[i])) {
            printf("Failed to set offset %d for servo %d!\n", offsets[i], m_servoIDs[i]);
            return false;
        }
    }
    return true;
}

static bool angle_to_servo_pos(int servoId, double angle, int& pos, bool silent)
{
     if (std::isnan(angle)) {
        if (!silent) printf("angle_to_servo_pos(servoId=%d): angle %f is NAN\n", servoId, angle);
        return false;
    }

    switch (servoId) {
        case  1: pos = 500  - IK::toDeg(angle       ) * kDegToServo * kWheelRatio;       break;
        case  2: pos = 750  + IK::toDeg(angle - PI/2) * kDegToServo * kWheelRatio;       break;
        case  3: pos = 1200 - IK::toDeg(angle       ) * kDegToServo * kWheelRatio;       break;

        case  4: pos = 500  - IK::toDeg(-angle      ) * kDegToServo * kWheelRatio;       break;
        case  5: pos = 250  - IK::toDeg(angle - PI/2) * kDegToServo * kWheelRatio;       break;
        case  6: pos =        IK::toDeg(angle       ) * kDegToServo * kWheelRatio - 200; break;

        case  7: pos = 500  + IK::toDeg(angle       ) * kDegToServo * kWheelRatio;       break;
        case  8: pos = 800  + IK::toDeg(angle - PI/2) * kDegToServo * kWheelRatio;       break;
        case  9: pos = 1200 - IK::toDeg(angle       ) * kDegToServo * kWheelRatio;       break;

        case 10: pos = 500  + IK::toDeg(-angle      ) * kDegToServo * kWheelRatio;       break;
        case 11: pos = 250  - IK::toDeg(angle - PI/2) * kDegToServo * kWheelRatio;       break;
        case 12: pos =        IK::toDeg(angle       ) * kDegToServo * kWheelRatio - 200; break;

        default:
            printf("angle_to_servo_pos(): Unknown servo id %d\n", servoId);
            return false;
    }

    //silent = false;
    if (pos < 0) {
        if (!silent) printf("Servo %d out of range (%d), adjust to 0\n", servoId, pos);
        pos = 0;
        return false;
    }
    else if (pos > 1000) {
        if (!silent) printf("Servo %d out of range (%d), adjust to 1000\n", servoId, pos);
        pos = 1000;
        return false;
    }

    return true;
}

bool LegV2::move(double angles[3], int move_time, bool simulate)
{
    int pos[3];
    for (int i = 0; i < 3; ++i) {
        if (!angle_to_servo_pos(m_servoIDs[i], angles[i], pos[i], simulate)) return false;
    }

    if (!simulate) {
        for (int i = 0; i < 3; ++i) {
            if (!m_lx16a->move(m_servoIDs[i], pos[i], move_time)) return false;
        }
    }

    return true;
}

