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

#pragma once

class GCodeController;

#include "IK.h"

const double kMaxSpeed = 0.12; // m/s

class RobotState
{
public:
    struct Leg {
        Leg() {} //: onGround(true) {}

        IK::Vector pos;
        //bool   onGround;
    };

    RobotState(GCodeController& gcodeCtrl);

    bool Init();

    IK::Vector GetMoveVector (int legId, IK::Vector pos);
    IK::Vector GetPosByVector(int legId, IK::Vector v);

    int GetMoveTimeInMS(IK::Vector v, double speed);

    bool MoveLeg(int legId, IK::Vector pos, double speed);

    bool MoveBase(IK::Vector   v, double speed); // Relative move
    bool SetBase (IK::Vector pos, double speed); // Absolute move

    IK::Vector GetLegPositionInBaseFrame(int legId, IK::Vector pos);
    IK::Vector GetLegPositionInLegFrame (int legId, IK::Vector pos);

    IK::Vector GetAdjustedLegPosition(int legId, IK::Vector pos);

    void WaitForMove(double k = 1.0);

    bool GetCurrentPos();

public:
    IK::Vector m_speed; // m/s
    Leg m_leg[4];           // Position in leg frame
    Leg m_legAdjusted[4];   // Position after adjusting to base rotation and offset
    bool m_legToAdjust[4];

    double m_baseLength, m_baseWidth;

    IK::Vector m_rot; // Base rotation in rad
    IK::Vector m_baseOffset;

    int m_moveTime; // in ms

    GCodeController& m_gcodeCtrl;
};
