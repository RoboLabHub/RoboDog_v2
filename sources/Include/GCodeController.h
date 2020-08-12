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

#include <map>
#include <math.h>
#include <memory>

#include "Leg.h"
#include "RobotState.h"
#include "MovePrimitives.h"

#define SET_DEF_PARAM(id, val) if (paramsInt.find(id) == paramsInt.end()) paramsInt[id] = val;

class GCodeController
{
public:
    struct MovePoint {
        int legId;
        double p[3];
        int moveTime;
    };

    typedef std::map<uint8_t, std::shared_ptr<Leg>> RobotLegs;

    GCodeController(RobotLegs legs);
    void reset();

    void set_legs_base(double length, double width);

    bool run(const std::string gcode, bool simulate = false);
    bool do_move(bool simulate);

    bool run_from_file(const std::string& file);

    bool add_point(int legId, IK::Vector pos, int moveTime);
    bool look_at(IK::Vector vec);

public:
    int m_speedInPercent;

    RobotState     m_robotState;
    MovePrimitives m_primitives;

    RobotLegs m_legs;

    std::vector<MovePoint> m_movePoints;
};
