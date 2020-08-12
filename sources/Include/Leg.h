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

#include "LX16a.h"
#include "IK.h"

const double kDegToServo = 1.0 / 0.24;

class Leg
{
public:
    Leg(std::shared_ptr<LX16a> lx16a, bool inverseX);

    void set_leg_location(double x, double y, double z); // In base frame (in m)

    void set_geometry(double link1, double link2, double link3);

    virtual bool move(double angles[3], int move_time, bool simulate = false) = 0;

    bool move(double x, double y, double z, int move_time = 1000, bool simulate = false);

    void tune_pos_1(bool useIK = false);
    void tune_pos_2(bool useIK = false);

public:
    double m_x, m_y, m_z;
    double m_x0, m_y0, m_z0;
    bool m_inverseX;

    std::shared_ptr<IK>    m_ik;
    std::shared_ptr<LX16a> m_lx16a;
};
