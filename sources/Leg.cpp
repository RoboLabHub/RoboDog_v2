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

#include "Leg.h"
#include "Utils.h"

Leg::Leg(std::shared_ptr<LX16a> lx16a, bool inverseX) :
    m_lx16a(lx16a),
    m_inverseX(inverseX),
    m_x(0), m_y(0), m_z(0),
    m_x0(0), m_y0(0), m_z0(0)
{
}

void Leg::set_leg_location(double x, double y, double z)
{
    m_x0 = x;
    m_y0 = y;
    m_z0 = z;
}

void Leg::set_geometry(double link1, double link2, double link3)
{
    m_ik = std::make_shared<IK>(link1, link2, link3);
}

bool Leg::move(double x, double y, double z, int move_time, bool simulate)
{
    double adjustedX = m_inverseX ? (-x - m_ik->m_L1) : (x - m_ik->m_L1);

    double angles[3];
    m_ik->inverseKinematics(adjustedX, y, z, angles);

    bool ret = move(angles, move_time, simulate);

    // If move succeeded, then save the leg's new position
    if (ret && !simulate) {
        m_x = x;
        m_y = y;
        m_z = z;
    }

    return ret;
}

// Upper leg down, lower leg forward (parallel to body)
void Leg::tune_pos_1(bool useIK)
{
    if (useIK) {
        move(0, m_ik->m_L2, m_ik->m_L3, 1000);
    }
    else {
        double angles[] = { 0, 0, PI/2 };
        move(angles, 1000);
    }
}

// Upper leg backward (parallel to body), lower leg down
void Leg::tune_pos_2(bool useIK)
{
    if (useIK) {
        move(0, m_ik->m_L3, -m_ik->m_L2, 1000);
    }
    else {
        double angles[] = { 0, PI/2, PI/2 };
        move(angles, 1000);
    }
}
