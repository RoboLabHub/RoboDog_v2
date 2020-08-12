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

const double PI = 3.141592653589793238462;

#define half_pi     (PI / 2.0)
#define degtorad    (PI / 180.0)
#define radtodeg    (180.0 / PI)

class IK
{
public:
    struct Vector {
        Vector() : x(0), y(0), z(0) {}

        double x, y, z;
    };

    IK(double link1, double link2, double link3);

    void inverseKinematics(double x, double y, double z, double angles[3]);

    static void vectorToEuler(double x, double y, double z, double& pitch, double& yaw);

    static IK::Vector rotate(IK::Vector pos, IK::Vector rot);

    static double toRad(double a);
    static double toDeg(double a);

public:
    double m_L1, m_L2, m_L3;
};
