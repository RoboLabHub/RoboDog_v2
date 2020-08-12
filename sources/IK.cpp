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
#include "IK.h"

IK::IK(double link1, double link2, double link3) :
    m_L1(link1), m_L2(link2), m_L3(link3)
{

}

void IK::inverseKinematics(double x, double y, double z, double angles[3])
{
    double x2 = x * x;
    double y2 = y * y;
    double z2 = z * z;

    double L1_2 = m_L1 * m_L1;
    double L2_2 = m_L2 * m_L2;
    double L3_2 = m_L3 * m_L3;

    double D = (x2 + y2 - L1_2 + z2 - L2_2 - L3_2) / (2 * m_L2 * m_L3);

    double Q1 =-atan2l(-y, x) - atan2l(sqrt(x2 + y2 - L1_2), -m_L1);
    double Q3 = atan2l(sqrt(1 - D*D), D);
    double Q2 = atan2l(z, sqrt(x2 +y2 - L1_2)) - atan2l(m_L3 * sin(Q3), m_L2 + m_L3 * cos(Q3));

    angles[0] = -Q1;
    angles[1] = -Q2;
    angles[2] = PI - Q3;
}

void IK::vectorToEuler(double x, double y, double z, double& pitch, double& yaw)
{
    // if (x == 0) x = 0.00001;
    // if (y == 0) y = 0.00001;
    // if (z == 0) z = 0.00001;

    pitch = atan2(z, x);
    yaw   = atan2(y, x);
}

// http://planning.cs.uiuc.edu/node102.html
// A roll  is a counterclockwise rotation of gamma about the x-axis.
// A pitch is a counterclockwise rotation of beta  about the y-axis.
// A yaw   is a counterclockwise rotation of alpha about the z-axis.
IK::Vector IK::rotate(IK::Vector pos, IK::Vector rot)
{
    double y = rot.x; // roll
    double b = rot.y; // pitch
    double a = rot.z; // yaw

    double ca = cos(a);
    double cb = cos(b);
    double cy = cos(y);

    double sa = sin(a);
    double sb = sin(b);
    double sy = sin(y);

    double m11 = ca * cb;
    double m12 = ca * sb * sy - sa * cy;
    double m13 = ca * sb * cy + sa * sy;

    double m21 = sa * cb;
    double m22 = sa * sb * sy + ca * cy;
    double m23 = sa * sb * cy - ca * sy;

    double m31 = -sb;
    double m32 = cb * sy;
    double m33 = cb * cy;

    IK::Vector out;
    out.x = m11 * pos.x + m12 * pos.y + m13 * pos.z;
    out.y = m21 * pos.x + m22 * pos.y + m23 * pos.z;
    out.z = m31 * pos.x + m32 * pos.y + m33 * pos.z;

    return out;
}

double IK::toRad(double a) { return a * degtorad; }
double IK::toDeg(double a) { return a * radtodeg; }
