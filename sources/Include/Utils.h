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

#include <string>
//#include <bits/types/struct_timespec.h>

namespace utils {

void delay_ms(int ms);

double get_distance(double x1, double y1, double z1, double x2, double y2, double z2);

void printf(std::string &dst, const char *format, va_list ap);
void printf(std::string &dst, const char *format, ...);
std::string printf(const char *format, ...);

std::string get_trim(std::string s);

template <typename T>
T constrain(T x, T min, T max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

class Time
{
public:
    Time() { ReInit(); }

    void ReInit() { m_time = GetNow(); }
    static timespec GetNow();
    int GetPassedInMS();

private:
    timespec m_time;
};

} // utils
