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
#include <algorithm>

#include <unistd.h>
#include <time.h>

#include <cstdarg>

#include "Utils.h"

namespace utils {

void delay_ms(int ms)
{
  usleep(ms * 1000);
}

double get_distance(double x1, double y1, double z1, double x2, double y2, double z2)
{
  return sqrt( pow(x2-x1, 2) + pow(y2-y1, 2) + pow(z2-z1, 2) );
}

void printf(std::string &dst, const char *format, va_list ap)
{
  int length;
  va_list apStrLen;
  va_copy(apStrLen, ap);
  length = vsnprintf(NULL, 0, format, apStrLen);
  va_end(apStrLen);
  if (length > 0) {
    dst.resize(length);
    vsnprintf((char *)dst.data(), dst.size() + 1, format, ap);
  }
}

void printf(std::string &dst, const char *format, ...)
{
  va_list ap;
  va_start(ap, format);
  utils::printf(dst, format, ap);
  va_end(ap);
}

std::string printf(const char *format, ...)
{
  std::string dst;
  va_list ap;
  va_start(ap, format);
  utils::printf(dst, format, ap);
  va_end(ap);
  return dst;
}

std::string get_trim(std::string s)
{
  s.erase(s.begin(), std::find_if_not(s.begin(), s.end(), [](char c){ return std::isspace(c); }));
  s.erase(std::find_if_not(s.rbegin(), s.rend(), [](char c){ return std::isspace(c); }).base(), s.end());
  return s;
}

int Time::GetPassedInMS()
{
  timespec now(GetNow());
  __int64_t millis = (now.tv_sec - m_time.tv_sec) * 1e3;
  millis += (now.tv_nsec - m_time.tv_nsec) / 1e6;
  return millis;
}

timespec Time::GetNow()
{
  timespec time;
  clock_gettime(CLOCK_MONOTONIC, &time);
  return time;
}

} // utils
