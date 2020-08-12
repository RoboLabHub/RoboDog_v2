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
#include <memory>
#include <serial/serial.h>

class LX16a
{
public:
    LX16a() : m_unitTestRun(false) {}
    virtual ~LX16a() {
    }

    bool open(const std::string port, serial::Timeout timeout = serial::Timeout::simpleTimeout(1000));

    bool move(uint8_t id, uint16_t angle, uint16_t time = 0);
    bool set_offset(int8_t id, uint8_t angle);

    bool get_pos(uint8_t id, int16_t& pos);

    bool get_id(uint8_t& id);

    bool get_temp(uint8_t id, uint8_t& temp);
    bool get_vin(uint8_t id, uint16_t& vin);

private:
    bool send_packet(uint8_t id, int8_t cmd, const uint8_t* params = nullptr, size_t param_cnt = 0);
    bool read_packet(uint8_t id, uint8_t cmd, uint8_t *params = nullptr, size_t param_cnt = 0);

    bool read_packet_with_retries(uint8_t id, uint8_t cmd, uint8_t *params, size_t param_cnt, int retries);

    uint8_t read_byte();

    std::shared_ptr<serial::Serial> m_serial;
    bool m_unitTestRun;
};
