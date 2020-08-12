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

#include <iostream>
#include <serial/impl/unix.h>

#include "Utils.h"
#include "LX16a.h"

//#define DEBUG_LOG

using namespace std;

bool LX16a::open(const std::string port, serial::Timeout timeout)
{
    if (port.length() == 0) {
        m_unitTestRun = true;
        return true;    // For unit tests
    }

    try {
        m_serial = std::make_shared<serial::Serial>(port, 115200, timeout);
        return m_serial->isOpen();
    }
    catch (serial::PortNotOpenedException ex) {
        cout << "serial::PortNotOpenedException: " << port.c_str() << "\n";
    }
    catch (serial::IOException ex) {
        cout << "serial::IOException: " << port.c_str() << "\n";
    }

    return false;
}

uint8_t LX16a::read_byte()
{
    if (!m_serial) return m_unitTestRun;

    uint8_t byte;
    m_serial->read(&byte, 1);
    return byte;
}

bool LX16a::send_packet(uint8_t id, int8_t cmd, const uint8_t* params, size_t param_cnt)
{
   if (!m_serial) return m_unitTestRun;

   if (param_cnt > 4)
        return false;

    int buflen = 6 + param_cnt;
    uint8_t buf[buflen];
    buf[0] = 0x55;
    buf[1] = 0x55;
    buf[2] = id;
    buf[3] = buflen-3;
    buf[4] = cmd;

    for (int i = 0; i < param_cnt; i++)
        buf[5+i] = params[i];

    uint8_t cksum = 0;
    for (int i = 2; i < buflen-1; i++)
        cksum += buf[i];

    buf[buflen-1] = ~cksum;

#ifdef DEBUG_LOG
        printf("SEND: ");
        for (int i = 0; i < buflen; i++)
            printf(" %02X", buf[i]);
        printf("\n");
#endif

    try {
        // Clear input buffer
        while (m_serial->available())
            read_byte();

        // Send command packet
        return (m_serial->write(buf, buflen) == buflen);
    }
    catch (serial::PortNotOpenedException ex) {
        cout << "serial::PortNotOpenedException\n";
    }
    catch (serial::IOException ex) {
        cout << "serial::IOException\n";
    }
    catch (serial::SerialException ex) {
        cout << "serial::SerialException\n";
    }

    return false;
}

bool LX16a::read_packet(uint8_t id, uint8_t cmd, uint8_t *params, size_t param_cnt)
{
    if (!m_serial) return m_unitTestRun;

#ifdef DEBUG_LOG
    printf("RECV: ");
#endif

    int got = 0;
    int len = 7; // Minimum length
    uint8_t sum = 0;

    serial::MillisecondTimer timeoutTimer(15*(param_cnt+6) + 20); // +20ms for the servo to think
    while (got < len && timeoutTimer.remaining() > 0)
    {
        if (m_serial->available())
        {
            uint8_t byte;
            try {
                byte = read_byte();
            }
            catch (serial::PortNotOpenedException ex) {
                cout << "serial::PortNotOpenedException\n";
                return false;
            }
            catch (serial::IOException ex) {
                cout << "serial::IOException\n";
                return false;
            }
            catch (serial::SerialException ex) {
                cout << "serial::SerialException\n";
                return false;
            }
#ifdef DEBUG_LOG
            printf(" %02X", byte);
#endif
            switch (got)
            {
            case 0:
            case 1:
                if (byte != 0x55)
                {
#ifdef DEBUG_LOG
                    printf(" ERR (hdr)\n");
#endif
                    return false;
                }
                break;
            case 2:
                if (byte != id && id != 0xfe)
                {
#ifdef DEBUG_LOG
                    printf(" ERR (id)\n");
#endif
                    return false;
                }
                break;
            case 3:
                if (byte < 3 || byte > 7) {
#ifdef DEBUG_LOG
                    printf(" ERR (len)\n");
#endif
                    return false;
                }
                len = byte + 3;
                if (len > param_cnt+6) {
#ifdef DEBUG_LOG
                    printf(" ERR (param_len)\n");
#endif
                    return false;
                }
                break;
            case 4:
                if (byte != cmd) {
#ifdef DEBUG_LOG
                    printf(" ERR (cmd)\n");
#endif
                    return false;
                }
                break;
            default:
                if (got == len-1) {
                   if (byte == (uint8_t)~sum) {
#ifdef DEBUG_LOG
                        printf(" OK\n");
#endif
                        return true;
                    }
                   else {
#ifdef DEBUG_LOG
                       printf(" ERR (cksum!=%02x)\n", ~sum);
#endif
                       return false;
                    }
                }
                if (got-5 > param_cnt) {
#ifdef DEBUG_LOG
                    printf(" ERR (cksum)\n");
#endif
                    return false;
                }
                params[got-5] = byte;
            }

            if (got > 1)
                sum += byte;
            got++;
        }
    }
#ifdef DEBUG_LOG
    printf(" TIMEOUT\n");
#endif
    return false;
}

bool LX16a::read_packet_with_retries(uint8_t id, uint8_t cmd, uint8_t *params, size_t param_cnt, int retries)
{
    for (int i = 0; i < retries; ++i) {
        if (read_packet(id, cmd, params, param_cnt)) return true;
    }
    return false;
}

// 0..1000
bool LX16a::move(uint8_t id, uint16_t angle, uint16_t time)
{
    uint8_t params[4] = { (uint8_t)angle, (uint8_t)(angle>>8), (uint8_t)time, (uint8_t)(time>>8) };

    if (!send_packet(id, 1, params, sizeof(params)))
        return false;

    return true;
}

// -125...125
bool LX16a::set_offset(int8_t id, uint8_t angle)
{
    uint8_t params[1] = { (uint8_t)angle };

    if (!send_packet(id, 17, params, sizeof(params)))
        return false;

    if (!send_packet(id, 18, params, 0))
        return false;

    utils::delay_ms(100);

    return true;
}

bool LX16a::get_pos(uint8_t id, int16_t& pos)
{
    int i = 0;
    uint8_t params[2];
    for (; i < 3; ++i) {
        if (!send_packet(id, 28)) {
            continue;
        }
        if (!read_packet(id, 28, params, sizeof(params))) {
            continue;
        }
        break;
    }

    if (i == 3) {
        printf("get_pos: read_packet(id=%d) failed\n", id);
        return false;
    }

    pos = ( (int16_t)params[0] | ((int16_t)params[1]<<8) );
    return true;
}

bool LX16a::get_id(uint8_t& id)
{
    if (!send_packet(0xFE, 14))
        return false;

    uint8_t params[1];
    if (!read_packet(0xFE, 14, params, sizeof(params)))
        return false;

    id = params[0];
    return true;
}

bool LX16a::get_temp(uint8_t id, uint8_t& temp)
{
    if (!send_packet(id, 26))
        return false;

    uint8_t params[1];
    if (!read_packet(id, 26, params, sizeof(params)))
        return false;

    temp = params[0];
    return true;
}

bool LX16a::get_vin(uint8_t id, uint16_t& vin)
{
    if (!send_packet(id, 27))
        return false;

    uint8_t params[2];
    if (!read_packet(id, 27, params, sizeof(params)))
        return false;

    vin = ( (int16_t)params[0] | ((int16_t)params[1]<<8) );
    return true;
}
