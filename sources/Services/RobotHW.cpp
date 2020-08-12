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

#include "LegV2.h"
#include "Utils.h"
#include "GCodeController.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"

GCodeController* g_ctrl = nullptr;

std::shared_ptr<GCodeController> CreateRobotV2(std::shared_ptr<LX16a> lx16a)
{
    int servoIDs1[] = {  1,  2,  3 };
    int servoIDs2[] = {  4,  5,  6 };
    int servoIDs3[] = {  7,  8,  9 };
    int servoIDs4[] = { 10, 11, 12 };

    auto leg1 = std::make_shared<LegV2>(lx16a, servoIDs1, false);
    auto leg2 = std::make_shared<LegV2>(lx16a, servoIDs2,  true);
    auto leg3 = std::make_shared<LegV2>(lx16a, servoIDs3, false);
    auto leg4 = std::make_shared<LegV2>(lx16a, servoIDs4,  true);

    int offsets1[] = { -50,   10,  40 };
    int offsets2[] = {  20,   0, -50 };
    int offsets3[] = { -20, -30,  -60 };
    int offsets4[] = { -50,  -10, -80 };

    leg1->set_servo_offsets(offsets1);
    leg2->set_servo_offsets(offsets2);
    leg3->set_servo_offsets(offsets3);
    leg4->set_servo_offsets(offsets4);

    leg1->set_geometry(0.043, 0.165, 0.187); // + leg foot ball (20mm radius)
    leg2->set_geometry(0.043, 0.165, 0.187);
    leg3->set_geometry(0.043, 0.165, 0.187);
    leg4->set_geometry(0.043, 0.165, 0.187);

    std::map<uint8_t, std::shared_ptr<Leg>> legs;
    legs[1] = leg1;
    legs[2] = leg2;
    legs[3] = leg3;
    legs[4] = leg4;

    auto ctrl = std::make_shared<GCodeController>(legs);
    ctrl->set_legs_base(0.330, 0.223); //0.140);

    return ctrl;
}

int main(int argc, char **argv)
{
    auto lx16a = std::make_shared<LX16a>();
    if (!lx16a->open("/dev/ttyUSB0")) return -1;

    auto ctrl = CreateRobotV2(lx16a);
    g_ctrl = ctrl.get();

    // Test code, comment it after robot tune up completion
    for (int i = 1; i <= 4; ++i) {
        ctrl->m_legs[i]->tune_pos_1(false);
        //ctrl->m_legs[i]->tune_pos_2(false);
    }

    // Uncomment it to run test gcode
    //ctrl->run_from_file("src/RoboDog_v2/sources/GCODE/test.gcode");

    return 1;
}
