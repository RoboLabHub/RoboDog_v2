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

#include "MoveParams.h"

class GCodeController;

class MovePrimitives
{
public:
    MovePrimitives(GCodeController& ctrl): m_ctrl(ctrl) {}

    bool run_primitive(std::string cmd, std::map<int, int> params, bool simulate);

    bool set_pose(int posNum);

    std::string swing_leg(int legId, int height, int dist, double speedMultiply);
    std::string swing_leg_2(int legId1, int legId2, int height, int dist, double speedMultiply);

    bool walk     (int steps, bool forward);
    bool walk_2   (int steps, bool forward);
    bool side_walk(int steps, bool move_right);

    bool rotate(int steps, bool clockwise);

    bool forward_jump(int steps, bool forward);
    bool Circle(int r);

    bool run_gcode(std::string gcode);
    bool run_with_flush(std::string gcode);
    bool validate_gcode(std::string gcode);

private:
    MoveParams m_params;
    GCodeController& m_ctrl;
};
