// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <AP_VisPos/AP_VisPos_Backend.h>

class AP_VisPos_MAVLink : public AP_VisPos_Backend
{
public:
   AP_VisPos_MAVLink(AP_VisPos *vispos):
   AP_VisPos_Backend(vispos) {}
   virtual void handle_raw_vispos_report(mavlink_channel_t chan, mavlink_message_t *msg);
   float get_target_yaw() {return target_yaw; }
private:
   float target_yaw;
};
