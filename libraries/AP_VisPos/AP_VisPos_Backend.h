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

#include <AP_HAL/AP_HAL.h>
#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_VisPos.h"

class AP_VisPos_Backend
{
public:

   AP_VisPos_Backend(AP_VisPos *vispos):
   _frontend(vispos) {}

   //Initialise user selected visual position
   virtual void init(void) {}

   virtual void handle_raw_vispos_report(mavlink_channel_t chan, mavlink_message_t *msg) {}

   virtual Vector3f get_local_pos(void) { return _local_pos; }

   virtual Vector3f get_global_pos(void) { return _global_pos; }

   virtual uint32_t get_last_pos_time(void) { return _last_pos_msg_time_ms; }

   virtual void set_local_pos(Vector3f lpos) { _local_pos = lpos; }

   virtual void set_last_pos_msg_time_ms(uint32_t ts_ms) { _last_pos_msg_time_ms = ts_ms; }

   virtual float get_target_yaw() { return 0.0f; }
protected:

   AP_VisPos* _frontend;

   Vector3f _local_pos;    // in meters

   Vector3f _global_pos;    // in meters

   uint32_t _last_pos_msg_time_ms;

   Vector3f _pos_acc;    // in the form of variance

   bool _initialised;
};