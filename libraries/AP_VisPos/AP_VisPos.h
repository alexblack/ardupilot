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
#include <DataFlash/DataFlash.h>
#include <AP_AHRS/AP_AHRS.h>

#define VISPOS_TYPE_HIL    0
#define VISPOS_TYPE_MAV    1

class AP_VisPos_Backend;
class AP_VisPos_HIL;

class AP_VisPos
{
public:
   friend class AP_VisPos_Backend;
   friend class AP_VisPos_HIL;
   friend class AP_VisPos_MAVLink;

   AP_VisPos(DataFlash_Class &dataflash,AP_AHRS_NavEKF &ahrs);

   //Initialise user selected visual position
   void init(void);

   void handle_raw_vispos_report(mavlink_channel_t chan, mavlink_message_t *msg);

   // Reference Frame(wrt Vehicle Frame): X-axis facing front, Y-axis Right and Z axis Down
   Vector3f get_local_pos();

   void setHIL(Vector3f raw_pos, uint32_t timestamp_ms);

   bool enabled() { return _enable; }

   Vector3f get_target_pos();

   float get_target_yaw();
   // parameter var info table
   static const struct AP_Param::GroupInfo var_info[];

private:
   AP_Int8 _vispos_type;
   AP_Int8 _enable;
   AP_VisPos_Backend* _backend;
   AP_AHRS_NavEKF &_ahrs;
   DataFlash_Class &_dataflash;
};