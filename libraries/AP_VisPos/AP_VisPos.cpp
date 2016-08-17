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

#include <AP_VisPos/AP_VisPos.h>
#include "AP_VisPos_HIL.h"
#include "AP_VisPos_MAVLink.h"
#include "AP_VisPos_Backend.h"
#include <stdio.h>
extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_VisPos::var_info[] = {
   // @Param: _ENABLE
   // @DisplayName: Visual Positioning enable/disable
   // @Description: Setting this to Enabled(1) will enable visual positioning. Setting this to Disabled(0) will disable visual positioning
   // @Values: 0/1
   // @User: Standard
   AP_GROUPINFO("_ENABLE", 0,  AP_VisPos,    _enable,   1),

   // @Param: _TYPE
   // @DisplayName: Visual Pos Sensor Type
   // @Description: Selecting type of Vis Pos sensor
   // @Values: 0:VISPOS_TYPE_HIL, 1:VISPOS_TYPE_MAV
   // @User: Standard
   AP_GROUPINFO("_TYPE", 1,  AP_VisPos,    _vispos_type,   0),

   AP_GROUPEND
};
AP_VisPos::AP_VisPos(DataFlash_Class &dataflash, AP_AHRS_NavEKF &ahrs) :
_dataflash(dataflash),
_ahrs(ahrs)
{
    _backend = NULL;
}

//Initialise user selected visual position
void AP_VisPos::init(void)
{
   printf("Initialising Visual Positioning....\n");
	if(_vispos_type == VISPOS_TYPE_HIL) {
		_backend = new AP_VisPos_HIL(this);
	} else if (_vispos_type == VISPOS_TYPE_MAV) {
      _backend = new AP_VisPos_MAVLink(this);
   }
}

void AP_VisPos::handle_raw_vispos_report(mavlink_channel_t chan, mavlink_message_t *msg)
{
   if (_backend) {
   	_backend->handle_raw_vispos_report(chan, msg);
   }
}

// Reference Frame(wrt Vehicle Frame): X-axis facing front, Y-axis Right and Z axis Down
Vector3f AP_VisPos::get_local_pos()
{
   if (_backend) {
      return _backend->get_local_pos();
   } else {
      return Vector3f(0,0,0);
   }
}

Vector3f AP_VisPos::get_target_pos() {
   return _ahrs.get_NavEKF2().get_vp_target_pos();
}

float AP_VisPos::get_target_yaw() {
   if(_backend) {
      return _backend->get_target_yaw();
   }
   return 0.0f;
}

void AP_VisPos::setHIL(Vector3f raw_pos, uint32_t timestamp_ms)
{
   if (_backend) {
   	_backend->set_local_pos(raw_pos);
   	_backend->set_last_pos_msg_time_ms(timestamp_ms);
      Vector3f rotated_pos;
      
      Matrix3f rot_mat( 0.0, 0.0,  1.0,
                        1.0, 0.0,  0.0,
                        0.0, 1.0,  0.0);
   /*
      Matrix3f rot_mat( 1.0, 0.0,  0.0,
                        0.0, 1.0,  0.0,
                        0.0, 0.0,  1.0);*/
      rotated_pos = rot_mat*raw_pos;
      _ahrs.get_NavEKF2().writeVisPosMeas(Vector3f(rotated_pos.x,rotated_pos.y,rotated_pos.z), timestamp_ms);
   }
}
