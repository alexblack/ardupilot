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
#include "AP_VisPos_MAVLink.h"
extern const AP_HAL::HAL& hal;

void AP_VisPos_MAVLink::handle_raw_vispos_report(mavlink_channel_t chan, mavlink_message_t *msg)
{
	mavlink_local_position_ned_t pkt;
	mavlink_msg_local_position_ned_decode(msg, &pkt);
	struct log_VPOS pkt_vispos = {
	    LOG_PACKET_HEADER_INIT(LOG_VPOS_MSG),
	    time_us 		:	AP_HAL::micros(),
	    sample_time_us	:	pkt.time_boot_ms,
	    x				:	pkt.x,
	    y 			:	pkt.y,
	    z				:	pkt.z,
	    roll		:	pkt.vx,
	    pitch		:	pkt.vy,
	    yaw			:	pkt.vz
    };
    Vector3f rotated_pos;
    Matrix3f rot_mat( 0.0, 0.0,  1.0,
    				          1.0, 0.0,  0.0,
                      0.0, 1.0,  0.0);
    rotated_pos = rot_mat*Vector3f(pkt.x,pkt.y,pkt.z);
    _frontend->_dataflash.WriteBlock(&pkt_vispos, sizeof(pkt_vispos));
	_frontend->_ahrs.get_NavEKF2().writeVisPosMeas(Vector3f(rotated_pos.x,rotated_pos.y,rotated_pos.z), pkt.time_boot_ms);
}
