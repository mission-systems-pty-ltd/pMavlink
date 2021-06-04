/*****************************************************************/
/*    NAME: David Battle                                         */
/*    ORGN: Mission Systems Pty Ltd                              */
/*    (based on pMavlinkConverter by Muthukumaran Chandrasekaran)*/
/*    FILE: translation.cpp                                      */
/*    DATE: 14 Dec 2018                                          */
/*****************************************************************/
#include <stdio.h>
#include "Mavlink.h"
#include "translation.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"

using namespace std;
using json = nlohmann::json;

uint64_t get_time_usec() {
  static struct timeval _time_stamp;
  gettimeofday(&_time_stamp, NULL);
  return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

void Mavlink::TranslateToMoos(mavlink_message_t* msg)
{
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT:

        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(msg, &hb);

        //fprintf(stdout, "HEARTBEAT - Hello Drone World!\n");
        //fprintf(stdout, "    Autopilot: ");
        switch (hb.autopilot) {
        case MAV_AUTOPILOT_ARDUPILOTMEGA:
            fprintf(stdout, "APM\n");
            break;
        case MAV_AUTOPILOT_PX4:
            fprintf(stdout, "PX4\n");
            break;
        default:
            fprintf(stdout, "UNKNOWN\n");
            break;
        }

        // armed_state = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
        // fprintf(stdout, "    Armed? %s.\n", armed_state ? "Yes" : "No");
        break;

    case MAVLINK_MSG_ID_SYS_STATUS:
        {
            float bat_remaining = mavlink_msg_sys_status_get_battery_remaining(msg);
            Notify("BAT_REMAINING", bat_remaining);
        }
        break;

    case MAVLINK_MSG_ID_SYSTEM_TIME:
        {
            double moosTime = m_moosTimePX4DATA;
            uint64_t unix_time_us = mavlink_msg_system_time_get_time_unix_usec(msg);
            uint32_t boot_time_ms = mavlink_msg_system_time_get_time_boot_ms(msg);
            m_boot_pxunix_offset_ms = ((double)unix_time_us)/1000.0 - ((double)boot_time_ms);
            m_moosunix_pxunix_offset_s = moosTime - ((double)unix_time_us)/1000000.0;
            initial_sys_time_received = true;

            // Calculate the average of the pxunix to moosunix connection
            if (m_number_moosunix_pxunix_offset_s_to_ignore > 0) {
                --m_number_moosunix_pxunix_offset_s_to_ignore;
                m_moosunix_pxunix_offset_average_s = m_moosunix_pxunix_offset_s;
                break;
            } else if (m_number_moosunix_pxunix_offset_s_receive == 0) {
                m_moosunix_pxunix_offset_average_s = m_moosunix_pxunix_offset_s;
                ++m_number_moosunix_pxunix_offset_s_receive;
            } else {
                m_moosunix_pxunix_offset_average_s = (m_number_moosunix_pxunix_offset_s_receive*m_moosunix_pxunix_offset_average_s + m_moosunix_pxunix_offset_s)/(++m_number_moosunix_pxunix_offset_s_receive);
                Notify("OFFSET_MS_MOOSUNIX_PIXHAWKUNIX_AVERAGE", m_moosunix_pxunix_offset_average_s*1000.0);
            }

            if (m_verbose_output) {
                std::cout << std::fixed << "SYSTEM_TIME Received: MOOSTime() (s)=" << moosTime << " unix_time_us (s)" << ((double)unix_time_us)/1000000.0 << " diff (s)=" << m_moosunix_pxunix_offset_s << "\n";
            }
            Notify("OFFSET_MS_PIXHAWK_BOOT_UNIX", m_boot_pxunix_offset_ms);
            Notify("OFFSET_MS_MOOSUNIX_PIXHAWKUNIX", m_moosunix_pxunix_offset_s*1000.0);
        }
        break;

    case MAVLINK_MSG_ID_SCALED_IMU:

        break;

    case MAVLINK_MSG_ID_ATTITUDE:
        {
            uint32_t boot_time_ms = mavlink_msg_attitude_get_time_boot_ms(msg);
            double unix_time_s = getCorrectedUnixTimestamp(boot_time_ms);

            float roll = mavlink_msg_attitude_get_roll(msg);
            Notify("NAV_ROLL", roll, unix_time_s);

            float pitch = mavlink_msg_attitude_get_pitch(msg);
            Notify("NAV_PITCH", pitch, unix_time_s);

            float yaw = mavlink_msg_attitude_get_yaw(msg);
            Notify("NAV_YAW", yaw, unix_time_s);
        }
        break;

    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        {
            float x = mavlink_msg_local_position_ned_get_x(msg);
            Notify("LOCAL_X_NED", x);

            float y = mavlink_msg_local_position_ned_get_y(msg);
            Notify("LOCAL_Y_NED", y);

            float z = mavlink_msg_local_position_ned_get_z(msg);
            Notify("LOCAL_Z_NED", z);
        }
        break;

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
            int32_t int_lat = mavlink_msg_global_position_int_get_lat(msg);
            double lat = ((double)int_lat) / 1.0e7;
            Notify("NAV_LAT", lat);

            int32_t int_lon = mavlink_msg_global_position_int_get_lon(msg);
            double lon = ((double)int_lon) / 1.0e7;
            Notify("NAV_LONG", lon);

            int32_t int_alt_amsl = mavlink_msg_global_position_int_get_alt(msg);
            double alt_amsl = ((double)int_alt_amsl) / 1.0e3;
            Notify("ALT_AMSL", alt_amsl);

            int32_t alt_rel = mavlink_msg_global_position_int_get_relative_alt(msg);
            Notify("ALT_REL", ((double)alt_rel) / 1.0e3);

            int16_t hdg = mavlink_msg_global_position_int_get_hdg(msg);
            Notify("NAV_HEADING", ((double)hdg) / 1.0e2);

            double nav_x, nav_y;

            // Conversion to local grid coordinates
            m_geodesy.LatLong2LocalGrid(lat, lon, nav_y, nav_x);
            Notify("NAV_X", nav_x);
            Notify("NAV_Y", nav_y);
            Notify("NAV_Z", alt_amsl);
        }
        break;

    case MAVLINK_MSG_ID_GPS_RAW_INT:
    	{
            uint64_t time_usec = mavlink_msg_gps_raw_int_get_time_usec(msg);
    		Notify("GPS_UNIX_TIME", time_usec);
			if (m_verbose_output) {
                std::cout << "GPS_RAW_INT received (time_usec = " << time_usec << ")\n";
            }
		}
        break;

    case MAVLINK_MSG_ID_HIGHRES_IMU:

        break;

    case MAVLINK_MSG_ID_MISSION_CURRENT:

        break;

    case MAVLINK_MSG_ID_ALTITUDE:
        // {
        //     float alt_amsl = mavlink_msg_altitude_get_altitude_amsl(msg);
        //     Notify("ALT_AMSL", alt_amsl);
        // }
        break;

    case MAVLINK_MSG_ID_TIMESYNC:
    	{
    		int64_t tc1 = mavlink_msg_timesync_get_tc1(msg); // written at offboard
			int64_t ts1 = mavlink_msg_timesync_get_ts1(msg); // written at pixhawk

            if (m_verbose_output) {
                std::cout << "Received timesync message (tc1: " << tc1/1000000000.0 << " s, ts1: " << ts1/1000000000.0 << " s";
            }
			if (tc1 > 0) {
				double diff = (tc1-ts1)/1000000.0;
                if (m_number_timesyncs_to_ignore > 0) {
                    --m_number_timesyncs_to_ignore;
                    if (m_verbose_output) {
                        std::cout << " -> ignored as one of first timesync messages)\n";
                    }
                    m_timesync_delay_average_ms = diff;
                    break;
                } else if (m_number_timesync_receive == 0) {
                    m_timesync_delay_average_ms = diff;
					m_smallest_diff = diff;
                    m_largest_diff = diff;
					++m_number_timesync_receive;
				} else {
					m_timesync_delay_average_ms = (m_number_timesync_receive*m_timesync_delay_average_ms + diff)/(++m_number_timesync_receive);
					if (diff > m_largest_diff) {
						m_largest_diff = diff;
					}
					if (diff < m_smallest_diff) {
						m_smallest_diff = diff;
					}
				}
                if (m_verbose_output) {
    				std::cout << ", diff: " << diff << " ms, ave: " << m_timesync_delay_average_ms << " ms, smallest: " << m_smallest_diff << " ms, largest: " << m_largest_diff << " ms)\n";
                }
                Notify("TIMESYNC_DELAY_AVERAGE_MS", m_timesync_delay_average_ms);
			} else {
                // Default message
    			mavlink_message_t mav_msg;
    			// Pack
    			uint16_t length = mavlink_msg_timesync_pack(m_system_id, m_component_id, &mav_msg, tc1, ts1);
    			// Prepare to send over serial (iPX4 does the sending)
    			unsigned len = mavlink_msg_to_send_buffer(m_buf_tx, &mav_msg);
    			// Publish to MOOSDB
                if (m_verbose_output) {
                    std::cout << ")\n";
                    std::cout << "Sent timesync message (tc1 " << tc1/1000000000.0 << " s, ts1: " << ts1/1000000000.0 << " s)\n";
                }
    			Notify("MAVLINK_TRANSMIT", m_buf_tx, len);
			}
        }
        break;

    case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:

        break;

    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
        {
            uint8_t landed_state = mavlink_msg_extended_sys_state_get_landed_state(msg);
            Notify("LANDED_STATE", landed_state);
        }
        break;

    case MAVLINK_MSG_ID_ESTIMATOR_STATUS:

        break;

    case MAVLINK_MSG_ID_HOME_POSITION:

        break;

    case MAVLINK_MSG_ID_BATTERY_STATUS:

        break;

    case MAVLINK_MSG_ID_VIBRATION:

        break;

    case MAVLINK_MSG_ID_COMMAND_ACK:
        {
            uint8_t result = mavlink_msg_command_ack_get_result(msg);
            Notify("COMMAND_ACK", result);
        }
        break;
	case MAVLINK_MSG_ID_PING:
		{
            double moosTime = m_moosTimePX4DATA; // time at which px4 data comes in

            mavlink_ping_t ping;
	        mavlink_msg_ping_decode(msg, &ping);
            if (m_verbose_output) {
                std::cout << "Ping (time_usec = " << (int)ping.time_usec << " | seq = " << (int)ping.seq << " | target_system = " << (int)ping.target_system << " | target_component = " << (int)ping.target_component << ")\n";
            }
            if ((ping.target_system == 0) && (ping.target_component == 0)) {
                // Ping request, send one back
                //ping.target_system = msg->sysid;
		        //ping.target_component = msg->compid;
                mavlink_message_t mav_msg;
                uint64_t time_to_send_us = (uint64_t)(MOOSTime()*1000000.0); // use newest time
                uint16_t length =  mavlink_msg_ping_pack(m_system_id, m_component_id, &mav_msg, time_to_send_us, ping.seq, 0, 0);
                unsigned len = mavlink_msg_to_send_buffer(m_buf_tx, &mav_msg);
                Notify("MAVLINK_TRANSMIT", m_buf_tx, len);
            } else if ((ping.target_system == m_system_id) && (ping.target_component == m_component_id)) {
                // Ping return from this system
                // Calculate round trip time
                uint64_t ping_time = (uint64_t)(moosTime*1000000.0) - ping.time_usec;
                if (m_verbose_output) {
                    std::cout << std::fixed << "Return Ping received: MoosTime() (s): " << moosTime << " ping time (s): " << ((long double)ping.time_usec)/1000000.0 << " | ping.seq = " << (int)ping.seq << " | ping time = " << ping_time << std::endl;
                }
                if (m_number_pings_to_ignore > 0 || m_number_pings_received == 0) {
                    --m_number_pings_to_ignore;
                    m_ping_average_us = ping_time;
                    m_number_pings_received = 1;
                    m_ping_min_us = ping_time;
                    m_ping_max_us = ping_time;
                } else {
                    m_ping_average_us = (m_number_pings_received * m_ping_average_us + ping_time)/(++m_number_pings_received);
                    if (ping_time < m_ping_min_us) {
                        m_ping_min_us = ping_time;
                    } else if (ping_time > m_ping_max_us) {
                        m_ping_max_us = ping_time;
                    }
                    if (m_verbose_output) {
                        std::cout << "Average ping (us) = " << m_ping_average_us << " | Max ping (us) = " << m_ping_max_us << " | Min ping (us) = " << m_ping_min_us << " | #Pings = " << m_number_pings_received << std::endl;
                    }
                    Notify("PING_AVERAGE_MS", m_ping_average_us/1000.0);
                }
            }
		}
		break;
    default:
        {
            uint8_t id = msg->msgid;
            // std::cout << "Caught unknown message!" << "(" << (int)id << ")" << std::endl;

            if (id == 46)
                std::cout << "MISSION_ITEM_REACHED" << "(" << (int)id << ")" << std::endl;
        }
        break;
    }
}

void Mavlink::TranslateToMavlink(CMOOSMsg &msg)
{
    string key = msg.GetKey();

    if (key == "DESIRED_SETPOINT") {

        string sval = msg.GetString();
        json data = json::parse(sval);

        float x   = data["x"];
        float y   = data["y"];
        float z   = data["z"];
        float yaw = data["yaw"];

        float yaw_rate = 0.0;

        float vx = 0.0;
        float vy = 0.0;
        float vz = 0.0;

        float afx = 0.0;
        float afy = 0.0;
        float afz = 0.0;

        mavlink_message_t mav_msg;

        // This is actually the time now...
        uint32_t time_boot_ms = (uint32_t) (get_time_usec()/1000);
        uint16_t type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_SETPOINT;
        // Virtually the only message type available for PX4 off-board control
        uint16_t length = mavlink_msg_set_position_target_local_ned_pack(m_system_id, m_component_id, &mav_msg,
                                                                        time_boot_ms, m_target_system, m_target_component,
                                                                        m_coordinate_frame, type_mask, x, y, z,
                                                                        vx, vy, vz, afx, afy, afz, yaw, yaw_rate);

        unsigned len = mavlink_msg_to_send_buffer(m_buf_tx, &mav_msg);

        // Send binary Mavlink message to MOOSDB for transmission by iPX4
        Notify("MAVLINK_TRANSMIT", m_buf_tx, len);
    }

    if (key == "RC_CHAN")
    {
        // set_channel_override
        string sval = msg.GetString();
        json data = json::parse(sval);
        std::cerr << sval << std::endl;
        mavlink_message_t tx_msg;
        mavlink_rc_channels_override_t setch;

        int rudder = -1; // Default is -1 if we don't have a new msg
        int elevator = -1;
        int throttle = -1;
        int chan2 = -1;
        int chan5 = -1;
        int chan6 = -1;
        int chan7 = -1;
        int chan8 = -1;
        //if (!data["rudder"].empty())
            rudder   = data["rudder"];
        //else if (!data["elevator"].empty())
            elevator = data["elevator"];
        //else if (!data["throttle"].empty())
            throttle = data["throttle"];

        setch.target_system = m_system_id;
        setch.target_component = m_component_id;

        if (elevator == -1)
            setch.chan1_raw=0; //UINT16_MAX is the default for unused channels
        else
            setch.chan1_raw=(uint16_t) (elevator);

        if (chan2 == -1)
            setch.chan2_raw=0;
        else
            setch.chan2_raw=(uint16_t) chan2;
        if (throttle == -1)
            setch.chan3_raw=0;
        else
            setch.chan3_raw=(uint16_t) (throttle);

        if (rudder == -1)
            setch.chan4_raw=0;
        else
            setch.chan4_raw=(uint16_t) (rudder);

        if ( chan5 == -1)
            setch.chan5_raw=0;
        else
            setch.chan5_raw=(uint16_t) chan5;

        if ( chan6 == -1)
            setch.chan6_raw=0;
        else
            setch.chan6_raw=(uint16_t) chan6;

        if ( chan7 == -1)
            setch.chan7_raw=0;
        else
            setch.chan7_raw=(uint16_t) chan7;

        if ( chan7 == -1)
            setch.chan8_raw=0;
        else
            setch.chan8_raw=(uint16_t) chan7;

        mavlink_msg_rc_channels_override_encode(255, 0, &tx_msg, &setch);
        std::cerr << to_string(m_system_id) << " " << to_string(m_component_id) << " " << setch.chan1_raw <<
           " " << setch.chan2_raw  << " " << setch.chan3_raw  << " " << setch.chan4_raw  << " " << setch.chan5_raw
            << " " << setch.chan6_raw  << " " << setch.chan7_raw <<  " " << setch.chan8_raw << std::endl;
        unsigned len = mavlink_msg_to_send_buffer(m_buf_tx, &tx_msg);

        // Send binary Mavlink message to MOOSDB for transmission by iPX4
        Notify("MAVLINK_TRANSMIT", m_buf_tx, len);

    }
}

// This returns the MOOS unix timestamp equivalent as if made at the time of the measurement
// The accuracy of this depends on the accuracy of m_moosunix_pxunix_offset_s which varies by approx 20 ms
double Mavlink::getCorrectedUnixTimestamp(uint32_t boot_time_ms) {
    //return ((double)boot_time_ms + m_boot_pxunix_offset_ms)/1000.0 + m_moosunix_pxunix_offset_s;
    if (m_number_moosunix_pxunix_offset_s_to_ignore > 0 || m_number_pings_to_ignore > 0) {
        return ((double)boot_time_ms + m_boot_pxunix_offset_ms)/1000.0 + m_moosunix_pxunix_offset_s;
        //return ((double)boot_time_ms + m_boot_pxunix_offset_ms + m_timesync_delay_average_ms/2.0)/1000.0 + m_moosunix_pxunix_offset_s;
        //return ((double)boot_time_ms + m_boot_pxunix_offset_ms - (double)m_ping_average_us/(1000.0*2.0))/1000.0 + m_moosunix_pxunix_offset_s;
    }
    //return ((double)boot_time_ms + m_boot_pxunix_offset_ms)/1000.0 + m_moosunix_pxunix_offset_average_s;
    //return ((double)boot_time_ms + m_boot_pxunix_offset_ms + m_timesync_delay_average_ms/2.0)/1000.0 + m_moosunix_pxunix_offset_average_s;
    double one_way_ping_ms = (double)m_ping_average_us/(2.0*1000.0);

    // THis should be the moos time of data creation
    return ((double)boot_time_ms + m_boot_pxunix_offset_ms)/1000.0 + m_moosunix_pxunix_offset_average_s - one_way_ping_ms/1000.0;
}