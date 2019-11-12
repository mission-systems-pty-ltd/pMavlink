/*****************************************************************/
/*    NAME: David Battle                                         */
/*    ORGN: Mission Systems Pty Ltd                              */
/*    (based on pMavlinkConverter by Muthukumaran Chandrasekaran)*/
/*    FILE: translation.h                                        */
/*    DATE: 14 Dec 2018                                          */
/*****************************************************************/
#ifndef PARSE_MSG_HEADER
#define PARSE_MSG_HEADER

#include "mavlink.h"
#include <nlohmann/json.hpp>

// Force modes are not operational
// Velocity modes are not operational
#define MAVLINK_MSG_IGNORE_EVERYTHING                          0b0000110111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_CUSTOM       0b0000100111000000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_SETPOINT     0b0000000111111000

void TranslateToMoos(mavlink_message_t *msg);
void TranslateToMavlink(CMOOSMsg & moos_msg);
double getCorrectedUnixTimestamp(uint32_t boot_time_ms);

#endif
