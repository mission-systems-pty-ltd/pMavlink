## SYNOPSIS:

**pMavlink** is a translation layer between MOOS and PX4 software which
is used in conjunction with iPX4 to handle Mavlink data streams.

Supported messages are:

### MOOS -> Mavlink


| **MOOS**         | **Mavlink**                                        |
| ---------------- | -------------------------------------------------- |
| DESIRED_SETPOINT | MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_SETPOINT |

### Mavlink -> MOOS

| **Mavlink**                        | **MOOS**                               |
| ---------------------------------- | -------------------------------------- |
| MAVLINK_MSG_ID_SYS_STATUS          | BAT_REMAINING                          |
|                                    |                                        |
| MAVLINK_MSG_ID_SYSTEM_TIME         | OFFSET_MS_PIXHAWK_BOOT_UNIX            |
|                                    | OFFSET_MS_MOOSUNIX_PIXHAWKUNIX         |
|                                    | OFFSET_MS_MOOSUNIX_PIXHAWKUNIX_AVERAGE |
|                                    |                                        |
| MAVLINK_MSG_ID_ATTITUDE            | NAV_ROLL                               |
|                                    | NAV_PITCH                              |
|                                    | NAV_YAW                                |
|                                    |                                        |
| MAVLINK_MSG_ID_LOCAL_POSITION_NED  | LOCAL_X_NED                            |
|                                    | LOCAL_Y_NED                            |
|                                    | LOCAL_Z_NED                            |
|                                    |                                        |
| MAVLINK_MSG_ID_GLOBAL_POSITION_INT | NAV_LAT                                |
|                                    | NAV_LONG                               |
|                                    | ALT_AMSL                               |
|                                    | ALT_REL                                |
|                                    | NAV_HEADING                            |
|                                    | NAV_X                                  |
|                                    | NAV_Y                                  |
|                                    | NAV_Z                                  |
|                                    |                                        |
| MAVLINK_MSG_ID_GPS_RAW_INT         | GPS_UNIX_TIME                          |
| MAVLINK_MSG_ID_TIMESYNC            | TIMESYNC_DELAY_AVERAGE_MS              |
| MAVLINK_MSG_ID_EXTENDED_SYS_STATE  | LANDED_STATE                           |
| MAVLINK_MSG_ID_COMMAND_ACK         | COMMAND_ACK                            |
| MAVLINK_MSG_ID_PING                | PING_AVERAGE_MS                        |

All messages are read from or recorded in the MOOSDB. iPX4 is used to
transport to or from MOOSDB via Mavlink.

### Installation
For installation of the Mavlink library, consult iPX4 README located at
https://github.com/mission-systems-pty-ltd/iPX4
