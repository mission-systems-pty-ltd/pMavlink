/****************************************************************/
/*   NAME:                                              */
/*   ORGN: MIT Cambridge MA                                     */
/*   FILE: Mavlink_Info.cpp                               */
/*   DATE: Dec 29th 1963                                        */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "Mavlink_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:");
  blk("------------------------------------");
  blk("pMavlink is a translation layer between MOOS and PX4 software which");
  blk("is used in conjunction with iPX4 to handle Mavlink data streams.");
  blk("");
  blk("Supported messages are:");
  blk("");
  blk("MOOS -> Mavlink");
  blk("");
  blk("| MOOS             | Mavlink                                            |");
  blk("|------------------+----------------------------------------------------|");
  blk("| DESIRED_SETPOINT | MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_SETPOINT |");
  blk("|------------------+----------------------------------------------------|");
  blk("");
  blk("Mavlink -> MOOS");
  blk("");
  blk("| Mavlink                            | MOOS                                   |");
  blk("|------------------------------------+----------------------------------------|");
  blk("| MAVLINK_MSG_ID_SYS_STATUS          | BAT_REMAINING                          |");
  blk("|------------------------------------+----------------------------------------|");
  blk("| MAVLINK_MSG_ID_SYSTEM_TIME         | OFFSET_MS_PIXHAWK_BOOT_UNIX            |");
  blk("|                                    | OFFSET_MS_MOOSUNIX_PIXHAWKUNIX         |");
  blk("|                                    | OFFSET_MS_MOOSUNIX_PIXHAWKUNIX_AVERAGE |");
  blk("|------------------------------------+----------------------------------------|");
  blk("| MAVLINK_MSG_ID_ATTITUDE            | NAV_ROLL                               |");
  blk("|                                    | NAV_PITCH                              |");
  blk("|                                    | NAV_YAW                                |");
  blk("|------------------------------------+----------------------------------------|");
  blk("| MAVLINK_MSG_ID_LOCAL_POSITION_NED  | LOCAL_X_NED                            |");
  blk("|                                    | LOCAL_Y_NED                            |");
  blk("|                                    | LOCAL_Z_NED                            |");
  blk("|------------------------------------+----------------------------------------|");
  blk("| MAVLINK_MSG_ID_GLOBAL_POSITION_INT | NAV_LAT                                |");
  blk("|                                    | NAV_LONG                               |");
  blk("|                                    | ALT_AMSL                               |");
  blk("|                                    | ALT_REL                                |");
  blk("|                                    | NAV_HEADING                            |");
  blk("|                                    | NAV_X                                  |");
  blk("|                                    | NAV_Y                                  |");
  blk("|                                    | NAV_Z                                  |");
  blk("|------------------------------------+----------------------------------------|");
  blk("| MAVLINK_MSG_ID_GPS_RAW_INT         | GPS_UNIX_TIME                          |");
  blk("|------------------------------------+----------------------------------------|");
  blk("| MAVLINK_MSG_ID_TIMESYNC            | TIMESYNC_DELAY_AVERAGE_MS              |");
  blk("|------------------------------------+----------------------------------------|");
  blk("| MAVLINK_MSG_ID_EXTENDED_SYS_STATE  | LANDED_STATE                           |");
  blk("|------------------------------------+----------------------------------------|");
  blk("| MAVLINK_MSG_ID_COMMAND_ACK         | COMMAND_ACK                            |");
  blk("|------------------------------------+----------------------------------------|");
  blk("| MAVLINK_MSG_ID_PING                | PING_AVERAGE_MS                        |");
  blk("|------------------------------------+----------------------------------------|");
  blk("");
  blk("All messages are read from or recorded in the MOOSDB. iPX4 is used to");
  blk("transport to or from MOOSDB via Mavlink.");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pMavlink file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pMavlink with the given process name         ");
  blk("      rather than pMavlink.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pMavlink.        ");
  blk("                                                                ");
  blk("Note: If argv[2] does not otherwise match a known option,       ");
  blk("      then it will be interpreted as a run alias. This is       ");
  blk("      to support pAntler launching conventions.                 ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pMavlink Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pMavlink                              ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}


//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pMavlink INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NODE_MESSAGE = src_node=alpha,dest_node=bravo,var_name=FOO,   ");
  blk("                 string_val=BAR                                 ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  Publications are determined by the node message content.      ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pMavlink", "gpl");
  exit(0);
}

