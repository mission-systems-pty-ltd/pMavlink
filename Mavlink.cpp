/*****************************************************************/
/*    NAME: David Battle                                         */
/*    ORGN: Mission Systems Pty Ltd                              */
/*    (based on pMavlinkConverter by Muthukumaran Chandrasekaran)*/
/*    FILE: Mavlink.cpp                                          */
/*    DATE: 14 Dec 2018                                          */
/*****************************************************************/
#include <iterator>
#include <cstring>
#include "MBUtils.h"
#include "Mavlink.h"
#include "translation.h"
#include "math.h"

#define BUFFER_LENGTH 279

using namespace std;
void printByteStream(unsigned char * buffer, size_t size, size_t concise_length=0);
void printMavlinkStatus(mavlink_status_t mav_status);

Mavlink::Mavlink()
{
  m_system_id        = 2;
  m_component_id     = 0;
  m_target_system    = 1;
  m_target_component = 1;
  m_coordinate_frame = 7;
  send_rc = 0;
  //rc_msg = NULL;

  // Allocate mavlink message buffer
  m_buf = new unsigned char [BUFFER_LENGTH];
  m_buf_tx = new unsigned char [BUFFER_LENGTH];
}

//---------------------------------------------------------
// Destructor

Mavlink::~Mavlink()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool Mavlink::OnNewMail(MOOSMSG_LIST &NewMail)
{
	// Mavlink message structure
	mavlink_message_t mav_msg;
	mavlink_status_t  mav_status;

	MOOSMSG_LIST::iterator p;
	for (p=NewMail.begin(); p!=NewMail.end(); p++) {

		CMOOSMsg &msg = *p;
		string key = msg.GetKey();

		if (key == "MAVLINK_RECEIVE") {
            m_moosTimePX4DATA = msg.GetTime(); // This is the moos time of creation of the px4 serial stream

		    size_t len = p->GetBinaryDataSize();
		    memcpy(m_buf, p->GetBinaryData(), len);

			if (m_verbose_output) {
				std::cout << "m_buf:";
				printByteStream(m_buf, len, 50);
			}

			// Loop through input_buffer and update index of next non-processed byte
			int packet_drops{0}; // when a new packet drop is detected, increment idx
			for (int i = 0; i < len; ++i) {
				//std::cout << "Message # " << std::setw(5) << mav_status.packet_rx_success_count << endl;
                if (mavlink_parse_char(MAVLINK_COMM_0, m_buf[i], &mav_msg, &mav_status)) {
					if (m_verbose_output) {
						std::cout << "Message # " << std::setw(5) << mav_status.packet_rx_success_count << ": (index=";
						if (next_msg_idx > i) {
							std::cout << "(prev)";
						} else {
							std::cout << "      ";
						}
						std::cout << std::setw(3) << next_msg_idx << "-" << std::setw(3) << i << ",size=";
						if (next_msg_idx > i) {
							std::cout << std::setw(3) << (i + (BUFFER_LENGTH - next_msg_idx)) << ",";
						} else {
							std::cout << std::setw(3) << (i - next_msg_idx) << ",";
						}
						printf("ID %3d, sequence: %3d from component %2d of system %2d)\n", mav_msg.msgid, mav_msg.seq, mav_msg.compid, mav_msg.sysid);
					}
				    TranslateToMoos(&mav_msg);
				    next_msg_idx = i+1;
			    } else if (mav_status.packet_rx_drop_count > packet_drops) {
			        packet_drops = mav_status.packet_rx_drop_count;
			        next_msg_idx = i+1;
					if (m_verbose_output) {
						std::cout << "mavlink status after packet drop detection:" << std::endl;
						printMavlinkStatus(mav_status);
					}
			    }
			}
		} else {
            std::cerr << "GOT NET MAIL\n";
            std::cerr << key;
			TranslateToMavlink(msg);
            if (key == "RC_CHAN")
            {
                send_rc = 1;
                rc_msg = msg;
            }
		}
	}
	return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool Mavlink::OnConnectToServer()
{
	RegisterVariables();

	return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool Mavlink::Iterate()
{
	// check if a rc chan command has been sent
    // send in loop
    //if (send_rc)
    //    TranslateToMavlink(rc_msg);
    return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool Mavlink::OnStartUp()
{
    SetAppFreq(0); // Fast as we can
    SetIterateMode(REGULAR_ITERATE_AND_COMMS_DRIVEN_MAIL);


  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string original_line = *p;
      string param = stripBlankEnds(toupper(biteString(*p, '=')));
      string value = stripBlankEnds(*p);
      double dval  = atof(value.c_str());

      if(param == "SYS_ID") {
        m_system_id = dval;
      }
      else if(param == "COMP_ID") {
        m_component_id = dval;
      }
      else if(param == "TARGET_SYS") {
        m_target_system = dval;
      }
      else if(param == "TARGET_COMP") {
        m_target_component = dval;
      } else if(param == "VERBOSE_OUTPUT") {
        value = toupper(value);
        if (value == "ON" || value == "TRUE") {
        	m_verbose_output = true;
        } else if (value == "OFF" || value == "FALSE") {
        	m_verbose_output = false;
        }
      }
    }
  }

  // look for latitude, longitude global variables
  double latOrigin, longOrigin;

  if(!m_MissionReader.GetValue("LatOrigin", latOrigin)) {

    MOOSTrace("pGridViewer: LatOrigin not set in *.moos file.\n");
    m_geo_ok = false;
  }
  else if(!m_MissionReader.GetValue("LongOrigin", longOrigin)) {

    MOOSTrace("pGridViewer: LongOrigin not set in *.moos file\n");
    m_geo_ok = false;
  }
  else {

    m_geo_ok = true;

    // initialize m_geodesy
    if(!m_geodesy.Initialise(latOrigin, longOrigin)) {
      MOOSTrace("pGridViewer: Geodesy init failed.\n");
      m_geo_ok = false;
    }
  }

  RegisterVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void Mavlink::RegisterVariables()
{
  Register("DESIRED_SPEED",0.0);
  Register("DESIRED_HEADING",0.0);
  Register("DESIRED_SETPOINT",0.0);
  Register("MAVLINK_RECEIVE",0.0);
  Register("RC_CHAN",0.0);
}

void printByteStream(unsigned char * buffer, size_t size, size_t concise_length) {
	char fill_char{'?'};
	size_t end_first_chunk{size};
	size_t start_end_chunk{size};
	if (concise_length > 0 && concise_length < size/2) {
		end_first_chunk = concise_length;
		start_end_chunk = size - concise_length;
	}
	for (size_t i = 0; i < end_first_chunk; ++i) {
		std::cout << "|" <<
		std::hex <<           // output in hex
		std::setw(2) <<       // each byte prints as two characters
		std::setfill(fill_char) <<  // fill with 0 if not enough characters
		static_cast<unsigned int>(buffer[i]);
	}
	if (concise_length > 0 && concise_length < size/2) {
		std::cout << "| ... ";
	}
	for (size_t i = start_end_chunk; i < size; ++i) {
		std::cout << "|" <<
		std::hex <<           // output in hex
		std::setw(2) <<       // each byte prints as two characters
		std::setfill(fill_char) <<  // fill with 0 if not enough characters
		static_cast<unsigned int>(buffer[i]);
	}
	if (size != 0) {
		std::cout << "|";
	}
	// reset iostream properties and print a new line
	std::cout << std::dec << std::setw(0) << std::setfill(' ') << std::endl;
}

void printMavlinkStatus(mavlink_status_t mav_status) {
	std::cout << "mavlink_status_t:"															<< std::endl;
	std::cout << "\tbuffer_overrun = "          << unsigned(mav_status.buffer_overrun)			<< std::endl;
	std::cout << "\tcurrent_rx_seq = "          << unsigned(mav_status.current_rx_seq)			<< std::endl;
	std::cout << "\tcurrent_tx_seq = "          << unsigned(mav_status.current_tx_seq)			<< std::endl;
	// std::cout << "\tflags = "                   << unsigned(mav_status.flags)					<< std::endl;
	std::cout << "\tmsg_received = "            << unsigned(mav_status.msg_received)			<< std::endl;
	std::cout << "\tpacket_idx = "              << unsigned(mav_status.packet_idx)				<< std::endl;
	std::cout << "\tpacket_rx_drop_count = "    << unsigned(mav_status.packet_rx_drop_count)	<< std::endl;
	std::cout << "\tpacket_rx_success_count = " << unsigned(mav_status.packet_rx_success_count)	<< std::endl;
	std::cout << "\tparse_error = "             << unsigned(mav_status.parse_error)				<< std::endl;
	std::cout << "\tparse_state = "             << mav_status.parse_state               		<< std::endl;
	// std::cout << "\tsignature_wait = "          << unsigned(mav_status.signature_wait)			<< std::endl;
	std::cout << "\tsigning = "                 << "struct __mavlink_signing *"					<< std::endl;
	std::cout << "\tsigning_streams = "         << "struct __mavlink_signing_streams *"			<< std::endl;
}
