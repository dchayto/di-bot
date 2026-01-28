/*								serialMSG.hpp								//
	defining custom structs/data types for passing messages between pi and due

	auth: @dchayto

*/

#ifndef __SERIAL_MSGS_H__
#define __SERIAL_MSGS_H__

#include <cstdio>
#include <cinttypes>
#include <cstdint>

// largest data string: "-128 -128 -128 -128" [19] -> go with 22 if sending chars
// working with fixed with, 8bit integers... should be able to send directly 
#define WHEELSPEED_MSG_SIZE 5 

namespace serialMSG		{
	struct WheelSpeed	{	 // 
		// vars for handling message formatting
		char msg[WHEELSPEED_MSG_SIZE]; // first element for struct packing
		static const uint8_t MAGIC_NUMBER { 0xDC };
		static const uint8_t MSG_SIZE { WHEELSPEED_MSG_SIZE };	

		// input wheel speed data
		// could store as int8_t[4], but this seems clearer usage-wise
		int8_t FR { 0 }; // front right wheel speed command
		int8_t FL { 0 }; // front left
		int8_t BR { 0 }; // back right
		int8_t BL { 0 }; // back left
		
		void encodeMsg() {
			//using namespace std; // arduino defaults snprintf into global ns
			//snprintf(msg, MSG_SIZE, "%c %+" PRId8 " %+" PRId8 " %+" PRId8 
			//			" %+" PRId8, MAGIC_NUMBER, FR, FL, BR, BL);
			msg[0] = MAGIC_NUMBER;
			msg[1] = FR;
			msg[2] = FL;
			msg[3] = BR;
			msg[4] = BL;
		}

		void decodeMsg() {
			//std::sscanf(msg, "%*c %hhd %hhd %hhd %hhd", &FR, &FL, &BR, &BL);
			FR = msg[1];
			FL = msg[2];
			BR = msg[3];
			BL = msg[4];
		}

		void setWheelSpeed(int8_t FR, int8_t FL, int8_t BR, int8_t BL) {
			this->FR = FR;
			this->FL = FL;
			this->BR = BR;
			this->BL = BL;
			encodeMsg();
		} 
	}; // </struct WheelSpeed>
} // serialMSG

#endif
