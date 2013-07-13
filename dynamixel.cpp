//##########################################################
//##                      R O B O T I S                   ##
//##         SyncWrite Example code for Dynamixel.        ##
//##                                           2009.11.10 ##
//##########################################################
#include <stdio.h>
#include <unistd.h>
#include <cmath>
#include <termio.h>
#include <dynamixel.h>
#include "init.hpp"

using namespace std;

// Control table address
#define P_GOAL_POSITION_L	30
#define P_GOAL_POSITION_H	31
#define P_GOAL_SPEED_L		32
#define P_GOAL_SPEED_H		33

// Defulat setting
#define DEFAULT_BAUDNUM		1  // 1Mbps
#define NUM_SERVOS			18 // Number of actuators
 

void PrintCommStatus(int CommStatus);
void PrintErrorCode(void);


int id[NUM_SERVOS];
int CommStatus;


void syncWriteServos()
{
	// Make syncwrite packet
	dxl_set_txpacket_id(BROADCAST_ID);
	dxl_set_txpacket_instruction(INST_SYNC_WRITE);
	dxl_set_txpacket_parameter(0, P_GOAL_POSITION_L);
	dxl_set_txpacket_parameter(1, 2);

	// RF
	dxl_set_txpacket_parameter(2+3*0, RF_COXA_ID);
	dxl_set_txpacket_parameter(2+3*0+1, dxl_get_lowbyte(leg[RIGHT_FRONT].servoPos.coxa));
	dxl_set_txpacket_parameter(2+3*0+2, dxl_get_highbyte(leg[RIGHT_FRONT].servoPos.coxa));
	dxl_set_txpacket_parameter(2+3*1, RF_FEMUR_ID);
	dxl_set_txpacket_parameter(2+3*1+1, dxl_get_lowbyte(leg[RIGHT_FRONT].servoPos.femur));
	dxl_set_txpacket_parameter(2+3*1+2, dxl_get_highbyte(leg[RIGHT_FRONT].servoPos.femur));
	dxl_set_txpacket_parameter(2+3*2, RF_TIBIA_ID);
	dxl_set_txpacket_parameter(2+3*2+1, dxl_get_lowbyte(leg[RIGHT_FRONT].servoPos.tibia));
	dxl_set_txpacket_parameter(2+3*2+2, dxl_get_highbyte(leg[RIGHT_FRONT].servoPos.tibia));
	
	// RM
	dxl_set_txpacket_parameter(2+3*3, RM_COXA_ID);
	dxl_set_txpacket_parameter(2+3*3+1, dxl_get_lowbyte(leg[RIGHT_MIDDLE].servoPos.coxa));
	dxl_set_txpacket_parameter(2+3*3+2, dxl_get_highbyte(leg[RIGHT_MIDDLE].servoPos.coxa));
	dxl_set_txpacket_parameter(2+3*4, RM_FEMUR_ID);
	dxl_set_txpacket_parameter(2+3*4+1, dxl_get_lowbyte(leg[RIGHT_MIDDLE].servoPos.femur));
	dxl_set_txpacket_parameter(2+3*4+2, dxl_get_highbyte(leg[RIGHT_MIDDLE].servoPos.femur));
	dxl_set_txpacket_parameter(2+3*5, RM_TIBIA_ID);
	dxl_set_txpacket_parameter(2+3*5+1, dxl_get_lowbyte(leg[RIGHT_MIDDLE].servoPos.tibia));
	dxl_set_txpacket_parameter(2+3*5+2, dxl_get_highbyte(leg[RIGHT_MIDDLE].servoPos.tibia));
	
	// RR
	dxl_set_txpacket_parameter(2+3*6, RR_COXA_ID);
	dxl_set_txpacket_parameter(2+3*6+1, dxl_get_lowbyte(leg[RIGHT_REAR].servoPos.coxa));
	dxl_set_txpacket_parameter(2+3*6+2, dxl_get_highbyte(leg[RIGHT_REAR].servoPos.coxa));
	dxl_set_txpacket_parameter(2+3*7, RR_FEMUR_ID);
	dxl_set_txpacket_parameter(2+3*7+1, dxl_get_lowbyte(leg[RIGHT_REAR].servoPos.femur));
	dxl_set_txpacket_parameter(2+3*7+2, dxl_get_highbyte(leg[RIGHT_REAR].servoPos.femur));
	dxl_set_txpacket_parameter(2+3*8, RR_TIBIA_ID);
	dxl_set_txpacket_parameter(2+3*8+1, dxl_get_lowbyte(leg[RIGHT_REAR].servoPos.tibia));
	dxl_set_txpacket_parameter(2+3*8+2, dxl_get_highbyte(leg[RIGHT_REAR].servoPos.tibia));
	
	// LR
	dxl_set_txpacket_parameter(2+3*9, LR_COXA_ID);
	dxl_set_txpacket_parameter(2+3*9+1, dxl_get_lowbyte(leg[LEFT_REAR].servoPos.coxa));
	dxl_set_txpacket_parameter(2+3*9+2, dxl_get_highbyte(leg[LEFT_REAR].servoPos.coxa));
	dxl_set_txpacket_parameter(2+3*10, LR_FEMUR_ID);
	dxl_set_txpacket_parameter(2+3*10+1, dxl_get_lowbyte(leg[LEFT_REAR].servoPos.femur));
	dxl_set_txpacket_parameter(2+3*10+2, dxl_get_highbyte(leg[LEFT_REAR].servoPos.femur));
	dxl_set_txpacket_parameter(2+3*11, LR_TIBIA_ID);
	dxl_set_txpacket_parameter(2+3*11+1, dxl_get_lowbyte(leg[LEFT_REAR].servoPos.tibia));
	dxl_set_txpacket_parameter(2+3*11+2, dxl_get_highbyte(leg[LEFT_REAR].servoPos.tibia));
	
	// LM
	dxl_set_txpacket_parameter(2+3*12, LM_COXA_ID);
	dxl_set_txpacket_parameter(2+3*12+1, dxl_get_lowbyte(leg[LEFT_MIDDLE].servoPos.coxa));
	dxl_set_txpacket_parameter(2+3*12+2, dxl_get_highbyte(leg[LEFT_MIDDLE].servoPos.coxa));
	dxl_set_txpacket_parameter(2+3*13, LM_FEMUR_ID);
	dxl_set_txpacket_parameter(2+3*13+1, dxl_get_lowbyte(leg[LEFT_MIDDLE].servoPos.femur));
	dxl_set_txpacket_parameter(2+3*13+2, dxl_get_highbyte(leg[LEFT_MIDDLE].servoPos.femur));
	dxl_set_txpacket_parameter(2+3*14, LM_TIBIA_ID);
	dxl_set_txpacket_parameter(2+3*14+1, dxl_get_lowbyte(leg[LEFT_MIDDLE].servoPos.tibia));
	dxl_set_txpacket_parameter(2+3*14+2, dxl_get_highbyte(leg[LEFT_MIDDLE].servoPos.tibia));
	
	// LF
	dxl_set_txpacket_parameter(2+3*15, LF_COXA_ID);
	dxl_set_txpacket_parameter(2+3*15+1, dxl_get_lowbyte(leg[LEFT_FRONT].servoPos.coxa));
	dxl_set_txpacket_parameter(2+3*15+2, dxl_get_highbyte(leg[LEFT_FRONT].servoPos.coxa));
	dxl_set_txpacket_parameter(2+3*16, LF_FEMUR_ID);
	dxl_set_txpacket_parameter(2+3*16+1, dxl_get_lowbyte(leg[LEFT_FRONT].servoPos.femur));
	dxl_set_txpacket_parameter(2+3*16+2, dxl_get_highbyte(leg[LEFT_FRONT].servoPos.femur));
	dxl_set_txpacket_parameter(2+3*17, LF_TIBIA_ID);
	dxl_set_txpacket_parameter(2+3*17+1, dxl_get_lowbyte(leg[LEFT_FRONT].servoPos.tibia));
	dxl_set_txpacket_parameter(2+3*17+2, dxl_get_highbyte(leg[LEFT_FRONT].servoPos.tibia));
	
	dxl_set_txpacket_length((2+1)*NUM_SERVOS+4);
	dxl_txrx_packet();
	
	CommStatus = dxl_get_result();
	if( CommStatus == COMM_RXSUCCESS )
	{
		PrintErrorCode();
	}
	else
	{
		PrintCommStatus(CommStatus);
	}
			
}


// Print communication result
void PrintCommStatus(int CommStatus)
{
	switch(CommStatus)
	{
	case COMM_TXFAIL:
		printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
		break;

	case COMM_TXERROR:
		printf("COMM_TXERROR: Incorrect instruction packet!\n");
		break;

	case COMM_RXFAIL:
		printf("COMM_RXFAIL: Failed get status packet from device!\n");
		break;

	case COMM_RXWAITING:
		printf("COMM_RXWAITING: Now recieving status packet!\n");
		break;

	case COMM_RXTIMEOUT:
		printf("COMM_RXTIMEOUT: There is no status packet!\n");
		break;

	case COMM_RXCORRUPT:
		printf("COMM_RXCORRUPT: Incorrect status packet!\n");
		break;

	default:
		printf("This is unknown error code!\n");
		break;
	}
}

// Print error bit of status packet
void PrintErrorCode()
{
	if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
		printf("Input voltage error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
		printf("Angle limit error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
		printf("Overheat error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
		printf("Out of range error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
		printf("Checksum error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
		printf("Overload error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
		printf("Instruction code error!\n");
}
