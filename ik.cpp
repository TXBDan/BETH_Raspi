#include <cmath>
#include <iostream>
#include "init.hpp"
#include "ik.hpp"
#include "CommanderEx.hpp"
#include <dynamixel.h>

using namespace std;

/******************************************************************************
 * Inverse Kinematics for hexapod
 *
 * FRONT VIEW       ^        ==0         0==
 *     /\___/\      |       |  0==[___]==0  |
 *    /       \     -Z      |               |
 *
 * TOP VIEW
 *    \       /     ^
 *     \_____/      |
 *  ___|     |___   X
 *     |_____|
 *     /     \      Y->
 *    /       \
 *****************************************************************************/


/*********************************************************************************************************
    runIK()
**********************************************************************************************************/
void runIK(){
  
    bodyFK();
    legIK(); 
    driveServos();
}

/**********************************************************************************************************
    bodyFK()
    Calculates necessary foot position (leg space) to acheive commanded body rotations, translations, and gait inputs
***********************************************************************************************************/
void bodyFK(){
	
	//cout << "Body FK" << endl;
    
    float sinRotX, cosRotX, sinRotY, cosRotY, sinRotZ, cosRotZ;
    int totalX, totalY, totalZ;
    float bodyRotOffsetX[6], bodyRotOffsetY[6], bodyRotOffsetZ[6];
  
    sinRotX = sin(radians(-commanderInput.bodyRotX));
    cosRotX = cos(radians(-commanderInput.bodyRotX));
    sinRotY = sin(radians(-commanderInput.bodyRotY));
    cosRotY = cos(radians(-commanderInput.bodyRotY));
    sinRotZ = sin(radians(-commanderInput.bodyRotZ));
    cosRotZ = cos(radians(-commanderInput.bodyRotZ));
  
    for( int legNum=0; legNum<6; legNum++){ 
        
        //cout << "bodyFK() Leg: " << legNum+1 << endl;
    
        totalX = leg[legNum].initialFootPos.x + leg[legNum].legBasePos.x;       
        totalY = leg[legNum].initialFootPos.y + leg[legNum].legBasePos.y;
        totalZ = leg[legNum].initialFootPos.z + leg[legNum].legBasePos.z;

        bodyRotOffsetX[legNum] = ( totalY*cosRotY*sinRotZ + totalY*cosRotZ*sinRotX*sinRotY + totalX*cosRotZ*cosRotY - totalX*sinRotZ*sinRotX*sinRotY - totalZ*cosRotX*sinRotY) - totalX;     
        bodyRotOffsetY[legNum] =   totalY*cosRotX*cosRotZ - totalX*cosRotX*sinRotZ         + totalZ*sinRotX         - totalY;      
        bodyRotOffsetZ[legNum] = ( totalY*sinRotZ*sinRotY - totalY*cosRotZ*cosRotY*sinRotX + totalX*cosRotZ*sinRotY + totalX*cosRotY*sinRotZ*sinRotX + totalZ*cosRotX*cosRotY) - totalZ;       
        
        // Calculated foot positions to acheive xlation/rotation input. Not coxa mounting angle corrected
        bodyRotOffsetX[legNum] = leg[legNum].initialFootPos.x + bodyRotOffsetX[legNum] - commanderInput.bodyTransX + leg[legNum].footPos.x;                                              
        bodyRotOffsetY[legNum] = leg[legNum].initialFootPos.y + bodyRotOffsetY[legNum] - commanderInput.bodyTransY + leg[legNum].footPos.y;
        bodyRotOffsetZ[legNum] = leg[legNum].initialFootPos.z + bodyRotOffsetZ[legNum] - commanderInput.bodyTransZ + leg[legNum].footPos.z;
        //cout << "bodyRotOffsetX: " << bodyRotOffsetX[legNum] << endl;
        //cout << "bodyRotOffsetY: " << bodyRotOffsetY[legNum] << endl;
        //cout << "bodyRotOffsetZ: " << bodyRotOffsetZ[legNum] << endl;
    }
     
    // Rotates X,Y about coxa to compensate for coxa mounting angles.
    leg[0].footPosCalc.x = round( bodyRotOffsetY[0]*cos(radians(COXA_ANGLE))   - bodyRotOffsetX[0]*sin(radians(COXA_ANGLE)) ); 
    leg[0].footPosCalc.y = round( bodyRotOffsetY[0]*sin(radians(COXA_ANGLE))   + bodyRotOffsetX[0]*cos(radians(COXA_ANGLE)) );
    leg[0].footPosCalc.z = bodyRotOffsetZ[0];
    leg[1].footPosCalc.x = round( bodyRotOffsetY[1]*cos(radians(COXA_ANGLE*2)) - bodyRotOffsetX[1]*sin(radians(COXA_ANGLE*2)) );
    leg[1].footPosCalc.y = round( bodyRotOffsetY[1]*sin(radians(COXA_ANGLE*2)) + bodyRotOffsetX[1]*cos(radians(COXA_ANGLE*2)) );
    leg[1].footPosCalc.z = bodyRotOffsetZ[1];
    leg[2].footPosCalc.x = round( bodyRotOffsetY[2]*cos(radians(COXA_ANGLE*3)) - bodyRotOffsetX[2]*sin(radians(COXA_ANGLE*3)) );
    leg[2].footPosCalc.y = round( bodyRotOffsetY[2]*sin(radians(COXA_ANGLE*3)) + bodyRotOffsetX[2]*cos(radians(COXA_ANGLE*3)) );
    leg[2].footPosCalc.z = bodyRotOffsetZ[2];
    leg[3].footPosCalc.x = round( bodyRotOffsetY[3]*cos(radians(COXA_ANGLE*5)) - bodyRotOffsetX[3]*sin(radians(COXA_ANGLE*5)) );
    leg[3].footPosCalc.y = round( bodyRotOffsetY[3]*sin(radians(COXA_ANGLE*5)) + bodyRotOffsetX[3]*cos(radians(COXA_ANGLE*5)) );
    leg[3].footPosCalc.z = bodyRotOffsetZ[3];
    leg[4].footPosCalc.x = round( bodyRotOffsetY[4]*cos(radians(COXA_ANGLE*6)) - bodyRotOffsetX[4]*sin(radians(COXA_ANGLE*6)) );
    leg[4].footPosCalc.y = round( bodyRotOffsetY[4]*sin(radians(COXA_ANGLE*6)) + bodyRotOffsetX[4]*cos(radians(COXA_ANGLE*6)) );
    leg[4].footPosCalc.z = bodyRotOffsetZ[4];
    leg[5].footPosCalc.x = round( bodyRotOffsetY[5]*cos(radians(COXA_ANGLE*7)) - bodyRotOffsetX[5]*sin(radians(COXA_ANGLE*7)) );
    leg[5].footPosCalc.y = round( bodyRotOffsetY[5]*sin(radians(COXA_ANGLE*7)) + bodyRotOffsetX[5]*cos(radians(COXA_ANGLE*7)) );
    leg[5].footPosCalc.z = bodyRotOffsetZ[5];
    
/*    for( int legNum=0; legNum<6; legNum++){ 
        cout << "bodyFK() Leg: " << legNum+1 << endl;
        cout << "footPosCalcX: " << leg[legNum].footPosCalc.x << endl;  //these are off by +/- 1
        cout << "footPosCalcY: " << leg[legNum].footPosCalc.y << endl;
        cout << "footPosCalcZ: " << leg[legNum].footPosCalc.z << endl;
    }*/
    
}


/**************************************************************************************************************
    legIK()
    Translates foot x,y,z positions (body space) to leg space and adds goal foot positon input (leg space).
    Calculates the coxa, femur, and tibia angles for these foot positions (leg space).
***************************************************************************************************************/
void legIK(){
	
	//cout << "Leg IK" << endl;
  
    float CoxaFootDist, IKSW, IKA1, IKA2, tibAngle;
                                                         
    for( int legNum=0; legNum<6; legNum++ ){
      
        //cout << "legIK() Leg: " << legNum+1 << endl;
   
        CoxaFootDist = sqrt( pow(leg[legNum].footPosCalc.y,2) + pow(leg[legNum].footPosCalc.x,2) );
        //cout << "CoxaFootDist: " << CoxaFootDist << endl;
        IKSW = sqrt( pow(CoxaFootDist-LENGTH_COXA,2) + pow(leg[legNum].footPosCalc.z,2) );
        //cout << "IKSW : " << IKSW  << endl;
        IKA1 = atan2( (CoxaFootDist - LENGTH_COXA) , leg[legNum].footPosCalc.z );
        //cout << "IKA1: " << IKA1 << endl;
        IKA2 = acos( (pow(LENGTH_TIBIA,2) - pow(LENGTH_FEMUR,2) - pow(IKSW,2) ) / (-2*IKSW*LENGTH_FEMUR) );
        //cout << "IKA2: " << IKA2 << endl;
        tibAngle = acos( (pow(IKSW,2) - pow(LENGTH_TIBIA,2) - pow(LENGTH_FEMUR,2)) / (-2*LENGTH_FEMUR*LENGTH_TIBIA) );
        //cout << "tibAngle: " << tibAngle << endl;
        
        leg[legNum].jointAngles.coxa  = 90 - degrees( atan2( leg[legNum].footPosCalc.y , leg[legNum].footPosCalc.x) ); 
        leg[legNum].jointAngles.femur = 90 - degrees( IKA1 + IKA2 );
        leg[legNum].jointAngles.tibia = 90 - degrees( tibAngle );
                      
        //cout << "Coxa Angle: " << leg[legNum].jointAngles.coxa << endl;
        //cout << "Femur Angle: " << leg[legNum].jointAngles.femur << endl; 
        //cout << "Tibia Angle: " << leg[legNum].jointAngles.tibia << endl;
    }
    
    // Applies necessary corrections to servo angles to account for hardware
    for( int legNum=0; legNum<3; legNum++ ){
        leg[legNum].jointAngles.coxa  = leg[legNum].jointAngles.coxa;
        leg[legNum].jointAngles.femur = leg[legNum].jointAngles.femur - 13.58;              // accounts for offset servo bracket on femur
        leg[legNum].jointAngles.tibia = leg[legNum].jointAngles.tibia - 48.70 + 13.58 + 90; //counters offset servo bracket on femur, accounts for 90deg mounting, and bend of tibia
    }    
   for( int legNum=3; legNum<6; legNum++ ){
     
        leg[legNum].jointAngles.coxa  =   leg[legNum].jointAngles.coxa;
        leg[legNum].jointAngles.femur = -(leg[legNum].jointAngles.femur - 13.58);
        leg[legNum].jointAngles.tibia = -(leg[legNum].jointAngles.tibia - 48.70 + 13.58 + 90);
    }
    
}



/*************************************************
    driveServos()
    Commands servos to angles in joinitAngles
    converts to AX12 definition of angular rotation and divides by number of degrees per bit.
    0deg = 512 = straight servo
    ax12SyncWrite() writes all servo values out to the AX12 bus using the SYNCWRITE instruction
**************************************************/
void driveServos(){
  
    for( int legNum=0; legNum<6; legNum++ ){
        leg[legNum].servoPos.coxa  = round((abs( leg[legNum].jointAngles.coxa  - 210) - 60 )  / 0.293);
        leg[legNum].servoPos.femur = round((abs( leg[legNum].jointAngles.femur - 210) - 60 )  / 0.293);
        leg[legNum].servoPos.tibia = round((abs( leg[legNum].jointAngles.tibia - 210) - 60 )  / 0.293);
    }
      
    syncWriteServos();  
}
