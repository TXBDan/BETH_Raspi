/*
 * B.E.T.H.
 *
 */

#include <iostream>
#include <cmath>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <dynamixel.h>
#include "init.hpp"
#include "CommanderEx.hpp"

using namespace std;

Commander command = Commander();

commanderStruct commanderInput;
legStruct leg[6];
unsigned long currentTime;
unsigned long currentMicros;
unsigned long previousTime;
int tick;
int mode = 0;
int gaitSelect = 0;
int caseStep[6] = {1,2,1,2,1,2}; //for tripod gait
int runLoop = 1;

char xbeeDevice[] = "/dev/ttyAMA0";


/************************
 * Main
 * **********************/

int main()
{
	wiringPiSetup();
	
	pinMode(pwrLEDpin, OUTPUT);
	pinMode(faultLEDpin, OUTPUT);
	digitalWrite(pwrLEDpin, HIGH);
	digitalWrite(faultLEDpin, LOW);
	
	//// Setup UART for Xbee ////
	if( command.begin(xbeeDevice, B38400) )
		cout << "Initializing Xbee UART... OK" << endl;
	else{
		cout << "Initializing Xbee UART... FAIL" << endl;
		digitalWrite(faultLEDpin, HIGH);
	}
	
	//// Initialize USB2AX //// 
	if( dxl_initialize(0, 1) == 0 ){ // (/dev/ttyACM"0", "1"MBPs)
		cout << "Initializing USB2AX... FAIL" << endl;
		digitalWrite(faultLEDpin, HIGH);
	}
	else
		cout << "Initializing USB2AX... OK" << endl; 
	
	//// Check Servo Voltage (LiPO safety) ////
    delay (500);
    float voltage = (dxl_read_byte (1, 42)) / 10.0;
    if (voltage < 10.0){
		cout << "Checking System Voltage... " << voltage << "V FAIL" << endl;
		digitalWrite(faultLEDpin, HIGH);
	}
    else
		cout << "Checking System Voltage... " << voltage << "V OK" <<endl;
		
	
	cout << "B.E.T.H. IS ALIVE!" << endl;
	
	/* INITIAL FOOT POSITIONS */
    leg[RIGHT_FRONT].initialFootPos.x = round( sin(radians(COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR+initLegStretch) );
    leg[RIGHT_FRONT].initialFootPos.y = round( cos(radians(COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR+initLegStretch) );
    leg[RIGHT_FRONT].initialFootPos.z = LENGTH_TIBIA + rideHeightOffset;
    leg[RIGHT_FRONT].legBasePos.x = X_COXA;
    leg[RIGHT_FRONT].legBasePos.y = Y_COXA_FB;
    leg[RIGHT_FRONT].legBasePos.z = 0;
    
    leg[RIGHT_MIDDLE].initialFootPos.x = 0;
    leg[RIGHT_MIDDLE].initialFootPos.y = (LENGTH_COXA+LENGTH_FEMUR+initLegStretch);
    leg[RIGHT_MIDDLE].initialFootPos.z = LENGTH_TIBIA + rideHeightOffset;
    leg[RIGHT_MIDDLE].legBasePos.x = 0;
    leg[RIGHT_MIDDLE].legBasePos.y = Y_COXA_M;
    leg[RIGHT_MIDDLE].legBasePos.z = 0;
    
    leg[RIGHT_REAR].initialFootPos.x = round( sin(radians(-COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR+initLegStretch) );
    leg[RIGHT_REAR].initialFootPos.y = round( cos(radians(COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR+initLegStretch) );
    leg[RIGHT_REAR].initialFootPos.z = LENGTH_TIBIA + rideHeightOffset;
    leg[RIGHT_REAR].legBasePos.x = -X_COXA;
    leg[RIGHT_REAR].legBasePos.y = Y_COXA_FB;
    leg[RIGHT_REAR].legBasePos.z = 0;
    
    leg[LEFT_REAR].initialFootPos.x = round( sin(radians(-COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR+initLegStretch) );
    leg[LEFT_REAR].initialFootPos.y = -round( cos(radians(COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR+initLegStretch) );
    leg[LEFT_REAR].initialFootPos.z = LENGTH_TIBIA + rideHeightOffset;
    leg[LEFT_REAR].legBasePos.x = -X_COXA;
    leg[LEFT_REAR].legBasePos.y = -Y_COXA_FB;
    leg[LEFT_REAR].legBasePos.z = 0;
    
    leg[LEFT_MIDDLE].initialFootPos.x = 0;
    leg[LEFT_MIDDLE].initialFootPos.y = -(LENGTH_COXA+LENGTH_FEMUR+initLegStretch);
    leg[LEFT_MIDDLE].initialFootPos.z = LENGTH_TIBIA + rideHeightOffset;
    leg[LEFT_MIDDLE].legBasePos.x = 0;
    leg[LEFT_MIDDLE].legBasePos.y = -Y_COXA_M;
    leg[LEFT_MIDDLE].legBasePos.z = 0;
    
    leg[LEFT_FRONT].initialFootPos.x = round( sin(radians(COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR+initLegStretch) );
    leg[LEFT_FRONT].initialFootPos.y = -round( cos(radians(COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR+initLegStretch) );
    leg[LEFT_FRONT].initialFootPos.z = LENGTH_TIBIA + rideHeightOffset;
    leg[LEFT_FRONT].legBasePos.x = X_COXA;
    leg[LEFT_FRONT].legBasePos.y = -Y_COXA_FB;
    leg[LEFT_FRONT].legBasePos.z = 0;
    
	
	while(runLoop == 1){
		
		currentTime = millis();
		if(currentTime - previousTime >= SERVO_UPDATE_PERIOD){
			previousTime = currentTime;
			
			//currentMicros = micros();
			
			readCommandInputs();         // Read in input from controller
			
			if( gaitSelect == 0){
				tripodGait();
			}
			else if( gaitSelect == 1){
				waveGait(); 
			}
			else if( gaitSelect == 2){
				rippleGait();
			}
			
			runIK();
			

			//cout << "Crunch time (us): " << (micros()-currentMicros) << endl;
			

		}
		else usleep(10000); //10ms sleep
		
	} // end of loop
	
	
	
	// SHUTTING DOWN!!! 
	
	for(int id=1; id<=18; id++){
		dxl_write_word( id, 24, 0); // turn off torque
		usleep(1000);

	}
	dxl_terminate();
	
	command.end();
	
	
	
	
	return 0;
}


  


/*************************************************
  tripodGait()
  
**************************************************/
void tripodGait(){
  
    float sinRotZ, cosRotZ;
    intCoordsStruct gblInitFootPos;
    float rotSpeedOffsetX[6], rotSpeedOffsetY[6];
    floatCoordsStruct amplitude;
    int duration;
    int numTicks;
    int speedX, speedY;
    float speedR;

          
    if( (abs(commanderInput.Xspeed) > 5) || (abs(commanderInput.Yspeed) > 5) || (abs(commanderInput.Rspeed) > 5 ) ){
                             
        duration = 250;                               //duration of pull back step cycle (ms)
        numTicks = round(duration / SERVO_UPDATE_PERIOD ); //number of ticks in pull back step cycle
              
        speedX = 300*commanderInput.Xspeed/127;        //300mm/s top speed for 180mmm stride in one sec
        speedY = 300*commanderInput.Yspeed/127;        //
        speedR = 30*commanderInput.Rspeed/127;         //30deg/s top rotation speed
 
        
        amplitude.x = (speedX*duration/1000.0)/2.0; //amplitude is half of stride length
        amplitude.y = (speedY*duration/1000.0)/2.0;
        
        if( abs(commanderInput.Rspeed) > abs(commanderInput.Xspeed) && abs(commanderInput.Rspeed) > abs(commanderInput.Yspeed))
            amplitude.z = -abs(50*commanderInput.Rspeed/127);
        else if( abs(commanderInput.Xspeed) > abs(commanderInput.Yspeed) )
            amplitude.z = -abs(50*commanderInput.Xspeed/127);
        else
            amplitude.z = -abs(50*commanderInput.Yspeed/127);
        
        //cout << "X Speed: " << (amplitudeX*2/duration)*1000 << "mm/s" << endl;
        
        
        for( int legNum=0; legNum<6; legNum++){
			
			//cout << "Leg: " << legNum+1 << endl;
          
            //foot position in global space
            gblInitFootPos.x = leg[legNum].initialFootPos.x + leg[legNum].legBasePos.x;
            gblInitFootPos.y = leg[legNum].initialFootPos.y + leg[legNum].legBasePos.y;
            				          
                      
            switch (caseStep[legNum]){
            
                case 1: // forward raise and lower
                        
                    //cout << "Case 1, tick: " << tick << endl;
                    
                    //angle of rotation starting at half a stride back and building toward half a stride forward
                    sinRotZ = sin(-radians(-speedR/2.0 + speedR*((float)tick/numTicks)));
                    cosRotZ = cos(-radians(-speedR/2.0 + speedR*((float)tick/numTicks)));
                    
                    //rotate foot position about global Z axis
                    rotSpeedOffsetX[legNum] = gblInitFootPos.x*cosRotZ - gblInitFootPos.y*sinRotZ - gblInitFootPos.x;
                    rotSpeedOffsetY[legNum] = gblInitFootPos.x*sinRotZ + gblInitFootPos.y*cosRotZ - gblInitFootPos.y;

                    //footPos is stride plus rotational offset
                    leg[legNum].footPos.x = -amplitude.x*cos(M_PI*tick/numTicks) + rotSpeedOffsetX[legNum];
                    leg[legNum].footPos.y = -amplitude.y*cos(M_PI*tick/numTicks) + rotSpeedOffsetY[legNum];
                    leg[legNum].footPos.z =  amplitude.z*sin(M_PI*tick/numTicks);
                        
                    if( tick >= numTicks-1 ) caseStep[legNum] = 2;
                    break;
    
                    
                case 2: // pull back
                
					//cout << "Case 2, tick: " << tick << endl;
                    
                    sinRotZ = sin(-radians(speedR/2.0 - speedR*((float)tick/numTicks)));
                    cosRotZ = cos(-radians(speedR/2.0 - speedR*((float)tick/numTicks)));
                    
                    //rotate foot position about global Z axis
                    rotSpeedOffsetX[legNum] = gblInitFootPos.x*cosRotZ - gblInitFootPos.y*sinRotZ - gblInitFootPos.x;
                    rotSpeedOffsetY[legNum] = gblInitFootPos.x*sinRotZ + gblInitFootPos.y*cosRotZ - gblInitFootPos.y;
                
                    leg[legNum].footPos.x = amplitude.x - 2*amplitude.x*tick/numTicks  + rotSpeedOffsetX[legNum];
                    leg[legNum].footPos.y = amplitude.y - 2*amplitude.y*tick/numTicks  + rotSpeedOffsetY[legNum];
                    leg[legNum].footPos.z = 0;
                     
                    if( tick >= numTicks-1 ) caseStep[legNum] = 1;
                    break;             
       
          }// end of case statement
                             
        }// end of loop over legs
        if (tick < numTicks-1) tick++;
        else tick = 0;
      
    }//end if joystick active

}



/*************************************************
  waveGait()
  
**************************************************/
void waveGait(){
    
    float sinRotZ, cosRotZ;
    intCoordsStruct gblInitFootPos;
    float rotSpeedOffsetX[6], rotSpeedOffsetY[6];
    floatCoordsStruct amplitude;
    int duration;
    int numTicks;
    int speedX, speedY;
    float speedR;
    int numCasesPb;

          
    if( (abs(commanderInput.Xspeed) > 5) || (abs(commanderInput.Yspeed) > 5) || (abs(commanderInput.Rspeed) > 5 ) ){
                             
        duration = 250;                               //duration of one case (ms)      
        numTicks = round( duration / SERVO_UPDATE_PERIOD ); //total ticks divided into the two cases   
        numCasesPb = 5;
              
        speedX = 100*commanderInput.Xspeed/127;        //100mm/s top speed 
        speedY = 100*commanderInput.Yspeed/127;        //100mm/s top speed
        speedR = 30*commanderInput.Rspeed/127;         //15deg/s top rotation speed
        
        amplitude.x = (speedX*duration*numCasesPb/1000.0)/2.0; //amplitude is half of stride length
        amplitude.y = (speedY*duration*numCasesPb/1000.0)/2.0;
        
        if( abs(commanderInput.Rspeed) > abs(commanderInput.Xspeed) && abs(commanderInput.Rspeed) > abs(commanderInput.Yspeed))
            amplitude.z = -abs(50*commanderInput.Rspeed/127);
        else if( abs(commanderInput.Xspeed) > abs(commanderInput.Yspeed) )
            amplitude.z = -abs(50*commanderInput.Xspeed/127);
        else
            amplitude.z = -abs(50*commanderInput.Yspeed/127);
        
        //cout << "X Speed: " << (amplitude.x*2/(duration*numCasesPb))*1000 << "mm/s" << endl;

        
        for( int legNum=0; legNum<6; legNum++){
			
			//cout << "Leg: " << legNum+1 << endl;
            
            //foot position in global space
            gblInitFootPos.x = leg[legNum].initialFootPos.x + leg[legNum].legBasePos.x;
            gblInitFootPos.y = leg[legNum].initialFootPos.y + leg[legNum].legBasePos.y;
			
                      
            switch (caseStep[legNum]){
            
                case 1: // forward raise and lower
                
					//if(legNum==1) cout << "Case 1, tick: " << tick << endl;
                    
                    //angle of rotation starting at half a stride back and building toward half a stride forward
                    sinRotZ = sin(-radians( -speedR/2.0 + speedR*tick/numTicks ));
                    cosRotZ = cos(-radians( -speedR/2.0 + speedR*tick/numTicks ));
                    
                    //rotate foot position about global Z axis
                    rotSpeedOffsetX[legNum] = gblInitFootPos.x*cosRotZ - gblInitFootPos.y*sinRotZ - gblInitFootPos.x;
                    rotSpeedOffsetY[legNum] = gblInitFootPos.x*sinRotZ + gblInitFootPos.y*cosRotZ - gblInitFootPos.y;
                    
                    leg[legNum].footPos.x = round( -amplitude.x*cos(M_PI*tick/numTicks) + rotSpeedOffsetX[legNum] );
                    leg[legNum].footPos.y = round( -amplitude.y*cos(M_PI*tick/numTicks) + rotSpeedOffsetY[legNum] );
                    leg[legNum].footPos.z = round(  amplitude.z*sin(M_PI*tick/numTicks) );
                    //if(legNum==1) cout << "footPos X, Y, Z: " << leg[legNum].footPos.x << ", " << leg[legNum].footPos.y << ", " << leg[legNum].footPos.z << endl;
                        
                    if( tick >= numTicks-1 ) caseStep[legNum] = 2;
                    break;
                    
                case 2: // pull back slowly
                
					//if(legNum==1) cout << "Case 2, tick: " << tick << endl;
                    
                    //angle of rotation starting at half a stride back and building toward half a stride forward
                    sinRotZ = sin(-radians( speedR/2.0 - speedR*tick/(5.0*numTicks) ));
                    cosRotZ = cos(-radians( speedR/2.0 - speedR*tick/(5.0*numTicks) ));
                    
                    //rotate foot position about global Z axis
                    rotSpeedOffsetX[legNum] = gblInitFootPos.x*cosRotZ - gblInitFootPos.y*sinRotZ - gblInitFootPos.x;
                    rotSpeedOffsetY[legNum] = gblInitFootPos.x*sinRotZ + gblInitFootPos.y*cosRotZ - gblInitFootPos.y;
                
                    leg[legNum].footPos.x = round( amplitude.x - 2.0*amplitude.x*tick/(5.0*numTicks) + rotSpeedOffsetX[legNum] );
                    leg[legNum].footPos.y = round( amplitude.y - 2.0*amplitude.y*tick/(5.0*numTicks) + rotSpeedOffsetY[legNum] );
                    leg[legNum].footPos.z = 0;

                     
                    if( tick >= numTicks-1 ) caseStep[legNum] = 3;
                    break;             
       
				case 3: // pull back slowly
                
					//if(legNum==1) cout << "Case 3, tick: " << tick << endl;
                    
                    //angle of rotation starting at half a stride back and building toward half a stride forward
                    sinRotZ = sin(-radians( speedR/2.0 - speedR*(tick+numTicks)/(5.0*numTicks) ));
                    cosRotZ = cos(-radians( speedR/2.0 - speedR*(tick+numTicks)/(5.0*numTicks) ));
                    
                    //rotate foot position about global Z axis
                    rotSpeedOffsetX[legNum] = gblInitFootPos.x*cosRotZ - gblInitFootPos.y*sinRotZ - gblInitFootPos.x;
                    rotSpeedOffsetY[legNum] = gblInitFootPos.x*sinRotZ + gblInitFootPos.y*cosRotZ - gblInitFootPos.y;
                
                    leg[legNum].footPos.x = round( amplitude.x - 2.0*amplitude.x*(tick+numTicks)/(5.0*numTicks) + rotSpeedOffsetX[legNum] );
                    leg[legNum].footPos.y = round( amplitude.y - 2.0*amplitude.y*(tick+numTicks)/(5.0*numTicks) + rotSpeedOffsetY[legNum] );
                    leg[legNum].footPos.z = 0;

                    
                    if( tick >= numTicks-1 ) caseStep[legNum] = 4;
                    break;             
                    
                case 4: // pull back slowly
                
					//if(legNum==1) cout << "Case 4, tick: " << tick << endl;
                    
                    //angle of rotation starting at half a stride back and building toward half a stride forward
                    sinRotZ = sin(-radians( speedR/2.0 - speedR*(tick+2.0*numTicks)/(5.0*numTicks) ));
                    cosRotZ = cos(-radians( speedR/2.0 - speedR*(tick+2.0*numTicks)/(5.0*numTicks) ));
                    
                    //rotate foot position about global Z axis
                    rotSpeedOffsetX[legNum] = gblInitFootPos.x*cosRotZ - gblInitFootPos.y*sinRotZ - gblInitFootPos.x;
                    rotSpeedOffsetY[legNum] = gblInitFootPos.x*sinRotZ + gblInitFootPos.y*cosRotZ - gblInitFootPos.y;
                
                    leg[legNum].footPos.x = round( amplitude.x - 2.0*amplitude.x*(tick+2.0*numTicks)/(5.0*numTicks) + rotSpeedOffsetX[legNum] );
                    leg[legNum].footPos.y = round( amplitude.y - 2.0*amplitude.y*(tick+2.0*numTicks)/(5.0*numTicks) + rotSpeedOffsetY[legNum] );
                    leg[legNum].footPos.z = 0;

                     
                    if( tick >= numTicks-1 ) caseStep[legNum] = 5;
                    break;             
                    
				case 5: // pull back slowly
                
					//if(legNum==1) cout << "Case 5, tick: " << tick << endl;
                
                    //angle of rotation starting at half a stride back and building toward half a stride forward
                    sinRotZ = sin(-radians( speedR/2.0 - speedR*(tick+3.0*numTicks)/(5.0*numTicks) ));
                    cosRotZ = cos(-radians( speedR/2.0 - speedR*(tick+3.0*numTicks)/(5.0*numTicks) ));
                    
                    //rotate foot position about global Z axis
                    rotSpeedOffsetX[legNum] = gblInitFootPos.x*cosRotZ - gblInitFootPos.y*sinRotZ - gblInitFootPos.x;
                    rotSpeedOffsetY[legNum] = gblInitFootPos.x*sinRotZ + gblInitFootPos.y*cosRotZ - gblInitFootPos.y;
                    
                    leg[legNum].footPos.x = round( amplitude.x - 2.0*amplitude.x*(tick+3.0*numTicks)/(5.0*numTicks) + rotSpeedOffsetX[legNum] );
                    leg[legNum].footPos.y = round( amplitude.y - 2.0*amplitude.y*(tick+3.0*numTicks)/(5.0*numTicks) + rotSpeedOffsetY[legNum] );
                    leg[legNum].footPos.z = 0;

                     
                    if( tick >= numTicks-1 ) caseStep[legNum] = 6;
                    break;             
                    
                case 6: // pull back
                
					//if(legNum==1) cout << "Case 6, tick: " << tick << endl;
                
                    //angle of rotation starting at half a stride back and building toward half a stride forward
                    sinRotZ = sin(-radians( speedR/2.0 - speedR*(tick+4.0*numTicks)/(5.0*numTicks) ));
                    cosRotZ = cos(-radians( speedR/2.0 - speedR*(tick+4.0*numTicks)/(5.0*numTicks) ));
                    
                    //rotate foot position about global Z axis
                    rotSpeedOffsetX[legNum] = gblInitFootPos.x*cosRotZ - gblInitFootPos.y*sinRotZ - gblInitFootPos.x;
                    rotSpeedOffsetY[legNum] = gblInitFootPos.x*sinRotZ + gblInitFootPos.y*cosRotZ - gblInitFootPos.y;
                    
                    leg[legNum].footPos.x = round( amplitude.x - 2.0*amplitude.x*(tick+4.0*numTicks)/(5.0*numTicks) + rotSpeedOffsetX[legNum] );
                    leg[legNum].footPos.y = round( amplitude.y - 2.0*amplitude.y*(tick+4.0*numTicks)/(5.0*numTicks) + rotSpeedOffsetY[legNum] );
                    leg[legNum].footPos.z = 0;

                     
                    if( tick >= numTicks-1 ) caseStep[legNum] = 1;
                    break;             
          }// end of case statement
                             
        }// end of loop over legs
        if (tick < numTicks-1) tick++;
        else tick = 0;
      
    }//end if joystick active

}



/*************************************************
  rippleGait()
  
**************************************************/
void rippleGait(){
    
    float sinRotZ, cosRotZ;
    intCoordsStruct gblInitFootPos;
    float rotSpeedOffsetX[6], rotSpeedOffsetY[6];
    floatCoordsStruct amplitude;
    int duration;
    int numTicks;
    int speedX, speedY;
    float speedR;
    int numCasesPb;
    
    if( (abs(commanderInput.Xspeed) > 5) || (abs(commanderInput.Yspeed) > 5) || (abs(commanderInput.Rspeed) > 5 ) ){
                             
        duration = 125;                               //duration of one case (ms)     
        numTicks = round( duration / SERVO_UPDATE_PERIOD ); //total ticks in one case
        numCasesPb = 4;  
              
        speedX = 200*commanderInput.Xspeed/127;        //(mm/s) top speed
        speedY = 200*commanderInput.Yspeed/127;        //(mm/s) top speed
        speedR = 30*commanderInput.Rspeed/127;         //(deg/s) top rotation speed
                    
        amplitude.x = (speedX*duration*numCasesPb/1000.0)/2.0; //amplitude is half of stride length
        amplitude.y = (speedY*duration*numCasesPb/1000.0)/2.0;
        
        if( abs(commanderInput.Rspeed) > abs(commanderInput.Xspeed) && abs(commanderInput.Rspeed) > abs(commanderInput.Yspeed))
            amplitude.z = -abs(50*commanderInput.Rspeed/127);
        else if( abs(commanderInput.Xspeed) > abs(commanderInput.Yspeed) )
            amplitude.z = -abs(50*commanderInput.Xspeed/127);
        else
            amplitude.z = -abs(50*commanderInput.Yspeed/127);
        
        //cout << "X Speed: " << (amplitude.x*2/(duration*numCasesPb))*1000 << "mm/s" << endl;

        
        for( int legNum=0; legNum<6; legNum++){
			
			//cout << "Leg: " << legNum+1 << endl;
            
            //foot position in global space
            gblInitFootPos.x = leg[legNum].initialFootPos.x + leg[legNum].legBasePos.x;
            gblInitFootPos.y = leg[legNum].initialFootPos.y + leg[legNum].legBasePos.y;
            
            
            switch (caseStep[legNum]){
            
                case 1: //forward raise
                
					//cout << "Case 1, Leg: " << legNum << " Tick: " << tick << endl;
                    
                    //angle of rotation starting at half a stride back and building toward half a stride forward
                    sinRotZ = sin(-radians( -speedR/2.0 + speedR*tick/(2.0*numTicks) ));
                    cosRotZ = cos(-radians( -speedR/2.0 + speedR*tick/(2.0*numTicks) ));
                    
                    //rotate foot position about global Z axis
                    rotSpeedOffsetX[legNum] = gblInitFootPos.x*cosRotZ - gblInitFootPos.y*sinRotZ - gblInitFootPos.x;
                    rotSpeedOffsetY[legNum] = gblInitFootPos.x*sinRotZ + gblInitFootPos.y*cosRotZ - gblInitFootPos.y;
                    
                    leg[legNum].footPos.x = round( -amplitude.x*cos(M_PI*tick/(2.0*numTicks)) + rotSpeedOffsetX[legNum] );
                    leg[legNum].footPos.y = round( -amplitude.y*cos(M_PI*tick/(2.0*numTicks)) + rotSpeedOffsetY[legNum] );
                    leg[legNum].footPos.z = round(  amplitude.z*sin(M_PI*tick/(2.0*numTicks)) );
                    //if(legNum==1) cout << "footPos X, Y, Z: " << leg[legNum].footPos.x << ", " << leg[legNum].footPos.y << ", " << leg[legNum].footPos.z << endl;

					
                    if( tick >= numTicks-1 ) caseStep[legNum] = 2;
                    break;
                    
                case 2: // forward lower
                
					//cout << "Case 2, Leg: " << legNum << " Tick: " << tick << endl;
                    
                    //angle of rotation starting at half a stride back and building toward half a stride forward
                    sinRotZ = sin(-radians( -speedR/2.0 + speedR*(tick+numTicks)/(2.0*numTicks) ));
                    cosRotZ = cos(-radians( -speedR/2.0 + speedR*(tick+numTicks)/(2.0*numTicks) ));
                    
                    //rotate foot position about global Z axis
                    rotSpeedOffsetX[legNum] = gblInitFootPos.x*cosRotZ - gblInitFootPos.y*sinRotZ - gblInitFootPos.x;
                    rotSpeedOffsetY[legNum] = gblInitFootPos.x*sinRotZ + gblInitFootPos.y*cosRotZ - gblInitFootPos.y;
                    
                    leg[legNum].footPos.x = round( -amplitude.x*cos(M_PI*(tick+numTicks)/(2.0*numTicks)) + rotSpeedOffsetX[legNum] );
                    leg[legNum].footPos.y = round( -amplitude.y*cos(M_PI*(tick+numTicks)/(2.0*numTicks)) + rotSpeedOffsetY[legNum] );
                    leg[legNum].footPos.z = round(  amplitude.z*sin(M_PI*(tick+numTicks)/(2.0*numTicks)) );
                    //if(legNum==1) cout << "footPos X, Y, Z: " << leg[legNum].footPos.x << ", " << leg[legNum].footPos.y << ", " << leg[legNum].footPos.z << endl;
                    
				 
                    if( tick >= numTicks-1 ) caseStep[legNum] = 3;
                    break;             
       
				case 3: // pull back slowly
                
					//cout << "Case 2, tick: " << tick << endl;
                    
                    //angle of rotation starting at half a stride back and building toward half a stride forward
                    sinRotZ = sin(-radians( speedR/2.0 - speedR*tick/(4.0*numTicks) ));
                    cosRotZ = cos(-radians( speedR/2.0 - speedR*tick/(4.0*numTicks) ));
                    
                    //rotate foot position about global Z axis
                    rotSpeedOffsetX[legNum] = gblInitFootPos.x*cosRotZ - gblInitFootPos.y*sinRotZ - gblInitFootPos.x;
                    rotSpeedOffsetY[legNum] = gblInitFootPos.x*sinRotZ + gblInitFootPos.y*cosRotZ - gblInitFootPos.y;
                    
                    leg[legNum].footPos.x = round( amplitude.x - 2.0*amplitude.x*tick/(4.0*numTicks) + rotSpeedOffsetX[legNum] );
                    leg[legNum].footPos.y = round( amplitude.y - 2.0*amplitude.y*tick/(4.0*numTicks) + rotSpeedOffsetY[legNum] );
                    leg[legNum].footPos.z = 0;

                     
                    if( tick >= numTicks-1 ) caseStep[legNum] = 4;
                    break;             
                    
                case 4: // pull back slowly
                
					//cout << "Case 2, tick: " << tick << endl;
                    
                    //angle of rotation starting at half a stride back and building toward half a stride forward
                    sinRotZ = sin(-radians( speedR/2.0 - speedR*(tick+numTicks)/(4.0*numTicks) ));
                    cosRotZ = cos(-radians( speedR/2.0 - speedR*(tick+numTicks)/(4.0*numTicks) ));
                    
                    //rotate foot position about global Z axis
                    rotSpeedOffsetX[legNum] = gblInitFootPos.x*cosRotZ - gblInitFootPos.y*sinRotZ - gblInitFootPos.x;
                    rotSpeedOffsetY[legNum] = gblInitFootPos.x*sinRotZ + gblInitFootPos.y*cosRotZ - gblInitFootPos.y;
                    
                    leg[legNum].footPos.x = round( amplitude.x - 2.0*amplitude.x*(tick+numTicks)/(4.0*numTicks) + rotSpeedOffsetX[legNum] );
                    leg[legNum].footPos.y = round( amplitude.y - 2.0*amplitude.y*(tick+numTicks)/(4.0*numTicks) + rotSpeedOffsetY[legNum] );
                    leg[legNum].footPos.z = 0;

                     
                    if( tick >= numTicks-1 ) caseStep[legNum] = 5;
                    break;             
                    
				case 5: // pull back slowly
                
					//cout << "Case 2, tick: " << tick << endl;
                    
                    //angle of rotation starting at half a stride back and building toward half a stride forward
                    sinRotZ = sin(-radians( speedR/2.0 - speedR*(tick+2.0*numTicks)/(4.0*numTicks) ));
                    cosRotZ = cos(-radians( speedR/2.0 - speedR*(tick+2.0*numTicks)/(4.0*numTicks) ));
                    
                    //rotate foot position about global Z axis
                    rotSpeedOffsetX[legNum] = gblInitFootPos.x*cosRotZ - gblInitFootPos.y*sinRotZ - gblInitFootPos.x;
                    rotSpeedOffsetY[legNum] = gblInitFootPos.x*sinRotZ + gblInitFootPos.y*cosRotZ - gblInitFootPos.y;
                    
                    leg[legNum].footPos.x = round( amplitude.x - 2.0*amplitude.x*(tick+2.0*numTicks)/(4.0*numTicks) + rotSpeedOffsetX[legNum] );
                    leg[legNum].footPos.y = round( amplitude.y - 2.0*amplitude.y*(tick+2.0*numTicks)/(4.0*numTicks) + rotSpeedOffsetY[legNum] );
                    leg[legNum].footPos.z = 0;

                     
                    if( tick >= numTicks-1 ) caseStep[legNum] = 6;
                    break;             
                    
                case 6: // pull back
                
					//cout << "Case 2, tick: " << tick << endl;
                    
                    //angle of rotation starting at half a stride back and building toward half a stride forward
                    sinRotZ = sin(-radians( speedR/2.0 - speedR*(tick+3.0*numTicks)/(4.0*numTicks) ));
                    cosRotZ = cos(-radians( speedR/2.0 - speedR*(tick+3.0*numTicks)/(4.0*numTicks) ));
                    
                    //rotate foot position about global Z axis
                    rotSpeedOffsetX[legNum] = gblInitFootPos.x*cosRotZ - gblInitFootPos.y*sinRotZ - gblInitFootPos.x;
                    rotSpeedOffsetY[legNum] = gblInitFootPos.x*sinRotZ + gblInitFootPos.y*cosRotZ - gblInitFootPos.y;
                    
                    leg[legNum].footPos.x = round( amplitude.x - 2.0*amplitude.x*(tick+3.0*numTicks)/(4.0*numTicks) + rotSpeedOffsetX[legNum] );
                    leg[legNum].footPos.y = round( amplitude.y - 2.0*amplitude.y*(tick+3.0*numTicks)/(4.0*numTicks) + rotSpeedOffsetY[legNum] );
                    leg[legNum].footPos.z = 0;

                     
                    if( tick >= numTicks-1 ) caseStep[legNum] = 1;
                    break;      
                                          
                           
          }// end of case statement
                             
        }// end of loop over legs
        if (tick < numTicks-1) tick++;
        else tick = 0;
      
    }//end if joystick active

}




/*************************************************
  readCommandInputs()
  Reads input from Commander controller and saves them as variables
**************************************************/
void readCommandInputs(){
  
  
	if(command.ReadMsgs() > 0){
	  
		//digitalWrite( ledPin, HIGH-digitalRead(ledPin) );
		
		
		if( command.buttons&BUT_R1 ){
			mode = 0;   
			cout << "Mode 0, Normal Operation" << endl;   
		}
		if( command.buttons&BUT_R2 ){
			mode = 1;
			cout << "Mode 1, Translation" << endl;
		}
		if( command.buttons&BUT_R3 ){
			mode = 2;
			cout << "Mode 2, Rotation" << endl;
		}
		
		if( command.buttons&BUT_L6 ){
			gaitSelect = 0;   
			cout << "Tripod Gait" << endl;   
			caseStep[0] = 1; 
			caseStep[1] = 2;
			caseStep[2] = 1;
			caseStep[3] = 2;
			caseStep[4] = 1;
			caseStep[5] = 2; 
		}
		if( command.buttons&BUT_L5 ){
			gaitSelect = 1;
			cout << "Wave Gait" << endl;
			caseStep[0] = 4; 
			caseStep[1] = 5;
			caseStep[2] = 6;
			caseStep[3] = 3;
			caseStep[4] = 2;
			caseStep[5] = 1;
		}
		if( command.buttons&BUT_L4 ){
			gaitSelect = 2;
			cout << "Ripple Gait" << endl;
			caseStep[0] = 2; 
			caseStep[1] = 6;
			caseStep[2] = 4;
			caseStep[3] = 1;
			caseStep[4] = 3;
			caseStep[5] = 5;
		}
		if( command.buttons&BUT_LT && command.buttons&BUT_RT ){
			
			cout << "SHUTTING DOWN!" << endl;		
			runLoop = 0;			
		}
          
		if( mode == 1 ){
			commanderInput.bodyTransX = command.leftV / 3;
			commanderInput.bodyTransY = command.leftH / 3;
			commanderInput.bodyTransZ = command.rightV / 3;  
			commanderInput.Rspeed = 0;
			commanderInput.Xspeed = 0;
			commanderInput.Yspeed = 0;  
			commanderInput.bodyRotX = 0;
			commanderInput.bodyRotY = 0;
			commanderInput.bodyRotZ = 0; 
		}
		else if( mode == 2 ){
			commanderInput.bodyRotX = command.leftH / 10.0;
			commanderInput.bodyRotY = command.leftV / 10.0;
			commanderInput.bodyRotZ = -command.rightH / 7.0;      
			commanderInput.Rspeed = 0;
			commanderInput.Xspeed = 0;
			commanderInput.Yspeed = 0;  
			commanderInput.bodyTransX = 0;
			commanderInput.bodyTransY = 0;
			commanderInput.bodyTransZ = 0; 
		}
		else{		
			commanderInput.bodyTransX = 0; 
			commanderInput.bodyTransY = 0;
			commanderInput.bodyTransZ = 0; 
			commanderInput.bodyRotX = 0;
			commanderInput.bodyRotY = 0;
			commanderInput.bodyRotZ = 0; 		
			
			if( abs(command.leftV) > 5 ){
				commanderInput.Xspeed = command.leftV;
			}
			else commanderInput.Xspeed = 0;
     
			if( abs(command.leftH) > 5 ){   
				commanderInput.Yspeed = command.leftH;
			}
			else commanderInput.Yspeed = 0;
      
			if( abs(command.rightH) > 5 ){
				commanderInput.Rspeed = -command.rightH;
			}
			else commanderInput.Rspeed = 0;
 
		}
  }
}


float radians( float deg){
	return deg*M_PI/180.0;
}

float degrees( float rad){
	return rad*180.0/M_PI;
}

