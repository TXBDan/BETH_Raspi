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
int caseStep[6] = {1,3,1,3,1,3}; //for tripod gait

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
	
	// Setup UART for Xbee //
	if( command.begin(xbeeDevice, B38400) )
		cout << "Initializing Xbee UART... OK" << endl;
	else{
		cout << "Initializing Xbee UART... FAIL" << endl;
		digitalWrite(faultLEDpin, HIGH);
	}
	
	
	// Initialize USB2AX // 
	if( dxl_initialize(0, 1) == 0 ){ // (/dev/ttyACMX, XMBPs)
		cout << "Initializing USB2AX... FAIL" << endl;
		digitalWrite(faultLEDpin, HIGH);
	}
	else
		cout << "Initializing USB2AX... OK" << endl; 
	
	
	//// Servo voltage check (LiPO safety) ////
    delay (500);
    float voltage = (dxl_read_word (1, 42)) / 10.0;
    if (voltage < 10.0){
		cout << "Checking System Voltage... FAIL " << voltage << "V" << endl;
		digitalWrite(faultLEDpin, HIGH);
	}
    else
		cout << "Checking System Voltage... OK " << voltage << "V" << endl;
		
	
	cout << "B.E.T.H. IS ALIVE!" << endl;
	
	/* INITIAL FOOT POSITIONS */
    leg[RIGHT_FRONT].initialFootPos.x = round( sin(radians(COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR) );
    leg[RIGHT_FRONT].initialFootPos.y = round( cos(radians(COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR) );
    leg[RIGHT_FRONT].initialFootPos.z = LENGTH_TIBIA + rideHeightOffset;
    leg[RIGHT_FRONT].legBasePos.x = X_COXA;
    leg[RIGHT_FRONT].legBasePos.y = Y_COXA_FB;
    leg[RIGHT_FRONT].legBasePos.z = 0;
    
    leg[RIGHT_MIDDLE].initialFootPos.x = 0;
    leg[RIGHT_MIDDLE].initialFootPos.y = (LENGTH_COXA+LENGTH_FEMUR);
    leg[RIGHT_MIDDLE].initialFootPos.z = LENGTH_TIBIA + rideHeightOffset;
    leg[RIGHT_MIDDLE].legBasePos.x = 0;
    leg[RIGHT_MIDDLE].legBasePos.y = Y_COXA_M;
    leg[RIGHT_MIDDLE].legBasePos.z = 0;
    
    leg[RIGHT_REAR].initialFootPos.x = round( sin(radians(-COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR) );
    leg[RIGHT_REAR].initialFootPos.y = round( cos(radians(COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR) );
    leg[RIGHT_REAR].initialFootPos.z = LENGTH_TIBIA + rideHeightOffset;
    leg[RIGHT_REAR].legBasePos.x = -X_COXA;
    leg[RIGHT_REAR].legBasePos.y = Y_COXA_FB;
    leg[RIGHT_REAR].legBasePos.z = 0;
    
    leg[LEFT_REAR].initialFootPos.x = round( sin(radians(-COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR) );
    leg[LEFT_REAR].initialFootPos.y = -round( cos(radians(COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR) );
    leg[LEFT_REAR].initialFootPos.z = LENGTH_TIBIA + rideHeightOffset;
    leg[LEFT_REAR].legBasePos.x = -X_COXA;
    leg[LEFT_REAR].legBasePos.y = -Y_COXA_FB;
    leg[LEFT_REAR].legBasePos.z = 0;
    
    leg[LEFT_MIDDLE].initialFootPos.x = 0;
    leg[LEFT_MIDDLE].initialFootPos.y = -(LENGTH_COXA+LENGTH_FEMUR);
    leg[LEFT_MIDDLE].initialFootPos.z = LENGTH_TIBIA + rideHeightOffset;
    leg[LEFT_MIDDLE].legBasePos.x = 0;
    leg[LEFT_MIDDLE].legBasePos.y = -Y_COXA_M;
    leg[LEFT_MIDDLE].legBasePos.z = 0;
    
    leg[LEFT_FRONT].initialFootPos.x = round( sin(radians(COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR) );
    leg[LEFT_FRONT].initialFootPos.y = -round( cos(radians(COXA_ANGLE))*(LENGTH_COXA+LENGTH_FEMUR) );
    leg[LEFT_FRONT].initialFootPos.z = LENGTH_TIBIA + rideHeightOffset;
    leg[LEFT_FRONT].legBasePos.x = X_COXA;
    leg[LEFT_FRONT].legBasePos.y = -Y_COXA_FB;
    leg[LEFT_FRONT].legBasePos.z = 0;
	
	
	while(1){
		
		currentTime = millis();
		//currentMicros = micros();
		if(currentTime - previousTime >= SERVO_UPDATE_PERIOD){
			previousTime = currentTime;
	
			//cout << "Main Loop" << endl;
	
	
			
			readCommandInputs();                        // Read in input from controller
			
			runIK();
			
			tripodGait();  
			//rippleGait();
			//cout << "Crunch time (us): " << (micros()-currentMicros) << endl;
			

		}
		else usleep(5000);
	}
	
	return 0;
}

/*************************************************
  tripodGait()
  
**************************************************/
void tripodGait(){
  
    float sinRotZ, cosRotZ;
    int totalX, totalY;
    float strideRotOffsetX[6], strideRotOffsetY[6];
    int height = -35;
    int duration;
    int numTicks;
    int speedX, speedY, speedR;
          
    if( (abs(commanderInput.Xspeed) > 5) || (abs(commanderInput.Yspeed) > 5) || (abs(commanderInput.Rspeed) > 5 ) ){
                             
        duration = 800;                               //duration of one step cycle (ms)      
        numTicks = round(duration / SERVO_UPDATE_PERIOD / 4.0); //total ticks divided into the four cases   
              
        speedX = 180*commanderInput.Xspeed/127;        //200mm/s top speed
        speedY = 180*commanderInput.Yspeed/127;        //200mm/s top speed
        speedR = 40*commanderInput.Rspeed/127;         //40deg/s top rotation speed
                    
        sinRotZ = sin(radians(speedR));
        cosRotZ = cos(radians(speedR));
                 
        for( int legNum=0; legNum<6; legNum++){
          
            totalX = leg[legNum].initialFootPos.x + leg[legNum].legBasePos.x; 
            totalY = leg[legNum].initialFootPos.y + leg[legNum].legBasePos.y;
            
            strideRotOffsetX[legNum] = totalY*sinRotZ + totalX*cosRotZ - totalX;  
            strideRotOffsetY[legNum] = totalY*cosRotZ - totalX*sinRotZ - totalY; 
            
            if(abs(speedR*5) > abs(speedX) && abs(speedR*5) > abs(speedY))  
				height = -abs(speedR);  
            else{
				if(abs(speedX) >= abs(speedY)) 
					height = -abs(speedX/5);
				else 
					height = -abs(speedY/5);  
            } 
            
             
            switch (caseStep[legNum]){
            
                case 1: //forward raise
                                      
                    leg[legNum].footPos.x = ((speedX + strideRotOffsetX[legNum])*tick*SERVO_UPDATE_PERIOD)/1000.0 - (speedX + strideRotOffsetX[legNum])/4.0; 
                    leg[legNum].footPos.y = ((speedY + strideRotOffsetY[legNum])*tick*SERVO_UPDATE_PERIOD)/1000.0 - (speedY + strideRotOffsetY[legNum])/4.0;
                    leg[legNum].footPos.z = (height*tick)/numTicks;
                        
                    if( tick >= numTicks-1 ) caseStep[legNum] = 2;
                    break;
                    
                case 2: // forward lower
                
                    leg[legNum].footPos.x = ((speedX + strideRotOffsetX[legNum])*tick*SERVO_UPDATE_PERIOD)/1000.0;
                    leg[legNum].footPos.y = ((speedY + strideRotOffsetY[legNum])*tick*SERVO_UPDATE_PERIOD)/1000.0;
                    leg[legNum].footPos.z = height - (height*tick)/numTicks ;
                     
                    if( tick >= numTicks-1 ) caseStep[legNum] = 3;
                    break;
                  
                case 3: // down pull back
                
                    leg[legNum].footPos.x = -((speedX + strideRotOffsetX[legNum])*tick*SERVO_UPDATE_PERIOD)/1000.0 + (speedX + strideRotOffsetX[legNum])/4.0;
                    leg[legNum].footPos.y = -((speedY + strideRotOffsetY[legNum])*tick*SERVO_UPDATE_PERIOD)/1000.0 + (speedY + strideRotOffsetY[legNum])/4.0;
                    leg[legNum].footPos.z = 0;
                        
                    if( tick >= numTicks-1 ) caseStep[legNum] = 4;
                    break;
                    
                case 4: // down pull back
                    
                    leg[legNum].footPos.x = -((speedX + strideRotOffsetX[legNum])*tick*SERVO_UPDATE_PERIOD)/1000.0;
                    leg[legNum].footPos.y = -((speedY + strideRotOffsetY[legNum])*tick*SERVO_UPDATE_PERIOD)/1000.0;
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
  
  //cout << "Read Commander Inputs" << endl;
  
  //commanderInput.Xspeed = 80;
  //commanderInput.Yspeed = 100;
  //commanderInput.Rspeed = 80;
  
  
  if(command.ReadMsgs() > 0){
	  cout << "We have message!!!" << endl;
      //digitalWrite( ledPin, HIGH-digitalRead(ledPin) );
      
      //read in Command controller inputs
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
          
      if( command.buttons&BUT_RT ){
          commanderInput.bodyRotX = -(float)command.leftH / 10.0;
          commanderInput.bodyRotY = (float)command.leftV / 10.0;
          commanderInput.bodyRotZ = -(float)command.rightH / 7.0;      
          commanderInput.Rspeed = 0;
          commanderInput.Xspeed = 0;
          commanderInput.Yspeed = 0;  
      }
      else{
          commanderInput.bodyRotX = 0.0;
          commanderInput.bodyRotY = 0.0;
          commanderInput.bodyRotZ = 0.0;  
      }
      
      if( command.buttons&BUT_LT ){
          commanderInput.bodyTransX = command.leftV / 3;
          commanderInput.bodyTransY = command.leftH / 3;
          commanderInput.bodyTransZ = command.rightV / 3;  
          commanderInput.Rspeed = 0;
          commanderInput.Xspeed = 0;
          commanderInput.Yspeed = 0;  
      }
      else{
          commanderInput.bodyTransX = 0; 
          commanderInput.bodyTransY = 0;
          commanderInput.bodyTransZ = 0;  
      }
  }
}



float radians( float deg){
	return deg*M_PI/180.0;
}

float degrees( float rad){
	return rad*180.0/M_PI;
}

