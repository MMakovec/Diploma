#include <Pololu3piPlus32U4.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

using namespace Pololu3piPlus32U4;

Motors motors;
Encoders encoders;

// DIMENSIONS
// m_CPR = 12, ratio 29.86:1 -> CPR = 12 x 29.86 = 358.3
float CPR = 358.3;
float WHEEL_RADIUS = 0.016; // in meters
//float WHEEL_DISTANCE = 0.075-0.0068; //96mm from outer sides of wheels - one wheel width to compensate
float WHEEL_DISTANCE = 0.083;
//float WHEEL_DISTANCE = 0.096-0.0068;

// Variables to store the encoder tick counts for each wheel
int16_t encoderLeft = 0;
int16_t encoderRight = 0;

// variables controlling the functionality
bool custom = 0;
bool robot = 0;
bool computer = 1;
bool odometry = 0;
bool finished = 1;
bool start = 0;

// variables for tracking orientation and position
float theta = 0;
float x = 0;
float y = 0;
char x_str[10] = "0", y_str[10] = "0", theta_str[10] = "0";
float distanceLeft, distanceRight, deltaX, deltaTheta;
float target_theta;

unsigned long lastRecievedTime = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  // Read initial encoder tick counts
  encoderLeft = encoders.getCountsLeft();
  encoderRight = encoders.getCountsRight();
}

void loop() {
  tracking(100);
  // If data is available read it
  if (Serial1.available()>0){
    delay(1);    
    char inData[Serial1.available()+1];
    int i = 0;
    while(Serial1.available()){
      inData[i] = char(Serial1.read());
      delay(1);
      i++;
    }
    inData[i] = '\0';
    Serial.print("Message: ");
    Serial.println(inData);

    phone_controls(inData);

    // STOP
    if(inData[0] == 's' && inData[1] == '\0'){
      motors.setSpeeds(0,0);

      if (odometry && computer){
        // send location to computer
        char* location = x_str;
        strcat(location,",");
        strcat(location,y_str);
        strcat(location,",");
        strcat(location,theta_str);
  
        Serial1.write(location);
        delay(1);
      }
      else{
        Serial1.write('1');
        delay(1);
      }
    }

    // RESET
    if(inData[0] == 'R' && inData[1] == '\0'){
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();
      encoderLeft = 0;
      encoderRight = 0;
      x = 0;
      y = 0;
      theta = 0;
      motors.setSpeeds(0,0);
    }
     
    //custom and pos func
    if(custom || robot || computer){
      /* 
       information will come in form of "a,b,c" where
       if c = 1
        a is left motor speed
        b is right motor speed
        c is delay
       if robot = 1
        a is x coordinate
        b is y coordinate
        c is orientation
       if computer = 1
        a is translational velocity
        b is angular velocity
        c is orientation
      */
       
      char a_str[10] = "";
      char b_str[10] = "";
      char c_str[10] = "";
      for(int i=0, case_=0;i<sizeof(inData)/sizeof(inData[0]);i++){
        if(inData[i] == ','){
          case_++;
          continue;
        }
        switch(case_){
          case 0:
            strncat(a_str,&inData[i],1);
            break;
          case 1:
            strncat(b_str,&inData[i],1);
            break;
          case 2:
            strncat(c_str,&inData[i],1);
            break;
        }
      }
      if(custom){
        int lSpeed = atoi(a_str), rSpeed = atoi(b_str), Delay = atoi(c_str);
        motors.setSpeeds(lSpeed,rSpeed);
        if(Delay != 0){
          delay(Delay);
          motors.setSpeeds(0,0);
        }
        custom = 0;
        Serial.println("Custom mode disabled");
        Serial1.write("Custom mode disabled");
      }
      if(robot){
        // NAVIGATION ON ROBOT
        Serial.println("Begin guide.");
        guide(atof(a_str),atof(b_str),atof(c_str));
      }
      if(computer){
        // INSTRUCTIONS FROM COMPUTER
  
        double v = atof(a_str);
        double w = atof(b_str);
        
        float wr = (2*v+w*WHEEL_DISTANCE)/(2*WHEEL_RADIUS);
        float wl = (2*v-w*WHEEL_DISTANCE)/(2*WHEEL_RADIUS);

        float vr = wr*WHEEL_RADIUS;
        float vl = wl*WHEEL_RADIUS;
        
        float ticks_per_meter = CPR/(2*PI*WHEEL_RADIUS); // number of ticks per meter
        
        float vr_ticks = vr*ticks_per_meter;
        float vl_ticks = vl*ticks_per_meter;

        Serial.println("ticks");
        Serial.println(vr_ticks);
        Serial.println(vl_ticks);
 
        // Check if data is relevant
        if (start){
          if ((millis()-lastRecievedTime) < 500){
            motors.setSpeeds(vl_ticks/8.075,vr_ticks/8.075);
          }
          // else wait for relevant data
          else{
            motors.setSpeeds(0,0);
          }
        }
        else{
          motors.setSpeeds(vl_ticks/8.075,vr_ticks/8.075);
        }

        start = 1;

        lastRecievedTime = millis();

        if (odometry) {
          // send location to computer
          char* location = x_str;
          strcat(location,",");
          strcat(location,y_str);
          strcat(location,",");
          strcat(location,theta_str);
      
          Serial1.write(location);
          delay(1);
        }
        else{
          Serial1.write('1');
          delay(1);
        }
      }
    }
    
    if(inData[0] == 'c' && inData[1] == '\0'){
      custom = 1;
      robot = 0;
      Serial.println("Custom mode enabled");
      Serial1.write("Custom mode enabled");
      while(Serial1.available()==0){
      } 
    }

    if(inData[0] == 'p' && inData[1] == '\0'){
      robot = 1;
      custom = 0;
      Serial.println("Robot control enabled");
      Serial1.write("Robot control enabled");
      while(Serial1.available()==0){
      }
    }
  }
}


void tracking(float freq){
  static uint8_t lastTrackTime = 0;

  if ((uint8_t)(millis() - lastTrackTime) >= (1000./freq)){
    
    // Check for encoder error
    bool errorLeft = encoders.checkErrorLeft();
    bool errorRight = encoders.checkErrorRight();
    if (errorLeft || errorRight){
      Serial1.write("Encoder error");
      return;
    }
    
    // Calculate the distance travelled by each wheel
    distanceRight = (encoders.getCountsRight()-encoderRight)/CPR * 2*PI*WHEEL_RADIUS;
    distanceLeft = (encoders.getCountsLeft()-encoderLeft)/CPR * 2*PI*WHEEL_RADIUS;
  
    // Calculate the change in position of the robot
    deltaX = (distanceLeft + distanceRight) / 2;
    deltaTheta = (distanceRight - distanceLeft) / WHEEL_DISTANCE;
  
    // Update the position and orientation of the robot
    x += deltaX * cos(theta);
    y += deltaX * sin(theta);
    theta += deltaTheta;

    // Makes sure theta is in [-PI,PI] range
    if (theta<-PI){
      theta = 2*PI + theta;
    }
    if (theta>PI){
      theta = theta - 2*PI;
    }

    // write float values to strings
    dtostrf(x,4,2,x_str);
    dtostrf(y,4,2,y_str);
    dtostrf(theta,4,2,theta_str);

    lastTrackTime = millis();
    encoderLeft = encoders.getCountsLeft();
    encoderRight = encoders.getCountsRight();
  }
}


void guide(float a, float b, float c){  

  // transform into robot coordinate sistem
  float x_r = (a-x)*cos(theta)+(b-y)*sin(theta);
  float y_r = (b-y)*cos(theta)-(a-x)*sin(theta); 
  
  // the angle we need to reach
  target_theta = atan2(y_r,x_r);

  // regulation
  float reg_val;

  // Align with our target and drive to the target in a straight line
  Serial.println("Aligning with target");
  Serial1.write("Aligning with target");
  Serial1.flush();
  while(abs(atan2(y_r,x_r))>=0.02){ // decide on accuracy

    // P regulation
    float Kp_theta = 10;
    reg_val = abs(atan2(y_r,x_r))*Kp_theta;

    // max reg_val
    if(reg_val>40) {
      reg_val = 40;
    }

    // minimal reg_val
    if(reg_val<25){
      reg_val = 25;
    }
    
    if (y_r<0){
      // We need to turn right
      motors.setSpeeds(reg_val,-reg_val);
    }
    else{
      // We need to turn left
      motors.setSpeeds(-reg_val,reg_val);
    }
    
    // Track the change in position with 200Hz frequency
    tracking(200);
    x_r = (a-x)*cos(theta)+(b-y)*sin(theta);
    y_r = (b-y)*cos(theta)-(a-x)*sin(theta);
  }
  Serial.println("Driving to target");
  Serial1.write("Driving to target");
  Serial1.flush();
  while (sqrt((a-x)*(a-x)+(b-y)*(b-y))>=0.02){ // decide on accuracy 
    // P regulation
    float Kp_dist = 50;
    reg_val = sqrt((a-x)*(a-x)+(b-y)*(b-y)) * Kp_dist;

    // minimal speed
    if(reg_val<60){
      reg_val = 60;
    }

    // adjust orientation
    // transform into robot coordinate sistem
    x_r = (a-x)*cos(theta)+(b-y)*sin(theta);
    y_r = (b-y)*cos(theta)-(a-x)*sin(theta);

    
    float Kp_adj = 20;
    float adj = abs(atan2(y_r,x_r))*Kp_adj;
    //adj = 0;
    // Check how we need to adjust
    if (y_r<0.01){
      // We need to adjust right
      motors.setSpeeds(reg_val+adj,reg_val-adj);
    }
    if (y_r>-0.01){
      // We need to adjust left
      motors.setSpeeds(reg_val-adj,reg_val+adj);
    }
    else{
      motors.setSpeeds(reg_val,reg_val);
    }
    tracking(100);  
  }
  Serial1.write("Set motor speed to zero");
  delay(3);
  motors.setSpeeds(0,0);
  
  Serial1.write(1);
  delay(1);
}

void phone_controls(char* inData){
      // PHONE CONTROLS
    //forwards
    if(inData[0] == 'f' && inData[1] == '\0' ){
      motors.setSpeeds(100,100);
      delay(500);
      motors.setSpeeds(0,0);
    }
    //backwards
    if(inData[0] == 'b' && inData[1] == '\0'){
      motors.setSpeeds(-100,-100);
      delay(500);
      motors.setSpeeds(0,0);
    }  
    //right
    if(inData[0] == 'r' && inData[2] == '\0'){
      motors.setSpeeds(100,-100);
      delay(300*0.75*float(inData[1]-'0'));
      motors.setSpeeds(0,0);
    }
    //left
    if(inData[0] == 'l' && inData[2] == '\0'){
      motors.setSpeeds(-100,100);
      delay(300*0.75*float(inData[1]-'0'));
      motors.setSpeeds(0,0);
    }
}
