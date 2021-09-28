/////////////  MPU6050 /////////////////
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 18  // use pin 18 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
int mpu_a;
int mpu_180;
float mpu_ori;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


///////// Ps2 Shield ///////
#include <SoftwareSerial.h>
#include <Cytron_PS2Shield.h>
Cytron_PS2Shield ps2(10, 11); // SoftwareSerial: Rx and Tx pin


////////MOTOR DRIVER///////
const int M1_pwm=4;//4
const int M2_pwm=5;//5
const int M3_pwm=6;//6
const int M4_pwm=7 ;//7
 
const int M1_dir=22;//22
const int M2_dir=23;//23
const int M3_dir=24;//24//////
const int M4_dir=25;//25

//AXIS ANALOG CONTROLLER//
int xr_axis ;
int xl_axis;
int yr_axis;
int yl_axis;

/////////////  PID /////////////////
#include <PIDController.h>

// Objects
PIDController pid; // Create an instance of the PID controller class, called "pid"
int pwmOut;
int Setpoint = 180;
int Output;


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  
///////////////////////// MPU 6050 /////////////////////////
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
   
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(100); //220
    mpu.setYGyroOffset(88); //76
    mpu.setZGyroOffset(113); //-85
    mpu.setXAccelOffset(457);
    mpu.setYAccelOffset(-1252);
    mpu.setZAccelOffset(1811); //1826 
    

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        } 
    else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    
///////////////////////// MOTOR DECLARATION ///////////////////////// 
  pinMode(M1_pwm,OUTPUT);
  pinMode(M1_dir,OUTPUT);
  pinMode(M2_pwm,OUTPUT);
  pinMode(M2_dir,OUTPUT);
  pinMode(M3_pwm,OUTPUT);
  pinMode(M3_dir,OUTPUT);
  pinMode(M4_pwm,OUTPUT);
  pinMode(M4_dir,OUTPUT);

  // Pin for (pwm,value)value in initial coondition
  analogWrite(22, 0);
  analogWrite(23, 0);
  analogWrite(24, 0);
  analogWrite(25, 0);

  ps2.begin(57600); //ori 57600
  Serial.begin(57600);


///////////////////////// PID DECLARATION /////////////////////////
  pid.begin();          // initialize the PID instance
  pid.setpoint(Setpoint);    // The "goal" the PID controller tries to "reach"
  pid.tune(4, 0.2, 0);    // Tune the PID, arguments: kP, kI, kD             // 1, 0.31, 0/
  pid.limit(-255, 255);    // Limit the PID output between 0 and 255, this is important to get rid of integral windup!
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
   
  MPU();
  AnalogMovement();
  PID_MPU();

  
}


/*
///////////////////////////////////////////////////////////////////////////////////////////////////
                                    PID MPU
///////////////////////////////////////////////////////////////////////////////////////////////////
*/ 
void PID_MPU(){
    // PID //
//    Input = mpu_a;
    Output = pid.compute(mpu_180);

    // action (pwm Setting)//
      if (Output > 8){
        pwmOut = 80;
        }
      else if (Output < -8){
        pwmOut = 80;
        }
      else{
        pwmOut = abs(Output);
        }  

    if(ps2.readButton(PS2_UP) == 0)   //Moves Forward
   {
      // Direction Setting //
       if (Output < 9 && Output > -9){
        D_FW();     // move foward
        Serial.print("Depann");
       }
       
       else if (mpu_180 < Setpoint){    //angle correction
        D_R();
        analogWrite(M1_pwm,pwmOut);
        analogWrite(M2_pwm,pwmOut);
        analogWrite(M3_pwm,pwmOut);
        analogWrite(M4_pwm,pwmOut);
        }
   
       else if (mpu_180 > Setpoint){    //angle correction
        D_L();
        analogWrite(M1_pwm,pwmOut);
        analogWrite(M2_pwm,pwmOut);
        analogWrite(M3_pwm,pwmOut);
        analogWrite(M4_pwm,pwmOut);
        }
   }

   else if(ps2.readButton(PS2_DOWN)==0)
   {
    // Direction Setting //
       if (Output < 9 && Output > -9){
        D_BW();                      // move backward
        Serial.print("Belakang");
       }
       
       else if (mpu_180 < Setpoint){    //angle correction
        D_R();
        analogWrite(M1_pwm,pwmOut);
        analogWrite(M2_pwm,pwmOut);
        analogWrite(M3_pwm,pwmOut);
        analogWrite(M4_pwm,pwmOut);
        }
   
       else if (mpu_180 > Setpoint){    //angle correction
        D_L();
        analogWrite(M1_pwm,pwmOut);
        analogWrite(M2_pwm,pwmOut);
        analogWrite(M3_pwm,pwmOut);
        analogWrite(M4_pwm,pwmOut);
       }
   }

   else if (ps2.readButton(PS2_LEFT)==0){
    // Direction Setting //
       if (Output < 9 && Output > -9){
        D_TL();                      // crab left
        Serial.print("crab left");
       }
       
       else if (mpu_180 < Setpoint){    //angle correction
        D_R();
        analogWrite(M1_pwm,pwmOut);
        analogWrite(M2_pwm,pwmOut);
        analogWrite(M3_pwm,pwmOut);
        analogWrite(M4_pwm,pwmOut);
        }
   
       else if (mpu_180 > Setpoint){    //angle correction
        D_L();
        analogWrite(M1_pwm,pwmOut);
        analogWrite(M2_pwm,pwmOut);
        analogWrite(M3_pwm,pwmOut);
        analogWrite(M4_pwm,pwmOut);
       }
   }

    else if (ps2.readButton(PS2_RIGHT)==0){
    // Direction Setting //
       if (Output < 9 && Output > -9){
        D_TR();                      // crab left
        Serial.print("crab right");
       }
       
       else if (mpu_180 < Setpoint){    //angle correction
        D_R();
        analogWrite(M1_pwm,pwmOut);
        analogWrite(M2_pwm,pwmOut);
        analogWrite(M3_pwm,pwmOut);
        analogWrite(M4_pwm,pwmOut);
        }
   
       else if (mpu_180 > Setpoint){    //angle correction
        D_L();
        analogWrite(M1_pwm,pwmOut);
        analogWrite(M2_pwm,pwmOut);
        analogWrite(M3_pwm,pwmOut);
        analogWrite(M4_pwm,pwmOut);
       }
   }
   
   else if(ps2.readButton(PS2_RIGHT_1)==0) //SPIN CW
    {
      D_R();
      analogWrite(M1_pwm,140);
      analogWrite(M2_pwm,160);
      analogWrite(M3_pwm,165);
      analogWrite(M4_pwm,160);
    } 

  else if(ps2.readButton(PS2_LEFT_1)==0) //SPIN CCW
  {
    D_L();
    analogWrite(M1_pwm,140);
    analogWrite(M2_pwm,150);
    analogWrite(M3_pwm,147);
    analogWrite(M4_pwm,147);
   
  }
   else {
    Brake();
   }

//  Serial.print(Output);
//  Serial.print(" Out   ");
//  Serial.print(pwmOut);
//  Serial.print(" pwm_out   ");
//  Serial.print("MPU_180:  ");
//  Serial.print(mpu_180);
//  Serial.print("°  ");
//  Serial.print(mpu_a);
//  Serial.println("° ");
  
}



/*
///////////////////////////////////////////////////////////////////////////////////////////////////
                                    MOTOR DIRECTION
///////////////////////////////////////////////////////////////////////////////////////////////////
*/
void D_FW() // FORWARD
{
  digitalWrite(M1_dir,LOW);
  digitalWrite(M2_dir,LOW);//LOW
  digitalWrite(M3_dir,LOW);//LOW
  digitalWrite(M4_dir,HIGH);

  analogWrite(M1_pwm,180); //180
  analogWrite(M2_pwm,160); //160
  analogWrite(M3_pwm,180); //180
  analogWrite(M4_pwm,160); //160
}

void D_BW() // BACKWARD
{
  digitalWrite(M1_dir,HIGH);
  digitalWrite(M2_dir,HIGH);//LOW
  digitalWrite(M3_dir,HIGH);//LOW
  digitalWrite(M4_dir,LOW);

  analogWrite(M1_pwm,157); //180   //157
  analogWrite(M2_pwm,150); //160   //150
  analogWrite(M3_pwm,157); //180   //157
  analogWrite(M4_pwm,150); //160   //150
}

void D_R()// SPIN CW
{
  digitalWrite(M1_dir,LOW);
  digitalWrite(M2_dir,HIGH);
  digitalWrite(M3_dir,LOW);
  digitalWrite(M4_dir,LOW);
}

void D_L()// SPIN CCW
{
  digitalWrite(M1_dir,HIGH);
  digitalWrite(M2_dir,LOW);
  digitalWrite(M3_dir,HIGH);
  digitalWrite(M4_dir,HIGH);
}
void D_TL() // MOVE LEFT
{
  digitalWrite(M1_dir,LOW);
  digitalWrite(M2_dir,HIGH);
  digitalWrite(M3_dir,HIGH);
  digitalWrite(M4_dir,HIGH);

  analogWrite(M1_pwm,140);
  analogWrite(M2_pwm,140);
  analogWrite(M3_pwm,140);
  analogWrite(M4_pwm,140);
  
}

void D_TR() // MOVE RIGHT
{
  digitalWrite(M1_dir,HIGH);
  digitalWrite(M2_dir,LOW);
  digitalWrite(M3_dir,LOW);
  digitalWrite(M4_dir,LOW);

  analogWrite(M1_pwm,140);
  analogWrite(M2_pwm,160);
  analogWrite(M3_pwm,165);
  analogWrite(M4_pwm,160);
}

void Brake()
{
 digitalWrite(M1_dir,HIGH);
 digitalWrite(M2_dir,HIGH);
 digitalWrite(M3_dir,HIGH);
 digitalWrite(M4_dir,HIGH);

 analogWrite(M1_pwm,0);
 analogWrite(M2_pwm,0);
 analogWrite(M3_pwm,0);
 analogWrite(M4_pwm,0);
}

/*
///////////////////////////////////////////////////////////////////////////////////////////////////
                                    ANALOG MOVEMENT
///////////////////////////////////////////////////////////////////////////////////////////////////
*/
void AnalogMovement()
{
  int x_axis = ps2.readButton(PS2_JOYSTICK_LEFT_X_AXIS);//analog LEFT for x axis
  int xl_axis = map(x_axis, 128, 0, 0, 255);//nom origin analog untuk xl adalah 128-0,bila letak map,kita boleh setkan dari 0-255,....(128,0)nom origin,(0,255) nom yg kita set.
  int xr_axis = map(x_axis, 128, 255, 0, 255);//nom origin analog untuk xr adalah 128-255,bila letak map,kita boleh setkan dari 255-0,....(128,255)nom origin,(255,0) nom yg kita set.
 
  int y_axis = ps2.readButton(PS2_JOYSTICK_LEFT_Y_AXIS);//analog LEFT y_axis
  int yu_axis = map(y_axis,128,0,0,255);//nom origin analog untuk yl adalah 128-0,bila letak map,kita boleh setkan dari 0-255,....(128,0)nom origin,(0,255) nom yg kita set.
  int yd_axis = map(y_axis,128,255,0,255);//nom origin analog untuk yr adalah 128-255,bila letak map,kita boleh setkan dari 255-0,....(128,255)nom origin,(255,0) nom yg kita set.
  
  int x_axisR = ps2.readButton(PS2_JOYSTICK_RIGHT_X_AXIS);//analog RIGHT for x axis
  int xl_axisR = map(x_axisR, 128, 0, 0, 255);
  int xr_axisR = map(x_axisR,128,255,0,255);

// ANALOG DIRECTION & MOTOR

/*
                0
                y
                |
                | 
                |128
         0-------------- x 255
                |
                |
                |
                255
                
*/
 if (xr_axis > 70)//kalau xr axis lebih dr 120,motor akan bergerak lebih laju,kadang2 kondisi (>100) motor tk bergerak,kene pandai cari nilai yg untuk motor bergerak.
  {                 //motor MOVE RIGHT
    digitalWrite(M1_dir,HIGH); // MOVE RIGHT
    digitalWrite(M2_dir,LOW);
    digitalWrite(M3_dir,LOW);
    digitalWrite(M4_dir,LOW);

    analogWrite(M1_pwm,xr_axis);
    analogWrite(M2_pwm,xr_axis);
    analogWrite(M3_pwm,xr_axis);
    analogWrite(M4_pwm,xr_axis);
  }
  
  else if (xl_axis > 70)//MOTOR MOVE TO LEFT
  {
    digitalWrite(M1_dir,LOW); // MOVE LEFT
    digitalWrite(M2_dir,HIGH);
    digitalWrite(M3_dir,HIGH);
    digitalWrite(M4_dir,HIGH);

    analogWrite(M1_pwm,xl_axis);//pwm akan bergrak bila xr axis lebih dr 100
    analogWrite(M2_pwm,xl_axis);
    analogWrite(M3_pwm,xl_axis);
    analogWrite(M4_pwm,xl_axis);
  }
//  else if(xr_axis == 0 && xl_axis == 0)//MOTOR STOP                        //clashing with pid mpu
//  {
//    digitalWrite(M1_dir,HIGH);
//    digitalWrite(M2_dir,HIGH);//HIGH
//    digitalWrite(M3_dir,HIGH);//HIGH
//    digitalWrite(M4_dir,HIGH);
//    
//    analogWrite(M1_pwm,xr_axis);//pwm akan bergrak bila xr axis lebih dr 100
//    analogWrite(M2_pwm,xl_axis);
//    analogWrite(M3_pwm,xr_axis);
//    analogWrite(M4_pwm,xr_axis);
//    
//  }
    if (yd_axis > 70)//kalau yr axis lebih dr 120,motor akan bergerak lebih laju,kadang2 kondisi (>100) motor tk bergerak,kene pandai cari nilai yg untuk motor bergerak.
                         //motor backward   
  {
    digitalWrite(M1_dir,HIGH); // BACKWARD
    digitalWrite(M2_dir,HIGH);
    digitalWrite(M3_dir,HIGH);
    digitalWrite(M4_dir,LOW);
    
    analogWrite(M1_pwm,yd_axis);//pwm akan bergrak bila xr axis lebih dr 120
    analogWrite(M2_pwm,yd_axis);
    analogWrite(M3_pwm,yd_axis);
    analogWrite(M4_pwm,yd_axis);
  }
  else if (yu_axis > 70)//kalau yr axis lebih dr 120,motor akan bergerak lebih laju,kadang2 kondisi (>100) motor tk bergerak,kene pandai cari nilai yg untuk motor bergerak.
                         //motor MOVE FOWARD
  {
    digitalWrite(M1_dir,LOW); // FORWARD
    digitalWrite(M2_dir,LOW);
    digitalWrite(M3_dir,LOW);
    digitalWrite(M4_dir,HIGH);
    
    analogWrite(M1_pwm,yu_axis);//pwm akan bergrak bila xr axis lebih dr 120
    analogWrite(M2_pwm,yu_axis);
    analogWrite(M3_pwm,yu_axis);
    analogWrite(M4_pwm,yu_axis);
  }
  
//  else if (yu_axis == 0 && yr_axis == 0)                       //clashing with pid mpu
//  {
//    digitalWrite(M1_dir,LOW); // MOVE LEFT
//    digitalWrite(M2_dir,HIGH);
//    digitalWrite(M3_dir,HIGH);
//    digitalWrite(M4_dir,HIGH);
//
//    analogWrite(M1_pwm,yl_axis);//pwm akan bergrak bila xr axis lebih dr 120
//    analogWrite(M2_pwm,yr_axis);
//    analogWrite(M3_pwm,xl_axis);
//    analogWrite(M4_pwm,xl_axis);
//
//  }
  
  if ( xr_axisR > 120)//MOTOR SPIN LEFT
  {
    digitalWrite(M1_dir,LOW); 
    digitalWrite(M2_dir,HIGH);
    digitalWrite(M3_dir,LOW);
    digitalWrite(M4_dir,LOW);
    
    analogWrite(M1_pwm,xr_axisR);//pwm akan bergrak bila xr axis lebih dr 120
    analogWrite(M2_pwm,xr_axisR);
    analogWrite(M3_pwm,xr_axisR);
    analogWrite(M4_pwm,xr_axisR);
  }

  else if (xl_axisR > 120)//MOTOR SPIN RIGHT
  {
    digitalWrite(M1_dir,HIGH);
    digitalWrite(M2_dir,LOW);
    digitalWrite(M3_dir,HIGH);
    digitalWrite(M4_dir,HIGH);
    
    analogWrite(M1_pwm,xl_axisR);//pwm akan bergrak bila xr axis lebih dr 120
    analogWrite(M2_pwm,xl_axisR);
    analogWrite(M3_pwm,xl_axisR);
    analogWrite(M4_pwm,xl_axisR);
  }

  // recalibration analog utk buang minor error value //  
  if (yu_axis < 20 && yd_axis < 20 )
  {
    yu_axis=0;
    yd_axis=0;
  }


  Serial.print("    xl: ");
  Serial.print(xl_axis);
  Serial.print("    xr: ");
  Serial.print(xr_axis);
  Serial.print("    yu: ");
  Serial.print(yu_axis);
  Serial.print("    yd: ");
  Serial.println(yd_axis);
}



/*
///////////////////////////////////////////////////////////////////////////////////////////////////
                                    MPU6050
///////////////////////////////////////////////////////////////////////////////////////////////////
*/
void MPU(){
  
      if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          fifoCount = mpu.getFIFOCount();
        }  
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
  if(fifoCount < packetSize){
  }
    else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } 
    else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT))
    {
       while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
       mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
     }
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
       
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            mpu_ori = (ypr[0] * 180/M_PI);

            if(mpu_ori < 0){
                mpu_a = mpu_ori + 360;
               }
            else {
                mpu_a = mpu_ori;
               }

            if(mpu_a >= 180){
                mpu_180 = mpu_a - 180;
               }
            else {
                mpu_180 = mpu_a+180;
               }
//            Serial.print("angle\t");
//            Serial.print(mpu_a);
//            Serial.print("  ori\t");
//            Serial.println(ypr[0] * 180/M_PI);
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
