#include <Cytron_SmartDriveDuo.h>
#include <VarSpeedServo.h>
#include <SoftwareSerial.h>
#include <Cytron_PS2Shield.h>
Cytron_PS2Shield ps2(10, 11); 


///////////THROWERS//////////
const int T1_pwm=2;
const int T2_pwm=3;
const int T3_pwm=13;

const int T1_dir=40;
const int T2_dir=42;
const int T3_dir=44;


//////////IR SENSOR /////////
const int IR1 = 35;
const int IR2 = 37;
const int IR3 = 39;
const int IR4 = 41; 
const int IR5 = 43;

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

// POWER WINDOW //
const int PW_pwm=8;  //8
const int PW_dir=47;  //47

// SERVO //
VarSpeedServo myservo;
int servo_state = 1;

//////////ANGLE SERVO/////////
VarSpeedServo AngleServo;
int buttonNew1;
int buttonOld1 = LOW;
int servo = HIGH;


//////////THROWER CASE //////
int task = 1;
int count_value=1;
int Reset=1;

void setup() {

  //IR DECLARATION
  pinMode(IR1,INPUT);
  pinMode(IR2,INPUT);
  pinMode(IR3,INPUT);
  pinMode(IR4,INPUT);
  pinMode(IR5,INPUT);

  //THROWER DECLARATION
  pinMode(T1_pwm,OUTPUT);
  pinMode(T2_pwm,OUTPUT);
  pinMode(T3_pwm,OUTPUT);
  
  pinMode(T1_dir,OUTPUT);
  pinMode(T2_dir,OUTPUT);
  pinMode(T3_dir,OUTPUT);

  digitalWrite(T1_dir,HIGH);
  digitalWrite(T2_dir,HIGH);
  digitalWrite(T3_dir,HIGH);
  analogWrite(T1_pwm,0);
  analogWrite(T2_pwm,0);
  analogWrite(T3_pwm,0);

  // MOTOR DECLARATION
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

  // POWER WINDOW
  pinMode(PW_pwm,OUTPUT);
  pinMode(PW_dir,OUTPUT);

  
  ps2.begin(57600);
  Serial.begin(57600);
}

void loop() {
  
  Thrower();
  AngleThrowServo();
  AnalogMovement();
  ManualMovement();
  ServoMotor();
  power_window();
  
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////                    MAIN FUNCTION THROWING                       /////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void Thrower(){
  
  switch(count_value)
{
  
  //------------------------------------------------------------------------------//
  ///////       THROWING ARROW STARTING FROM THROWER 1 UNTIl THROWER 5      ////////
  //////////////////////////////////////////////////////////////////////////////////
  
    case 1:

     if(ps2.readButton(PS2_SQUARE) == 0 && ps2.readButton(PS2_SELECT)==0)
     {
       count_value = count_value + 1;
      }
      else{}
      break;
  //------------------------------------------------------------------------------
    case 2:
      Throw(T1_dir, T1_pwm,LOW, IR1); //thrower1 LOW
      
      if(task==3)
      { //AFTER THROWING, THROWING STOP SBB TASK=1
        count_value = count_value + 1;
      }
     
       break;
  //------------------------------------------------------------------------------
    case 3:
      task = 1;

      if(ps2.readButton(PS2_SQUARE) == 0 && ps2.readButton(PS2_SELECT)==0)
      {
       count_value = count_value + 1;
      }
      else{}
      break;

  //------------------------------------------------------------------------------
    case 4:
      Throw(T1_dir, T1_pwm,HIGH, IR2);  //Thrower2 HIGH

      if(task==3)
      { //AFTER THROWING, THROWING STOP SBB TASK=1
        count_value = count_value + 1;
      }
     
       break;

  //------------------------------------------------------------------------------
    case 5:
      task = 1;

      if(ps2.readButton(PS2_SQUARE) == 0 && ps2.readButton(PS2_SELECT)==0)
      {
       count_value = count_value + 1;
      }
      else{}
       
      break;

      //------------------------------------------------------------------------------
    case 6:
      Throw(T2_dir, T2_pwm,HIGH, IR3); //Thrower3

      if(task==3)
      { //AFTER THROWING, THROWING STOP SBB TASK=1
        count_value = count_value + 1;
      }
     
       break;


     //------------------------------------------------------------------------------
      case 7:
        task = 1;

      if(ps2.readButton(PS2_SQUARE) == 0 && ps2.readButton(PS2_SELECT)==0)
      {
       count_value = count_value + 1;
      }
      else{}
       
      break;


      //------------------------------------------------------------------------------
    case 8:
      Throw(T2_dir, T2_pwm,LOW, IR4); //Thrower4

      if(task==3)
      { //AFTER THROWING, THROWING STOP SBB TASK=1
        count_value = count_value + 1;
      }
     
       break;


      //------------------------------------------------------------------------------
      case 9:
        task = 1;

      if(ps2.readButton(PS2_SQUARE) == 0 && ps2.readButton(PS2_SELECT)==0)
      {
       count_value = count_value + 1;
      }
      else{}
       
      break;


     //------------------------------------------------------------------------------
    case 10:
      Throw(T3_dir, T3_pwm,LOW, IR5);

      if(task==3)
      { //AFTER THROWING, THROWING STOP SBB TASK=1
        count_value = count_value + 1;
      }

      else{}
     
       break;

  //------------------------------------------------------------------------------//
  ///////  RESET THROWER POSITION STARTING FROM THROWER 1 UNTIL THROWER 5   ////////
  //////////////////////////////////////////////////////////////////////////////////
  
    case 11:
      task = 1;
      
      if(ps2.readButton(PS2_SQUARE) == 0 && ps2.readButton(PS2_SELECT)==0)
      {
       count_value = count_value + 1;
      }
      else{}
       
      break;

      //------------------------------------------------------------------------------
    case 12:
      Resetting(T1_dir, T1_pwm,LOW,140, 2000);   //AKAN RESET THROWER1 UNTUK PEGANG ARROW

      if(Reset==3)
      { //LEPAS RESET THROWER, THROWER AKAN STOP SBB RESET=1
        count_value = count_value + 1;
      }
     
       break;

       //------------------------------------------------------------------------------
      case 13:
        Reset = 1;

      if(ps2.readButton(PS2_SQUARE) == 0 && ps2.readButton(PS2_SELECT)==0)
      {
       count_value = count_value + 1;
      }
      else{}
       
      break;

      //------------------------------------------------------------------------------
     case 14:
      Resetting(T1_dir, T1_pwm,HIGH,140, 2000);   //AKAN RESET THROWER2 UNTUK PEGANG ARROW

      if(Reset==3)
      { //LEPAS RESET THROWER, THROWER AKAN STOP SBB RESET=1
        count_value = count_value + 1;
      }
     
       break;

       //------------------------------------------------------------------------------
      case 15:
        Reset = 1;

      if(ps2.readButton(PS2_SQUARE) == 0 && ps2.readButton(PS2_SELECT)==0)
      {
       count_value = count_value + 1;
      }
      else{}
       
      break;

      //------------------------------------------------------------------------------
      case 16:
      Resetting(T2_dir, T2_pwm,HIGH,140, 2000);   //AKAN RESET THROWER3 UNTUK PEGANG ARROW

      if(Reset==3)
      { //LEPAS RESET THROWER, THROWER AKAN STOP SBB RESET=1
        count_value = count_value + 1;
      }
     
       break;

       //------------------------------------------------------------------------------
      case 17:
        Reset = 1;

      if(ps2.readButton(PS2_SQUARE) == 0 && ps2.readButton(PS2_SELECT)==0)
      {
       count_value = count_value + 1;
      }
      else{}
       
      break;

      //------------------------------------------------------------------------------
      case 18:
      Resetting(T2_dir, T2_pwm,LOW,140, 2000);    //AKAN RESET THROWER4 UNTUK PEGANG ARROW

      if(Reset==3)
      { //LEPAS RESET THROWER, THROWER AKAN STOP SBB RESET=1
        count_value = count_value + 1;
      }
     
       break;

       //------------------------------------------------------------------------------
      case 19:
        Reset = 1;

      if(ps2.readButton(PS2_SQUARE) == 0 && ps2.readButton(PS2_SELECT)==0)
      {
       count_value = count_value + 1;
      }
      else{}
       
      break;

      //------------------------------------------------------------------------------
    case 20:
      Resetting(T3_dir, T3_pwm,LOW,140, 2000);    //AKAN RESET THROWER5 UNTUK PEGANG ARROW

      if(Reset==3)
      { //LEPAS RESET THROWER, THROWER AKAN STOP SBB RESET=1
        count_value = count_value + 1;
      }
     
       break;
       
    //------------------------------------------------------------------------------//
    ////        HABIS DAH SEQUENCE, NANTI AKAN START DARI THROWER 1 BALIK         ////
    //////////////////////////////////////////////////////////////////////////////////
    
    case 21:
      task = 1;
      count_value=1;
      Reset=1;
       
      break;
//---------------------------------------------------------------------------------------------------------------------------------------------------//
}

//if(ps2.readButton(PS2_CIRCLE) == 0 && ps2.readButton(PS2_SELECT)==0)
if(ps2.readButton(PS2_CIRCLE) == 0 && ps2.readButton(PS2_SELECT)==0) //RESET ALL TO START AT THROWER 1
  {
  count_value=1;
  task=1;
  Reset=1;
  }
 


Serial.print("count: ");
Serial.print(count_value);
Serial.print("\t task: ");
Serial.print(task);
Serial.print("\t\t Reset: ");
Serial.println(Reset);

  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////                    SUBFUNCTION THROWING                         /////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Throw(int T_DIRpin, int T_PWMpin, int Tstate, int Infra){

    switch (task)
  {
    case 1:
      digitalWrite(T_DIRpin,Tstate);
      analogWrite(T_PWMpin,0);                    //THROWER WILL NOT THROWING BCS IT FOLLOW THE SEQUENCE FROM THROWER 1 to THROWER 5
    if( count_value == 2 || count_value == 4 || count_value == 6  || count_value == 8  || count_value == 10)
    {
       Serial.println("1");
       task = task + 1;  //2
    }
    else{}
    break;
  //------------------------------------------------------------------------------
    case 2:
      Serial.println("2");                         //THROWER WILL THROW AT FULL SPEED OF MOTOR
      digitalWrite(T_DIRpin,Tstate);
      analogWrite(T_PWMpin,255);
      
      if(digitalRead(Infra) == LOW) 
      {     
        digitalWrite(T_DIRpin,Tstate);
        analogWrite(T_PWMpin,0);
        task = task + 1;  //3
      }
      else{} 
    break;
  //------------------------------------------------------------------------------
    case 3:                                        //NULL CONDITION TO MAKE CONDITION 
   
    break;
 //------------------------------------------------------------------------------

  }
  
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////                 FUNCTION RESET THROWERS                         /////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Resetting(int R_DIRpin, int R_PWMpin, int Rstate, int R_PWMVal, int R_delay)  //FUNCTION RESET THROWER UNTUK PEGANG ARROW
 {
  switch (Reset)
    {
      case 1:
      digitalWrite(R_DIRpin,Rstate);
      analogWrite(R_PWMpin,0);
    if( count_value == 12 || count_value == 14 || count_value == 16  || count_value == 18  || count_value == 20)
    {
       Serial.println("1");
       Reset = Reset + 1;  //2
    }
    else{}
    break;
    
    //------------------------------------------------------------------------------
      case 2: 
      digitalWrite(R_DIRpin,Rstate);
      analogWrite(R_PWMpin,R_PWMVal);
      delay(R_delay);
          
      digitalWrite(R_DIRpin,Rstate);
      analogWrite(R_PWMpin,0);
      Reset = Reset + 1;  //3
       
    break;

    //------------------------------------------------------------------------------
       case 3:                                    //NULL CONDITION TO MAKE CONDITION
    break;
    
    }
 
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
  digitalWrite(M3_dir,HIGH);//LOW
  digitalWrite(M4_dir,HIGH);
  
  analogWrite(M1_pwm,160);
  analogWrite(M2_pwm,160);
  analogWrite(M3_pwm,160);
  analogWrite(M4_pwm,160);

}

void D_BW() // BACKWARD
{
  digitalWrite(M1_dir,HIGH);
  digitalWrite(M2_dir,HIGH);//LOW
  digitalWrite(M3_dir,LOW);//LOW
  digitalWrite(M4_dir,LOW);
    
  analogWrite(M1_pwm,160);
  analogWrite(M2_pwm,160);
  analogWrite(M3_pwm,160);
  analogWrite(M4_pwm,160);
}

void D_R()// SPIN CW
{
  digitalWrite(M1_dir,LOW);
  digitalWrite(M2_dir,HIGH);
  digitalWrite(M3_dir,LOW);
  digitalWrite(M4_dir,HIGH);

  analogWrite(M1_pwm,130);
  analogWrite(M2_pwm,130);
  analogWrite(M3_pwm,130);
  analogWrite(M4_pwm,130);
}

void D_L()// SPIN CCW
{
  digitalWrite(M1_dir,HIGH);
  digitalWrite(M2_dir,LOW);
  digitalWrite(M3_dir,HIGH);
  digitalWrite(M4_dir,LOW);

  analogWrite(M1_pwm,130);
  analogWrite(M2_pwm,130);
  analogWrite(M3_pwm,130);
  analogWrite(M4_pwm,130);
}
void D_TL() // MOVE LEFT
{
  digitalWrite(M1_dir,HIGH);
  digitalWrite(M2_dir,LOW);
  digitalWrite(M3_dir,LOW);
  digitalWrite(M4_dir,HIGH);
  
  analogWrite(M1_pwm,160);
  analogWrite(M2_pwm,160);
  analogWrite(M3_pwm,160);
  analogWrite(M4_pwm,160);
}

void D_TR() // MOVE RIGHT
{
  digitalWrite(M1_dir,LOW);
  digitalWrite(M2_dir,HIGH);
  digitalWrite(M3_dir,HIGH);
  digitalWrite(M4_dir,LOW);
  
  analogWrite(M1_pwm,160);
  analogWrite(M2_pwm,160);
  analogWrite(M3_pwm,160);
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

void ManualMovement()
{
  // LED on main board will light up if 'Select' button is pressed
  if(ps2.readButton(PS2_SELECT) == 0) // 0 = pressed, 1 = released
  {
//   digitalWrite(LEDPIN, HIGH);
  }
  else if(ps2.readButton(PS2_UP)==0) //Moves Forward
  {
   D_FW();  
   Serial.println("depann");
  }
  
  else if(ps2.readButton(PS2_DOWN)==0) //Moves Backward
  {
   D_BW();
   Serial.println("dbelakang");
  }
  
  else if(ps2.readButton(PS2_LEFT)==0) //Moves Right
  {
   D_TL();
  }
  
  else if(ps2.readButton(PS2_RIGHT)==0) //Moves Left
  {
   D_TR();
  }
  
  else if(ps2.readButton(PS2_RIGHT_1)==0) //SPIN CW
  {
   D_R();
  } 
   
  else if(ps2.readButton(PS2_LEFT_1)==0) //SPIN CCW
  {
   D_L();
  }
  else
  {
    Brake();//MOTOR INITIAL
    Serial.println("tidurrrr");
  }
}

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
    digitalWrite(M1_dir,LOW); // MOVE RIGHT
    digitalWrite(M2_dir,HIGH);
    digitalWrite(M3_dir,HIGH);
    digitalWrite(M4_dir,LOW);

    analogWrite(M1_pwm,xr_axis);
    analogWrite(M2_pwm,xr_axis);
    analogWrite(M3_pwm,xr_axis);
    analogWrite(M4_pwm,xr_axis);
  }
  
  else if (xl_axis > 70)//MOTOR MOVE TO LEFT
  {
    digitalWrite(M1_dir,HIGH); // MOVE LEFT
    digitalWrite(M2_dir,LOW);
    digitalWrite(M3_dir,LOW);
    digitalWrite(M4_dir,HIGH);

    analogWrite(M1_pwm,xl_axis);//pwm akan bergrak bila xr axis lebih dr 100
    analogWrite(M2_pwm,xl_axis);
    analogWrite(M3_pwm,xl_axis);
    analogWrite(M4_pwm,xl_axis);
  }

    if (yd_axis > 70)//kalau yr axis lebih dr 120,motor akan bergerak lebih laju,kadang2 kondisi (>100) motor tk bergerak,kene pandai cari nilai yg untuk motor bergerak.
                         //motor backward   
  {
    digitalWrite(M1_dir,HIGH); // BACKWARD
    digitalWrite(M2_dir,HIGH);
    digitalWrite(M3_dir,LOW);
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
    digitalWrite(M3_dir,HIGH);
    digitalWrite(M4_dir,HIGH);
    
    analogWrite(M1_pwm,yu_axis);//pwm akan bergrak bila xr axis lebih dr 120
    analogWrite(M2_pwm,yu_axis);
    analogWrite(M3_pwm,yu_axis);
    analogWrite(M4_pwm,yu_axis);
  }
  
  
  if ( xr_axisR > 120)//MOTOR SPIN LEFT
  {
    digitalWrite(M1_dir,LOW); 
    digitalWrite(M2_dir,HIGH);
    digitalWrite(M3_dir,LOW);
    digitalWrite(M4_dir,HIGH);
    
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
    digitalWrite(M4_dir,LOW);
    
    analogWrite(M1_pwm,xl_axisR);//pwm akan bergrak bila xr axis lebih dr 120
    analogWrite(M2_pwm,xl_axisR);
    analogWrite(M3_pwm,xl_axisR);
    analogWrite(M4_pwm,xl_axisR);
  }
}

 
/*
///////////////////////////////////////////////////////////////////////////////////////////////////
                                    POWER WINDOW
///////////////////////////////////////////////////////////////////////////////////////////////////
*/

void power_window(){
  if (ps2.readButton(PS2_RIGHT_2) == 0){  
         digitalWrite(PW_dir,HIGH);
         analogWrite(PW_pwm,60);
     }
 
    else if (ps2.readButton(PS2_LEFT_2) == 0){  
        digitalWrite(PW_dir,LOW);
        analogWrite(PW_pwm,60);
     }

    else {
        digitalWrite(PW_dir,LOW);
        analogWrite(PW_pwm,0);}
}




/*
///////////////////////////////////////////////////////////////////////////////////////////////////
                                    SERVO MOTOR
///////////////////////////////////////////////////////////////////////////////////////////////////
*/

void spin_tutup() //servo
{
 myservo.write(0,150);  //(degree,speed)
 delay(800);
}

void spin_sikit()
{
  myservo.write(30,200);  //(degree,speed)
 delay(800);
}


void spin_bukak()// servo
{
 myservo.write(170,200);  //(degree,speed)
 delay(800);
}

void ServoMotor()
{ 
  myservo.detach();
  myservo.attach(46); // ori 44, PIN 46 no more twitching

switch(servo_state)
  {
  case 1:
    spin_bukak();
    servo_state = servo_state + 1;
  break;
//------------------------------------------------------------------------------
  case 2:
    if(ps2.readButton(PS2_TRIANGLE)==0){
      spin_tutup();
      servo_state = servo_state + 1;
    }
    else{} 
    break;
//------------------------------------------------------------------------------
  case 3:
    if(ps2.readButton(PS2_TRIANGLE)==0){
      spin_sikit();
      servo_state = servo_state + 1;
    }
    else{}
    break;
//------------------------------------------------------------------------------
  case 4:
    if(ps2.readButton(PS2_TRIANGLE)==0){
      spin_bukak();
      servo_state = 1;
    }
    else{}
    break;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////                   FUNCTION ANGLE THROWER                        /////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 void angleInitial() //servo
{
 AngleServo.write(0,150);  //(degree,speed)bukak
 delay(800);
}


void angleFinal()// servo
{
  
 AngleServo.write(40,200);  //(degree,speed)tutup
 delay(800);
}

void AngleThrowServo()
{ 
  buttonNew1 = ps2.readButton(PS2_CROSS);// button ps2
  
  //AngleServo.detach();
  AngleServo.attach(45); // ori 44, PIN 46 no more twitching
  
  if (buttonNew1 == HIGH && buttonOld1 == LOW)//toggle funtion
  {
   if(servo == HIGH)//if want to servo funtion 
    {
     servo = LOW;//condition servo in intial condition
     angleInitial();//rotate to 180
    }
   
   else 
    {
     servo = HIGH;//condition servo funtionc 
     angleFinal();//rotate to 90
    }
  }     
     buttonOld1 = buttonNew1;//reset
} 
