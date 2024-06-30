// LIBRARIES
#include <PS4Controller.h>
#include <ESP32Servo.h> // Servo available on: 2,4,5,12-19,21-23,25-27,32-33

/*
  The programming mode can be accessed through pressing PS4 tringle twice before powering it on, as it will put 
  in into max speed. In that case after hearing special sound press PS4 cross twice to go to neutral position and her 
  single beep sound, then turn off the pwoer and turn on the power again you will hear several beep soudds and one long one, 
  which means you exited programming mode. For furher information check official documentation: 
  chrome-extension://efaidnbmnnnibpcajpcglclefindmkaj/https://hobbymania.com.ua/file/FlyColor_boat_ESC.pdf

*/

/*
  Start up sequence

  1. Switch to Calibration Mode
  2. Connect PS4
  3. Push Right Y stick to top
  4. Turn on motor power
  5. Once you see solid green light on C620, push Right Y stick to bottom, hold until you see blinking ornage
  6. Release right stick
  7. When you see blinking green, okay
  8. Switch to operation mode

*/



// #############################
// DEFINITIONS, GLOBAL VARIABLES
// #############################

//Uart
//HardwareSerial Sender(1);
//#define Sender_Txd_pin 23
//#define Sender_Rxd_pin 22

// esp now
#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x08, 0x3A, 0x8D, 0x0D, 0x7F, 0x50};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int b;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 


//Gripper Servo
Servo GripperServo;
Servo GripperServo2;
int GripperServoPin = 16;
int GripperServoPin2 = 32;
#define Gripper_Open 1
#define Gripper_Close 0
int Gripper_Status1 = Gripper_Close;
int Gripper_Status2 = Gripper_Close;

// PS4
bool lastSpeedUpButtonState = 0;
bool lastSpeedDownButtonState = 0;
bool lastSquareButtonState = 0;
bool lastCircleButtonState = 0;
bool lastStepperUpButtonState = 0;
bool lastStepperDownButtonState = 0;
bool lastLeftButtonState = 0;
bool lastRightButtonState = 0 ;
bool lastR1ButtonState = 0 ;
bool lastR2ButtonState = 0 ;
bool lastL1ButtonState = 0 ;



// SHOOTER (BLDC), TAKE 5V SIGNALS
Servo topRoller, bottomRoller;
#define TOP_ROLLER 18
#define BOTTOM_ROLLER 17
int rollerSpeeds[4] = {-500, -400, 100, 500};
int speedSelect = 0;

// PUSHER (STEPPER), TAKE 5V SIGNALS
// #define STEPPER_DIR 16
// #define STEPPER_PUL 17

// int stepper_speed = 32000;

// C620, take 3.3V PWM signals
#define C620_PWM_1 33 // frontLeft
#define C620_PWM_2 25 // backLeft
#define C620_PWM_3 26 // frontRight
#define C620_PWM_4 27 // backRight

#define FORWARD 0
#define BACKWARD 1
#define LEFT 2
#define RIGHT 3
#define FORWARD_LEFT 4
#define FORWARD_RIGHT 5
#define BACKWARD_LEFT 6
#define BACKWARD_RIGHT 7
#define ROTATE_CW 8
#define ROTATE_CCW 9
#define STOP 10

Servo frontLeftWheel; //33
Servo backLeftWheel; //25
Servo frontRightWheel; //26
Servo backRightWheel; //27

//define project servo parameter
int  projectservo_angle1 = 500;
int projectservo_angle2 = 1600;
int projectBall = 21;
bool projectservo_status1 = 0;
float projectservoSmoothed= 1600.0; 
float projectservoSmoothedPrev = 1600.0;
float projectservoCurrentAngle = 1600.0;
Servo projectServo;

// Check PS4 connection
void onConnect()
{
  Serial.println("Connected!.");
}

void onDisConnect()
{
  Serial.println("Disconnected!.");    
}
void calibrateC620() {

  int wheelSpeed = map(PS4.RStickY(), -128, 128, 1000, 2000);

  // Serial.println(wheelSpeed);

  frontLeftWheel.writeMicroseconds(wheelSpeed);
  backLeftWheel.writeMicroseconds(wheelSpeed);
  frontRightWheel.writeMicroseconds(wheelSpeed);
  backRightWheel.writeMicroseconds(wheelSpeed);

}






// #############################
// SETUP
// #############################



void setup() {

  Serial.begin(115200);
 // Sender.begin(115200, SERIAL_8N1, Sender_Txd_pin, Sender_Rxd_pin);
  // PS4
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);
  PS4.begin();
  Serial.println("Ready.");

  if (!PS4.isConnected()) delay(500);
//esp now
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  } 

  // C620
  frontLeftWheel.attach(C620_PWM_1);
  backLeftWheel.attach(C620_PWM_2);
  frontRightWheel.attach(C620_PWM_3);
  backRightWheel.attach(C620_PWM_4);

  // Gripper Servo
  GripperServo.attach(GripperServoPin);
  GripperServo2.attach(GripperServoPin2);

  projectServo.attach(projectBall);


  // -------------------- SHOOTER (BLDC) --------------------
  topRoller.attach(TOP_ROLLER);
  bottomRoller.attach(BOTTOM_ROLLER);
  topRoller.writeMicroseconds(1500);
  bottomRoller.writeMicroseconds(1500);
}



// #############################
// LOOP
// #############################



void loop() {

  // while (debugPS4) {
  //   int up = PS4.Up();
  //   int leftX = PS4.LStickX();
  //   int square = PS4.Square();

  //   Serial.print(up); Serial.print('/t');
  //   Serial.print(leftX); Serial.print('/t');
  //   Serial.print(square); Serial.print('/t');
  //   Serial.println();

  //   delay(200);
  // }

  // while (digitalRead(CALIBRATION_SWITCH) == 1) {
  //   calibrateC620();
  // }


  // -------------------- SHOOTER (BLDC) --------------------

  if (PS4.Triangle() != lastSpeedUpButtonState) {
    if (PS4.Triangle() == 1) {
      speedSelect++;
       speedSelect = constrain(speedSelect, 0, 3);
    }
    lastSpeedUpButtonState = PS4.Triangle();
    Serial.print("Speed Select Increased: ");
    Serial.println(speedSelect);
  }
  else if (PS4.Cross() != lastSpeedDownButtonState) {
    if (PS4.Cross() == 1) {
      speedSelect--;
      speedSelect = constrain(speedSelect, 0, 3);
    }
    lastSpeedDownButtonState = PS4.Cross();
    Serial.print("Speed Select Decreased: ");
    Serial.println(speedSelect);
  }

  // ---------------- SERVO (GRIPPER) -----------
  if (PS4.Square() != lastSquareButtonState) {
    if (PS4.Square() == 1) {
       myData.b = 3;
     }

     lastSquareButtonState = PS4.Square();
   }
  
  if (PS4.Circle() != lastCircleButtonState) {
    if (PS4.Circle() == 1) {
      if (Gripper_Status1 == Gripper_Close){
        GripperServo.write(110);
        GripperServo2.write(110);
        Gripper_Status1 = Gripper_Open;
      }
      else if(Gripper_Status1 == Gripper_Open){
        GripperServo.write(0);
        GripperServo2.write(6);
        Gripper_Status1 = Gripper_Close; 
      }
    }
    lastCircleButtonState = PS4.Circle();
  }


  // ---------------- Stepper -----------
  if (PS4.Down() != lastStepperDownButtonState) {
    if (PS4.Down() == 1) {
      myData.b = 5;
    }
    else{
      myData.b = 0;
    }
    lastStepperDownButtonState = PS4.Down();
  }

  else if (PS4.Up() != lastStepperUpButtonState) {
    if (PS4.Up() == 1) {
      myData.b = 6;
    }
    else{
      myData.b = 0;
    }
    lastStepperUpButtonState = PS4.Up();
}
// ---------------- SERVO (Rotate) -----------
  else if (PS4.Left() != lastLeftButtonState) {
    if (PS4.Left() == 1) {
      myData.b = 7;
    }
    else{
      myData.b = 0;
    }
    lastLeftButtonState = PS4.Left();
} 

  else if (PS4.Right() != lastRightButtonState) {
    if (PS4.Right() == 1) {
      myData.b = 8;
    }
    else{
      myData.b = 0;
    }
    lastRightButtonState = PS4.Right();
} 


// Project Servo & PNEUMATICS
  // Check if the button state has changed (pressed/released)
  if (PS4.R1() != lastR1ButtonState) {
    // Check if the button is currently pressed
    if (PS4.R1() == 1) {
      myData.b = 9;
    }
    else{
      myData.b = 0;
    }
  }
  // Update the last button state
  lastR1ButtonState = PS4.R1();

  if (PS4.R2() != lastR2ButtonState){
    if(PS4.R2() == 1){
      if (projectservo_status1 == 0){
        projectservoCurrentAngle = projectservo_angle2;
        projectservo_status1 = 1;
      }
      else if (projectservo_status1 == 1){
        projectservoCurrentAngle = projectservo_angle1;
        projectservo_status1 = 0;
      }
      myData.b = 10;
    }
    else {
    myData.b = 0;
    }
  }
  lastR2ButtonState = PS4.R2(); 
  projectservoSmoothed = (projectservoCurrentAngle*0.05)+ (projectservoSmoothedPrev * 0.95);
  projectservoSmoothedPrev = projectservoSmoothed;
  Serial.println(projectservoSmoothed);
  projectServo.writeMicroseconds(projectservoSmoothed);


// Shooting (PNEUMATICS)
  if (PS4.L1() != lastL1ButtonState) {
    // Check if the button is currently pressed
    if (PS4.L1() == 1) {
       myData.b = 11;
    }
    else {
    myData.b = 0;
    }
  }
  lastL1ButtonState = PS4.L1(); 

//.............esp now
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  // if (PS4.Triangle()) rollerSpeed += 10; // need long delay and small increment otherwise speed changes rapidly, causing motor jerk
  // else if (PS4.Cross()) rollerSpeed -= 10;

  // rollerSpeed = constrain(rollerSpeed, 0, 500);
  int topRollerSpeed = 1500 + rollerSpeeds[speedSelect];
  int bottomRollerSpeed = 1500 + rollerSpeeds[speedSelect];

  topRoller.writeMicroseconds(topRollerSpeed);
  bottomRoller.writeMicroseconds(bottomRollerSpeed);


  // Serial.print("Top Roller Speed (microseconds): ");
  // Serial.println(topRollerSpeed);
  // Serial.print("Bottom Roller Speed (microseconds): ");
  // Serial.println(bottomRollerSpeed);


  // // -------------------- C620 --------------------

  // // read PS4 and set drive mode
  const int deadband = 40;
  int driveMode = STOP;

  if (abs(PS4.LStickX()) <= deadband && PS4.LStickY() >= deadband) driveMode = FORWARD;
  else if (abs(PS4.LStickX()) <= deadband && PS4.LStickY() <= -deadband) driveMode = BACKWARD;
  else if (abs(PS4.LStickY()) <= deadband && PS4.LStickX() >= deadband) driveMode = RIGHT;
  else if (abs(PS4.LStickY()) <= deadband && PS4.LStickX() <= -deadband) driveMode = LEFT;
  else if (PS4.LStickX() <= -deadband && PS4.LStickY() >= deadband) driveMode = FORWARD_LEFT;
  else if (PS4.LStickX() <= -deadband && PS4.LStickY() <= -deadband) driveMode = BACKWARD_LEFT;
  else if (PS4.LStickX() >= deadband && PS4.LStickY() >= deadband) driveMode = FORWARD_RIGHT;
  else if (PS4.LStickX() >= deadband && PS4.LStickY() <= -deadband) driveMode = BACKWARD_RIGHT;
  else if (PS4.RStickX() >= deadband) driveMode = ROTATE_CCW;
  else if (PS4.RStickX() <= -deadband) driveMode = ROTATE_CW;


    // calculate motor speeds according to drive mode
  int nominalSpeed = 120;
  int speedMultiplier = 1;

  if (PS4.Options() == 1) speedMultiplier = 1.5;
  else speedMultiplier = 1;

  short c620targetSpeeds[4];

  switch (driveMode) {
    case FORWARD:
      c620targetSpeeds[0] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[1] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[2] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[3] = -nominalSpeed * speedMultiplier;
      break;

    case BACKWARD:
      c620targetSpeeds[0] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[1] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[2] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[3] = nominalSpeed * speedMultiplier;
      break;

    case LEFT:
      c620targetSpeeds[0] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[1] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[2] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[3] = nominalSpeed * speedMultiplier;
      break;

    case RIGHT:
      c620targetSpeeds[0] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[1] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[2] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[3] = -nominalSpeed * speedMultiplier;
      break;

    case FORWARD_LEFT:
      c620targetSpeeds[0] = 0;
      c620targetSpeeds[1] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[2] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[3] = 0;
      break;

    case FORWARD_RIGHT:
      c620targetSpeeds[0] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[1] = 0;
      c620targetSpeeds[2] = 0;
      c620targetSpeeds[3] = -nominalSpeed * speedMultiplier;
      break;

    case BACKWARD_LEFT:
      c620targetSpeeds[0] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[1] = 0;
      c620targetSpeeds[2] = 0;
      c620targetSpeeds[3] = nominalSpeed * speedMultiplier;
      break;

    case BACKWARD_RIGHT:
      c620targetSpeeds[0] = 0;
      c620targetSpeeds[1] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[2] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[3] = 0;
      break;

    case ROTATE_CW:
      c620targetSpeeds[0] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[1] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[2] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[3] = -nominalSpeed * speedMultiplier;
      break;

    case ROTATE_CCW:
      c620targetSpeeds[0] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[1] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[2] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[3] = nominalSpeed * speedMultiplier;
      break;

    default:
      c620targetSpeeds[0] = 0;    
      c620targetSpeeds[1] = 0;    
      c620targetSpeeds[2] = 0;    
      c620targetSpeeds[3] = 0;    
      break;
  }  

  // // Serial.println(driveMode);
  // // Serial.println(c620targetSpeeds[1] + 1500);

  frontLeftWheel.writeMicroseconds(c620targetSpeeds[0] + 1500);
  backLeftWheel.writeMicroseconds(c620targetSpeeds[1] + 1500);
  frontRightWheel.writeMicroseconds(c620targetSpeeds[2] + 1500);
  backRightWheel.writeMicroseconds(c620targetSpeeds[3] + 1500);


  delay(50);
}