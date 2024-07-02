// library
#include <ESP32Servo.h>

// Uart
//HardwareSerial Receiver(2); 
//#define Receiver_Txd_pin 27
//#define Receiver_Rxd_pin 14

// Setup Gripper servo and Pins

//Servo GripperServo3;
//Servo GripperServo4;
//int GripperServoPin3 = 32;
//int GripperServoPin4 = 33;
//#define Gripper_Open 1
//#define Gripper_Close 0
//int Gripper_Status1 = Gripper_Close;
//int Gripper_Status2 = Gripper_Close;

// Setup Rotate servo and pins
Servo RotateServo;
Servo RotateServo2;
//Servo RotateServo3;
//Servo RotateServo4;
int RotateServoPin = 17;
int RotateServoPin2 = 5;
//int RotateServoPin3 = 18;
//int RotateServoPin4 = 19;
bool pinModeStatus = 1;

// Setup Stepper
#define STEPPER_DIR 23
#define STEPPER_PUL 22
const int stepper_speed = 60000;  // Microseconds between steps

// Pneumatics
const int relayPin1 = 25; // Ball Gripper
const int relayPin2 = 26; // Ball Pusher
bool relayState = false;
bool relay2State = false;

// define the receiving value from esp sender
//#define Trianglebuttonvalue 1
//#define Crossbuttonvalue 2
#define Squarebuttonvalue 3
#define Circlebuttonvalue 4
#define Upbuttonvalue 5
#define Downbuttonvalue 6
#define Leftbuttonvalue 7
#define Rightbuttonvalue 8
#define R1buttonvalue 9
#define R2buttonvalue1 10
#define L1buttonvalue1 11


//........... esp now
#include <esp_now.h>
#include <WiFi.h>
int last_received_data =0;
// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    int b;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Int: ");
  Serial.println(myData.b);
}
 


void setup() {
  
  Serial.begin(115200);
  pinMode(STEPPER_DIR,OUTPUT);
  pinMode(STEPPER_PUL,OUTPUT);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);  
  
  //............esp now
 // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

   
  //Serial.begin(Baud Rate, Data Protocol, Txd pin, Rxd pin);      
  //Receiver.begin(115200, SERIAL_8N1, Receiver_Txd_pin, Receiver_Rxd_pin); // Define and start Receiver serial port

  // Extend the Rotate Servos after intialize the Robot
  RotateServo.attach(RotateServoPin);  
  RotateServo2.attach(RotateServoPin2);
 


  // -------------------- PNEUMATICS (RELAYS & SERVO) --------------------
  pinMode(relayPin1,OUTPUT);
  pinMode(relayPin2,OUTPUT);
  digitalWrite(relayPin1, 0); 
  digitalWrite(relayPin2, 0);

  // test code
 // RotateServo3.write(90);
 // RotateServo4.write(0); 

  
//reset to the default setting
//RotateServo2.write(0);
//RotateServo2.write(90);
}

void loop() {
 while (myData.b!=last_received_data) {                // Wait for the Receiver to get the characters
     // Display the Receivers characters
    
    switch (myData.b) {
      // Press the Sqaure button , if the Grippers are closed,then extend them 
      case Squarebuttonvalue:

        pinMode(RotateServoPin,INPUT);
        pinMode(RotateServoPin2,INPUT);      
        break;

      // Press the Up button, the stepper will go up  
      case Upbuttonvalue:
        digitalWrite(STEPPER_DIR, LOW);
        tone(STEPPER_PUL, stepper_speed);
        Serial.println("up");
        break;
      // Press the Down button, the stepper will go down
      case Downbuttonvalue:
        digitalWrite(STEPPER_DIR, HIGH);
        tone(STEPPER_PUL, stepper_speed);
        Serial.println("down");
        break;
      // Press the Left button, reset the rotate servos
     case Leftbuttonvalue:
        RotateServo.write(1);
        RotateServo2.write(180);
       break;

      // Press the Right button to extend or retract them  
      case Rightbuttonvalue:
        RotateServo.write(83);
        RotateServo2.write(90);
        break;

      // Project Servo Relay 
      case R1buttonvalue:
          relayState = !relayState;
          digitalWrite(relayPin1, relayState ? 1 : 0);
          relayState = relayState;
          break;
      case R2buttonvalue1:   
          //digitalWrite(relayPin1, 1); //Ball Gripper Open
          digitalWrite(relayPin2, 0); // 1: Extend , 0:Retract
          Serial.println("r1 open");
          Serial.println("r2 open");
          break;
      case L1buttonvalue1:
          relay2State = !relay2State;
          digitalWrite(relayPin2, 1);
          delay(1000);
          digitalWrite(relayPin2, 0);
          relay2State = relay2State;
          break;
      case 0:
          noTone(STEPPER_PUL);
          Serial.println("stop");
          break;
        
    
  }   
      last_received_data = myData.b;
      Serial.println(myData.b);              // Display the result on the serial monitor
  }

  // delay(1000);
}