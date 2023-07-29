#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>        // Instantiate the Wire library
#include <TFLI2C.h>      // TFLuna-I2C Library v.0.1.1

/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/ 
 * Arduino Mecanum Omni Direction Wheel Robot Car
 * Tutorial URL http://osoyoo.com/?p=43404
 * CopyRight www.osoyoo.com
 *  
 * In this project we will connect Robot Car to Wifi and Use 
 * an APP to control the car through an internal WiFi hot spot and use LIDAR and 
 * ultrasonic sensor to detect objects and stop car. 
 * 
 */


/* Define used to substitute serial.print for debug when debugging in the pre-processor */
#define DEBUG 1  //Switch this using 1 for debugging and 0 for not bebugging
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)        // substitutes nothing if DEBUG not == 1
#define debugln(x)
#endif

const int distancelimit = 10;      //distance limit in inches for obstacles in front
const int sidedistancelimit = 10;  //minimum distance in in to obstacles at both sides
int distance;

/* LIDAR sensor setup */
TFLI2C tflI2C;  // TFL = Time of FLight, controlled by I2C
int16_t  tfDist;    // distance in centimeters
int16_t  tfAddr = TFL_DEF_ADR;  // Use this default I2C address


/* Ultrasonic sensor for rear watching setup  */
#define Echo_PIN 31                // Ultrasonic Echo pin connect to A5
#define Trig_PIN 30                // Ultrasonic Trig pin connect to A4
long echo_distance;
  

/*function for detection of LIDAR distance */
int watch() {
  if(tflI2C.getData(tfDist, tfAddr)){
    distance = tfDist/2.54;  //how far away is the object in in
  //  debugln("Lidar Distance "+ String(tfDist)+" cm / " + String(tfDist/2.54)+" inches");
  return round(distance);
  }
 } 

/*function for detection of ultrasonic distance when in reverse*/
int rearWatch() {
  // long echo_distance;
  digitalWrite(Trig_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(Trig_PIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(Trig_PIN, LOW);
  echo_distance = pulseIn(Echo_PIN, HIGH);
  echo_distance = echo_distance * 0.01657 / 2.54;  //how far away is the object in in
//  Serial.print("Sonic Distance = " + String((int)echo_distance));
//  Serial.println(" in");
  //debug("Distance = " + String((int)echo_distance));
  //debugln(" in");
  return round(echo_distance);
}

// Motor definitions
 

/* servo setup */
#define SERVO_PIN 13  //servo connect to D5
Servo head;

// Tail light definitions


/*motor control*/
void right_shift(int speed_fl_fwd, int speed_rl_bck, int speed_rr_fwd, int speed_fr_bck) {
  FL_fwd(speed_fl_fwd);
  RL_bck(speed_rl_bck);
  FR_bck(speed_fr_bck);
  RR_fwd(speed_rr_fwd);
}
void left_shift(int speed_fl_bck, int speed_rl_fwd, int speed_rr_bck, int speed_fr_fwd) {
  FL_bck(speed_fl_bck);
  RL_fwd(speed_rl_fwd);
  FR_fwd(speed_fr_fwd);
  RR_bck(speed_rr_bck);
}
void go_advance(int speed, long distance) {
  RL_fwd(speed);
  RR_fwd(speed);
  FR_fwd(speed);
  FL_fwd(speed);
}
void go_back(int speed) {
  RL_bck(speed);
  RR_bck(speed);
  FR_bck(speed);
  FL_bck(speed);
}
void left_turn(int speed) {
  RL_bck(0);
  RR_fwd(speed);
  FR_fwd(speed);
  FL_bck(0);
  
}
void right_turn(int speed) {
  RL_fwd(speed);
  RR_bck(0);
  FR_bck(0);
  FL_fwd(speed);
}
void left_back(int speed) {
  RL_fwd(0);
  RR_bck(speed);
  FR_bck(speed);
  FL_fwd(0);
}
void right_back(int speed) {
  RL_bck(speed);
  RR_fwd(0);
  FR_fwd(0);
  FL_bck(speed);
}

void clockwise(int speed) {
  RL_fwd(speed);
  RR_bck(speed);
  FR_bck(speed);
  FL_fwd(speed);
}
void countclockwise(int speed) {
  RL_bck(speed);
  RR_fwd(speed);
  FR_fwd(speed);
  FL_bck(speed);
}
void FR_bck(int speed)  //front-right wheel forward turn
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
  analogWrite(speedPinR, speed);
}
void FR_fwd(int speed)  // front-right wheel backward turn
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  analogWrite(speedPinR, speed);
}
void FL_bck(int speed)  // front-left wheel forward turn
{
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);
  analogWrite(speedPinL, speed);
}
void FL_fwd(int speed)  // front-left wheel backward turn
{
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
  analogWrite(speedPinL, speed);
}
void RR_bck(int speed)  //rear-right wheel forward turn
{
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B, HIGH);
  analogWrite(speedPinRB, speed);
}
void RR_fwd(int speed)  //rear-right wheel backward turn
{
  digitalWrite(RightMotorDirPin1B, HIGH);
  digitalWrite(RightMotorDirPin2B, LOW);
  analogWrite(speedPinRB, speed);
}
void RL_bck(int speed)  //rear-left wheel forward turn
{
  digitalWrite(LeftMotorDirPin1B, LOW);
  digitalWrite(LeftMotorDirPin2B, HIGH);
  analogWrite(speedPinLB, speed);
}
void RL_fwd(int speed)  //rear-left wheel backward turn
{
  digitalWrite(LeftMotorDirPin1B, HIGH);
  digitalWrite(LeftMotorDirPin2B, LOW);
  analogWrite(speedPinLB, speed);
}

void stop_Stop()  //Stop
{
  analogWrite(speedPinLB, 0);
  analogWrite(speedPinRB, 0);
  analogWrite(speedPinL, 0);
  analogWrite(speedPinR, 0);
}

//Motor Pins initialize
void init_GPIO() {
  pinMode(RightMotorDirPin1, OUTPUT);
  pinMode(RightMotorDirPin2, OUTPUT);
  pinMode(speedPinL, OUTPUT);

  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  pinMode(RightMotorDirPin1B, OUTPUT);
  pinMode(RightMotorDirPin2B, OUTPUT);
  pinMode(speedPinLB, OUTPUT);

  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT);
  pinMode(speedPinRB, OUTPUT);

  stop_Stop();
}

// Wi-Fi stuff
#include "WiFiEsp.h"
#include "WiFiEspUDP.h"
char ssid[] = "osoyoo_robot";  

int status = WL_IDLE_STATUS;
// use a ring buffer to increase speed and reduce memory allocation
 char packetBuffer[5]; 
WiFiEspUDP Udp;
unsigned int localPort = 8888;  // local port to listen on


void printWifiStatus() {
    // print the SSID of the network you're attached to
    debug("SSID: ");
    debugln(WiFi.SSID());

    // print your WiFi shield's IP address
    IPAddress ip = WiFi.localIP();
    debug("IP Address: ");
    debugln(ip);

    // print where to go in the browser
    debugln();
    debug("To see this page in action, open a browser to http://");
    debugln(ip);
    debugln();
  }

// ===============================================================================
void setup() {
init_GPIO();
  Serial.begin(9600);   // initialize serial for debugging
    Serial1.begin(115200);
    Serial1.write("AT+UART_DEF=9600,8,1,0,0\r\n");
  delay(200);
  Serial1.write("AT+RST\r\n");
  delay(200);
  Serial1.begin(9600);    // initialize serial for ESP module
  WiFi.init(&Serial1);    // initialize ESP module
  tail_Lights_On();       // turn on the tail lights
  Wire.begin();           // Initalize Wire library
  
  // check for the presence of the WiFi shield
  if (WiFi.status() == WL_NO_SHIELD) {
    debugln("WiFi shield not present");
    // don't continue
    while (true);
  }

  debug("Attempting to start WiFi Access Point ");
  debugln(ssid);
  //AP mode
  status = WiFi.beginAP(ssid, 10, "", 0);

  debugln("You're connected to the network");
  printWifiStatus();
  Udp.begin(localPort);

  debug("Listening on port ");
  debugln(localPort);

  // servo initialize
  head.attach(SERVO_PIN);
   debugln("Initializing servo ");
   delay(500);
   head.write(90);  //set servo to middle and wait
   delay(500);
   head.write(10);  //set servo to far right and wait
   delay(500);
   head.write(90);  //set servo to far middle and wait
   delay(500);
   head.write(170);  //set servo to far left and wait
   delay(500);
   head.write(90);  //set servo to far middle and wait
   delay(500);

  /*init Aucostic Sensor - HC-SR04*/
  pinMode(Trig_PIN, OUTPUT);
  pinMode(Echo_PIN, INPUT);
  /*init buzzer*/
  digitalWrite(Trig_PIN, LOW);
  delay(2000);  // Delay so DHT-22 sensor can stabalize
}

// =====================================================================
void loop() {
watch();  //sensor checking for obsticles
rearWatch();
if (distance < 10) stop_Stop();


    //Check Wifi Input
  int packetSize = Udp.parsePacket();
  if (packetSize) {  // if you get a client,
  debug("Received packet of size ");
  debugln(packetSize);
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;

      char c = packetBuffer[0];
 
      // Move sensor to new direction
      switch (c) {
        case 'A':
          head.write(90);
          debugln("Look straight ahead");
          break;  //ahead
        case 'L':
          head.write(180);
          debugln("Look left");
          break;  //left
        case 'R':
          head.write(0);
          debugln("Look right");
          break;  //right
        case 'B':
          head.write(90);
          debugln("Look backwards someday");
          break;  //backwards
        case 'E':
          stop_Stop();
          head.write(90);
          debugln("stop");
          break;  //stop
        case 'F':
          head.write(135);
          debugln("Look left ahead");
          break;  //left ahead
        case 'G':
          head.write(0);
          debugln("Look right");
          break;  //clockwise
        case 'H':
          head.write(45);
          debugln("Look right ahead");
          break;  //right ahead
        case 'I':
          head.write(180);
          debugln("Look left back");
          break;  //left back
        case 'J':
          head.write(180);
          debugln("Look left back");
          break;  //counter clockwise
        case 'K':
          head.write(0);
          debugln("Look right back");
          break;  //right back
        case 'O':
          head.write(180);
          debugln("Look left shift");
          break;//left shift
        case 'T':
          head.write(0);
          debugln("Look right shift");
          break;  //right shift
        default: break;
      }
      // Tail light controls
       switch (c) {
        case 'A': //ahead
          tail_Lights_On();
          break; 
        case 'R': //right
          previousTime = right_tail_light_blink(previousTime); //right turn signal
          break; 
        case 'L':  //left
          previousTime = left_tail_light_blink(previousTime); //left turn signal
          break; 
        case 'B': //backwards
          previousTime = both_tail_light_blink(previousTime); //emergency signal
          break;  
        case 'E': //stop
          tail_Lights_On();
          break;  
        case 'F': //left ahead
         previousTime = left_tail_light_blink(previousTime); //left turn signal
          break;  
        case 'G':  //clockwise
          previousTime = right_tail_light_blink(previousTime); //right turn signal
          break; 
        case 'H': //right ahead
          previousTime = right_tail_light_blink(previousTime); //right turn signal
          break;  
        case 'I':  //left back
          previousTime = left_tail_light_blink(previousTime); //left turn signal
          break; 
        case 'J': //counter clockwise
          previousTime = left_tail_light_blink(previousTime); //left turn signal
          break;  
        case 'K':  //right back
          previousTime = right_tail_light_blink(previousTime); //right turn signal
          break; 
        case 'O': //left shift
          previousTime = left_tail_light_blink(previousTime); //left turn signal
          break;
        case 'T': //right shift
          previousTime = right_tail_light_blink(previousTime); //right turn signal
          break;  

        default: break;
      }

   


  // Move car in selected direction if there's space ============================   
debugln("Lidar Distance in direction area: " + String(distance) +" inches");
  if (distance > 10) {
      
      switch (c)  //serial control instructions
      {
      case 'A':
        go_advance(SPEED, distance);
        debugln("advance");
        break;
      case 'L':
        left_turn(TURN_SPEED);
        debugln("left turn");
        break;
      case 'R':
        right_turn(TURN_SPEED);
        debugln("right turn");
        break;
      case 'B':
        go_back(SPEED);
        debugln("reverse");
        break;
      case 'E':
        stop_Stop();
        debugln("stop");
        break;
      case 'F':
        left_shift(0, 150, 0, 150);
        debugln("left ahead");
        break;  //left ahead
      case 'G':
        clockwise(TURN_SPEED);
        debugln("clockwise");
        break;
      case 'H':
        right_shift(180, 0, 150, 0);
        debugln("right ahead");
        break;  //right ahead
      case 'I':
        left_shift(150, 0, 150, 0);
        debugln("left back");
        break;  //left back
      case 'J':
        countclockwise(TURN_SPEED);
        debugln("counterclockwise");
        break;
      case 'K':
        right_shift(0, 130, 0, 130);
        debugln("right back");
        break;  //right back
      case 'O':
        left_shift(200, 150, 150, 200);
        debugln("left shift");
        break;  //left shift
      case 'T':
        right_shift(200, 200, 200, 200);
        debugln("right shift");
        break;  //right shift
      default: break;
    }
   
  //}
  } else stop_Stop();
 } // end of checking for new direction letter
}  // end of checking for packet
}


