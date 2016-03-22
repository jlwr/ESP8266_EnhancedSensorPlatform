/*
* This sketch will enable the user to read the sensor data on a webserver and control the robot by clicking the commands on the webserver
* The server will receive the data when the webserver has been refreshed
* http://192.168.43.5 to obtain reading  
* Open the link on the PC/Mobile that has the same WiFi connection as the ESP8266
*
* Developed by: Josiah Lee
* Version: 23
* Created on: 04-Feb-2016
* Last Modified: 12-Mar-2016
*/
 
#include <ESP8266WiFi.h> //Library extension to use the ESP8266 WiFi functions
#include <Wire.h> //Library extension for the use of I2C communication
#include <SparkFunLSM6DS3.h> //SparkFun LSM6DS3 library extension to obtain the Accelerometer/Gyroscope and Temperature readings from the sensor

#define PIN 16 // Corresponds to GPIO16 labelled pin D0 on NodeMCU board this pin is also connected to the LED cathode on the NodeMCU board
#define LED 10 //Green LED on Prototype board
#define IND 5 //Red LED 

volatile int state1 = LOW; //State used for Right wheel optical sensor
volatile int state2 = LOW; //State used for Left wheel optical sensor
float rwDist = 0; //Measured right wheel distance 
float lwDist = 0; //Measured left wheel distance
double speedLw = 0; //Calculated left wheel speed
double speedRw = 0; //Calculated right wheel speed
int setzeroLW = 0; //reset left wheel 
int setzeroRW = 0; //reset right wheel
int trigger = 0; //Trigger for Straight movement
int lwTrig = 0; //Trigger for left turn to prevent increment in distance
int rwTrig = 0; //Trigger for right turn to prevent increment in distance

unsigned long timeStart; //current record of time
unsigned long timeEndRw; //previous record of time before exit of interrupt
unsigned long timeDiffRw; //to calculate time difference between each record of rw wheel count
unsigned long timePrevRw = 0; //storing of previous time

unsigned long timeEndLw;
unsigned long timeDiffLw;
unsigned long timePrevLw = 0;

//Initialise methods for optical sensor interrupt
void changeINT1();
void changeINT2();

//Initialise methods for proximity sensor
void WriteByte(uint8_t ad, uint8_t r, byte d);
void ReadByte(byte add, byte re, byte *bd);
void ReadWord(byte adr, byte rg, int *da);

//Initialise WiFi Router
const char* ssid = "AndroidAP"; //Change to the SSID of your router
const char* password = "yjzo9236"; //Change to the password of your router
 
//Create an instance of the server
//specify the port to listen on as an argument
WiFiServer server(80);

//Create an instance of the 3-D accelerometer/gyroscope sensor 
LSM6DS3 myIMU;
 
void setup() {
Serial.begin(115200); //Set baud rate to 115200
Serial.swap(); //Swapping the internal UART of ESP8266 to RXD2 and TXD2 - GPIO13 & GPIO15 
delay(10);
 
//Prepare GPIO PIN
pinMode(PIN, OUTPUT);
digitalWrite(PIN, 0); //Ground GPIO 16
pinMode(LED, OUTPUT); //Initialise GREEN LED
pinMode(IND, OUTPUT); //Initialise RED LED

//Prepare Interrupt for optical wheel sensors
attachInterrupt(digitalPinToInterrupt(14),changeINT1,FALLING); //For left optical sensor
attachInterrupt(digitalPinToInterrupt(12),changeINT2,FALLING); //For right optical sensor

//Prepare I2C for proximity sensor
Wire.begin(0, 2); //SDA (GPIO 0), SCL (GPIO 2)
byte PTIME;
byte WTIME;
byte PPULSE;
 
WTIME = 0xff; // 2.7 ms - minimum Wait time
PTIME = 0xff; // 2.7 ms - minimum Prox integration time
PPULSE = 1; // Minimum prox pulse count

WriteByte(0x39, 0, 0); //Disable and Powerdown
WriteByte (0x39, 2, PTIME);
WriteByte (0x39, 3, WTIME);
WriteByte (0x39, 0xe, PPULSE);

byte PDRIVE;
byte PDIODE;
byte PGAIN;

PDRIVE = 0; //100mA of LED Power
PDIODE = 0x20; // CH1 Diode
PGAIN = 10; //8x Prox gain

WriteByte (0x39, 0xf, PDRIVE | PDIODE | PGAIN );

byte WEN, PEN, PON;
WEN = 8; // Enable Wait
PEN = 4; // Enable Prox
PON = 1; // Enable Power On

WriteByte (0x39, 0, WEN | PEN | PON);
delay(120); //Wait for 12 ms

//Prepare LSM6DS3 IMU (Accel/Gyro Sensor)
myIMU.begin();

//Prepare connection to WiFi network
WiFi.begin(ssid, password);
 
while (WiFi.status() != WL_CONNECTED) {
delay(500);
}
 
// Start the server
server.begin();
} //END SETUP

void loop() {
// Check if a client has connected
WiFiClient client = server.available();
timeStart = millis(); //get time in milliseconds

float rev= 3.5; //the circumference of the wheel
int prox;
byte IDvalue, val3, val4;
client.setTimeout(50); //setting timeout per transmission to 50ms (timeout for connection to webserver)
String req = client.readStringUntil('/r'); //Read client's request

//Proximity sensor read byte
ReadByte(0x39, 0x12, &IDvalue);
ReadByte(0x39, 0x00, &val4);
ReadByte(0x39, 0x13, &val3);
ReadWord(0x39, 0x18, &prox);

//For every detection of wheel spoke, increase the distance by 3.5cm
//Check if all wheel spokes are detected. If so, then each count = circumference of wheel/3 (cm)

if(state1 == HIGH) //Left optical wheel sensor detects spoke, interrupt program
{
   //Set time comparison to last interrupt
   timeEndLw = timePrevLw;
   timePrevLw = timeStart;

   //Indicate interrupt through RED LED
   digitalWrite(IND, state1);
   delay(60);
   state1 = LOW;
   digitalWrite(IND, state1);
   //Left wheel distance calculation
   lwDist = lwDist + rev;

   //Calculation for time comparison to previous interrupt to prevent extra count
   timeDiffLw = timePrevLw - timeEndLw;
   //If optical wheel sensor calls for interrupt earlier than expected
   //then ignore the count
   if(timeDiffLw<170 || lwTrig==1)
   {
    lwDist = lwDist - rev;
    lwTrig = 0;
   }

   //If reset of wheels was called, reset distance covered and set wheels
   //to initial position
   if(setzeroLW == 1)
   {
    lwDist = 0;
    setzeroLW = 0;
   }
}

//Same comments as left wheel as stated above. This conditions are meant for
//the right wheel
if(state2 == HIGH)
{
   timeEndRw = timePrevRw;
   timePrevRw = timeStart;
   digitalWrite(LED, state2);
   delay(60);
   state2 = LOW;
   digitalWrite(LED, state2);
   rwDist = rwDist + rev;

   timeDiffRw = timePrevRw - timeEndRw;
   if(timeDiffRw<170 || rwTrig==1) //Compensate for double count
   {
    rwDist = rwDist - rev;
    rwTrig = 0;
   }
   if(setzeroRW == 1) //when RESET command has been executed
   {
    rwDist = 0;
    setzeroRW = 0;
   }
}

//Calculation for average distance covered
float avg = (lwDist+rwDist)/2;

//Speed calculation in cm/s
speedLw = (3.5/(timeDiffLw))*1000;
speedRw = (3.5/(timeDiffRw))*1000;

//Calibration of proximity sensor. The proximity detected relates to different distance from an obstacle
String proxDist;
if(prox>800 && prox<=1023)
  proxDist = "0 to 2 cm";
else if(prox>730 && prox<=800)
  proxDist = "3 cm";
else if(prox>390 && prox<=730)
  proxDist = "4 cm";
else if(prox>248 && prox<=390)
  proxDist = "5 cm";
else if(prox>160 && prox<=248)
  proxDist = "6 cm";
else if(prox>110 && prox<=160)
  proxDist = "7 cm";
else if(prox>80 && prox<=110)
  proxDist = "8 cm";
else if(prox>64 && prox<=80)
  proxDist = "9 cm";
else if(prox>39 && prox<=64)
  proxDist = "10 cm";
else if(prox>32 && prox<=39)
  proxDist = "11 cm";
else if(prox>16 && prox<=32)
  proxDist = "12 cm";
else if(prox>11 && prox<=16)
  proxDist = "13 cm";
else if(prox>7 && prox<=11)
  proxDist = "14 cm";
else
  proxDist = ">15 cm";

//Obstacle avoidance 
if(prox>30) 
{
      //Light both LEDS to signal obstacle
      digitalWrite(IND, HIGH); 
      digitalWrite(LED, HIGH);
      delay(50);
      digitalWrite(IND, LOW);
      digitalWrite(LED, LOW);
      
      //Stop motors then turn right
      Serial.printf("%c%c%c%c", 0xC5, 0x01, 0xC1, 0x01);
      delay(150);
      Serial.printf("%c%c%c%c", 0xC1, 0x10, 0xC6, 0x10);
      delay(400);   
      Serial.printf("%c%c%c%c", 0xC5, 0x01, 0xC1, 0x01);
      delay(150);   
      //Turn right calculation:
      //radius from centre of bot: 4.75cm
      //Speed of bot: 20.35cm/secs
      //1/4 circumference = (pi*(4.75)*2)/4 = 7.6413cm
      //time taken to cover 7.6413cm = 7.6413/20.35 = 370ms 
}

// Read the first line of the request on Webserver
// Match the request
//Pololu 3pi robot right wheel hexadecimal command: C5 - forward, C6 - reverse
//Pololu 3pi robot left wheel hexadecimal command: C1 - forward, C2 - reverse
if (req.indexOf("/move") != -1){ //Moving forward
  Serial.printf("%c%c%c%c", 0xC5, 0x15, 0xC1, 0x15); //forward drive
}
else if (req.indexOf("/stop") != -1){ //Stop
  Serial.printf("%c%c%c%c", 0xC5, 0x01, 0xC1, 0x01); //forward drive
}
else if (req.indexOf("/yaw") != - 1){ //Turn Right
  //delay(10);
  lwTrig = 1;
  Serial.printf("%c%c%c%c", 0xC1, 0x10, 0xC5, 0x01);
}
else if (req.indexOf("/left") != - 1){ //Turn Left
  rwTrig = 1;
  Serial.printf("%c%c%c%c", 0xC5, 0x10, 0xC1, 0x01);
}
//Reset the wheels to the same starting point, to ensure that the distance calculation for both wheels will be similar.
else if (req.indexOf("/setnew") != -1){ 
  while(1){
    Serial.printf("%c%c", 0xC1, 0x15); //Reset left wheel position
    if(state1==HIGH)
    {//if spoke of wheel is detected, stop motor
      Serial.printf("%c%c", 0xC1, 0x01);
      setzeroLW = 1; //reset left wheel distance travelled to zero
      break;
    }
  }
  while(1){
    Serial.printf("%c%c", 0xC5, 0x15); //Reset right wheel position
    if(state2==HIGH)
    {//if spoke of wheel is detected, stop motor
      Serial.printf("%c%c", 0xC5, 0x01);
      setzeroRW = 1; //reset right wheel distance travelled to zero
      break;
    }
  }
}
else if (req.indexOf("/zhi") != -1){ //Straight line algorithm
  if(rwDist>lwDist) //if the distance covered by the right wheel is higher
  {
    Serial.printf("%c%c%c%c", 0xC1, 0x12, 0xC5, 0x10); //increase speed of left motor
    delay(500);
    //Reset distance travelled
    setzeroRW = 1;
    setzeroLW = 1;
  }
  else if(lwDist>rwDist) //if the distance covered by the left wheel is higher
  {
    Serial.printf("%c%c%c%c", 0xC1, 0x10, 0xC5, 0x12); //increase speed of right motor
    delay(500);
    //Reset distance travelled
    setzeroRW = 1;
    setzeroLW = 1;
  }
  //if the distance for both wheels are the same, both motors will drive at the same speed
  else
  {
    Serial.printf("%c%c%c%c", 0xC1, 0x10, 0xC5, 0x10);
    delay(500);
  }
}
else { //disconnect from server if non of the request matches
client.stop(); //stop client
return;
}

delay(10);

//HTML Webserver printing of commands and data
if(client){ //got client?
  boolean currentLineIsBlank = true;
  while(client.connected()){
    if(client.available()){ //client data available to read
      char c = client.read(); //read 1 byte (character) from client
      if(c == '\n' && currentLineIsBlank){
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html"); //text type HTML language
        client.println("Connection: close");  // the connection will be closed after completion of the response
        client.println("Refresh: 1");  // refresh the page automatically every 1 sec
        client.println();
        client.println("<!DOCTYPE HTML>");
        client.println("<html>"); //start html text
        // output the value of each analog input pin
        //<br>: space
        //<strong>: bold words
        client.println("<strong>DISTANCE COVERED: WHEEL OPTICAL SENSOR</strong><br><br>");
        client.println("Right wheel distance: ");
        client.print(rwDist); //print right wheel distance data
        client.println("cm <br>Left wheel distance: ");
        client.print(lwDist); //print left wheel distance data
        client.println("cm <br>Average distance: ");
        client.print(avg); //print average distance data
        client.println("cm <br><strong>PROXIMITY TO OBJECT: FRONT PROXIMITY SENSOR</strong><br><br>");
        client.println("Current Prox: ");
        client.print(prox); //print proximity count data
        client.println("<br>Distance to object (+/- 1 cm): ");
        client.print(proxDist); //print distance away from obstacle 
        client.println("<br><strong>TEMPERATURE SENSOR (Degree C): </strong>");
        client.print(myIMU.readTempC(), 4); //print temperature read from LSM6DS3 sensor
        client.println("<br><br>");
        //Links to execute request
        client.println("<a href=\"/move\">MOVE</a>"); 
        client.println("&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp<a href=\"/setnew\">RESET</a><br>");
        client.println("<a href=\"/stop\">STOP</a>"); 
        client.println("&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp&nbsp<a href=\"/zhi\">STRAIGHT</a><br>");
        client.println("<a href=\"/yaw\">RIGHT</a><br>");
        client.println("<a href=\"/left\">LEFT</a><br>");        
        client.println("<br>Time diff RW: "); //print time taken between each spoke of the wheel, used for speed calculation
        client.print(timeDiffRw);
        client.println("<br>Time diff LW: "); //print time taken between each spoke of the wheel, used for speed calculation
        client.print(timeDiffLw);
        client.println("<br>LW Speed: ");
        client.print(speedLw); //Print speed of left wheel
        client.print(" cm/s");
        client.println("<br>RW Speed: ");
        client.print(speedRw); //Print speed of right wheel
        client.print(" cm/s");
        client.println("</html>"); //close html text
        break;
      }
      if(c=='\n'){
        // last character on line of received text
        // starting new line with next character read
        currentLineIsBlank = true;
      }
      else if(c!='\r'){
        // a text character was received from client
        currentLineIsBlank = false;
      }
    }
  }
  delay(1); //give web browser time to reeive the data
  client.stop(); // close the connection
}
 
// The client will actually be disconnected
// when the function returns and 'client' object is destroyed
} //END LOOP

//Function for Left optical wheel sensor interrupt
void changeINT1()  {
  state1 = HIGH;
}

//Function for Right optical wheel sensor interrupt
void changeINT2()  {
  state2 = HIGH;
}

//Read word function for Proximity sensor
void ReadWord(byte addr, byte reg, int *data) {
  int reading;
  Wire.beginTransmission(addr);
  Wire.write(0xA0 | reg);
  Wire.endTransmission(); // Read a byte from the device.
  Wire.requestFrom(addr, 2);    // request 2 bytes from slave device #112

  if(2 <= Wire.available())    // if two bytes were received
  {
     reading = Wire.read();  // receive low byte (overwrites previous reading)
     reading |= Wire.read()<<8; // receive high byte as higher 8 bits
     *data = reading;
  }
}

//Read byte function for proximity sensor
void ReadByte(byte addr, byte reg, byte *bytedata) {
  Wire.beginTransmission(addr);
  Wire.write(0x80 | reg);
  Wire.endTransmission(); // Read a byte from the device.

  Wire.requestFrom(addr, 2);    // request 2 bytes from slave device #112

  if(1 <= Wire.available())    // if two bytes were received
  {
     *bytedata = Wire.read();  // receive low byte (overwrites previous reading)
  }
}

// Write a byte on the i2c interface of proximity sensor
void WriteByte(uint8_t addr, uint8_t reg, byte data) {
  // Begin the write sequence
  Wire.beginTransmission(addr); 
  Wire.write(0x80 | reg); // First byte is to set the register pointer
  Wire.write(data); // Write the data byte
  Wire.endTransmission(); // End the write sequence; bytes are actually transmitted now
  delay(50);
}
 
