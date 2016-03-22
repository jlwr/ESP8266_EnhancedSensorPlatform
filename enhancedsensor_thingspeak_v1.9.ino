/*
* This sketch will enable the user to send the data onto ThingSpeak
* The channel will update 15 seconds after the client has been connected.
* The link to the ThingSpeak channel is: https://thingspeak.com/channels/80181
* Credits to ThingSpeak (API) and the ESP8266 community forum.
*
* Developed by: Josiah Lee
* Version: 9
* Created on: 03-Jan-2016
* Last Modified: 12-Mar-2016
*/
 
#include <ESP8266WiFi.h> //Library extension to use the ESP8266 WiFi functions
#include <Wire.h> //Library extension for the use of I2C communication
#include <SparkFunLSM6DS3.h> //SparkFun LSM6DS3 library extension to obtain the Accelerometer/Gyroscope and Temperature readings from the sensor
 
#define PIN 16 // Corresponds to GPIO16 labelled pin D0 on NodeMCU board this pin is also connected to the LED cathode on the NodeMCU board
#define LED 10 //Green LED on Prototype board
#define IND 5 //Red LED on Prototype board

volatile int state1 = LOW; //State used for Right wheel optical sensor
volatile int state2 = LOW; //State used for Left wheel optical sensor
int count = 0; //initialise counter for left wheel
int count2 = 0; //initialise counter for right wheel
float lwDist = 0; //Measured left wheel distance
float rwDist = 0; //Measured right wheel distance

//Initialise methods for optical sensor interrupt
void changeINT(); //interrupt for left wheel optical-switch sensor
void switchINT(); //interrupt for right wheel optical-switch sensor

//Initialise methods to update ThingSpeak channel
void updateOpto(int i); //Method for left wheel distance update
void updateOpto2(int k); //Method for right wheel distance update
void updateProx(int j); //Method for proximity count update

//Initialise methods for proximity sensor
void WriteByte(uint8_t ad, uint8_t r, byte d);
void ReadByte(byte add, byte re, byte *bd);
void ReadWord(byte adr, byte rg, int *da);

//Initialise method to read tweet updated on ThingSpeak channel
void readData();

//Initialisation of Router
const char* ssid = "AndroidAP"; //Key in your router's SSID
const char* password = "yjzo9236"; //Key in your router's password

//ThingSpeak Settings
const char* tserver = "api.thingspeak.com";
String apiKey = "IVKFTYDIL5ODU5CM"; //API key unique to the user's channel, key in your own write API key
 
// Create an instance of the server
// specify the port to listen on as an argument
WiFiServer server(80);
WiFiClient client = server.available();

//Create an instance of the 3-D accelerometer/gyroscope sensor 
LSM6DS3 myIMU; 
 
void setup() {
Serial.begin(115200); // Set baud rate to 115200
Serial.swap(); //Swapping the internal UART of ESP8266 to RXD2 and TXD2 - GPIO13 & GPIO15  
//comment Serial.swap() out if you would like to use the Serial monitor of the Arduino IDE

// prepare GPIO & LED PIN
pinMode(PIN, OUTPUT);
digitalWrite(PIN, 0);
pinMode(LED, OUTPUT);
pinMode(IND, OUTPUT); //change 9 to IND

// PREPARE INTERRUPT FOR OPTICAL WHEEL SENSOR
attachInterrupt(digitalPinToInterrupt(14),changeINT,FALLING);
attachInterrupt(digitalPinToInterrupt(12),switchINT,FALLING);

// PREPARE I2C FOR PROX SENSOR
Wire.begin(0, 2);  // initialize I2C stuff: SDA, SCL (GPIO 0; GPIO 2)

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
PGAIN = 0; //1x Prox gain

WriteByte (0x39, 0xf, PDRIVE | PDIODE | PGAIN );

byte WEN, PEN, PON;
WEN = 8; // Enable Wait
PEN = 4; // Enable Prox
PON = 1; // Enable Power On

WriteByte (0x39, 0, WEN | PEN | PON);
delay(120); //Wait for 12 ms

//Prepare LSM6DS3 IMU (Accel/Gyro Sensor)
myIMU.begin();

WiFi.begin(ssid, password);
 
while (WiFi.status() != WL_CONNECTED) {
delay(500);
//Serial.print(".");
}
//Serial.println("");
//Serial.println("WiFi connected");
 
// Start the server
server.begin();

//Serial.println("Server started");
// Print the IP address
//Serial.println(WiFi.localIP());
//else{
//  Serial.println("connection failed");
//}
} //END SETUP
 
void loop() {
  
   float rev = 3.5; //Initialise wheel revolution, rev, variable
   //Initialise prox sensor variables
   byte IDvalue, val3, val4;
   int prox;

   //Proximity sensor read byte
   ReadByte(0x39, 0x12, &IDvalue);
   ReadByte(0x39, 0x00, &val4);
   ReadByte(0x39, 0x13, &val3);
   ReadWord(0x39, 0x18, &prox);

   //Uncomment the section below to read the twitter status update. Note: Remember to comment out Serial.swap() 
   //to allow Serial monitor to work.
   /*readData();
   delay(2000);*/

   //If statement to blink the Green LED when the optical sensor sends an interrupt to the microcontroller
   if(state1 == HIGH){
    digitalWrite(IND, state1);
    delay(100);
    state1 = LOW;
    digitalWrite(IND, state1);
   
    count++; //increment counter
       
    //Left wheel distance calculation
    lwDist = lwDist + rev;
   
    if(count==10) //after moving for 35cm
    {
      updateOpto(lwDist); //(Argument: left wheel distance travelled). Send data to the updateOpto() method
      count = 0; //reset counter
    }
   }//end if statement

   //If statement to blink the Red LED when the optical sensor sends an interrupt to the microcontroller
   if(state2 == HIGH){
     digitalWrite(LED, state2);
     delay(100);
     state2 = LOW;
     digitalWrite(LED, state2);

     count2++; //increment counter

     //Right wheel distance calculation
     rwDist = rwDist + rev;

     if(count2==10) //after moving for 35cm
     {
      updateOpto2(rwDist); //(Argument: left wheel distance travelled). Send data to the updateOpto2() method
      count2=0; //reset counter
     }
   }//end if statement

   //if statement for obstacle detected for a proximity count of more than 50.
   if(prox>50) {
      digitalWrite(IND, HIGH); //light both LED to signal obstacle
      digitalWrite(LED, HIGH);
      delay(100);
      digitalWrite(IND, LOW);
      digitalWrite(LED, LOW);

      //Stop both left and right motors of the Pololu 3pi robot
      Serial.printf("%c%c%c%c", 0xC5, 0x01, 0xC1, 0x01);
      delay(200);
     
      updateProx(prox); //(Argument: proximity count). Send data to the updateProx() method
   }//end if statement
}
  

// Update ThingSpeak channel with left wheel distance travelled
void updateOpto(int dist)
{
   if (client.connect(tserver, 80)) { // use ip 184.106.153.149 or api.thingspeak.com
   Serial.println("WiFi Client connected ");
   String postStr = apiKey;
   postStr += "&field1="; //Update field 1
   postStr += String(dist);
   postStr += "\r\n\r\n";
   
   client.print("POST /update HTTP/1.1\n"); //Post update using HTTP 1.1 protocol
   client.print("Host: api.thingspeak.com\n"); //onto the thingspeak host
   client.print("Connection: close\n");
   client.print("X-THINGSPEAKAPIKEY: " + apiKey + "\n"); //api write key
   client.print("Content-Type: application/x-www-form-urlencoded\n");
   client.print("Content-Length: ");
   client.print(postStr.length());
   client.print("\n\n");
   client.print(postStr);
   Serial.println(dist); //print the left wheel distance 
   }//end if
   client.stop();
}//end updateOpto

// Update ThingSpeak channel with right wheel distance travelled
void updateOpto2(int dist2)
{
   if (client.connect(tserver, 80)) { // use ip 184.106.153.149 or api.thingspeak.com
   Serial.println("WiFi Client connected ");
   String postStr = apiKey;
   postStr += "&field3="; //Update field 3
   postStr += String(dist2);
   postStr += "\r\n\r\n";
   
   client.print("POST /update HTTP/1.1\n"); //Post update using HTTP 1.1 protocol
   client.print("Host: api.thingspeak.com\n"); //onto the thingspeak host
   client.print("Connection: close\n");
   client.print("X-THINGSPEAKAPIKEY: " + apiKey + "\n"); //api write key
   client.print("Content-Type: application/x-www-form-urlencoded\n");
   client.print("Content-Length: ");
   client.print(postStr.length());
   client.print("\n\n");
   client.print(postStr);
   Serial.println(dist2); //print the right wheel distance
   }//end if
   client.stop();
}//end updateOpto2

// Update ThingSpeak channel with proximity count when obstacle is detected
void updateProx(int range)
{
  if (client.connect(tserver, 80)) { // use ip 184.106.153.149 or api.thingspeak.com
   Serial.println("WiFi Client connected ");
   String postStr = apiKey;
   postStr += "&field2="; //Update field 2
   postStr += String(range);
   postStr += "\r\n\r\n";
   
   client.print("POST /update HTTP/1.1\n"); //Post update using HTTP 1.1 protocol
   client.print("Host: api.thingspeak.com\n"); //onto the thingspeak host
   client.print("Connection: close\n");
   client.print("X-THINGSPEAKAPIKEY: " + apiKey + "\n"); //api write key
   client.print("Content-Type: application/x-www-form-urlencoded\n");
   client.print("Content-Length: ");
   client.print(postStr.length());
   client.print("\n\n");
   client.print(postStr);
   Serial.println(range); //print the proximity count 
   }//end if
   client.stop();
}//end updateProx

//Uncomment the section below to allow reading of ThingSpeak status update from twitter
//The trigger for the twitter update is "drivejl02"
/*void readData(){
  //Serial.println("in readData");
  if (client.connect(tserver, 80)){
    //Serial.println("HELLO connected to server");
    //client.println("GET /channels/80181/status.json HTTP/1.1");
    client.println("GET /channels/80181/field/1/last.json HTTP/1.1");
    client.println("Host: api.thingspeak.com");
    client.println("Connection: close");
    client.println();

   while(client.available()){
    char c = client.read();
    //Serial.write(c);
    Serial.print(c);
   }
   if(!client.connected()){
    Serial.println();
    Serial.println("disconnecting from server");
    client.stop();
   }
  }
}*/

//Method called when Interrupt occurs for HLC1395 left wheel Optical sensor
void changeINT()  {
  state1 = HIGH;
}

//Method called when Interrupt occurs for HLC1395 right wheel Optical sensor
void switchINT(){
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
