/* 

   <==QUIRON=====]}============
          --.   /|
         _\"/_.'/
       .'._._,.'
       :/ \{}/
      (L  /--',----._
          |          \\
         : /-\ .'-'\ / |
          \\, ||    \|
           \/ ||    ||
   
 This program controls the Leader on the Bicycle field experiments.
  the code was develope with four main branches and its been assemble on a single sketch, those branches are

  --Speedometer code, responsible for the speed camculations and sensor, it uses a single interrupt and some timers
  --Handlebars HCI motor controller, this code controls the speed and direction os the motor to comunicate the rider acceleration instructions
  --Xbee comunication module: responscible for sending ans recieving data, it will use AT mode and strings to send data and recieve instructions
  --GPS reading: the sole proporse its to update GPS readings from the GPS module once a second

Hardware and software developed at HCI Lab - ICESI university -  Cali, Colombia, by Juan Manuel Salamanca and Daniel Vinasco
*/
//------------------------LIBRARIES------------------------\\

//GPS
#include <SoftwareSerial.h> //viertual serial for the GPS
#include <TinyGPS++.h> //Helps with the handling of the GPS

//------------------------VARIABLES------------------------\\

//Debbugging
unsigned long time3; //keep track for USB serial COM
int reportCycle;// how often it sends data to USB serial com

//Speedometer
long time1; // Control for debouncing the sensor and tracking speed
float   mySpeed;// The actual speed calculated each interruption
int sensorPIN = 3; // this must be one of the interruptables 0,1,2,3,7
float circuit;// wheel diameter in cm to calculate covered distance vs time on each lap


//Xbee Com
String beaconMsg;
long time4; 
float timerBeaconLeader;

//GPS
SoftwareSerial serialGPS(8, 9); // Creates virtual serial for the GPS (RX, TX) pins, must jump gps pins (0,1) to Arduino pins (8,9)
TinyGPSPlus gps; //GPS object
float myLat; //My latitud
float myLng; //My longitud
float myDir; //My direction
int gpsOnOffPIN = 12; //this pin switch the gps on and off (one HIGH with switch)
long time2;



// Platoon Size its 4

int bikeID = 0; // bikes 1 throw 3 are fallowers, and 0 the leader
int beaconInterval=1000;


//------------------------SETUP------------------------\\

void setup() {
  //debugging
  reportCycle = 1000;
  Serial1.begin(9600); //USB serial
  time3 = millis();

  //speedometer Setup
  circuit = 1, 52;
  pinMode(sensorPIN, INPUT);
  attachInterrupt(1, speedSensor, RISING);//interrupt for pin 3
  time1 = millis();


  //Xbee Com
  Serial1.begin(9600); //XBee serial1
  time4 = millis();

  //GPS
  serialGPS.begin(4800); //GPS SoftSerial
  myLat = 0;
  myLng = 0;
  myDir = 0;
  pinMode(gpsOnOffPIN, OUTPUT); //starts GPS
  digitalWrite(gpsOnOffPIN, HIGH);
  delay(100);
  digitalWrite(gpsOnOffPIN, LOW);
  time2 = millis();
  time4 = millis();

  //control Formula
}
//------------------------Program Loop------------------------\\

void loop() {
  //debuggibg
 // sendToserial("Speed: " + String(mySpeed) + " GPS Lat: " + String(myLat) + " Lng: " + String(myLng) + " Dir: " + String(myDir));

  //speedometer
  speedometer();

  //GPS
  readFromGPS();
  sendBeacon();
 // readBeacons();
}

//------------------------Functions------------------------\\

//debugging

void sendToserial(String texto) {//send text to serial one each second, for debugging
  if (millis() - time3 > reportCycle) {
    time3 = millis();
    Serial.println(texto);
  }
}

//speedometer
void speedSensor() {// this fuction its executeded each time the sensor interrupts arduino

  if (millis() - time1 > 100) {
    mySpeed = (circuit * 1000) / ((millis() - time1));
    if (millis() - time1 > 3000)
      mySpeed = 0;
    time1 = millis();
  }
}
void speedometer() {
  if (millis() - time1 > 2000) //checks if the wheel hasn't move for 2 seconds and assumes it has stopt, resert speed to 0
    mySpeed = 0;
}


//Xbee Com
void sendBeacon()// pruebas de envio de datos por serial un char a la vez
{
  if (millis() - time4 > beaconInterval-1) {
    time4 = millis();
    Serial1.println(bikeID + String(myLat) + "|" + String(myLng) + "|" + String(mySpeed) + '*'+millis()+'#');//leader also sends its timer millis() after the 
    Serial.println(bikeID + String(myLat) + "|" + String(myLng) + "|" + String(mySpeed) + '*'+millis()+'#');//leader also sends its timer millis() after the 
  }
}
void readBeacons() {

  int beaconFrom;
  int index1;
  float timerTemp;
  if (Serial1.available())
  {

    String linea;
    linea = Serial.readStringUntil('#');
    Serial.println(linea);

  }
}


//GPS
void readFromGPS() {//reads gps serial and updates position information

  if (millis() - time2 > 999) {
    while (serialGPS.available())
    {
      gps.encode(serialGPS.read());
      digitalWrite(13, HIGH);
    }
    digitalWrite(13, LOW);
    myLat = gps.location.lat();
    myLng = gps.location.lng();
    myDir = gps.course.deg();
    time2 = millis();
  }
}

