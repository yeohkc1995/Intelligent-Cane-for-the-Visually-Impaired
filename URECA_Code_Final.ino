#include <Arduino.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <ThingSpeak.h>

#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  true
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


//Thingspeak Channel Info
unsigned long myChannelNumber = 416237;
const char * myWriteAPIKey = "8PZRL7BFZUSJBR8P";
TCPClient client;

float lat_marker = 0;
float long_marker = 0;


//code for GPS update onto Thingspeak Server
void GPS_update()
{
    ThingSpeak.setField(1,GPS.latitudeDegrees);
    ThingSpeak.setField(2,GPS.longitudeDegrees);
    ThingSpeak.setField(3,lat_marker);
    ThingSpeak.setField(4,long_marker);

    // Write the fields that you've set all at once.
    ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
}


void marker_update(void)
{
    lat_marker = GPS.latitudeDegrees;
    long_marker = GPS.longitudeDegrees;
}


float latR1, latR2, lonR1, lonR2, dlon,dlat, brRad, reqBear; // for Harversine Formula
double toDegrees = 57.295779; //for arversine Formula


void getBearing(){ //This is a haversine based distance calculation formula
  //This portion converts the current and destination GPS coords from decDegrees to Radians
  latR1 = lat_marker*(PI/180);
  lonR1 = long_marker*(PI/180);
  latR2 = GPS.latitude*(PI/180);
  lonR2 = GPS.longitude*(PI/180);

  //This portion calculates the differences for the Radian latitudes and longitudes and saves them to variables
  dlon = lonR2 - lonR1;
  dlat = latR2 - latR1;

  /*
  //This portion is the Haversine Formula for distance between two points. Returned value is in KM
  a = ((sin(dlat/2))*(sin(dlat/2))) + cos(latR1) * cos(latR2) * ((sin(dlon/2))*(sin(dlon/2)));
  e = 2 * atan2(sqrt(a), sqrt(1-a)) ;
  d = R * e;

  printf("Coordinates A:\nLatitude : %.6lf \nLongitude : %.6lf \n", lat1, lon1);
  printf("Coordinates B:\nLatitude : %.6lf \nLongitude : %.6lf \n", lat2, lon2);
  printf("Distance to destination(KM): ");
  //Serial.println(a);
  //Serial.println(e);
  printf("%.6f", d);
  printf("\n");
  */

  //This portion is the Haversine Formula for required bearing between current location and destination. Returned value is in Degrees
  float x = cos(latR2)*sin(lonR2-lonR1); //calculate x

  Serial.print("X = ");
  Serial.print(x);

  float y = cos(latR1)*sin(latR2)-sin(latR1)*cos(latR2)*cos(lonR2-lonR1); //calculate y

  Serial.print("Y = ");
  Serial.print(y);
  float brRad = atan2(x, y); //return atan2 result for bearing. Result at this point is in Radians

  Serial.print("atan2(x, y) (Radians) = ");
  Serial.print(brRad);

  float reqBear = toDegrees*brRad;
  Serial.print("Bearing: ");
  Serial.print(reqBear);
}

//to update timer every fixed duration
int TStimer;

//code for LSM303
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

//all the variables required for Ultrasonic module
unsigned long durationA, durationB, durationC, period = 25000;
int distanceA, distanceB;


int trig1 = D2;
int echo1 = D3;
int trig2 = D4;
int echo2 = D5;
int rightled = D6;
int leftled = A5;

//variables for vibration
int right_vib = D6;
int left_vib = D7;


void setup()
{
    /*
    ----------------------------------------------------------
    PINS Used:
    D0: SDA for Magnetometer
    D1: SCL for Magnetometer
    D2: Trig1
    D3: Echo1
    D4: Trig2
    D5: Echo2
    D6: Right Vibration
    D7: Left Vibration
    A4: Piezo
    A5: Left LED (temp)
    RX: GPS
    TX: GPS
    -------------------------------------------------------------
    */

    Serial.begin(115200);

    /*
    start of Ultraosnic Setup
    -------------------------------------------------------
    */

    pinMode(trig1, OUTPUT);    // Trig
    pinMode(echo1, INPUT);   // Echo

    pinMode(trig2, OUTPUT);  //Trig
    pinMode(echo2, INPUT);   // Echo

    pinMode(rightled, OUTPUT); // LED
    pinMode(leftled, OUTPUT); // LED

    pinMode(right_vib, OUTPUT);
    pinMode(left_vib, OUTPUT);

    /*End of Ultrasonic setup
    -----------------------------------------------------
    */


    /* Start of ThingSpeak setup
    ---------------------------------------------------
    */

    ThingSpeak.begin(client);

    /*End of Thingspeak setup
    ----------------------------------------------
    */


    /*Start of Test for Magnetometer
    ----------------------------------------------
    */

    /*
    Serial.println("Magnetometer Test"); Serial.println("");

    if(!mag.begin())
    {
        // There was a problem detecting the LSM303 ... check your connections
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while(1);
    }
    */


    /*End of test for Magnetomter
    --------------------------------------------
    */


    /*Start of GPS Marker Interrupt Code
    ---------------------------------------------------------
    */
    pinMode(A0, INPUT);
    attachInterrupt(A0, marker_update, CHANGE);

    /*
    ---------------------------------------------
    End of GPS Marker Interrupt Defination
    */



    /*Start of GPS setup
    ---------------------------------------------------
    */
    Serial.println("Adafruit GPS library basic test!");
    GPS.begin(9600);
    mySerial.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    GPS.sendCommand(PGCMD_ANTENNA);


    #ifdef __arm__
        usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
    #else
        useInterrupt(true);
    #endif

    delay(1000);
    // Ask for firmware version
    mySerial.println(PMTK_Q_RELEASE);

    /*End of GPS setup
    ---------------------------------------------------
    */
}



/*Start of GPS Interupt Functions
--------------------------------------------------------
*/
#ifdef __AVR__
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
    // writing direct to UDR0 is much much faster than Serial.print
    // but only one character can be written at a time.
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
#endif //#ifdef__AVR__

uint32_t timer = millis();

 /*End of GPS Interupt Functions
----------------------------------------------------------
*/


void loop()
{
        /*
        Start of Ultrasonic Code,
        ----------------------------
        */

        //start of Ultrasonic module code
        delay(10);                      // even cicuits need a break
        digitalWrite(trig1, HIGH);         // activate trigger for ultrasonic1
        delayMicroseconds(10);
        digitalWrite(trig1, LOW);          // de-activate trigger

        durationA =  pulseIn(echo1, HIGH);


        delay(10);                      // even cicuits need a break
        digitalWrite(trig2, HIGH);         // activate trigger for ultrasonic 2
        delayMicroseconds(10);
        digitalWrite(trig2, LOW);          // de-activate trigger


        durationB = pulseIn(echo2, HIGH);

        distanceA = durationA/58.2;


        distanceB = durationB/58.2;

        Serial.print("Ultrasonic Distance A: ");
        Serial.print(distanceA);
        Serial.print("\n");
        Serial.print("Ultrasonic Distance B: ");
        Serial.print(distanceB);
        Serial.print("\n\n");

        digitalWrite(right_vib, LOW);
        digitalWrite(left_vib, LOW);

        if ((distanceA < 100) || (distanceB < 100))
        {
            //  digitalWrite(D7, HIGH);     // D7 Blue LED on for 0.4sec if near
            //  delay(400);
            delay(100);
            //    digitalWrite(D7, LOW);
            //    delay(400);
            if (distanceA > distanceB)
              {
                digitalWrite(right_vib, HIGH);
                digitalWrite(left_vib, LOW);
              }
            else
              {
                digitalWrite(right_vib, LOW);
                digitalWrite(left_vib, HIGH);
              }

             delay(100);
        }

        else if ((distanceA < 150) || (distanceB < 150))
        {

            delay(200);
            //    digitalWrite(D7, LOW);
            //    delay(400);
            if (distanceA > distanceB)
              {
                digitalWrite(right_vib, HIGH);
                digitalWrite(left_vib, LOW);
              }
            else
              {
                digitalWrite(right_vib, LOW);
                digitalWrite(left_vib, HIGH);
              }

             delay(200);
        }

        else if ((distanceA < 200) || (distanceB < 200))
        {

            delay(300);
            //    digitalWrite(D7, LOW);
            //    delay(400);
            if (distanceA > distanceB)
              {
                digitalWrite(right_vib, HIGH);
                digitalWrite(left_vib, LOW);
              }
            else
              {
                digitalWrite(right_vib, LOW);
                digitalWrite(left_vib, HIGH);
              }

             delay(300);
        }
        /*
        End of Ultrasonic Module Code
        -------------------------------------------------
        */


    /*
    Start of GPS Code
    -----------------------------------------------------
    */

    // in case you are not using the interrupt above, you'll
    // need to 'hand query' the GPS, not suggested :(
    if (! usingInterrupt)
    {
        // read data from the GPS in the 'main loop'
        char c = GPS.read();
        // if you want to debug, this is a good time to do it!
        if (GPSECHO)
        if (c) Serial.print(c);
    }

  // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived())
    {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    }

    // if millis() or timer wraps around, we'll just reset it
    if (timer > millis())  timer = millis();

    // approximately every 2 seconds or so, print out the current stats
    if (millis() - timer > 2000)
    {
        timer = millis(); // reset the timer

        Serial.print("\nTime: ");
        Serial.print(GPS.hour, DEC); Serial.print(':');
        Serial.print(GPS.minute, DEC); Serial.print(':');
        Serial.print(GPS.seconds, DEC); Serial.print('.');
        Serial.println(GPS.milliseconds);
        Serial.print("Date: ");
        Serial.print(GPS.day, DEC); Serial.print('/');
        Serial.print(GPS.month, DEC); Serial.print("/20");
        Serial.println(GPS.year, DEC);
        Serial.print("Fix: "); Serial.print((int)GPS.fix);
        Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
        if (GPS.fix)
        {
            Serial.print("Location: ");
            Serial.print(GPS.latitudeDegrees, 4); Serial.print(GPS.lat);
            Serial.print(", ");
            Serial.print(GPS.longitudeDegrees, 4); Serial.println(GPS.lon);
            Serial.print("Speed (knots): "); Serial.println(GPS.speed);
            Serial.print("Angle: "); Serial.println(GPS.angle);
            Serial.print("Altitude: "); Serial.println(GPS.altitude);
            Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
        }
    }
     /* End of GPS code
    -------------------------------------------------------------
    */



    /*Start of LSM code
    ------------------------------------------------
    */

    /* Get a new sensor event */

    /*
    sensors_event_t event;
    mag.getEvent(&event);

    float Pi = 3.14159;

     // Calculate the angle of the vector y,x
    float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;

    // Normalize to 0-360
    if (heading < 0)
    {
        heading = 360 + heading;

    }
    Serial.print("Compass Heading: ");
    Serial.println(heading);
    */

    /*End of LSM code
    ---------------------------------------------------------------
    */





    /*Start of Thingspeak Update
    -------------------------------------------------------------
    */

     // if millis() or timer wraps around, we'll just reset it
    if (TStimer > millis())  TStimer = millis();

    // once very 60000 ms, or 5min, update the Thingspeak
    if (millis() - TStimer > 10000)
    {
        TStimer = millis();

        //Update Thingspeak server with self-written function
        GPS_update();
    }

    /*End of Thingspeak Update
    ------------------------------------------------------------
    */
     getBearing();


}
