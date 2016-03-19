/*  Written by Steve Lovejoy aka lovejoy777.
 *   
 *  Base station code for the Nodemcu amica r2 with
 *  rf24Lo1 rf module.
 *  
 *  connect 5v from power supply to vin amica and 0v to gnd amica.
 *  
 * rf24 pin config for nodemcu
 ******************************
 *  VCC = 3v3 amica           *
 *  GND = gnd amica           *
 *  CE = SD2 amica;           *
 *  CSN = SD3 amica;          *
 *  MOSI = D7 amica;          *
 *  MISO = D6 amica;          *    
 *  SCK = D5 amica;           *
 ******************************
 *  
 *  solder 10uf accross vcc & gnd at rf24L01
 *  
 *  connect DS18B20 temperature sensor as follows;
 *  pin 1 = gnd amica
 *  pin 2 = D1 amica
 *  pin 3 = vin amica (5v)
 *  
 *  I also have a test led to pin D2 amica for the return data.
 *  
 *  still need to move temperature sensor from base station to node
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <DallasTemperature.h>
#include <OneWire.h>

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#define LED1 4
// ard 5 = amica d1 for temp sensor
#define ONE_WIRE_BUS 5

const int pinCE = 9; // ard 5 = mcu D1, This pin is used to set the nRF24 to standby (0) or active mode (1)
const int pinCSN = 10;// ard 4 = mcu D2, This pin is used to tell the nRF24 whether the SPI communication is a command or message to send out
RF24 radio(pinCE, pinCSN); // Create your nRF24 object or wireless SPI connection

const byte txAddr[6] = "00001";
const byte rxAddr[6] = "00002";

const char* ssid = "wifivirus1";
const char* password = "domingo777";
unsigned int localPort = 8032;
boolean incoming = 0;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
static char outstr[15];

OneWire ourWire(ONE_WIRE_BUS);
DallasTemperature sensors(&ourWire);

WiFiUDP Udp;

void setup()
{
  Serial.begin(57600);
  
  pinMode(LED1, OUTPUT);

  radio.begin();
  radio.setRetries(15, 15);
  radio.openWritingPipe(txAddr);
  radio.openReadingPipe(0, rxAddr);

  delay(100);
  WiFi.begin(ssid, password);
  Udp.begin(localPort);

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  sensors.begin();
  
  Serial.println("Setup Done");
}

void loop()
{

radio.stopListening();
  delay(1000);

// if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {

    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);

    int i = atoi(&packetBuffer[0]);
    //delay(1000);
    if (i > 0 ) {
      Serial.println(i);
      

// Send On code to node
      if (i == 9997) { // ON CODE
        
  Serial.print("Sending ");
  const int relay1On = 9997;
  radio.write(&relay1On, sizeof(relay1On));
  Serial.println(relay1On);
  
  radio.startListening();
  delay(1000);
  int j = 0;
  radio.read(&j, sizeof(i));
  Serial.print("Received ");
  Serial.println(j);

  if (j == 9957) {
    digitalWrite(LED1,HIGH);
    radio.stopListening();
    delay(1000);

    // send ack back to android
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write("Heating is On");
        Udp.endPacket();
  }
      } // end on code


    // Send Off code to node
      if (i == 9998) { // OFF CODE
        
  Serial.print("Sending ");
  const int relay1Off = 9998;
  radio.write(&relay1Off, sizeof(relay1Off));
  Serial.println(relay1Off);

  radio.startListening();
  delay(1000);
  int j = 0;
  radio.read(&j, sizeof(j));
  Serial.print("Received ");
  Serial.println(j);

  if (j == 9958) {
    digitalWrite(LED1,LOW);
    radio.stopListening();
    delay(1000);

    // send ack back to android
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write("Heating is Off");
        Udp.endPacket();
  }
      } // end off code

          // Temperature code
      if (i == 9999) {
       Serial.println("Requesting temperature...");
       sensors.requestTemperatures(); // Send the command to get temperatures
       
        float temperatureC = (sensors.getTempCByIndex(0));
        dtostrf(temperatureC, 4, 2, outstr);
        Serial.print(outstr);
        Serial.println(" C");

        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(outstr);
        Udp.endPacket();
        
       } // End Temperature code

// if packet buffer is zero length
    }else {
      Serial.println("PB is empty");
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write("PB is empty");
      Udp.endPacket();
      }
    
  } // end if packet size.
} // end loop.
