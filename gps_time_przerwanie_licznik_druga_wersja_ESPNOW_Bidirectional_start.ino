#include <espnow.h>
#include <ESP8266WiFi.h>
#include <TimeLib.h>    // http://www.pjrc.com/teensy/td_libs_Time.html
#include <TinyGPS++.h>  // http://arduiniana.org/libraries/tinygpsplus/
#include <SPI.h>
#include <Adafruit_GFX.h>      // http://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_SSD1306.h>  // http://github.com/adafruit/Adafruit_SSD1306
#include <SoftwareSerial.h>

//#define Debug
// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress[] = { 0xE8, 0x9F, 0x6D, 0x93, 0x81, 0x5A };

bool readyStart = 0;
bool readyStart_bufor = 0;
bool readyMeta = 0;

// Define variables to store incoming readings
//bool wyscig = 0;
bool wyscig_lewa = 0;
bool wyscig_prawa = 0;

#define PPSPin 15    // D8
#define lewaPin 12   //D6
#define prawaPin 14  //D5

#define filter 2

#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

word licznik;
bool delivery = 0;
unsigned long lastButtonchange = 0, syncMillis = 0, pushTime = 0, lewa_czas = 0, lewa_czas_wyswietlacz = 0, prawa_czas = 0;
word lewa_licznik = 0, prawa_licznik = 0, lewa_licznik_bufor = 0, prawa_licznik_bufor = 0, prawa_licznik_filter = filter, lewa_licznik_filter = filter;
unsigned char LCDrefreshSecond = 0;
static const int RXPin = 13, TXPin = 13;
static const uint32_t GPSBaud = 9600;
unsigned long cutnumber();
//String timeString();
// String millisToTimeString();
const long interval = 5000;
long previousMillis;

//do obslugi seriala
String inputString = "";      // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

TinyGPSPlus gps;
tmElements_t tm;
SoftwareSerial ss(RXPin, TXPin);

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  bool status;
  bool wyscig;
  bool ready;
  long lewa_czas;
  long prawa_czas;
} struct_message;

// Create a struct_message called myData
struct_message myData;
// Create a struct_message to hold incoming readings
struct_message incomingMeta;

// ---Callback when data is sent---
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  //Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0) {
    //Serial.println("Delivery success");
    delivery = 1;
    myData.status = 0;
  } else {
#ifdef Debug
    Serial.println("Delivery fail");
#endif
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    // delivery = 0;
  }
}
//---------------------------------------------

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&incomingMeta, incomingData, sizeof(incomingMeta));
  readyMeta = incomingMeta.ready;
  //Serial.println("Bytes received: ");
  //Serial.println(len);
  // Serial.println(incomingMeta.lewa_czas);
}

//-----------------SETUP------------
void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
  pinMode(PPSPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPSPin), PPS, RISING);


  //------------- Init ESP-NOW--------------------------------
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Set ESP-NOW Role
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  // // Once ESPNow is successfully Init, we will register for Send CB to
  // // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  // // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  //---------------------------------------------

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.display();
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.println("start");
  display.display();
  delay(2000);

  pinMode(lewaPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(lewaPin), lewa, RISING);
  pinMode(prawaPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(prawaPin), prawa, FALLING);
}

//sprawdzamy bufor seriala
void readSerial() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}


//####################   glowna petla ##############
void loop() {

  //obsluga GPS uart
  while (ss.available() > 0) {
    gps.encode(ss.read());
    if (tm.Hour != 0) {
      readyStart = 1;
    }
  }

  //sprawdzamy czy wiecej niz sekunda od PPS
  if (millis() - syncMillis >= 1100) {
    readyStart = 0;
  }

  // if (millis() - previousMillis >= interval && wyscig == 1) {
  //   previousMillis = millis();
  //   //wyscig = 0;
  //   Serial.print("lewa_czas:  ");
  //   Serial.println(lewa_czas);
  //   Serial.print("prawa_czas: ");
  //   Serial.println(prawa_czas);
  //   Serial.print(tm.Hour);
  //   Serial.print(tm.Minute);
  //   Serial.println(tm.Second);
  // }


  if (lewa_licznik != lewa_licznik_bufor || prawa_licznik != prawa_licznik_bufor || readyStart != readyStart_bufor) {
    lewa_licznik_bufor = lewa_licznik;
    prawa_licznik_bufor = prawa_licznik;
    readyStart_bufor = readyStart;
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println(lewa_licznik_bufor);
    display.print("GPS ");
    if (readyStart) {
      display.println("OK");
    } else {
      display.println("X");
    }
    display.println(prawa_licznik_bufor);
    display.display();
  }


  /*
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 5000) {
    // save the last time you updated the DHT values
    previousMillis = currentMillis;
#ifdef Debug
    Serial.print("Start ready:");
    Serial.println(readyStart);
    Serial.print("Meta ready: ");
    Serial.println(readyMeta);
#endif
    //Set values to send
    myData.wyscig = 1;

    // Send message via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

    // Print incoming readings
    // printincomingMeta();
  }
*/


  /*
  if (LCDrefreshSecond != cutnumber(millis() - syncMillis, 4, 3)) {
    breakTime(now(), tm);

    display.println(millis() - syncMillis);
    display.println((tm.Hour * 360 + tm.Minute * 60 + tm.Second) * 1000);
    display.println(lewa_czas);

    display.display();

    LCDrefreshSecond = cutnumber(millis() - syncMillis, 4, 3);
  }
  */


  //sterowanie po serialu
  readSerial();  //musimy odczytać co mamy w serialu
  if (stringComplete) {
    if (inputString == "W\n") {
      Serial.println("warunek");
      //wyscig = 1;
      wyscig_lewa = 1;
      wyscig_prawa = 1;
      myData.wyscig = 1;             //do wyslania
      lewa_licznik_filter = filter;  //zerujemy filtry
      prawa_licznik_filter = filter;
      // Send message via ESP-NOW
      esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
      myData.wyscig = 0;  //
    }

    if (inputString == "S\n") {
      myData.status = 1;
      // Send message via ESP-NOW
      esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    }

    if (inputString == "R\n") {
      Serial.print("start: ");
      Serial.println(lewa_czas);
      Serial.print("meta:  ");
      Serial.println(incomingMeta.lewa_czas);
      Serial.print("roznica: ");
      Serial.println(incomingMeta.lewa_czas - lewa_czas);
      Serial.print("GPS_meta: ");
      Serial.println(incomingMeta.status);
      Serial.print("GPS_start: ");
      Serial.println(readyStart);
    }

    inputString = "";
    stringComplete = false;
  }
}

//########  podprogramy

ICACHE_RAM_ATTR void PPS() {  //przerwanie od PPS
  syncMillis = millis();

  licznik++;
  if (gps.time.isUpdated() && gps.date.year() > 2015 && !timeStatus()) {

    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
    //adjustTime(UTCcorrection + 1);
    //detachInterrupt(digitalPinToInterrupt(PPSPin));
    //syncUNIXtime = now();
  } else
    breakTime(now(), tm);
}

ICACHE_RAM_ATTR void lewa() {
  if (wyscig_lewa == 1) {
    lewa_licznik_filter--;
    if (lewa_licznik_filter == 0) {
      lewa_licznik_filter = filter;
      wyscig_lewa = 0;
      if (millis() % 1000 >= syncMillis % 1000) {
        lewa_czas = (tm.Hour * 3600 + tm.Minute * 60 + tm.Second) * 1000 + millis() % 1000 - syncMillis % 1000;
      } else {
        lewa_czas = (tm.Hour * 3600 + tm.Minute * 60 + tm.Second) * 1000 + 1000 + millis() % 1000 - syncMillis % 1000;
      }
    }
  }
  lewa_licznik++;
}

ICACHE_RAM_ATTR void prawa() {
  if (wyscig_prawa == 1) {
    prawa_licznik_filter--;
    if (prawa_licznik_filter == 0) {
      prawa_licznik_filter = filter;
      wyscig_prawa = 0;
      if (millis() % 1000 >= syncMillis % 1000) {
        prawa_czas = (tm.Hour * 3600 + tm.Minute * 60 + tm.Second) * 1000 + millis() % 1000 - syncMillis % 1000;
      } else {
        prawa_czas = (tm.Hour * 3600 + tm.Minute * 60 + tm.Second) * 1000 + 1000 + millis() % 1000 - syncMillis % 1000;
      }
    }
  }
  prawa_licznik++;
}


unsigned long cutnumber(unsigned long number, unsigned int cutbeg, unsigned int cutend) {
  number = number - pow(10, cutbeg) * (unsigned long)(number / pow(10, cutbeg));
  if (cutend != 0) {
    number = (unsigned long)(number / pow(10, cutend));
  }
  return number;
}