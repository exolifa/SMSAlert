#include <Arduino.h>
#include "Adafruit_FONA.h"

// definition des pins internes pour le modem SIM800L

#define SIM800L_RX     27
#define SIM800L_TX     26
#define SIM800L_PWRKEY 4
#define SIM800L_RST    5
#define SIM800L_POWER  23

// definition du port serie vers le modem 
HardwareSerial *sim800lSerial = &Serial1;
// initialisation de la librairie FONA - client GSM
Adafruit_FONA sim800l = Adafruit_FONA(SIM800L_PWRKEY);
// definition du code pin 
#define SIM800L_PIN  1111
//zone pour lire le code imei de la carte SIM
char imei[16] = {0};

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

#define LED_BLUE  13
long prevMillis = 0;
int interval = 1000;
char sim800lNotificationBuffer[64];          //for notifications from the FONA
char smsBuffer[250];
boolean ledState = false;

String smsString = "";

void setup()
{
  pinMode(LED_BLUE, OUTPUT);
  pinMode(SIM800L_POWER, OUTPUT);

  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(SIM800L_POWER, HIGH);

  Serial.begin(115200);
  Serial.println(F("LyliGo TTGO T-Call "));
  Serial.println(F("en cours d'initialisation...."));
  
  // il faudra tuner les 10 secondes ...c'est long
  delay(10000);

  // Initialisation de la communication vers le modem
  sim800lSerial->begin(9600, SERIAL_8N1, SIM800L_TX, SIM800L_RX);
  if (!sim800l.begin(*sim800lSerial)) {
    Serial.println("GSM SIM800L introuvable");
    while (1);
  }
  uint8_t imeiLen = sim800l.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("SIM card IMEI: "); Serial.println(imei);
  }
  Serial.println("GSM SIM800L is OK");
  uint8_t unlock = sim800l.unlockSIM("1111");
  Serial.println(unlock);
  if (unlock = 1) {
    Serial.println("SIM card ouverte");     
  }
  else {
    Serial.println("Mauvais code PIN");
  }
  delay(3000);
  // Set up the FONA to send a +CMTI notification
  // when an SMS is received
  sim800lSerial->print("AT+CNMI=2,1\r\n");

  Serial.println("GSM SIM800L Ready");
   char destination[32] ="+32476417968";
  sim800l.sendSMS(destination, "le systeme est pret");
}


void loop()
{
  if (millis() - prevMillis > interval) {
    ledState = !ledState;
    digitalWrite(LED_BLUE, ledState);

    prevMillis = millis();
    Serial.println("on verifie"); 
    char* bufPtr = sim800lNotificationBuffer;    //handy buffer pointer
    Serial.println(sim800l.available());
    if (sim800l.available()) {
      int slot = 0; // this will be the slot number of the SMS
     int charCount = 0;

    // Read the notification into fonaInBuffer
    do {
      *bufPtr = sim800l.read();
      Serial.write(*bufPtr);
      delay(1);
    } while ((*bufPtr++ != '\n') && (sim800l.available()) && (++charCount < (sizeof(sim800lNotificationBuffer)-1)));
    
    //Add a terminal NULL to the notification string
    *bufPtr = 0;
    Serial.println(sim800lNotificationBuffer);
    //Scan the notification string for an SMS received notification.
    //  If it's an SMS message, we'll get the slot number in 'slot'
    if (1 == sscanf(sim800lNotificationBuffer, "+CMTI: \"SM\",%d", &slot)) {
      Serial.print("slot: "); Serial.println(slot);
      
      char callerIDbuffer[32];  //we'll store the SMS sender number in here
      
      // Retrieve SMS sender address/phone number.
      if (!sim800l.getSMSSender(slot, callerIDbuffer, 31)) {
        Serial.println("Didn't find SMS message in slot!");
      }
      Serial.print(F("FROM: ")); Serial.println(callerIDbuffer);

      // Retrieve SMS value.
      uint16_t smslen;
      // Pass in buffer and max len!
      if (sim800l.readSMS(slot, smsBuffer, 250, &smslen)) {
        smsString = String(smsBuffer);
        Serial.println(smsString);
      }

      if (smsString == "INFO") {
        Serial.println("SMS recu demande info");
        // Send SMS for info
        if (!sim800l.sendSMS(callerIDbuffer, "Merci de votre demande.le systeme est operationnel")) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Sent!"));
        }
      }
      while (1) {
        if (sim800l.deleteSMS(slot)) {
          Serial.println(F("OK!"));
          break;
        }
        else {
          Serial.print(F("Couldn't delete SMS in slot ")); Serial.println(slot);
          sim800l.print(F("AT+CMGD=?\r\n"));
        }
      }
    }
  }
}
}
