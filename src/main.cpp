
/* SMS-Alert module
*interaction avec le système par SMS : 
*    - envoi d'SMS d'alerte qui sont générés par les différents composants sur le topic MQTT alert/<composant>/
*    - reception d'SMS de query ou de commande 
*
*
*/
#include <Arduino.h>
#include "Adafruit_FONA.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "SPIFFS.h"
#include "ESP8266FtpServer.h"


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
struct Destinataire{
  char mnemo[15];
  char numero[32];
};
struct Alerttopic{
  char topic[35];
};
struct Config {
  char serverName[15] ;
  int alerttopicscount ;
  int destinatairescount;
  Alerttopic alerttopic[10] ;
  char wifissid [20];
  char wifiuid[10];
  char wifipw[25] ;
  char mqttserver[15];
  char mqttuser[15];
  char mqttpw[15];
  char mqttcmd[35];
  char ftpuser [15];
  char ftppw [15];  
  char pincode[5];
  Destinataire destination [10];
};
Config config; 

// definition du code pin 
char SIM800L_PIN[5]  ="1111";
char destination[32] ="+32483207089";
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
//client WIFI,NTP & mqtt definition
WiFiClient espClient;
PubSubClient client(espClient);
FtpServer ftpSrv;

String findNumero (String mnemo ){
  for (int i = 0; i < config.destinatairescount; i++){
    if (mnemo = config.destination[i].mnemo) {
      return config.destination[i].numero;
    }
  }
  return mnemo;
}
void setup_wifi() {
 // WiFi.hostname( "smsalert");
  delay(100);
  // We start by connecting to a WiFi network
 // String ssid = wifissid;
 // String pw = wifipw;
  Serial.println();
  Serial.print("Connecting to ");
  Serial.print(config.wifissid);
  Serial.print(" with pw=");
  Serial.println(config.wifipw);
  WiFi.begin(config.wifissid,config.wifipw);
  delay(100);
  WiFi.printDiag(Serial);
  while (WiFi.status() != WL_CONNECTED) {
   delay(10000);  
   Serial.print(WiFi.status());
  }
 // Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.print(WiFi.localIP());
  Serial.print(" Gateway: ");
  Serial.print(WiFi.gatewayIP());
  Serial.print(" netmask: ");
  Serial.println(WiFi.subnetMask());
}
/*
    Connecting to MQTT server
*/
void reconnect() {
  // Loop until we're reconnected to mqtt server
  while (!client.connected()) {
    if (client.connect(config.serverName, config.mqttuser, config.mqttpw)) {
      // subscribing to mqtt commands topics if needed
      Serial.println("mqtt subscribing to :");
      Serial.println(config.mqttcmd);
      client.subscribe(config.mqttcmd);
      client.subscribe(config.alerttopic);
      Serial.println("mqqt connected");
    } else {
      Serial.print("failed mqtt with ");
      Serial.println(client.state());
      delay(5000);
    }
  }
}
/*
   Loading the parameters from the spiffs config.json file
*/
bool loadConfig() {

  DynamicJsonDocument cfg (3500);
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("Failed to open config file");
    return false;
  }

  size_t size = configFile.size();
  Serial.print("config size=");
  Serial.println(size);
  if (size > 3500) {
    Serial.println("Config file size is too large");
    return false;
  }

  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);
  configFile.readBytes(buf.get(), size);
  auto error = deserializeJson(cfg, buf.get());
  if (error) {
    Serial.print("Failed to parse config file");
    Serial.println(error.c_str());
    return false;
  }
  strlcpy(config.serverName,                  // <- destination
          cfg["devname"] | "DEV001",  // <- source
          sizeof(config.serverName));
  strlcpy(config.wifissid,                  // <- destination
          cfg["wifi"]["ssid"] | "WifiLan",  // <- source
          sizeof(config.wifissid));
  strlcpy(config.wifiuid,                  // <- destination
          cfg["wifi"]["uid"] | "uid",  // <- source
          sizeof(config.wifiuid));
  strlcpy(config.wifipw,                  // <- destination
          cfg["wifi"]["pw"] | "paswd",  // <- source
          sizeof(config.wifipw));
  strlcpy(config.mqttserver,                  // <- destination
          cfg["mqtt"]["server"] | "10.0.0.55",  // <- source
          sizeof(config.mqttserver));
  strlcpy(config.mqttuser,                  // <- destination
          cfg["mqtt"]["user"] | "DEV001",  // <- source
          sizeof(config.mqttuser));
  strlcpy(config.mqttpw,                  // <- destination
          cfg["mqtt"]["pw"] | "pass",  // <- source
          sizeof(config.mqttpw));
  strlcpy(config.mqttcmd,                  // <- destination
          cfg["mqtt"]["cmd"] | "pass",  // <- source
          sizeof(config.mqttcmd));
    strlcpy(config.ftpuser,                  // <- destination
          cfg["ftp"]["user"] | "DEV001",  // <- source
          sizeof(config.ftpuser)); 
    strlcpy(config.ftppw,                  // <- destination
          cfg["ftp"]["pw"] | "pass",  // <- source
          sizeof(config.ftppw));            
  strlcpy(config.pincode,                  // <- destination
          cfg["gsm"]["pincode"] | "10.0.0.55",  // <- source
          sizeof(config.pincode));
  config.alerttopicscount = cfg["alertcount"].as<int>();
  for ( int i = 0; i < config.alerttopicscount; i++)
  {  
   strlcpy(config.alerttopic[i].topic,                  // <- destination
          cfg["alerttopics"][i]["topic"] | "alert",  // <- source
          sizeof(config.alerttopic[i].topic));
  }
   config.destinatairescount = cfg["destcount"].as<int>();     
   for ( int i = 0; i < config.destinatairescount; i++)
  { 
  strlcpy(config.destination[i].mnemo,                  // <- destination
          cfg["gsm"]["destinataires"][i]["mnemo"] | "10.0.0.55",  // <- source
          sizeof(config.destination[i].mnemo));
  strlcpy(config.destination[i].numero,                  // <- destination
          cfg["gsm"]["destinataires"][i]["numero"] | "10.0.0.55",  // <- source
          sizeof(config.destination[i].numero));
  }
/*
               Device specific data load to be tuned together with the config declarations
*/
  configFile.close();
  return true;
}
/*
   Processing MQTT commands if needed
*/
void callback(char* topic, byte* payload, unsigned int length) {
  char cpayload[10];
  char ctopic[35];
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  payload[length] = '\0';
  strlcpy(cpayload, (char *)payload, sizeof(cpayload));
  strlcpy(ctopic, topic, sizeof(ctopic));
  Serial.print("payload=");
  Serial.println(cpayload);
  if (strcmp(config.mqttcmd, ctopic) == 0) {
      Serial.print("system request received");
      if (strcmp(cpayload, "REBOOT") == 0) {
          Serial.println("reboot requested");
          ESP.restart();
      }
  }
}

void setup()
{
    if (!SPIFFS.begin()) {
    Serial.println ("failed to mount FS");
    return;
  }
  if (!loadConfig()) {
    Serial.println("Failed to load config");
  } else {
    Serial.println("Config loaded");
  }

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

  sim800l.sendSMS(config.destination[0].numero, "le systeme SMS vient de démarrer");
  setup_wifi();
  Serial.println(config.mqttserver);
  client.setServer(config.mqttserver, 1883);
  client.setCallback(callback);
     //setup FTP service 
  ftpSrv.begin("esp32","esp32");    

}


void loop()
{
  // reconnect to mqtt if needed
  if (!client.connected()) {
    reconnect();
  }
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
ftpSrv.handleFTP();
client.loop();
}
