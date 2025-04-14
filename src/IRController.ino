#include <FS.h>                                               // This needs to be first, or it all crashes and burns

#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <WiFi.h>
#include <WiFiManager.h>                                      // https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ESPmDNS.h>                                      // Useful to access to ESP by hostname.local

#include <ArduinoJson.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoOTA.h>
#include "mbedtls/sha256.h"
#include "mbedtls/md.h" // Für HMAC

#include <Ticker.h>                                           // For LED status
#include <TimeLib.h>

#include <LittleFS.h>

// User settings are below here
//+=============================================================================
const bool getExternalIP = true;                               // Set to false to disable querying external IP

const bool getTime = true;                                     // Set to false to disable querying for the time
const int timeZone = -5;                                       // Timezone (-5 is EST)

const bool enableMDNSServices = true;                          // Use mDNS services, must be enabled for ArduinoOTA

const bool bypassLocalAuth = true;                             // Allow local traffic to bypass HMAC check

const unsigned int captureBufSize = 1024;                      // Size of the IR capture buffer.

const bool toggleRC = true;                                    // Toggle RC signals every other transmission

#if defined(ARDUINO_ESP8266_WEMOS_D1R1) || defined(ARDUINO_ESP8266_WEMOS_D1MINI) || defined(ARDUINO_ESP8266_WEMOS_D1MINIPRO) || defined(ARDUINO_ESP8266_WEMOS_D1MINILITE)
const uint16_t  pinr1 = D5;                                          // Receiving pin (GPIO14)
const uint16_t  pins1 = D6;                                          // Transmitting preset 1 (GPIO12)
const uint16_t  configpin = D2;                                      // Reset Pin (GPIO4)
const uint16_t  pins2 = 5;                                           // Transmitting preset 2
const uint16_t  pins3 = 12;                                          // Transmitting preset 3
const uint16_t  pins4 = 13;                                          // Transmitting preset 4
#else
const uint16_t  pinr1 = 15;                                          // Receiving pin
const uint16_t  pins1 = 13;                                           // Transmitting preset 1
const uint16_t  configpin = 10;                                      // Reset Pin
const uint16_t  pins2 = 5;                                           // Transmitting preset 2
const uint16_t  pins3 = 12;                                          // Transmitting preset 3
const uint16_t  pins4 = 4;                                          // Transmitting preset 4
#endif
//+=============================================================================
// User settings are above here

const int ledpin = LED_BUILTIN;                                // Built in LED defined for WEMOS people
const char *wifi_config_name = "IR Controller Configuration";
const char serverName[] = "checkip.dyndns.org";
int port = 80;
char passcode[20] = "";
char host_name[20] = "";
char port_str[6] = "80";
char user_id[60] = "";

// Do not modify these values with your own, they are placeholder values that WiFiManager will overwrite
char static_ip[16] = "10.0.1.10";
char static_gw[16] = "10.0.1.1";
char static_sn[16] = "255.255.255.0";
char static_dns[16] = "10.0.1.1";

DynamicJsonDocument deviceState(1024);

WiFiClient client;
WebServer *server = NULL;
Ticker ticker;

bool shouldSaveConfig = false;                                 // Flag for saving data
bool holdReceive = false;                                      // Flag to prevent IR receiving while transmitting

IRrecv irrecv(pinr1, captureBufSize, 35);
IRsend irsend1(pins1);
IRsend irsend2(pins2);
IRsend irsend3(pins3);
IRsend irsend4(pins4);

const unsigned long resetfrequency = 259200000;                // 72 hours in milliseconds for external IP reset
static const char ntpServerName[] = "time.google.com";
unsigned int localPort = 8888;                                 // Local port to listen for UDP packets
void sendNTPpacket(IPAddress &address);
time_t getNtpTime();
WiFiUDP ntpUDP;

bool _rc5toggle = false;
bool _rc6toggle = false;

char _ip[16] = "";

unsigned long lastupdate = 0;

bool authError = false;
time_t timeAuthError = 0;
bool externalIPError = false;
bool userIDError = false;
bool ntpError = false;

class Code {
  public:
    char encoding[14] = "";
    char address[20] = "";
    char command[40] = "";
    char data[40] = "";
    String raw = "";
    int bits = 0;
    time_t timestamp = 0;
    bool valid = false;
    // +++ NEUE MEMBER +++
    int repeat = 1; // Default repeat count
    int out = 1;    // Default output pin
};

// Declare prototypes
void sendCodePage(Code selCode);
void sendCodePage(Code selCode, int httpcode);
void cvrtCode(Code& codeData, decode_results *results);
void copyCode (Code& c1, Code& c2);

Code last_recv;
Code last_recv_2;
Code last_recv_3;
Code last_recv_4;
Code last_recv_5;
Code last_send;
Code last_send_2;
Code last_send_3;
Code last_send_4;
Code last_send_5;

//+=============================================================================
// Button Configuration
//+=============================================================================
const int MAX_BUTTONS = 9; // Maximale Anzahl an Buttons

struct ButtonConfig {
  char name[32] = "";      // Name des Buttons
  char type[14] = "";      // IR Protokoll (nec, sony, etc.)
  char data[40] = "";      // IR Daten (Hex String)
  int length = 0;          // Anzahl Bits
  char address[20] = "";   // Adresse (Hex String, optional)
  int repeat = 1;          // Wiederholungen
  int out = 1;             // Output Pin (1-4)
  // --- NEUE Felder für MAKRO ---
  bool isMacro = false;    // Ist dieser Button ein Makro?
  char macroJson[512] = ""; // JSON-String für das Makro (Größe ggf. anpassen)
  // --- Status ---
  bool configured = false; // Ist dieser Button-Slot konfiguriert?
};

ButtonConfig buttonConfigs[MAX_BUTTONS]; // Array für Button-Konfigurationen
//+=============================================================================


//+=============================================================================
// Callback notifying us of the need to save config
//
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}


//+=============================================================================
// Load Button Configuration from LittleFS
//
void loadButtonConfig() {
  if (!LittleFS.begin()) {
    Serial.println("Failed to mount LittleFS for button config loading.");
    return;
  }

  if (LittleFS.exists("/buttons.json")) {
    Serial.println("Reading button config file");
    File configFile = LittleFS.open("/buttons.json", "r");
    if (configFile) {
      DynamicJsonDocument jsonDoc(2048); // Größe ggf. anpassen (9 Buttons * ~150 Zeichen)
      DeserializationError error = deserializeJson(jsonDoc, configFile);
      configFile.close(); // Datei schließen, sobald gelesen

      if (!error) {
        JsonArray buttonArray = jsonDoc.as<JsonArray>();
        int count = 0;
        for (JsonObject buttonJson : buttonArray) {
          if (count >= MAX_BUTTONS) break; // Nicht mehr laden als Plätze vorhanden

          strncpy(buttonConfigs[count].name, buttonJson["name"] | "", sizeof(buttonConfigs[count].name) - 1);
          strncpy(buttonConfigs[count].type, buttonJson["type"] | "", sizeof(buttonConfigs[count].type) - 1);
          strncpy(buttonConfigs[count].data, buttonJson["data"] | "", sizeof(buttonConfigs[count].data) - 1);
          buttonConfigs[count].length = buttonJson["length"] | 0;
          strncpy(buttonConfigs[count].address, buttonJson["address"] | "", sizeof(buttonConfigs[count].address) - 1);
          buttonConfigs[count].repeat = buttonJson["repeat"] | 1;
          buttonConfigs[count].out = buttonJson["out"] | 1;
          buttonConfigs[count].configured = buttonJson["configured"] | false;

          // Sicherstellen, dass Strings null-terminiert sind
          buttonConfigs[count].name[sizeof(buttonConfigs[count].name) - 1] = '\0';
          buttonConfigs[count].type[sizeof(buttonConfigs[count].type) - 1] = '\0';
          buttonConfigs[count].data[sizeof(buttonConfigs[count].data) - 1] = '\0';
          buttonConfigs[count].address[sizeof(buttonConfigs[count].address) - 1] = '\0';

          // NEU: Lade Makro-Informationen
          buttonConfigs[count].isMacro = buttonJson["isMacro"] | false; // Default ist false
          strncpy(buttonConfigs[count].macroJson, buttonJson["macroJson"] | "", sizeof(buttonConfigs[count].macroJson) - 1);
          buttonConfigs[count].macroJson[sizeof(buttonConfigs[count].macroJson) - 1] = '\0'; // Sicherstellen, dass null-terminiert

          buttonConfigs[count].configured = buttonJson["configured"] | false;

          // Erweiterte Validierung:
          if (strlen(buttonConfigs[count].name) == 0 ||
          (!buttonConfigs[count].isMacro && (strlen(buttonConfigs[count].data) == 0 || buttonConfigs[count].length == 0)) ||
          (buttonConfigs[count].isMacro && strlen(buttonConfigs[count].macroJson) == 0) )
          {
          buttonConfigs[count].configured = false;
          }

          count++;
        }
        Serial.println("Button config loaded successfully.");
      } else {
        Serial.print("Failed to parse buttons.json: ");
        Serial.println(error.c_str());
        // Bei Fehler: Alle Buttons als nicht konfiguriert markieren
        for(int i=0; i<MAX_BUTTONS; ++i) buttonConfigs[i].configured = false;
      }
    } else {
      Serial.println("Failed to open buttons.json for reading.");
    }
  } else {
    Serial.println("buttons.json not found. Initializing with defaults.");
    // Datei existiert nicht, alle als nicht konfiguriert belassen
    for(int i=0; i<MAX_BUTTONS; ++i) buttonConfigs[i].configured = false;
  }
  // LittleFS.end(); // Nicht hier beenden, wird evtl. noch gebraucht
}

//+=============================================================================
// Save Button Configuration to LittleFS
//
void saveButtonConfig() {
  if (!LittleFS.begin()) {
    Serial.println("Failed to mount LittleFS for button config saving.");
    return;
  }

  DynamicJsonDocument jsonDoc(2048); // Größe ggf. anpassen
  JsonArray buttonArray = jsonDoc.to<JsonArray>();

  for (int i = 0; i < MAX_BUTTONS; ++i) {
    JsonObject buttonJson = buttonArray.createNestedObject();
    // Nur speichern, wenn konfiguriert (oder zumindest Name gesetzt ist)
    if (buttonConfigs[i].configured && strlen(buttonConfigs[i].name) > 0) {
      buttonJson["name"] = buttonConfigs[i].name;
      buttonJson["configured"] = true;
      buttonJson["isMacro"] = buttonConfigs[i].isMacro; // NEU

      if (buttonConfigs[i].isMacro) {
          // Speichere Makro-JSON
          buttonJson["macroJson"] = buttonConfigs[i].macroJson;
          // Optional: Setze die anderen Felder auf Defaults oder lasse sie weg
          buttonJson["type"] = "";
          buttonJson["data"] = "";
          buttonJson["length"] = 0;
          buttonJson["address"] = "";
          buttonJson["repeat"] = 1;
          buttonJson["out"] = 1;
      } else {
          // Speichere Einzel-Signal-Daten (wie bisher)
          buttonJson["type"] = buttonConfigs[i].type;
          buttonJson["data"] = buttonConfigs[i].data;
          buttonJson["length"] = buttonConfigs[i].length;
          buttonJson["address"] = buttonConfigs[i].address;
          buttonJson["repeat"] = buttonConfigs[i].repeat;
          buttonJson["out"] = buttonConfigs[i].out;
          // Optional: Setze macroJson auf leer oder lasse es weg
          buttonJson["macroJson"] = "";
      }
    } else {
        // Leeren Eintrag speichern, um die Position zu markieren
        buttonJson["configured"] = false;
        // Optional: Alle Felder auf Default/leer setzen
        buttonJson["name"] = "";
        buttonJson["isMacro"] = false;
        buttonJson["macroJson"] = "";
        buttonJson["type"] = "";
        buttonJson["data"] = "";
    }
  }

  File configFile = LittleFS.open("/buttons.json", "w");
  if (!configFile) {
    Serial.println("Failed to open buttons.json for writing.");
    return;
  }

  if (serializeJson(jsonDoc, configFile) == 0) {
    Serial.println("Failed to write to buttons.json.");
  } else {
    Serial.println("Button config saved successfully.");
  }
  configFile.close();
  // LittleFS.end(); // Nicht hier beenden
}


//+=============================================================================
// Reenable IR receiving
//
void resetReceive() {
  if (holdReceive) {
    Serial.println("Reenabling receiving");
    irrecv.resume();
    holdReceive = false;
  }
}


//+=============================================================================
// Valid user_id formatting
//
bool validUID(char* user_id) {
  if (!String(user_id).startsWith("amzn1.account.")) {
      Serial.println("Warning, user_id appears to be in the wrong format, security check will most likely fail. Should start with amzn1.account.***");
      return false;
    }
    return true;
}


//+=============================================================================
// Valid EPOCH time retrieval
//
bool validEPOCH(time_t timenow) {
  if (timenow < 922838400) {
    Serial.println("Epoch time from timeServer is unexpectedly old, probably failed connection to the time server. Check your network settings");
    Serial.println(timenow);
    return false;
  }
  return true;
}


//+=============================================================================
// EPOCH time to String
//
String epochToString(time_t timenow) {
  unsigned long hours = (timenow % 86400L) / 3600;
  String hourStr = hours < 10 ? "0" + String(hours) : String(hours);

  unsigned long minutes = (timenow % 3600) / 60;
  String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);

  unsigned long seconds = (timenow % 60);
  String secondStr = seconds < 10 ? "0" + String(seconds) : String(seconds);
  return hourStr + ":" + minuteStr + ":" + secondStr;
}

//+=============================================================================
// Valid command request using HMAC
//
bool validateHMAC(String epid, String mid, String timestamp, String signature, IPAddress clientIP) {
    userIDError = false;
    authError = false;
    ntpError = false;
    timeAuthError = 0;

    if (allowLocalBypass(clientIP)) {
      Serial.println("Bypassing HMAC security as this is a local network request");
      return true;
    }

    userIDError = !(validUID(user_id));

    time_t timethen = timestamp.toInt();
    time_t timenow = now() - (timeZone * SECS_PER_HOUR);
    time_t timediff = abs(timethen - timenow);
    if (timediff > 30) {
      Serial.println("Failed security check, signature is too old");
      Serial.print("Server: ");
      Serial.println(timethen);
      Serial.print("Local: ");
      Serial.println(timenow);
      Serial.print("MID: ");
      Serial.println(mid);
      timeAuthError = timediff;
      validEPOCH(timenow);
      return false;
    }

    uint8_t hash_output[32]; // SHA256 produces a 32-byte hash
    String key = String(user_id);
    String computedSignature = ""; // Initialize to empty

    mbedtls_md_context_t ctx;
    const mbedtls_md_info_t *md_info;

    mbedtls_md_init(&ctx);
    md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256); // Specify SHA256

    if (md_info == NULL) {
        Serial.println("ERROR: mbedtls_md_info_from_type failed");
        mbedtls_md_free(&ctx);
        return false; // Indicate failure
    }

    if (mbedtls_md_setup(&ctx, md_info, 1) != 0) { // 1 indicates HMAC mode
         Serial.println("ERROR: mbedtls_md_setup failed");
         mbedtls_md_free(&ctx);
         return false;
    }

    if (mbedtls_md_hmac_starts(&ctx, (const unsigned char *)key.c_str(), key.length()) != 0) {
         Serial.println("ERROR: mbedtls_md_hmac_starts failed");
         mbedtls_md_free(&ctx);
         return false;
    }

    // Update with the message parts in order
    if (mbedtls_md_hmac_update(&ctx, (const unsigned char *)epid.c_str(), epid.length()) != 0) {
         Serial.println("ERROR: mbedtls_md_hmac_update (epid) failed");
         mbedtls_md_free(&ctx);
         return false;
    }
     if (mbedtls_md_hmac_update(&ctx, (const unsigned char *)mid.c_str(), mid.length()) != 0) {
         Serial.println("ERROR: mbedtls_md_hmac_update (mid) failed");
         mbedtls_md_free(&ctx);
         return false;
    }
     if (mbedtls_md_hmac_update(&ctx, (const unsigned char *)timestamp.c_str(), timestamp.length()) != 0) {
         Serial.println("ERROR: mbedtls_md_hmac_update (timestamp) failed");
         mbedtls_md_free(&ctx);
         return false;
    }

    // Finalize the HMAC calculation and get the result
    if (mbedtls_md_hmac_finish(&ctx, hash_output) != 0) {
         Serial.println("ERROR: mbedtls_md_hmac_finish failed");
         mbedtls_md_free(&ctx);
         return false;
    }

    // Clean up the mbedtls context
    mbedtls_md_free(&ctx);

    // Convert the binary hash result to a hexadecimal string
    computedSignature = bin2hex(hash_output, 32); // Use your existing bin2hex function


    if (computedSignature != signature) {
      Serial.println("Failed security check, signatures do not match");
      Serial.print("1: ");
      Serial.println(signature);
      Serial.print("2: ");
      Serial.println(computedSignature);
      Serial.print("MID: ");
      Serial.println(mid);
      authError = true;
      return false;
    }

    Serial.println("Passed security check");
    Serial.print("MID: ");
    Serial.println(mid);
    return true;
}

//+=============================================================================
// Check if client request is coming from a local IP address
//
bool isInSubnet(IPAddress address) {
    Serial.print("Client IP: ");
    Serial.println(address.toString());

    uint32_t mask = uint32_t(WiFi.subnetMask());

    /*
    uint32_t net_lower = (uint32_t(WiFi.localIP()) & mask);
    uint32_t net_upper = (net_lower | (~mask));
    Serial.println(IPAddress(net_lower).toString());
    Serial.println(IPAddress(net_upper).toString());
    */

    return ((uint32_t(address) & mask) == (uint32_t(WiFi.localIP()) & mask));
}

//+=============================================================================
// Allow local traffic to bypass security
//
bool allowLocalBypass(IPAddress clientIP) {
  return (bypassLocalAuth && isInSubnet(clientIP));
}

//+=============================================================================
// Passcode valid check
//
bool isPasscodeValid(String pass) {
  return ((strlen(passcode) == 0) || (pass == passcode));
}

//+=============================================================================
// Get User_ID from Amazon Token (memory intensive and causes crashing)
//
String getUserID(String token)
{
  HTTPClient http;
  http.setTimeout(5000);
  String url = "https://api.amazon.com/user/profile?access_token=";
  String uid = "";
  http.begin(url + token);
  int httpCode = http.GET();
  String payload = http.getString();
  Serial.println(url + token);
  Serial.println(httpCode);
  Serial.println(payload);
  if (httpCode > 0 && httpCode == HTTP_CODE_OK) {
    DynamicJsonDocument json(1024);
    deserializeJson(json, payload);
    uid = json["user_id"].as<String>();
  } else {
    Serial.println("Error retrieving user_id");
    payload = "";
  }
  http.end();
  return uid;
}


//+=============================================================================
// Toggle state
//
void tick()
{
  int state = digitalRead(ledpin);  // get the current state of GPIO1 pin
  digitalWrite(ledpin, !state);     // set pin to the opposite state
}


//+=============================================================================
// Get External IP Address
//
String externalIP()
{
  if (!getExternalIP) {
    return "0.0.0.0"; // User doesn't want the external IP
  }

  if (strlen(_ip) > 0) {
    unsigned long delta = millis() - lastupdate;
    if (delta > resetfrequency || lastupdate == 0) {
      Serial.println("Reseting cached external IP address");
      strncpy(_ip, "", 16); // Reset the cached external IP every 72 hours
    } else {
      return String(_ip); // Return the cached external IP
    }
  }

  HTTPClient http;
  externalIPError = false;
  unsigned long start = millis();
  http.setTimeout(5000);
  http.begin(serverName, 8245);
  int httpCode = http.GET();

  if (httpCode > 0 && httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    int pos_start = payload.indexOf("IP Address") + 12; // add 10 for "IP Address" and 2 for ":" + "space"
    int pos_end = payload.indexOf("</body>", pos_start); // add nothing
    strncpy(_ip, payload.substring(pos_start, pos_end).c_str(), 16);
    Serial.print(F("External IP: "));
    Serial.println(_ip);
    lastupdate = millis();
  } else {
    Serial.println("Error retrieving external IP");
    Serial.print("HTTP Code: ");
    Serial.println(httpCode);
    Serial.println(http.errorToString(httpCode));
    externalIPError = true;
  }

  http.end();
  Serial.print("External IP address request took ");
  Serial.print(millis() - start);
  Serial.println(" ms");

  return _ip;
}


//+=============================================================================
// Turn off the Led after timeout
//
void disableLed()
{
  Serial.println("Turning off the LED to save power.");
  digitalWrite(ledpin, HIGH);                           // Shut down the LED
  ticker.detach();                                      // Stopping the ticker
}


//+=============================================================================
// Gets called when WiFiManager enters configuration mode
//
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}


// Callback function for WiFi events
void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.printf("[WiFi-event] event: %d\n", event);

  switch (event) {
    case SYSTEM_EVENT_STA_DISCONNECTED: // Older Cores might use this enum name directly
    // case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: // Newer Cores use this
        Serial.println("Lost Wifi - WiFi station disconnected");
        Serial.printf("Reason: %d\n", info.wifi_sta_disconnected.reason);
        // reset and try again
        ESP.restart();
        // delay(1000); // Delay likely won't execute after reset
        break;
    default:
        break;
  }
}


//+=============================================================================
// First setup of the Wifi.
// If return true, the Wifi is well connected.
// Should not return false if Wifi cannot be connected, it will loop
//
bool setupWifi(bool resetConf) {
  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.5, tick);
  
  WiFi.mode(WIFI_STA); // To make sure STA mode is preserved by WiFiManager and resets it after config is done.
  
  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  // set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);
  // set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  // Reset device if on config portal for greater than 3 minutes
  wifiManager.setConfigPortalTimeout(180);

  if (LittleFS.begin()) {
    Serial.println("mounted file system");
    if (LittleFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = LittleFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument json(1024);
        DeserializationError error = deserializeJson(json, buf.get());
        serializeJson(json, Serial);
        if (!error) {
          Serial.println("\nparsed json");

          if (json.containsKey("hostname")) strncpy(host_name, json["hostname"], 20);
          if (json.containsKey("passcode")) strncpy(passcode, json["passcode"], 20);
          if (json.containsKey("user_id")) strncpy(user_id, json["user_id"], 60);
          if (json.containsKey("port_str")) {
            strncpy(port_str, json["port_str"], 6);
            port = atoi(json["port_str"]);
          }
          if (json.containsKey("ip")) strncpy(static_ip, json["ip"], 16);
          if (json.containsKey("gw")) strncpy(static_gw, json["gw"], 16);
          if (json.containsKey("sn")) strncpy(static_sn, json["sn"], 16);
          if (json.containsKey("dns")) strncpy(static_dns, json["dns"], 16);
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }

  WiFiManagerParameter custom_hostname("hostname", "Choose a hostname to this IR Controller", host_name, 20);
  wifiManager.addParameter(&custom_hostname);
  WiFiManagerParameter custom_passcode("passcode", "Choose a passcode", passcode, 20);
  wifiManager.addParameter(&custom_passcode);
  WiFiManagerParameter custom_port("port_str", "Choose a port", port_str, 6);
  wifiManager.addParameter(&custom_port);
  WiFiManagerParameter custom_userid("user_id", "Enter your Amazon user_id", user_id, 60);
  wifiManager.addParameter(&custom_userid);

  wifiManager.setShowStaticFields(true);
  wifiManager.setShowDnsFields(true);

  IPAddress sip, sgw, ssn, dns;
  sip.fromString(static_ip);
  sgw.fromString(static_gw);
  ssn.fromString(static_sn);
  dns.fromString(static_dns);

  if (resetConf) {
    Serial.println("Reset triggered, launching in AP mode");
    wifiManager.startConfigPortal(wifi_config_name);
  } else {
    Serial.println("Setting static WiFi data from config");
    wifiManager.setSTAStaticIPConfig(sip, sgw, ssn, dns);
  }

  // fetches ssid and pass and tries to connect
  // if it does not connect it starts an access point with the specified name
  // and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect(wifi_config_name)) {
    Serial.println("Failed to connect and hit timeout");
    // reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(1000);
  }

  // if you get here you have connected to the WiFi
  strncpy(host_name, custom_hostname.getValue(), 20);
  strncpy(passcode, custom_passcode.getValue(), 20);
  strncpy(port_str, custom_port.getValue(), 6);
  strncpy(user_id, custom_userid.getValue(), 60);
  port = atoi(port_str);

  if (server != NULL) {
    delete server;
  }
  server = new WebServer(port);

// Register the WiFi event handler function for the disconnect event
WiFi.onEvent(WiFiEvent);
// Or if using newer core versions and the ARDUINO_EVENT_... enum:
// WiFi.onEvent(WiFiEvent, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
// If you kept the old function name:
// WiFi.onEvent(lostWifiCallback, SYSTEM_EVENT_STA_DISCONNECTED);

  Serial.println("WiFi connected! User chose hostname '" + String(host_name) + String("' passcode '") + String(passcode) + "' and port '" + String(port_str) + "'");

  // Save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println(" config...");
    DynamicJsonDocument json(1024);
    json["hostname"] = host_name;
    json["passcode"] = passcode;
    json["port_str"] = port_str;
    json["user_id"] = user_id;
    json["ip"] = WiFi.localIP().toString();
    json["gw"] = WiFi.gatewayIP().toString();
    json["sn"] = WiFi.subnetMask().toString();
    json["dns"] = WiFi.dnsIP().toString();

    File configFile = LittleFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    serializeJson(json, Serial);
    Serial.println("");
    Serial.println("Writing config file");
    serializeJson(json, configFile);
    configFile.close();
    json.clear();
    Serial.println("Config written successfully");
  }
  ticker.detach();

  // keep LED on
  digitalWrite(ledpin, LOW);
  return true;
}


//+=============================================================================
// Send CORS HTTP headers
//
void sendCorsHeaders() {
  server->sendHeader("Access-Control-Allow-Origin", "*");
  server->sendHeader("Access-Control-Allow-Methods", "GET, POST");
}

//+=============================================================================
// Handler for Button Configuration Page
//
void handleButtonConfigPage() {
  Serial.println("Connection received endpoint '/buttons' (GET)");

  // --- Security Check (optional) ---
  // if (!allowLocalBypass(server->client().remoteIP()) && !isPasscodeValid(server->arg("pass"))) { ... }

  sendHeader(); // Send standard HTML header

  server->sendContent("      <div class='row'>\n");
  server->sendContent("        <div class='col-md-12'>\n");
  server->sendContent("          <h2>Configure Remote Buttons</h2>\n");
  server->sendContent("          <p>Enter the details for each button you want to configure. Leave the 'Name' field empty to disable a button slot.</p>\n");
  server->sendContent("          <form class='form-horizontal' action='/savebuttons' method='post'>\n");

  // --- Helper Lambdas for Dropdowns (um Code-Duplizierung zu vermeiden) ---
  auto generateTypeDropdown = [&](const String& selectName, const String& selectedValue) {
    String html = "<select class='form-control' id='" + selectName + "' name='" + selectName + "'>\n";
    auto addSelected = [&](const String& val) { return val.equalsIgnoreCase(selectedValue) ? " selected" : ""; };
    // --- KORREKTUR: String(...) verwenden ---
    html += String("  <option value='nec'") + addSelected("nec") + ">NEC</option>\n";
    html += String("  <option value='sony'") + addSelected("sony") + ">SONY</option>\n";
    html += String("  <option value='rc5'") + addSelected("rc5") + ">RC5</option>\n";
    html += String("  <option value='rc6'") + addSelected("rc6") + ">RC6</option>\n";
    html += String("  <option value='panasonic'") + addSelected("panasonic") + ">PANASONIC</option>\n";
    html += String("  <option value='lg'") + addSelected("lg") + ">LG</option>\n";
    html += String("  <option value='jvc'") + addSelected("jvc") + ">JVC</option>\n";
    html += String("  <option value='samsung'") + addSelected("samsung") + ">SAMSUNG</option>\n";
    html += String("  <option value='whynter'") + addSelected("whynter") + ">WHYNTER</option>\n";
    html += String("  <option value='coolix'") + addSelected("coolix") + ">COOLIX</option>\n";
    html += String("  <option value='denon'") + addSelected("denon") + ">DENON</option>\n";
    html += String("  <option value='sharp'") + addSelected("sharp") + ">SHARP</option>\n";
    html += String("  <option value='sharpraw'") + addSelected("sharpraw") + ">SHARPRAW</option>\n";
    html += String("  <option value='dish'") + addSelected("dish") + ">DISH</option>\n";
    html += String("  <option value='gree'") + addSelected("gree") + ">GREE</option>\n";
    html += String("  <option value='lutron'") + addSelected("lutron") + ">LUTRON</option>\n";
    html += String("  <option value='roomba'") + addSelected("roomba") + ">ROOMBA</option>\n";
    html += String("  <option value='ecoclim'") + addSelected("ecoclim") + ">ECOCLIM</option>\n";
    // Füge hier weitere Typen hinzu, falls nötig.
    html += "</select>\n";
    return html;
  };

  auto generateOutDropdown = [&](const String& selectName, int selectedValue) {
      String html = "<select class='form-control' id='" + selectName + "' name='" + selectName + "'>\n";
      auto addOutSelected = [&](int val) { return (val == selectedValue) ? " selected" : ""; };
      
      // --- KORREKTUR: String(...) verwenden ---
      html += String("  <option value='1'") + addOutSelected(1) + ">1 (GPIO " + String(pins1) + ")</option>\n";
      html += String("  <option value='2'") + addOutSelected(2) + ">2 (GPIO " + String(pins2) + ")</option>\n";
      html += String("  <option value='3'") + addOutSelected(3) + ">3 (GPIO " + String(pins3) + ")</option>\n";
      html += String("  <option value='4'") + addOutSelected(4) + ">4 (GPIO " + String(pins4) + ")</option>\n";
      html += "</select>\n";
      return html;
  };
  // --- Ende Helper Lambdas ---


  for (int i = 0; i < MAX_BUTTONS; ++i) {
    String prefix = "btn" + String(i) + "_"; // Prefix für Feldnamen
    bool currentIsMacro = buttonConfigs[i].configured && buttonConfigs[i].isMacro;

    server->sendContent("            <hr><h4>Button " + String(i + 1) + "</h4>\n");

    // Name (bleibt)
    server->sendContent("            <div class='form-group'>\n");
    server->sendContent("              <label for='" + prefix + "name' class='col-sm-2 control-label'>Name</label>\n");
    server->sendContent("              <div class='col-sm-10'><input type='text' class='form-control' id='" + prefix + "name' name='" + prefix + "name' placeholder='Button Label (e.g., TV Power)' value='" + String(buttonConfigs[i].name) + "'></div>\n");
    server->sendContent("            </div>\n");

    // NEU: Modus-Auswahl (Radio Buttons)
    server->sendContent("            <div class='form-group'>\n");
    server->sendContent("              <label class='col-sm-2 control-label'>Mode</label>\n");
    server->sendContent("              <div class='col-sm-10'>\n");
    server->sendContent("                <label class='radio-inline'><input type='radio' name='" + prefix + "mode' value='single' " + (!currentIsMacro ? "checked" : "") + " onclick='toggleFields(\"" + prefix + "\", false)'> Single Code</label>\n");
    server->sendContent("                <label class='radio-inline'><input type='radio' name='" + prefix + "mode' value='macro' " + (currentIsMacro ? "checked" : "") + " onclick='toggleFields(\"" + prefix + "\", true)'> Macro (JSON)</label>\n");
    server->sendContent("              </div>\n");
    server->sendContent("            </div>\n");

    // Container für Single-Code-Felder
    server->sendContent("            <div id='" + prefix + "single_code_fields' style='" + (currentIsMacro ? "display: none;" : "") + "'>\n");
        // Type (Dropdown)
        server->sendContent("            <div class='form-group'>\n");
        server->sendContent("              <label for='" + prefix + "type' class='col-sm-2 control-label'>Type</label>\n");
        server->sendContent("              <div class='col-sm-10'>" + generateTypeDropdown(prefix + "type", String(buttonConfigs[i].type)) + "</div>\n");
        server->sendContent("            </div>\n");
        // Data (Hex)
        server->sendContent("            <div class='form-group'>\n");
        server->sendContent("              <label for='" + prefix + "data' class='col-sm-2 control-label'>Data (Hex)</label>\n");
        server->sendContent("              <div class='col-sm-10'><input type='text' class='form-control' id='" + prefix + "data' name='" + prefix + "data' placeholder='e.g., FF02FD' value='" + String(buttonConfigs[i].data) + "'></div>\n");
        server->sendContent("            </div>\n");
        // Length (Bits)
        server->sendContent("            <div class='form-group'>\n");
        server->sendContent("              <label for='" + prefix + "length' class='col-sm-2 control-label'>Length (Bits)</label>\n");
        server->sendContent("              <div class='col-sm-10'><input type='number' class='form-control' id='" + prefix + "length' name='" + prefix + "length' placeholder='e.g., 32' value='" + String(buttonConfigs[i].length) + "'></div>\n");
        server->sendContent("            </div>\n");
        // Address (Hex, optional)
        server->sendContent("            <div class='form-group'>\n");
        server->sendContent("              <label for='" + prefix + "address' class='col-sm-2 control-label'>Address (Hex, opt.)</label>\n");
        server->sendContent("              <div class='col-sm-10'><input type='text' class='form-control' id='" + prefix + "address' name='" + prefix + "address' placeholder='e.g., 0x404' value='" + String(buttonConfigs[i].address) + "'></div>\n");
        server->sendContent("            </div>\n");
        // Repeat
        server->sendContent("            <div class='form-group'>\n");
        server->sendContent("              <label for='" + prefix + "repeat' class='col-sm-2 control-label'>Repeat</label>\n");
        server->sendContent("              <div class='col-sm-10'><input type='number' class='form-control' id='" + prefix + "repeat' name='" + prefix + "repeat' value='" + String(buttonConfigs[i].repeat) + "' min='1'></div>\n");
        server->sendContent("            </div>\n");
        // Output Pin (Dropdown)
        server->sendContent("            <div class='form-group'>\n");
        server->sendContent("              <label for='" + prefix + "out' class='col-sm-2 control-label'>Output Pin</label>\n");
        server->sendContent("              <div class='col-sm-10'>" + generateOutDropdown(prefix + "out", buttonConfigs[i].out) + "</div>\n");
        server->sendContent("            </div>\n");
    server->sendContent("            </div>\n"); // Ende single_code_fields

    // Container für Makro-Feld
    server->sendContent("            <div id='" + prefix + "macro_field' style='" + (!currentIsMacro ? "display: none;" : "") + "'>\n");
        server->sendContent("            <div class='form-group'>\n");
        server->sendContent("              <label for='" + prefix + "macroJson' class='col-sm-2 control-label'>Macro JSON</label>\n");
        server->sendContent("              <div class='col-sm-10'>\n");
        server->sendContent("                <textarea class='form-control' id='" + prefix + "macroJson' name='" + prefix + "macroJson' rows='6' placeholder='Enter JSON array, e.g.,\\n[{&quot;type&quot;:&quot;nec&quot;, &quot;data&quot;:&quot;FF02FD&quot;, &quot;length&quot;:32}, {&quot;type&quot;:&quot;delay&quot;, &quot;rdelay&quot;:500}, {...}]'>" + String(buttonConfigs[i].macroJson) + "</textarea>\n");
        server->sendContent("                <span class='help-block'>Define a sequence of actions like in the /json endpoint. Use an array `[]` containing objects `{}`.</span>\n");
        server->sendContent("              </div>\n");
        server->sendContent("            </div>\n");
    server->sendContent("            </div>\n"); // Ende macro_field
  }

  // Submit Button
  server->sendContent("            <hr><div class='form-group'>\n");
  server->sendContent("              <div class='col-sm-offset-2 col-sm-10'>\n");
  server->sendContent("                <button type='submit' class='btn btn-success'>Save Button Configuration</button>\n");
  server->sendContent("                <a href='/' class='btn btn-default'>Cancel</a>\n");
  server->sendContent("              </div>\n");
  server->sendContent("            </div>\n");

  server->sendContent("          </form>\n");
  server->sendContent("        </div>\n");
  server->sendContent("      </div>\n");

  // NEU: JavaScript zum Umschalten der Felder hinzufügen (vor dem </form> oder am Ende der Seite)
server->sendContent("          <script>\n");
server->sendContent("            function toggleFields(prefix, isMacro) {\n");
server->sendContent("              document.getElementById(prefix + 'single_code_fields').style.display = isMacro ? 'none' : 'block';\n");
server->sendContent("              document.getElementById(prefix + 'macro_field').style.display = isMacro ? 'block' : 'none';\n");
server->sendContent("            }\n");
server->sendContent("          </script>\n");

  sendFooter(); // Send standard HTML footer
}


//+=============================================================================
// Handler to Save Button Configuration
//
void handleSaveButtons() {
  Serial.println("Connection received endpoint '/savebuttons' (POST)");

  // --- Security Check (optional) ---
  // if (!allowLocalBypass(server->client().remoteIP()) && !isPasscodeValid(server->arg("pass"))) { ... }

  bool changed = false;
for (int i = 0; i < MAX_BUTTONS; ++i) {
    String prefix = "btn" + String(i) + "_";

    String name = server->arg(prefix + "name");
    name.trim();

    // NEU: Modus auslesen
    String mode = server->arg(prefix + "mode"); // "single" oder "macro"
    bool isMacro = (mode == "macro");

    // Daten für den jeweiligen Modus auslesen
    String type = server->arg(prefix + "type");
    String data = server->arg(prefix + "data");
    int length = server->arg(prefix + "length").toInt();
    String address = server->arg(prefix + "address");
    int repeat = server->arg(prefix + "repeat").toInt();
    int out = server->arg(prefix + "out").toInt();
    String macroJson = server->arg(prefix + "macroJson");

    // Grundlegende Validierung basierend auf dem Modus
    bool isConfigured = (name.length() > 0 &&
                         ((!isMacro && data.length() > 0 && length > 0) ||
                          (isMacro && macroJson.length() > 0)));

    // Nur aktualisieren, wenn sich etwas geändert hat oder der Status sich ändert
    // (Diese Logik muss erweitert werden, um Änderungen in *allen* relevanten Feldern zu erkennen)
    bool configChanged = false;
    if (isConfigured != buttonConfigs[i].configured || name != buttonConfigs[i].name || isMacro != buttonConfigs[i].isMacro) {
        configChanged = true;
    } else if (isConfigured) {
        if (isMacro) {
            if (macroJson != buttonConfigs[i].macroJson) configChanged = true;
        } else {
            if (type != buttonConfigs[i].type || data != buttonConfigs[i].data ||
                length != buttonConfigs[i].length || address != buttonConfigs[i].address ||
                repeat != buttonConfigs[i].repeat || out != buttonConfigs[i].out) {
                configChanged = true;
            }
        }
    }

    if (configChanged)
    {
        changed = true; // Markiere, dass *irgendetwas* geändert wurde
        strncpy(buttonConfigs[i].name, name.c_str(), sizeof(buttonConfigs[i].name) - 1);
        buttonConfigs[i].name[sizeof(buttonConfigs[i].name) - 1] = '\0'; // Null-terminieren
        buttonConfigs[i].isMacro = isMacro; // NEU: Modus speichern
        buttonConfigs[i].configured = isConfigured; // NEU: Konfiguriert-Status setzen

        if (isConfigured) {
            if (isMacro) {
                // Makro-Daten speichern, Single-Code-Daten löschen/Defaults setzen
                strncpy(buttonConfigs[i].macroJson, macroJson.c_str(), sizeof(buttonConfigs[i].macroJson) - 1);
                buttonConfigs[i].macroJson[sizeof(buttonConfigs[i].macroJson) - 1] = '\0';
                buttonConfigs[i].type[0] = '\0';
                buttonConfigs[i].data[0] = '\0';
                buttonConfigs[i].length = 0;
                buttonConfigs[i].address[0] = '\0';
                buttonConfigs[i].repeat = 1;
                buttonConfigs[i].out = 1;
            } else {
                // Single-Code-Daten speichern, Makro-Daten löschen
                strncpy(buttonConfigs[i].type, type.c_str(), sizeof(buttonConfigs[i].type) - 1);
                strncpy(buttonConfigs[i].data, data.c_str(), sizeof(buttonConfigs[i].data) - 1);
                buttonConfigs[i].length = length;
                strncpy(buttonConfigs[i].address, address.c_str(), sizeof(buttonConfigs[i].address) - 1);
                buttonConfigs[i].repeat = (repeat > 0) ? repeat : 1;
                buttonConfigs[i].out = (out >= 1 && out <= 4) ? out : 1;
                buttonConfigs[i].macroJson[0] = '\0';

                // Sicherstellen, dass Strings null-terminiert sind
                buttonConfigs[i].type[sizeof(buttonConfigs[i].type) - 1] = '\0';
                buttonConfigs[i].data[sizeof(buttonConfigs[i].data) - 1] = '\0';
                buttonConfigs[i].address[sizeof(buttonConfigs[i].address) - 1] = '\0';
            }
        } else {
            // Button deaktivieren/leeren (wie bisher, aber auch Makro leeren)
            buttonConfigs[i].name[0] = '\0';
            buttonConfigs[i].type[0] = '\0';
            buttonConfigs[i].data[0] = '\0';
            buttonConfigs[i].length = 0;
            buttonConfigs[i].address[0] = '\0';
            buttonConfigs[i].repeat = 1;
            buttonConfigs[i].out = 1;
            buttonConfigs[i].isMacro = false;
            buttonConfigs[i].macroJson[0] = '\0';
            buttonConfigs[i].configured = false;
        }
    }
}

if (changed) {
    Serial.println("Button configuration changed, saving...");
    saveButtonConfig();
} else {
    Serial.println("No changes detected in button configuration.");
}

// Redirect back to home page after saving
server->sendHeader("Location", "/?status=buttons_saved"); // Optional: Status für Feedback
server->send(303); // 303 See Other
}


//+=============================================================================
// Handler to Send IR Code from a Remote Button (AJAX)
//
void handleSendButton() {
  Serial.println("Connection received endpoint '/sendbutton' (POST)");

  // --- Security Check (optional) ---
  // if (!allowLocalBypass(server->client().remoteIP()) && !isPasscodeValid(server->arg("pass"))) { ... }

  // --- Argument Parsing (aus JSON Body) ---
  if (server->hasArg("plain") == false || server->method() != HTTP_POST) {
    Serial.println("Invalid request to /sendbutton");
    server->send(400, "text/plain", "Bad Request: Missing JSON payload or wrong method.");
    return;
  }

  String body = server->arg("plain");
  DynamicJsonDocument jsonDoc(512); // Ausreichend für die Button-Parameter
  DeserializationError error = deserializeJson(jsonDoc, body);

  if (error) {
    Serial.print("Failed to parse JSON from /sendbutton: ");
    Serial.println(error.c_str());
    server->send(400, "text/plain", "Bad Request: Invalid JSON.");
    return;
  }

  // Parameter aus JSON extrahieren
  String type = jsonDoc["type"] | "";
  String dataStr = jsonDoc["data"] | "";
  unsigned int len = jsonDoc["length"] | 0;
  String addressStr = jsonDoc["address"] | "";
  int repeat = jsonDoc["repeat"] | 1;
  int out = jsonDoc["out"] | 1;

  // Adresse parsen (mit optionalem "0x")
  long address = 0;
  if (addressStr.length() > 0) {
      if (addressStr.startsWith("0x")) {
          address = strtoul(addressStr.c_str(), 0, 0);
      } else {
          address = strtoul(("0x" + addressStr).c_str(), 0, 0);
      }
  }

  // Default values for delays/pulse (könnten auch aus JSON kommen, wenn nötig)
  int rdelay = 1000;
  int pulse = 1;
  int pdelay = 100;

  // Validate inputs (basic)
  if (type.length() == 0 || dataStr.length() == 0 || len == 0) {
      Serial.println("Invalid arguments received via /sendbutton");
      server->send(400, "text/plain", "Bad Request: Missing or invalid IR parameters.");
      return;
  }
  if (repeat <= 0) repeat = 1;
  if (out < 1 || out > 4) out = 1;

  // --- Trigger IR Blast ---
  Serial.println("Calling irblast from button press...");
  digitalWrite(ledpin, LOW); // Turn LED on during send
  ticker.attach(0.5, disableLed); // Schedule LED turn off

  // Call the existing irblast function (stelle sicher, dass sie 'out_pin' akzeptiert)
  irblast(type, dataStr, len, rdelay, pulse, pdelay, repeat, address, pickIRsend(out), out);

  // --- Send Success Response ---
  sendCorsHeaders(); // Wichtig für AJAX von anderer Domain/Port (falls zutreffend)
  server->send(200, "text/plain", "OK");
}


//+=============================================================================
// Handler for the IR sending form
//
void handleSendIr() {
  Serial.println("Connection received endpoint '/sendir' (POST)");

  // --- Security Check (optional but recommended) ---
  // You might want to reuse parts of the security checks from /msg or /json
  // For simplicity, we'll skip strict HMAC for now, assuming access to the page was already authenticated.
  // Add passcode check if needed:
  /*
  if (!allowLocalBypass(server->client().remoteIP()) && !isPasscodeValid(server->arg("pass"))) { // 'pass' needs to be added to the form if used
      Serial.println("Unauthorized access (passcode)");
      sendCorsHeaders(); // May not be needed if not called via AJAX
      server->send(401, "text/plain", "Unauthorized, invalid passcode");
      return;
  }
  */

  // --- Argument Parsing ---
  if (!server->hasArg("type") || !server->hasArg("data") || !server->hasArg("length")) {
    Serial.println("Missing required arguments (type, data, length)");
    // Redirect back to home page with an error message
    server->sendHeader("Location", "/?status=error_missing_args");
    server->send(303); // 303 See Other
    return;
  }

  String type = server->arg("type");
  String dataStr = server->arg("data");
  unsigned int len = server->arg("length").toInt();
  long address = 0;
  if (server->hasArg("address") && server->arg("address").length() > 0) {
      // Handle potential "0x" prefix if users add it
      String addressStr = server->arg("address");
      if (addressStr.startsWith("0x")) {
          address = strtoul(addressStr.c_str(), 0, 0); // Base 0 auto-detects 0x
      } else {
          address = strtoul(("0x" + addressStr).c_str(), 0, 0); // Assume hex if not prefixed
      }
  }
  int repeat = (server->hasArg("repeat")) ? server->arg("repeat").toInt() : 1;
  int out = (server->hasArg("out")) ? server->arg("out").toInt() : 1;

  // Default values for delays/pulse if not included in the simple form
  int rdelay = 1000; // Default repeat delay
  int pulse = 1;     // Default pulse count
  int pdelay = 100;  // Default pulse delay

  // Validate inputs (basic)
  if (len == 0 || dataStr.length() == 0) {
      Serial.println("Invalid arguments (length or data empty)");
      server->sendHeader("Location", "/?status=error_invalid_args");
      server->send(303);
      return;
  }
  if (repeat <= 0) repeat = 1;
  if (out < 1 || out > 4) out = 1;


  // --- Trigger IR Blast ---
  Serial.println("Calling irblast from form...");
  digitalWrite(ledpin, LOW); // Turn LED on during send
  ticker.attach(0.5, disableLed); // Schedule LED turn off

  // Call the existing irblast function
// OLD: irblast(type, dataStr, len, rdelay, pulse, pdelay, repeat, address, pickIRsend(out));
// NEW: Pass 'out' as the last argument
irblast(type, dataStr, len, rdelay, pulse, pdelay, repeat, address, pickIRsend(out), out);


  // --- Redirect back to home page with success message ---
  server->sendHeader("Location", "/?status=success");
  server->send(303); // 303 See Other is appropriate for redirect after POST
}


//+=============================================================================
// Setup web server and IR receiver/blaster
//
void setup() {
  // Initialize serial
  Serial.begin(115200);
  Serial.println("\nBooting...");

  if (!LittleFS.begin()) {
    Serial.println("LittleFS Mount Failed! Formatting...");
    if (LittleFS.format()) {
       Serial.println("LittleFS Formatted. Please reset.");
    } else {
       Serial.println("LittleFS Format Failed.");
    }
    while(1) yield(); // Anhalten nach Formatierung
  }
  Serial.println("LittleFS Mounted.");

  // set led pin as output
  pinMode(ledpin, OUTPUT);

  Serial.println("");
  Serial.println("ESP32 IR Controller");
  pinMode(configpin, INPUT_PULLUP);
  Serial.print("Config pin GPIO");
  Serial.print(configpin);
  Serial.print(" set to: ");
  Serial.println(digitalRead(configpin));
  if (!setupWifi(digitalRead(configpin) == LOW))
    return;

  Serial.println("WiFi configuration complete");

  loadButtonConfig(); // Lade die Button-Konfigurationen

  // Set the hostname
  if (strlen(host_name) > 0) {
    // A hostname was loaded from config or set via WiFiManager
    if (!WiFi.setHostname(host_name)) {
      Serial.println("ERROR: Failed to set hostname!");
    } else {
      Serial.print("Hostname set to: ");
      Serial.println(host_name);
    }
  } else {
    // No hostname was provided, get the default hostname generated by ESP32
    // (usually based on MAC address like "ESP32-AABBCCDDEEFF")
    const char* defaultHostname = WiFi.getHostname();
    strncpy(host_name, defaultHostname, sizeof(host_name) - 1); // Copy default hostname to our buffer
    host_name[sizeof(host_name) - 1] = '\0'; // Ensure null termination
    Serial.print("Using default hostname: ");
    Serial.println(host_name);
    // Optional: Man könnte hier auch nochmals WiFi.setHostname(host_name) aufrufen,
    // aber getHostname sollte bereits den aktiven Namen liefern.
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  WiFi.setSleep(true);
  digitalWrite(ledpin, LOW);
  // Turn off the led in 2s
  ticker.attach(2, disableLed);

  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP().toString());
  Serial.print("DNS IP: ");
  Serial.println(WiFi.dnsIP().toString());
  Serial.println("URL to send commands: http://" + String(host_name) + ".local:" + port_str);

  if (enableMDNSServices) {
    // Configure OTA Update
    ArduinoOTA.setPort(8266);
    ArduinoOTA.setHostname(host_name);
    ArduinoOTA.onStart([]() {
      Serial.println("Start");
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
    Serial.println("ArduinoOTA started");

    // Configure mDNS
    MDNS.addService("http", "tcp", port); // Announce the ESP as an HTTP service
    Serial.println("MDNS http service added. Hostname is set to " + String(host_name) + ".local:" + String(port));
  }

  // Configure the server
  server->on("/json", []() { // JSON handler for more complicated IR blaster routines
    Serial.println("Connection received endpoint '/json'");

    int simple = 0;
    if (server->hasArg("simple")) simple = server->arg("simple").toInt();
    String signature = server->arg("auth");
    String epid = server->arg("epid");
    String mid = server->arg("mid");
    String timestamp = server->arg("time");

    if (!allowLocalBypass(server->client().remoteIP()) && !isPasscodeValid(server->arg("pass"))) {
      Serial.println("Unauthorized access");
      sendCorsHeaders();
      server->send(401, "text/plain", "Unauthorized, invalid passcode");
    } else if (strlen(user_id) != 0 && !validateHMAC(epid, mid, timestamp, signature, server->client().remoteIP())) {
      Serial.println("Unauthorized access");
      sendCorsHeaders();
      server->send(401, "text/plain", "Unauthorized, HMAC security authentication failed");
    } else {
      DynamicJsonDocument root(4096);
      DeserializationError error = deserializeJson(root, server->arg("plain"));
      int out = (server->hasArg("out")) ? server->arg("out").toInt() : 1;
      if (error) {
        Serial.println("JSON parsing failed");
        Serial.println(error.c_str());
        if (simple) {
          sendCorsHeaders();
          server->send(400, "text/plain", "JSON parsing failed, " + String(error.c_str()));
        } else {
          sendHomePage("JSON parsing failed", "Error", 3, 400); // 400
        }
        root.clear();
      } else {
        digitalWrite(ledpin, LOW);
        ticker.attach(0.5, disableLed);

        // Handle device state limitations for the global JSON command request
        if (server->hasArg("device")) {
          String device = server->arg("device");
          Serial.println("Device name detected " + device);
          int state = (server->hasArg("state")) ? server->arg("state").toInt() : 0;
          if (deviceState.containsKey(device)) {
            Serial.println("Contains the key!");
            Serial.println(state);
            int currentState = deviceState[device];
            Serial.println(currentState);
            if (state == currentState) {
              if (simple) {
                sendCorsHeaders();
                server->send(200, "text/html", "Not sending command to " + device + ", already in state " + state);
              } else {
                sendHomePage("Not sending command to " + device + ", already in state " + state, "Warning", 2); // 200
              }
              Serial.println("Not sending command to " + device + ", already in state " + state);
              return;
            } else {
              Serial.println("Setting device " + device + " to state " + state);
              deviceState[device] = state;
            }
          } else {
            Serial.println("Setting device " + device + " to state " + state);
            deviceState[device] = state;
          }
        }

        if (simple) {
          sendCorsHeaders();
          server->send(200, "text/html", "Success, code sent");
        }

        String message = "Code sent";

        for (size_t x = 0; x < root.size(); x++) {
          String type = root[x]["type"];
          String ip = root[x]["ip"];
          int rdelay = root[x]["rdelay"];
          int pulse = root[x]["pulse"];
          int pdelay = root[x]["pdelay"];
          int repeat = root[x]["repeat"];
          int xout = root[x]["out"];
          if (xout == 0) {
            xout = out;
          }
          int duty = root[x]["duty"];

          if (pulse <= 0) pulse = 1; // Make sure pulse isn't 0
          if (repeat <= 0) repeat = 1; // Make sure repeat isn't 0
          if (pdelay <= 0) pdelay = 100; // Default pdelay
          if (rdelay <= 0) rdelay = 1000; // Default rdelay
          if (duty <= 0) duty = 50; // Default duty

          // Handle device state limitations on a per JSON object basis
          String device = root[x]["device"];
          if (device != "null") {
            int state = root[x]["state"];
            if (deviceState.containsKey(device)) {
              int currentState = deviceState[device];
              if (state == currentState) {
                Serial.println("Not sending command to " + device + ", already in state " + state);
                message = "Code sent. Some components of the code were held because device was already in appropriate state";
                continue;
              } else {
                Serial.println("Setting device " + device + " to state " + state);
                deviceState[device] = state;
              }
            } else {
              Serial.println("Setting device " + device + " to state " + state);
              deviceState[device] = state;
            }
          }

          if (type == "delay") {
            delay(rdelay);
          } else if (type == "raw") {
            JsonArray raw = root[x]["data"]; // Array of unsigned int values for the raw signal
            int khz = root[x]["khz"];
            if (khz <= 0) khz = 38; // Default to 38khz if not set
            // OLD: rawblast(raw, khz, rdelay, pulse, pdelay, repeat, pickIRsend(xout),duty);
            // NEW: Pass 'xout' as the last argument
            rawblast(raw, khz, rdelay, pulse, pdelay, repeat, pickIRsend(xout), duty, xout);
          } else if (type == "pronto") {
            JsonArray pdata = root[x]["data"]; // Array of values for pronto
            // OLD: pronto(pdata, rdelay, pulse, pdelay, repeat, pickIRsend(xout));
            // NEW: Pass 'xout' as the last argument
            pronto(pdata, rdelay, pulse, pdelay, repeat, pickIRsend(xout), xout);
          } else if (type == "roku") {
            String data = root[x]["data"];
            rokuCommand(ip, data, repeat, rdelay);
          } else {
            String data = root[x]["data"];
            String addressString = root[x]["address"];
            long address = strtoul(addressString.c_str(), 0, 0);
            int len = root[x]["length"];
            // OLD: irblast(type, data, len, rdelay, pulse, pdelay, repeat, address, pickIRsend(xout));
            // NEW: Pass 'xout' as the last argument
            irblast(type, data, len, rdelay, pulse, pdelay, repeat, address, pickIRsend(xout), xout);
          }
        }

        if (!simple) {
          Serial.println("Sending home page");
          sendHomePage(message, "Success", 1); // 200
        }

        root.clear();
      }
    }
  });

  // Setup simple msg server to mirror version 1.0 functionality
  server->on("/msg", []() {
    Serial.println("Connection received endpoint '/msg'");

    int simple = 0;
    if (server->hasArg("simple")) simple = server->arg("simple").toInt();
    String signature = server->arg("auth");
    String epid = server->arg("epid");
    String mid = server->arg("mid");
    String timestamp = server->arg("time");

    if (!allowLocalBypass(server->client().remoteIP()) && !isPasscodeValid(server->arg("pass"))) {
      Serial.println("Unauthorized access");
      sendCorsHeaders();
      server->send(401, "text/plain", "Unauthorized, invalid passcode");
    } else if (strlen(user_id) != 0 && !validateHMAC(epid, mid, timestamp, signature, server->client().remoteIP())) {
      Serial.println("Unauthorized access");
      sendCorsHeaders();
      server->send(401, "text/plain", "Unauthorized, HMAC security authentication");
    } else {
      digitalWrite(ledpin, LOW);
      ticker.attach(0.5, disableLed);
      String type = server->arg("type");
      String data = server->arg("data");
      String ip = server->arg("ip");

      // Handle device state limitations
      if (server->hasArg("device")) {
        String device = server->arg("device");
        Serial.println("Device name detected " + device);
        int state = (server->hasArg("state")) ? server->arg("state").toInt() : 0;
        if (deviceState.containsKey(device)) {
          Serial.println("Contains the key!");
          Serial.println(state);
          int currentState = deviceState[device];
          Serial.println(currentState);
          if (state == currentState) {
            if (simple) {
              sendCorsHeaders();
              server->send(200, "text/html", "Not sending command to " + device + ", already in state " + state);
            } else {
              sendHomePage("Not sending command to " + device + ", already in state " + state, "Warning", 2); // 200
            }
            Serial.println("Not sending command to " + device + ", already in state " + state);
            return;
          } else {
            Serial.println("Setting device " + device + " to state " + state);
            deviceState[device] = state;
          }
        } else {
          Serial.println("Setting device " + device + " to state " + state);
          deviceState[device] = state;
        }
      }

      int len = server->arg("length").toInt();
      long address = 0;
      if (server->hasArg("address")) {
        String addressString = server->arg("address");
        address = strtoul(addressString.c_str(), 0, 0);
      }

      int rdelay = (server->hasArg("rdelay")) ? server->arg("rdelay").toInt() : 1000;
      int pulse = (server->hasArg("pulse")) ? server->arg("pulse").toInt() : 1;
      int pdelay = (server->hasArg("pdelay")) ? server->arg("pdelay").toInt() : 100;
      int repeat = (server->hasArg("repeat")) ? server->arg("repeat").toInt() : 1;
      int out = (server->hasArg("out")) ? server->arg("out").toInt() : 1;
      if (server->hasArg("code")) {
        String code = server->arg("code");
        char separator = ':';
        data = getValue(code, separator, 0);
        type = getValue(code, separator, 1);
        len = getValue(code, separator, 2).toInt();
      }

      if (simple) {
        sendCorsHeaders();
        server->send(200, "text/html", "Success, code sent");
      }

      if (type == "roku") {
        rokuCommand(ip, data, repeat, rdelay);
      } else {
        // OLD: irblast(type, data, len, rdelay, pulse, pdelay, repeat, address, pickIRsend(out));
        // NEW: Pass 'out' as the last argument
        irblast(type, data, len, rdelay, pulse, pdelay, repeat, address, pickIRsend(out), out);
      }

      if (!simple) {
        sendHomePage("Code Sent", "Success", 1); // 200
      }
    }
  });

  server->on("/received", []() {
    Serial.println("Connection received endpoint '/received'");
    String signature = server->arg("auth");
    String epid = server->arg("epid");
    String mid = server->arg("mid");
    String timestamp = server->arg("time");
    
    if (!allowLocalBypass(server->client().remoteIP()) && !isPasscodeValid(server->arg("pass"))) {
      Serial.println("Unauthorized access");
      sendCorsHeaders();
      server->send(401, "text/plain", "Unauthorized, invalid passcode");
    } else if (strlen(user_id) != 0 && !validateHMAC(epid, mid, timestamp, signature, server->client().remoteIP())) {
      Serial.println("Unauthorized access");
      sendCorsHeaders();
      server->send(401, "text/plain", "Unauthorized, HMAC security authentication");
    } else {
      int id = server->arg("id").toInt();
      String output;
      if (id == 1 && last_recv.valid) {
        sendCodePage(last_recv);
      } else if (id == 2 && last_recv_2.valid) {
        sendCodePage(last_recv_2);
      } else if (id == 3 && last_recv_3.valid) {
        sendCodePage(last_recv_3);
      } else if (id == 4 && last_recv_4.valid) {
        sendCodePage(last_recv_4);
      } else if (id == 5 && last_recv_5.valid) {
        sendCodePage(last_recv_5);
      } else {
        sendHomePage("Code does not exist", "Alert", 2, 404); // 404
      }
    }
  });

  server->on("/", []() {
    Serial.println("Connection received endpoint '/'");
    String signature = server->arg("auth");
    String epid = server->arg("epid");
    String mid = server->arg("mid");
    String timestamp = server->arg("time");
    
    if (!allowLocalBypass(server->client().remoteIP()) && !isPasscodeValid(server->arg("pass"))) {
      Serial.println("Unauthorized access");
      sendCorsHeaders();
      server->send(401, "text/plain", "Unauthorized, invalid passcode");
    } else if (strlen(user_id) != 0 && !validateHMAC(epid, mid, timestamp, signature, server->client().remoteIP())) {
      Serial.println("Unauthorized access");
      sendCorsHeaders();
      server->send(401, "text/plain", "Unauthorized, HMAC security authentication");
    } else {
      sendHomePage(); // 200
    }
  });

    // --- NEUE SERVER-HANDLER REGISTRIEREN ---
    server->on("/buttons", HTTP_GET, handleButtonConfigPage);
    server->on("/savebuttons", HTTP_POST, handleSaveButtons);
    server->on("/sendbutton", HTTP_POST, handleSendButton);
    // --- ENDE NEUE HANDLER ---

  server->begin();
  Serial.println("HTTP Server started on port " + String(port));

  server->on("/sendir", HTTP_POST, handleSendIr); // NEUE ZEILE: Handler für Formular-POST registrieren


  Serial.println("Starting UDP");
  ntpUDP.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(localPort);
  Serial.println("Waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(300);
  
  externalIP();

  if (strlen(user_id) > 0) {
    userIDError = !validUID(user_id);
    if (!userIDError) {
      Serial.println("No errors detected with security configuration");
    }

    // Validation check time
    time_t timenow = now() - (timeZone * SECS_PER_HOUR);
    bool validEpoch = validEPOCH(timenow);
    if (validEpoch) {
      Serial.println("EPOCH time obtained for security checks");
    } else {
      Serial.println("Invalid EPOCH time, security checks may fail if unable to sync with NTP server");
    }
  }

  irsend1.begin();
  irsend2.begin();
  irsend3.begin();
  irsend4.begin();
  irrecv.enableIRIn();
  Serial.println("Ready to send and receive IR signals");
}


//+=============================================================================
// NTP Code
//
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (ntpUDP.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = ntpUDP.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      ntpUDP.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}


//+=============================================================================
// Send an NTP request to the time server at the given address
//
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  ntpUDP.beginPacket(address, 123); //NTP requests are to port 123
  ntpUDP.write(packetBuffer, NTP_PACKET_SIZE);
  ntpUDP.endPacket();
}

//+=============================================================================
// Send command to local roku
//
int rokuCommand(String ip, String data, int repeat, int rdelay) {
  String url = "http://" + ip + ":8060/" + data;
  HTTPClient http;

  int output = 0;

  for (int r = 0; r < repeat; r++) {
    http.begin(url);
    Serial.println(url);
    Serial.println("Sending roku command");
  
    copyCode(last_send_4, last_send_5);
    copyCode(last_send_3, last_send_4);
    copyCode(last_send_2, last_send_3);
    copyCode(last_send, last_send_2);
  
    strncpy(last_send.data, data.c_str(), 40);
    last_send.bits = 1;
    strncpy(last_send.encoding, "roku", 14);
    strncpy(last_send.address, ip.c_str(), 20);
    last_send.timestamp = now();
    last_send.valid = true;
  
    output = http.POST("");
    http.end();

    if (r + 1 < repeat) delay(rdelay);
  }

  // +++ NEUE ZEILEN (nach dem Loop) +++
  // Store info about the *last* command sent in the loop
  copyCode(last_send_4, last_send_5);
  copyCode(last_send_3, last_send_4);
  copyCode(last_send_2, last_send_3);
  copyCode(last_send, last_send_2);

  strncpy(last_send.data, data.c_str(), 40);
  last_send.bits = 1; // Roku doesn't have bits in the same way
  strncpy(last_send.encoding, "roku", 14);
  strncpy(last_send.address, ip.c_str(), 20); // Store IP as address
  last_send.timestamp = now();
  last_send.valid = true;
  last_send.repeat = repeat; // Store repeat count
  last_send.out = 1; // Default out pin for Roku
  // +++ ENDE NEUE ZEILEN +++

  return output;
}

//+=============================================================================
// Split string by character
//
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}


//+=============================================================================
// Return which IRsend object to act on
//
IRsend pickIRsend (int out) {
  switch (out) {
    case 1: return irsend1;
    case 2: return irsend2;
    case 3: return irsend3;
    case 4: return irsend4;
    default: return irsend1;
  }
}


//+=============================================================================
// Display encoding type
//
String encoding(decode_results *results) {
  return typeToString(results->decode_type);
}

//+=============================================================================
// Code to string
//
void fullCode (decode_results *results)
{
  Serial.print("One line: ");
  serialPrintUint64(results->value, 16);
  Serial.print(":");
  Serial.print(encoding(results));
  Serial.print(":");
  Serial.print(results->bits, DEC);
  if (results->repeat) Serial.print(" (Repeat)");
  Serial.println("");
  if (results->overflow)
    Serial.println("WARNING: IR code too long. "
                   "Edit IRController.ino and increase captureBufSize");
}

//+=============================================================================
// Send header HTML
//
void sendHeader() {
  sendHeader(200);
}

void sendHeader(int httpcode) { 
  // Setze den Content-Type für die gesamte Seite
  server->sendHeader("Content-Type", "text/html; charset=utf-8");
  server->sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server->sendHeader("Pragma", "no-cache");
  server->sendHeader("Expires", "-1");
  // Beginne die Antwort (wichtig, damit sendContent funktioniert)
  // Sende den Header-Inhalt aus der Datei
  File headerFile = LittleFS.open("/header.html", "r");
  if (headerFile) {
    // Sende den Inhalt stückweise, falls er groß ist
    // server->sendContent_P(headerFile.readString().c_str()); // Einfach, aber speicherintensiv für große Dateien
    // Besser: Stückweise senden
    server->sendContent(""); // Beginnt die chunked response
    while(headerFile.available()){
        char buf[256]; // Puffergröße anpassen
        size_t len = headerFile.readBytes(buf, sizeof(buf));
        server->sendContent(buf, len);
    }
    headerFile.close();
  } else {
    Serial.println("ERROR: header.html not found!");
    server->sendContent("<!DOCTYPE html><html><head><title>Error</title></head><body><h1>Error: Header file not found!</h1>");
    // Sende hier nicht den Footer, da der Header fehlt.
  }
  // Die Verbindung bleibt offen für den eigentlichen Seiteninhalt
}


//+=============================================================================
// Send footer HTML
//
void sendFooter() {
  // Sende den Footer-Inhalt aus der Datei
  File footerFile = LittleFS.open("/footer.html", "r");
  if (footerFile) {
    // Sende den Inhalt stückweise
     while(footerFile.available()){
        char buf[256]; // Puffergröße anpassen
        size_t len = footerFile.readBytes(buf, sizeof(buf));
        server->sendContent(buf, len);
    }
    footerFile.close();
  } else {
    Serial.println("ERROR: footer.html not found!");
    server->sendContent("<p><em>Error: Footer file not found!</em></p></body></html>");
  }
  // Beende die Antwort (wird durch den letzten sendContent implizit gemacht)
  server->sendContent(""); // Sendet den finalen chunk und schließt die Antwort
}


// Helper function to send a file from LittleFS
bool sendFileFromFS(const String& path, const String& contentType) {
  // WICHTIG: Pfad muss mit '/' beginnen!
  String pathWithSlash = path;
  if (!pathWithSlash.startsWith("/")) {
    pathWithSlash = "/" + pathWithSlash;
  }

  Serial.print("Attempting to send: "); Serial.println(pathWithSlash);
  File file = LittleFS.open(pathWithSlash, "r");
  if (!file) {
    Serial.print("Failed to open file: "); Serial.println(pathWithSlash);
    // Sende keine 404 hier, da es Teil einer größeren Antwort sein könnte
    return false;
  }

  // Sende den Inhalt der Datei direkt. streamFile ist effizienter,
  // aber für Header/Footer, die oft mit anderem Content gesendet werden,
  // ist server->write() einfacher zu integrieren.
  size_t sent = server->streamFile(file, contentType);
  file.close();

  if (sent != file.size()) {
     Serial.println("Sent less data than expected!");
     return false;
  }

  Serial.print("Sent file: "); Serial.println(pathWithSlash);
  return true;
}


//+=============================================================================
// Stream home page HTML
//
void sendHomePage() {
  sendHomePage("", "");
}

void sendHomePage(String message, String header) {
  sendHomePage(message, header, 0);
}

void sendHomePage(String message, String header, int type) {
  sendHomePage(message, header, type, 200);
}

void sendHomePage(String message, String header, int type, int httpcode) {
  sendHeader(httpcode);


  // +++ FERNBEDIENUNGS-BUTTONS ANZEIGEN +++
  server->sendContent("      <div class='row'>\n");
  server->sendContent("        <div class='col-md-12'>\n");
  server->sendContent("          <h3>Remote Buttons</h3>\n");
  server->sendContent("          <div id='remote-buttons' class='text-center'>\n"); // Container für Buttons

  bool anyButtonConfigured = false;
  for (int i = 0; i < MAX_BUTTONS; ++i) {
    if (buttonConfigs[i].configured) {
      anyButtonConfigured = true;
      server->sendContent("            <button class='btn btn-primary btn-lg remote-button' style='margin: 5px;' ");
      // Speichere IR-Daten ODER Makro-Daten in data-Attributen
      if (buttonConfigs[i].isMacro) {
          server->sendContent("data-is-macro='true' ");
          // WICHTIG: JSON im Attribut korrekt escapen (Anführungszeichen!)
          // Eine einfache Ersetzung ist hier riskant. Besser wäre es,
          // das JSON im JavaScript direkt aus einer Datenstruktur zu holen,
          // aber für die direkte Einbettung versuchen wir es so:
          String escapedJson = String(buttonConfigs[i].macroJson);
          escapedJson.replace("'", "&#39;"); // Escape einfache Anführungszeichen
          escapedJson.replace("\"", "&quot;"); // Escape doppelte Anführungszeichen
          server->sendContent("data-macro-json='" + escapedJson + "'>");
      } else {
          server->sendContent("data-is-macro='false' "); // Explizit setzen
          server->sendContent("data-type='" + String(buttonConfigs[i].type) + "' ");
          server->sendContent("data-data='" + String(buttonConfigs[i].data) + "' ");
          server->sendContent("data-length='" + String(buttonConfigs[i].length) + "' ");
          server->sendContent("data-address='" + String(buttonConfigs[i].address) + "' ");
          server->sendContent("data-repeat='" + String(buttonConfigs[i].repeat) + "' ");
          server->sendContent("data-out='" + String(buttonConfigs[i].out) + "'>");
      }
      server->sendContent(String(buttonConfigs[i].name)); // Button-Beschriftung
      server->sendContent("</button>\n");
    }
}

  if (!anyButtonConfigured) {
      server->sendContent("            <p><em>No remote buttons configured yet.</em></p>\n");
  }

  server->sendContent("            <a href='/buttons' class='btn btn-default' style='margin: 5px;'>Configure Buttons</a>\n"); // Link zur Konfigurationsseite
  server->sendContent("          </div>\n");
  server->sendContent("        </div>\n");
  server->sendContent("      </div><hr />\n");
  // +++ ENDE FERNBEDIENUNGS-BUTTONS +++

    // +++ NEU: Feedback vom Formular anzeigen +++
    if (server->hasArg("status")) {
      String status = server->arg("status");
      if (status == "success") {
        server->sendContent("      <div class='row'><div class='col-md-12'><div class='alert alert-success'><strong>Success!</strong> IR code sent via form.</div></div></div>\n");
      } else if (status == "error_missing_args") {
        server->sendContent("      <div class='row'><div class='col-md-12'><div class='alert alert-danger'><strong>Error!</strong> Missing required form fields (type, data, length).</div></div></div>\n");
      } else if (status == "error_invalid_args") {
        server->sendContent("      <div class='row'><div class='col-md-12'><div class='alert alert-danger'><strong>Error!</strong> Invalid form data (e.g., length 0 or empty data).</div></div></div>\n");
      }
       // Weitere Statusmeldungen nach Bedarf hinzufügen...
    }
    // +++ ENDE NEU +++

  if (type == 1)
  server->sendContent("      <div class='row'><div class='col-md-12'><div class='alert alert-success'><strong>" + header + "!</strong> " + message + "</div></div></div>\n");
  if (type == 2)
  server->sendContent("      <div class='row'><div class='col-md-12'><div class='alert alert-warning'><strong>" + header + "!</strong> " + message + "</div></div></div>\n");
  if (type == 3)
  server->sendContent("      <div class='row'><div class='col-md-12'><div class='alert alert-danger'><strong>" + header + "!</strong> " + message + "</div></div></div>\n");
  server->sendContent("      <div class='row'>\n");
  server->sendContent("        <div class='col-md-12'>\n");
  server->sendContent("          <h3>Codes Transmitted</h3>\n");
  server->sendContent("          <table class='table table-striped' style='table-layout: fixed;'>\n");
  server->sendContent("            <thead><tr><th>Sent</th><th>Command</th><th>Type</th><th>Length</th><th>Address</th><th>Repeat</th><th>Out</th></tr></thead>\n"); // Added Repeat/Out
  server->sendContent("            <tbody>\n");
  // Helper to generate table row for sent codes
  auto generateSentRow = [&](const Code& code) {
      if (code.valid) {
          server->sendContent("              <tr class='text-uppercase'><td>" + epochToString(code.timestamp) + "</td><td><code>" + String(code.data) + "</code></td><td><code>" + String(code.encoding) + "</code></td><td><code>" + String(code.bits) + "</code></td><td><code>" + String(code.address) + "</code></td><td><code>" + String(code.repeat) + "</code></td><td><code>" + String(code.out) + "</code></td></tr>\n");
      }
  };
  generateSentRow(last_send);
  generateSentRow(last_send_2);
  generateSentRow(last_send_3);
  generateSentRow(last_send_4);
  generateSentRow(last_send_5);
  if (!last_send.valid && !last_send_2.valid && !last_send_3.valid && !last_send_4.valid && !last_send_5.valid)
  server->sendContent("              <tr><td colspan='7' class='text-center'><em>No codes sent</em></td></tr>"); // Colspan updated to 7
  server->sendContent("            </tbody></table>\n");
  server->sendContent("          </div></div>\n");
  server->sendContent("      <div class='row'>\n");
  server->sendContent("        <div class='col-md-12'>\n");
  server->sendContent("          <h3>Codes Received</h3>\n");
  server->sendContent("          <table class='table table-striped' style='table-layout: fixed;'>\n");
  server->sendContent("            <thead><tr><th>Received</th><th>Command</th><th>Type</th><th>Length</th><th>Address</th></tr></thead>\n"); //Title
  server->sendContent("            <tbody>\n");
  if (last_recv.valid)
  server->sendContent("              <tr class='text-uppercase'><td><a href='/received?id=1'>" + epochToString(last_recv.timestamp) + "</a></td><td><code>" + String(last_recv.data) + "</code></td><td><code>" + String(last_recv.encoding) + "</code></td><td><code>" + String(last_recv.bits) + "</code></td><td><code>" + String(last_recv.address) + "</code></td></tr>\n");
  if (last_recv_2.valid)
  server->sendContent("              <tr class='text-uppercase'><td><a href='/received?id=2'>" + epochToString(last_recv_2.timestamp) + "</a></td><td><code>" + String(last_recv_2.data) + "</code></td><td><code>" + String(last_recv_2.encoding) + "</code></td><td><code>" + String(last_recv_2.bits) + "</code></td><td><code>" + String(last_recv_2.address) + "</code></td></tr>\n");
  if (last_recv_3.valid)
  server->sendContent("              <tr class='text-uppercase'><td><a href='/received?id=3'>" + epochToString(last_recv_3.timestamp) + "</a></td><td><code>" + String(last_recv_3.data) + "</code></td><td><code>" + String(last_recv_3.encoding) + "</code></td><td><code>" + String(last_recv_3.bits) + "</code></td><td><code>" + String(last_recv_3.address) + "</code></td></tr>\n");
  if (last_recv_4.valid)
  server->sendContent("              <tr class='text-uppercase'><td><a href='/received?id=4'>" + epochToString(last_recv_4.timestamp) + "</a></td><td><code>" + String(last_recv_4.data) + "</code></td><td><code>" + String(last_recv_4.encoding) + "</code></td><td><code>" + String(last_recv_4.bits) + "</code></td><td><code>" + String(last_recv_4.address) + "</code></td></tr>\n");
  if (last_recv_5.valid)
  server->sendContent("              <tr class='text-uppercase'><td><a href='/received?id=5'>" + epochToString(last_recv_5.timestamp) + "</a></td><td><code>" + String(last_recv_5.data) + "</code></td><td><code>" + String(last_recv_5.encoding) + "</code></td><td><code>" + String(last_recv_5.bits) + "</code></td><td><code>" + String(last_recv_5.address) + "</code></td></tr>\n");
  if (!last_recv.valid && !last_recv_2.valid && !last_recv_3.valid && !last_recv_4.valid && !last_recv_5.valid)
  server->sendContent("              <tr><td colspan='5' class='text-center'><em>No codes received</em></td></tr>");
  server->sendContent("            </tbody></table>\n");
  server->sendContent("          </div></div><hr />\n"); // Moved HR here

// +++ KORRIGIERTES FORMULAR ZUM SENDEN +++
server->sendContent("      <div class='row'>\n");
server->sendContent("        <div class='col-md-12'>\n");
server->sendContent("          <h3>Send IR Code</h3>\n");
server->sendContent("          <form class='form-horizontal' action='/sendir' method='post'>\n");

// --- Hilfsvariablen für Pre-Filling ---
// Prüft, ob last_send gültig ist und weist entweder den letzten Wert oder einen Standardwert zu.
// --- KORREKTUR für toLowerCase() ---
String tempEncoding = "nec"; // Default
if (last_send.valid) {
    tempEncoding = String(last_send.encoding); // Zuerst kopieren
    tempEncoding.toLowerCase(); // Dann die Kopie ändern
}
String lastEncoding = tempEncoding; // Die (ggf. geänderte) Kopie zuweisen
// --- ENDE KORREKTUR ---
String lastData = last_send.valid ? String(last_send.data) : "";
String lastBits = last_send.valid ? String(last_send.bits) : "";
String lastAddress = last_send.valid ? String(last_send.address) : "";
// Optional: "0x" vom Adress-String entfernen für die Anzeige
// if (lastAddress.startsWith("0x")) { lastAddress = lastAddress.substring(2); }
String lastRepeat = last_send.valid ? String(last_send.repeat) : "1"; // Default 1
String lastOut = last_send.valid ? String(last_send.out) : "1";       // Default 1
// --- Ende Hilfsvariablen ---

// --- Lambda-Funktionen NACH den Variablen definieren ---
auto addSelected = [&](const String& val) {
    return (val.equalsIgnoreCase(lastEncoding)) ? " selected" : "";
};
auto addOutSelected = [&](const String& val) {
    return (val == lastOut) ? " selected" : "";
};
// --- Ende Lambda-Funktionen ---


// --- Encoding Type (Dropdown) ---
server->sendContent("            <div class='form-group'>\n");
server->sendContent("              <label for='type' class='col-sm-2 control-label'>Type</label>\n");
server->sendContent("              <div class='col-sm-10'>\n");
server->sendContent("                <select class='form-control' id='type' name='type'>\n");
// --- KORREKTUR: String(...) verwenden, um Verkettung zu ermöglichen ---
server->sendContent(String("                  <option value='nec'") + addSelected("nec") + ">NEC</option>\n");
server->sendContent(String("                  <option value='sony'") + addSelected("sony") + ">SONY</option>\n");
server->sendContent(String("                  <option value='rc5'") + addSelected("rc5") + ">RC5</option>\n");
server->sendContent(String("                  <option value='rc6'") + addSelected("rc6") + ">RC6</option>\n");
server->sendContent(String("                  <option value='panasonic'") + addSelected("panasonic") + ">PANASONIC</option>\n");
server->sendContent(String("                  <option value='lg'") + addSelected("lg") + ">LG</option>\n");
server->sendContent(String("                  <option value='jvc'") + addSelected("jvc") + ">JVC</option>\n");
server->sendContent(String("                  <option value='samsung'") + addSelected("samsung") + ">SAMSUNG</option>\n");
server->sendContent(String("                  <option value='whynter'") + addSelected("whynter") + ">WHYNTER</option>\n");
server->sendContent(String("                  <option value='coolix'") + addSelected("coolix") + ">COOLIX</option>\n");
server->sendContent(String("                  <option value='denon'") + addSelected("denon") + ">DENON</option>\n");
server->sendContent(String("                  <option value='sharp'") + addSelected("sharp") + ">SHARP</option>\n");
server->sendContent(String("                  <option value='sharpraw'") + addSelected("sharpraw") + ">SHARPRAW</option>\n");
server->sendContent(String("                  <option value='dish'") + addSelected("dish") + ">DISH</option>\n");
server->sendContent(String("                  <option value='gree'") + addSelected("gree") + ">GREE</option>\n");
server->sendContent(String("                  <option value='lutron'") + addSelected("lutron") + ">LUTRON</option>\n");
server->sendContent(String("                  <option value='roomba'") + addSelected("roomba") + ">ROOMBA</option>\n");
server->sendContent(String("                  <option value='ecoclim'") + addSelected("ecoclim") + ">ECOCLIM</option>\n");
// Füge hier weitere Typen hinzu, falls nötig. Stelle sicher, dass der 'value' mit dem in last_send.encoding gespeicherten String übereinstimmt.
server->sendContent("                </select>\n");
server->sendContent("              </div>\n");
server->sendContent("            </div>\n");

// --- Data (Hex String) ---
server->sendContent("            <div class='form-group'>\n");
server->sendContent("              <label for='data' class='col-sm-2 control-label'>Data (Hex)</label>\n");
// Fügt das 'value' Attribut mit dem letzten gesendeten Wert hinzu.
server->sendContent("              <div class='col-sm-10'><input type='text' class='form-control' id='data' name='data' placeholder='e.g., FF02FD' required value='" + lastData + "'></div>\n");
server->sendContent("            </div>\n");

// --- Length (Bits) ---
server->sendContent("            <div class='form-group'>\n");
server->sendContent("              <label for='length' class='col-sm-2 control-label'>Length (Bits)</label>\n");
// Fügt das 'value' Attribut mit dem letzten gesendeten Wert hinzu.
server->sendContent("              <div class='col-sm-10'><input type='number' class='form-control' id='length' name='length' placeholder='e.g., 32' required value='" + lastBits + "'></div>\n");
server->sendContent("            </div>\n");

// --- Address (Hex String, optional) ---
server->sendContent("            <div class='form-group'>\n");
server->sendContent("              <label for='address' class='col-sm-2 control-label'>Address (Hex, optional)</label>\n");
// Fügt das 'value' Attribut mit dem letzten gesendeten Wert hinzu.
server->sendContent("              <div class='col-sm-10'><input type='text' class='form-control' id='address' name='address' placeholder='e.g., 0x404 (for Panasonic)' value='" + lastAddress + "'></div>\n");
server->sendContent("            </div>\n");

// --- Repeat ---
server->sendContent("            <div class='form-group'>\n");
server->sendContent("              <label for='repeat' class='col-sm-2 control-label'>Repeat</label>\n");
// Fügt das 'value' Attribut mit dem letzten gesendeten Wert hinzu. min='1' stellt sicher, dass der Wert mindestens 1 ist.
server->sendContent("              <div class='col-sm-10'><input type='number' class='form-control' id='repeat' name='repeat' value='" + lastRepeat + "' min='1'></div>\n");
server->sendContent("            </div>\n");

// --- Output Pin ---
server->sendContent("            <div class='form-group'>\n");
server->sendContent("              <label for='out' class='col-sm-2 control-label'>Output Pin</label>\n");
server->sendContent("              <div class='col-sm-10'>\n");
server->sendContent("                 <select class='form-control' id='out' name='out'>\n");
// --- KORREKTUR: String(...) verwenden, um Verkettung zu ermöglichen ---
// Beachte: String(pinsX) ist bereits ein String-Objekt, daher funktioniert die Verkettung hier.
server->sendContent(String("                   <option value='1'") + addOutSelected("1") + ">1 (GPIO " + String(pins1) + ")</option>\n");
server->sendContent(String("                   <option value='2'") + addOutSelected("2") + ">2 (GPIO " + String(pins2) + ")</option>\n");
server->sendContent(String("                   <option value='3'") + addOutSelected("3") + ">3 (GPIO " + String(pins3) + ")</option>\n");
server->sendContent(String("                   <option value='4'") + addOutSelected("4") + ">4 (GPIO " + String(pins4) + ")</option>\n");
server->sendContent("                 </select>\n");
server->sendContent("              </div>\n");
server->sendContent("            </div>\n");

// --- Submit Button ---
server->sendContent("            <div class='form-group'>\n");
server->sendContent("              <div class='col-sm-offset-2 col-sm-10'>\n");
server->sendContent("                <button type='submit' class='btn btn-primary'>Send IR Code</button>\n");
server->sendContent("              </div>\n");
server->sendContent("            </div>\n");

server->sendContent("          </form>\n");
server->sendContent("        </div>\n");
server->sendContent("      </div><hr />\n"); // Trennlinie vor den Pin-Infos
// +++ ENDE KORRIGIERTES FORMULAR +++


  server->sendContent("      <div class='row'>\n");
  server->sendContent("        <div class='col-md-12'>\n");
  server->sendContent("          <ul class='list-unstyled'>\n");
  server->sendContent("            <li><span class='badge'>GPIO " + String(pinr1) + "</span> Receiving </li>\n");
  server->sendContent("            <li><span class='badge'>GPIO " + String(pins1) + "</span> Transmitter 1 </li>\n");
  server->sendContent("            <li><span class='badge'>GPIO " + String(pins2) + "</span> Transmitter 2 </li>\n");
  server->sendContent("            <li><span class='badge'>GPIO " + String(pins3) + "</span> Transmitter 3 </li>\n");
  server->sendContent("            <li><span class='badge'>GPIO " + String(pins4) + "</span> Transmitter 4 </li></ul>\n");
  server->sendContent("        </div>\n");
  server->sendContent("      </div>\n");

      // +++ JAVASCRIPT FÜR REMOTE BUTTONS (am Ende vor sendFooter()) +++
      server->sendContent("      <script>\n");
      server->sendContent("        document.getElementById('remote-buttons').addEventListener('click', function(event) {\n");
      server->sendContent("          if (event.target.classList.contains('remote-button')) {\n");
      server->sendContent("            event.preventDefault();\n");
      server->sendContent("            const button = event.target;\n");
      server->sendContent("            const isMacro = button.dataset.isMacro === 'true';\n"); // Prüfe das neue Attribut
      
      // Visuelles Feedback (optional)
      server->sendContent("            button.classList.add('btn-warning'); \n");
      server->sendContent("            setTimeout(() => { button.classList.remove('btn-warning'); }, 500);\n");
      
      server->sendContent("            if (isMacro) {\n"); // Wenn es ein Makro ist
      server->sendContent("              const macroJsonString = button.dataset.macroJson;\n");
      // Korrektur: HTML-Entities wieder in Zeichen umwandeln, bevor sie gesendet werden
      server->sendContent("              const tempElem = document.createElement('textarea'); tempElem.innerHTML = macroJsonString; const decodedJson = tempElem.value;\n");
      server->sendContent("              console.log('Sending Macro JSON:', decodedJson);\n");
      server->sendContent("              fetch('/json?pass=" + String(passcode) + "', { // Füge ggf. Passcode hinzu\n"); // Sende an /json
      server->sendContent("                method: 'POST',\n");
      server->sendContent("                headers: { 'Content-Type': 'text/plain' },\n"); // /json erwartet 'plain'
      server->sendContent("                body: decodedJson\n"); // Sende den JSON-String als Body
      server->sendContent("              })\n");
      server->sendContent("              .then(response => {\n");
      server->sendContent("                if (!response.ok) { console.error('Error sending macro command:', response.statusText); button.classList.add('btn-danger'); setTimeout(() => { button.classList.remove('btn-danger'); }, 1000); }\n");
      server->sendContent("                return response.text();\n");
      server->sendContent("              })\n");
      server->sendContent("              .then(data => console.log('Server response (macro):', data))\n");
      server->sendContent("              .catch(error => { console.error('Fetch error (macro):', error); button.classList.add('btn-danger'); setTimeout(() => { button.classList.remove('btn-danger'); }, 1000); });\n");
      server->sendContent("            } else {\n"); // Andernfalls (Single Code, wie bisher)
      server->sendContent("              const irData = {\n");
      server->sendContent("                type: button.dataset.type,\n");
      server->sendContent("                data: button.dataset.data,\n");
      server->sendContent("                length: parseInt(button.dataset.length, 10),\n");
      server->sendContent("                address: button.dataset.address,\n");
      server->sendContent("                repeat: parseInt(button.dataset.repeat, 10),\n");
      server->sendContent("                out: parseInt(button.dataset.out, 10)\n");
      server->sendContent("              };\n");
      server->sendContent("              console.log('Sending Single IR:', irData);\n");
      server->sendContent("              fetch('/sendbutton', {\n"); // Sende an /sendbutton
      server->sendContent("                method: 'POST',\n");
      server->sendContent("                headers: { 'Content-Type': 'application/json' },\n");
      server->sendContent("                body: JSON.stringify(irData)\n");
      server->sendContent("              })\n");
      server->sendContent("              .then(response => {\n");
      server->sendContent("                if (!response.ok) { console.error('Error sending IR command'); button.classList.add('btn-danger'); setTimeout(() => { button.classList.remove('btn-danger'); }, 1000); }\n");
      server->sendContent("                return response.text();\n");
      server->sendContent("              })\n");
      server->sendContent("              .then(data => console.log('Server response (single):', data))\n");
      server->sendContent("              .catch(error => { console.error('Fetch error (single):', error); button.classList.add('btn-danger'); setTimeout(() => { button.classList.remove('btn-danger'); }, 1000); });\n");
      server->sendContent("            }\n"); // Ende else (Single Code)
      server->sendContent("          }\n");
      server->sendContent("        });\n");
      server->sendContent("      </script>\n");
      // +++ ENDE JAVASCRIPT +++
  
      
  sendFooter();
}

//+=============================================================================
// Stream code page HTML
//
void sendCodePage(Code selCode) {
  sendCodePage(selCode, 200);
}

void sendCodePage(Code selCode, int httpcode){
  sendHeader(httpcode);
  server->sendContent("      <div class='row'>\n");
  server->sendContent("        <div class='col-md-12'>\n");
  server->sendContent("          <h2><span class='label label-success'>" + String(selCode.data) + ":" + String(selCode.encoding) + ":" + String(selCode.bits) + "</span></h2><br/>\n");
  server->sendContent("          <dl class='dl-horizontal'>\n");
  server->sendContent("            <dt>Data</dt>\n");
  server->sendContent("            <dd><code>" + String(selCode.data)  + "</code></dd></dl>\n");
  server->sendContent("          <dl class='dl-horizontal'>\n");
  server->sendContent("            <dt>Type</dt>\n");
  server->sendContent("            <dd><code>" + String(selCode.encoding)  + "</code></dd></dl>\n");
  server->sendContent("          <dl class='dl-horizontal'>\n");
  server->sendContent("            <dt>Length</dt>\n");
  server->sendContent("            <dd><code>" + String(selCode.bits)  + "</code></dd></dl>\n");
  server->sendContent("          <dl class='dl-horizontal'>\n");
  server->sendContent("            <dt>Address</dt>\n");
  server->sendContent("            <dd><code>" + String(selCode.address)  + "</code></dd></dl>\n");
  server->sendContent("          <dl class='dl-horizontal'>\n");
  server->sendContent("            <dt>Raw</dt>\n");
  server->sendContent("            <dd><code>" + String(selCode.raw)  + "</code></dd></dl>\n");
  server->sendContent("          <dl class='dl-horizontal'>\n");
  server->sendContent("            <dt>Timestamp</dt>\n");
  server->sendContent("            <dd><code>" + epochToString(selCode.timestamp)  + "</code></dd></dl>\n");
  server->sendContent("        </div></div>\n");
  server->sendContent("      <div class='row'>\n");
  server->sendContent("        <div class='col-md-12'>\n");
  server->sendContent("          <div class='alert alert-warning'>Don't forget to add your passcode to the URLs below if you set one</div>\n");
  server->sendContent("      </div></div>\n");
  if (String(selCode.encoding) == "UNKNOWN") {
  server->sendContent("      <div class='row'>\n");
  server->sendContent("        <div class='col-md-12'>\n");
  server->sendContent("          <ul class='list-unstyled'>\n");
  server->sendContent("            <li>Hostname <span class='label label-default'>JSON</span></li>\n");
  server->sendContent("            <li><pre>http://" + String(host_name) + ".local:" + String(port) + "/json?plain=[{data:[" + String(selCode.raw) + "],type:'raw',khz:38}]</pre></li>\n");
  server->sendContent("            <li>Local IP <span class='label label-default'>JSON</span></li>\n");
  server->sendContent("            <li><pre>http://" + WiFi.localIP().toString() + ":" + String(port) + "/json?plain=[{data:[" + String(selCode.raw) + "],type:'raw',khz:38}]</pre></li>\n");
  server->sendContent("            <li>External IP <span class='label label-default'>JSON</span></li>\n");
  server->sendContent("            <li><pre>http://" + externalIP() + ":" + String(port) + "/json?plain=[{data:[" + String(selCode.raw) + "],type:'raw',khz:38}]</pre></li></ul>\n");
  } else if (String(selCode.encoding) == "PANASONIC" || String(selCode.encoding) == "NEC") {
  //} else if (strtoul(selCode.address, 0, 0) > 0) {
  server->sendContent("      <div class='row'>\n");
  server->sendContent("        <div class='col-md-12'>\n");
  server->sendContent("          <ul class='list-unstyled'>\n");
  server->sendContent("            <li>Hostname <span class='label label-default'>MSG</span></li>\n");
  server->sendContent("            <li><pre>http://" + String(host_name) + ".local:" + String(port) + "/msg?code=" + String(selCode.data) + ":" + String(selCode.encoding) + ":" + String(selCode.bits) + "&address=" + String(selCode.address) + "</pre></li>\n");
  server->sendContent("            <li>Local IP <span class='label label-default'>MSG</span></li>\n");
  server->sendContent("            <li><pre>http://" + WiFi.localIP().toString() + ":" + String(port) + "/msg?code=" + String(selCode.data) + ":" + String(selCode.encoding) + ":" + String(selCode.bits) + "&address=" + String(selCode.address) + "</pre></li>\n");
  server->sendContent("            <li>External IP <span class='label label-default'>MSG</span></li>\n");
  server->sendContent("            <li><pre>http://" + externalIP() + ":" + String(port) + "/msg?code=" + selCode.data + ":" + String(selCode.encoding) + ":" + String(selCode.bits) + "&address=" + String(selCode.address) + "</pre></li></ul>\n");
  server->sendContent("          <ul class='list-unstyled'>\n");
  server->sendContent("            <li>Hostname <span class='label label-default'>JSON</span></li>\n");
  server->sendContent("            <li><pre>http://" + String(host_name) + ".local:" + String(port) + "/json?plain=[{data:'" + String(selCode.data) + "',type:'" + String(selCode.encoding) + "',length:" + String(selCode.bits) + ",address:'" + String(selCode.address) + "'}]</pre></li>\n");
  server->sendContent("            <li>Local IP <span class='label label-default'>JSON</span></li>\n");
  server->sendContent("            <li><pre>http://" + WiFi.localIP().toString() + ":" + String(port) + "/json?plain=[{data:'" + String(selCode.data) + "',type:'" + String(selCode.encoding) + "',length:" + String(selCode.bits) + ",address:'" + String(selCode.address) + "'}]</pre></li>\n");
  server->sendContent("            <li>External IP <span class='label label-default'>JSON</span></li>\n");
  server->sendContent("            <li><pre>http://" + externalIP() + ":" + String(port) + "/json?plain=[{data:'" + String(selCode.data) + "',type:'" + String(selCode.encoding) + "',length:" + String(selCode.bits) + ",address:'" + String(selCode.address) + "'}]</pre></li></ul>\n");
  } else {
  server->sendContent("      <div class='row'>\n");
  server->sendContent("        <div class='col-md-12'>\n");
  server->sendContent("          <ul class='list-unstyled'>\n");
  server->sendContent("            <li>Hostname <span class='label label-default'>MSG</span></li>\n");
  server->sendContent("            <li><pre>http://" + String(host_name) + ".local:" + String(port) + "/msg?code=" + String(selCode.data) + ":" + String(selCode.encoding) + ":" + String(selCode.bits) + "</pre></li>\n");
  server->sendContent("            <li>Local IP <span class='label label-default'>MSG</span></li>\n");
  server->sendContent("            <li><pre>http://" + WiFi.localIP().toString() + ":" + String(port) + "/msg?code=" + String(selCode.data) + ":" + String(selCode.encoding) + ":" + String(selCode.bits) + "</pre></li>\n");
  server->sendContent("            <li>External IP <span class='label label-default'>MSG</span></li>\n");
  server->sendContent("            <li><pre>http://" + externalIP() + ":" + String(port) + "/msg?code=" + selCode.data + ":" + String(selCode.encoding) + ":" + String(selCode.bits) + "</pre></li></ul>\n");
  server->sendContent("          <ul class='list-unstyled'>\n");
  server->sendContent("            <li>Hostname <span class='label label-default'>JSON</span></li>\n");
  server->sendContent("            <li><pre>http://" + String(host_name) + ".local:" + String(port) + "/json?plain=[{data:'" + String(selCode.data) + "',type:'" + String(selCode.encoding) + "',length:" + String(selCode.bits) + "}]</pre></li>\n");
  server->sendContent("            <li>Local IP <span class='label label-default'>JSON</span></li>\n");
  server->sendContent("            <li><pre>http://" + WiFi.localIP().toString() + ":" + String(port) + "/json?plain=[{data:'" + String(selCode.data) + "',type:'" + String(selCode.encoding) + "',length:" + String(selCode.bits) + "}]</pre></li>\n");
  server->sendContent("            <li>External IP <span class='label label-default'>JSON</span></li>\n");
  server->sendContent("            <li><pre>http://" + externalIP() + ":" + String(port) + "/json?plain=[{data:'" + String(selCode.data) + "',type:'" + String(selCode.encoding) + "',length:" + String(selCode.bits) + "}]</pre></li></ul>\n");
  }
  server->sendContent("        </div>\n");
  server->sendContent("     </div>\n");


    // +++ JAVASCRIPT FÜR REMOTE BUTTONS (am Ende vor sendFooter()) +++
    server->sendContent("      <script>\n");
    server->sendContent("        document.getElementById('remote-buttons').addEventListener('click', function(event) {\n");
    server->sendContent("          if (event.target.classList.contains('remote-button')) {\n");
    server->sendContent("            event.preventDefault();\n");
    server->sendContent("            const button = event.target;\n");
    server->sendContent("            const irData = {\n");
    server->sendContent("              type: button.dataset.type,\n");
    server->sendContent("              data: button.dataset.data,\n");
    server->sendContent("              length: parseInt(button.dataset.length, 10),\n");
    server->sendContent("              address: button.dataset.address,\n");
    server->sendContent("              repeat: parseInt(button.dataset.repeat, 10),\n");
    server->sendContent("              out: parseInt(button.dataset.out, 10)\n");
    server->sendContent("            };\n");
    server->sendContent("            console.log('Sending IR:', irData);\n");
    // Visuelles Feedback (optional)
    server->sendContent("            button.classList.add('btn-warning'); \n");
    server->sendContent("            setTimeout(() => { button.classList.remove('btn-warning'); }, 500);\n");
  
    server->sendContent("            fetch('/sendbutton', {\n");
    server->sendContent("              method: 'POST',\n");
    server->sendContent("              headers: {\n");
    server->sendContent("                'Content-Type': 'application/json'\n");
    // Optional: Wenn Passcode/Auth benötigt wird, hier hinzufügen
    // server->sendContent("                'Authorization': 'Bearer your_token_or_passcode'\n");
    server->sendContent("              },\n");
    server->sendContent("              body: JSON.stringify(irData)\n");
    server->sendContent("            })\n");
    server->sendContent("            .then(response => {\n");
    server->sendContent("              if (!response.ok) { console.error('Error sending IR command'); button.classList.add('btn-danger'); setTimeout(() => { button.classList.remove('btn-danger'); }, 1000); }\n");
    server->sendContent("              return response.text();\n");
    server->sendContent("            })\n");
    server->sendContent("            .then(data => console.log('Server response:', data))\n");
    server->sendContent("            .catch(error => { console.error('Fetch error:', error); button.classList.add('btn-danger'); setTimeout(() => { button.classList.remove('btn-danger'); }, 1000); });\n");
    server->sendContent("          }\n");
    server->sendContent("        });\n");
    server->sendContent("      </script>\n");
    // +++ ENDE JAVASCRIPT +++
  
    
  sendFooter();
}

//+=============================================================================
// Code to JsonObject
//
void cvrtCode(Code& codeData, decode_results *results) {
  strncpy(codeData.data, uint64ToString(results->value, 16).c_str(), 40);
  strncpy(codeData.encoding, encoding(results).c_str(), 14);
  codeData.bits = results->bits;
  String r = "";
      for (uint16_t i = 1; i < results->rawlen; i++) {
      r += results->rawbuf[i] * kRawTick;
      if (i < results->rawlen - 1)
        r += ",";                           // ',' not needed on last one
      //if (!(i & 1)) r += " ";
    }
  codeData.raw = r;
  if (results->decode_type != UNKNOWN) {
    strncpy(codeData.address, ("0x" + String(results->address, HEX)).c_str(), 20);
    strncpy(codeData.command, ("0x" + String(results->command, HEX)).c_str(), 40);
  } else {
    strncpy(codeData.address, "0x0", 20);
    strncpy(codeData.command, "0x0", 40);
  }
}

//+=============================================================================
// Dump out the decode_results structure.
//
void dumpInfo(decode_results *results) {
  if (results->overflow)
    Serial.println("WARNING: IR code too long. "
                   "Edit IRrecv.h and increase RAWBUF");

  // Show Encoding standard
  Serial.print("Encoding  : ");
  Serial.print(encoding(results));
  Serial.println("");

  // Show Code & length
  Serial.print("Code      : ");
  serialPrintUint64(results->value, 16);
  Serial.print(" (");
  Serial.print(results->bits, DEC);
  Serial.println(" bits)");
}


//+=============================================================================
// Dump out the decode_results structure.
//
void dumpRaw(decode_results *results) {
  // Print Raw data
  Serial.print("Timing[");
  Serial.print(results->rawlen - 1, DEC);
  Serial.println("]: ");

  for (uint16_t i = 1;  i < results->rawlen;  i++) {
    if (i % 100 == 0)
      yield();  // Preemptive yield every 100th entry to feed the WDT.
    uint32_t x = results->rawbuf[i] * kRawTick;
    if (!(i & 1)) {  // even
      Serial.print("-");
      if (x < 1000) Serial.print(" ");
      if (x < 100) Serial.print(" ");
      Serial.print(x, DEC);
    } else {  // odd
      Serial.print("     ");
      Serial.print("+");
      if (x < 1000) Serial.print(" ");
      if (x < 100) Serial.print(" ");
      Serial.print(x, DEC);
      if (i < results->rawlen - 1)
        Serial.print(", ");  // ',' not needed for last one
    }
    if (!(i % 8)) Serial.println("");
  }
  Serial.println("");  // Newline
}


//+=============================================================================
// Dump out the decode_results structure.
//
void dumpCode(decode_results *results) {
  // Start declaration
  Serial.print("uint16_t  ");              // variable type
  Serial.print("rawData[");                // array name
  Serial.print(results->rawlen - 1, DEC);  // array size
  Serial.print("] = {");                   // Start declaration

  // Dump data
  for (uint16_t i = 1; i < results->rawlen; i++) {
    Serial.print(results->rawbuf[i] * kRawTick, DEC);
    if (i < results->rawlen - 1)
      Serial.print(",");  // ',' not needed on last one
    if (!(i & 1)) Serial.print(" ");
  }

  // End declaration
  Serial.print("};");  //

  // Comment
  Serial.print("  // ");
  Serial.print(encoding(results));
  Serial.print(" ");
  serialPrintUint64(results->value, 16);

  // Newline
  Serial.println("");

  // Now dump "known" codes
  if (results->decode_type != UNKNOWN) {
    // Some protocols have an address &/or command.
    // NOTE: It will ignore the atypical case when a message has been decoded
    // but the address & the command are both 0.
    if (results->address > 0 || results->command > 0) {
      Serial.print("uint32_t  address = 0x");
      Serial.print(results->address, HEX);
      Serial.println(";");
      Serial.print("uint32_t  command = 0x");
      Serial.print(results->command, HEX);
      Serial.println(";");
    }

    // All protocols have data
    Serial.print("uint64_t  data = 0x");
    serialPrintUint64(results->value, 16);
    Serial.println(";");
  }
}


//+=============================================================================
// Binary value to hex
//
String bin2hex(const uint8_t* bin, const int length) {
  String hex = "";

  for (int i = 0; i < length; i++) {
    if (bin[i] < 16) {
      hex += "0";
    }
    hex += String(bin[i], HEX);
  }

  return hex;
}


//+=============================================================================
// Send IR codes to variety of sources
//
// OLD: void irblast(String type, String dataStr, unsigned int len, int rdelay, int pulse, int pdelay, int repeat, long address, IRsend irsend) {
// NEW: Add the 'out_pin' parameter
void irblast(String type, String dataStr, unsigned int len, int rdelay, int pulse, int pdelay, int repeat, long address, IRsend irsend, int out_pin) {
  Serial.println("Blasting off");
  type.toLowerCase();
  uint64_t data = strtoull(("0x" + dataStr).c_str(), 0, 0);
  holdReceive = true;
  Serial.println("Blocking incoming IR signals");
  // Repeat Loop
  for (int r = 0; r < repeat; r++) {
    // Pulse Loop
    for (int p = 0; p < pulse; p++) {
      serialPrintUint64(data, HEX);
      Serial.print(":");
      Serial.print(type);
      Serial.print(":");
      Serial.println(len);
      if (type == "nec") {
        irsend.sendNEC(data, len);
      } else if (type == "sony") {
        irsend.sendSony(data, len);
      } else if (type == "coolix") {
        irsend.sendCOOLIX(data, len);
      } else if (type == "whynter") {
        irsend.sendWhynter(data, len);
      } else if (type == "panasonic") {
        Serial.print("Address: ");
        Serial.println(address);
        irsend.sendPanasonic(address, data);
      } else if (type == "jvc") {
        irsend.sendJVC(data, len, 0);
      } else if (type == "samsung") {
        irsend.sendSAMSUNG(data, len);
      } else if (type == "sharpraw") {
        irsend.sendSharpRaw(data, len);
      } else if (type == "dish") {
        irsend.sendDISH(data, len);
      } else if (type == "rc5") {
        irsend.sendRC5(_rc5toggle ? data: irsend.toggleRC5(data), len);
      } else if (type == "rc6") {
        irsend.sendRC6(_rc6toggle ? data: irsend.toggleRC6(data, len), len);
      } else if (type == "denon") {
        irsend.sendDenon(data, len);
      } else if (type == "lg") {
        irsend.sendLG(data, len);
      } else if (type == "sharp") {
        irsend.sendSharpRaw(data, len);
      } else if (type == "rcmm") {
        irsend.sendRCMM(data, len);
      } else if (type == "gree") {
        irsend.sendGree(data, len);
      } else if (type == "lutron") {
        irsend.sendLutron(data, len);
      } else if (type == "roomba") {
        roomba_send(atoi(dataStr.c_str()), pulse, pdelay, irsend);
      } else if (type == "ecoclim") {
        irsend.sendEcoclim(data, len);
      }
      if (p + 1 < pulse) delay(pdelay);
    }
    if (r + 1 < repeat) delay(rdelay);

    if (toggleRC) {
      if (type == "rc5") { _rc5toggle = !_rc5toggle; }
      if (type == "rc6") { _rc6toggle = !_rc6toggle; }
    }
  }

  Serial.println("Transmission complete");

  copyCode(last_send_4, last_send_5);
  copyCode(last_send_3, last_send_4);
  copyCode(last_send_2, last_send_3);
  copyCode(last_send, last_send_2);

  strncpy(last_send.data, dataStr.c_str(), 40);
  last_send.bits = len;
  strncpy(last_send.encoding, type.c_str(), 14);
  strncpy(last_send.address, ("0x" + String(address, HEX)).c_str(), 20);
  last_send.timestamp = now();
  last_send.valid = true;

  // +++ NEUE ZEILEN +++
  last_send.repeat = repeat;
  last_send.out = out_pin;
  // +++ ENDE NEUE ZEILEN +++

  resetReceive();
}

// OLD: void pronto(JsonArray &pronto, int rdelay, int pulse, int pdelay, int repeat, IRsend irsend) {
// NEW: Add 'out_pin' parameter
void pronto(JsonArray &pronto, int rdelay, int pulse, int pdelay, int repeat, IRsend irsend, int out_pin) {
  // ... (rest of the function remains the same until the end)
  Serial.println("Pronto transmit");
  holdReceive = true;
  Serial.println("Blocking incoming IR signals");
  int psize = pronto.size();
  // Repeat Loop
  for (int r = 0; r < repeat; r++) {
    // Pulse Loop
    for (int p = 0; p < pulse; p++) {
      Serial.println("Sending pronto code");
      uint16_t output[psize];
      for (int d = 0; d < psize; d++) {
        String phexp = pronto[d];
        output[d] = strtoul(phexp.c_str(), 0, 0);
      }
      irsend.sendPronto(output, psize);
      if (p + 1 < pulse) delay(pdelay);
    }
    if (r + 1 < repeat) delay(rdelay);
  }
  Serial.println("Transmission complete");

  copyCode(last_send_4, last_send_5);
  copyCode(last_send_3, last_send_4);
  copyCode(last_send_2, last_send_3);
  copyCode(last_send, last_send_2);

  strncpy(last_send.data, "", 40);
  last_send.bits = psize;
  strncpy(last_send.encoding, "PRONTO", 14);
  strncpy(last_send.address, "0x0", 20);
  last_send.timestamp = now();
  last_send.valid = true;

  // +++ NEUE ZEILEN +++
  last_send.repeat = repeat;
  last_send.out = out_pin;
  // +++ ENDE NEUE ZEILEN +++

  resetReceive();
}

// OLD: void rawblast(JsonArray &raw, int khz, int rdelay, int pulse, int pdelay, int repeat, IRsend irsend,int duty) {
// NEW: Add 'out_pin' parameter
void rawblast(JsonArray &raw, int khz, int rdelay, int pulse, int pdelay, int repeat, IRsend irsend, int duty, int out_pin) {
  // ... (rest of the function remains the same until the end)
  Serial.println("Raw transmit");
  holdReceive = true;
  Serial.println("Blocking incoming IR signals");
  // Repeat Loop
  for (int r = 0; r < repeat; r++) {
    // Pulse Loop
    for (int p = 0; p < pulse; p++) {
      Serial.println("Sending code");
      irsend.enableIROut(khz,duty);
      for (unsigned int i = 0; i < raw.size(); i++) {
        int val = raw[i];
        if (i & 1) irsend.space(std::max(0, val));
        else       irsend.mark(val);
      }
      irsend.space(0);
      if (p + 1 < pulse) delay(pdelay);
    }
    if (r + 1 < repeat) delay(rdelay);
  }

  Serial.println("Transmission complete");

  copyCode(last_send_4, last_send_5);
  copyCode(last_send_3, last_send_4);
  copyCode(last_send_2, last_send_3);
  copyCode(last_send, last_send_2);

  strncpy(last_send.data, "", 40);
  last_send.bits = raw.size();
  strncpy(last_send.encoding, "RAW", 14);
  strncpy(last_send.address, "0x0", 20);
  last_send.timestamp = now();
  last_send.valid = true;

  // +++ NEUE ZEILEN +++
  last_send.repeat = repeat;
  last_send.out = out_pin;
  // +++ ENDE NEUE ZEILEN +++

  resetReceive();
}


void roomba_send(int code, int pulse, int pdelay, IRsend irsend)
{
  Serial.print("Sending Roomba code ");
  Serial.println(code);
  holdReceive = true;
  Serial.println("Blocking incoming IR signals");

  int length = 8;
  uint16_t raw[length * 2];
  unsigned int one_pulse = 3000;
  unsigned int one_break = 1000;
  unsigned int zero_pulse = one_break;
  unsigned int zero_break = one_pulse;
  uint16_t len = 15;
  uint16_t hz = 38;

  int arrayposition = 0;
  for (int counter = length - 1; counter >= 0; --counter) {
    if (code & (1 << counter)) {
      raw[arrayposition] = one_pulse;
      raw[arrayposition + 1] = one_break;
    }
    else {
      raw[arrayposition] = zero_pulse;
      raw[arrayposition + 1] = zero_break;
    }
    arrayposition = arrayposition + 2;
  }
  for (int i = 0; i < pulse; i++) {
    irsend.sendRaw(raw, len, hz);
    delay(pdelay);
  }

  resetReceive();
}

void copyCode (Code& c1, Code& c2) {
  strncpy(c2.data, c1.data, 40);
  strncpy(c2.encoding, c1.encoding, 14);
  //strncpy(c2.timestamp, c1.timestamp, 40);
  strncpy(c2.address, c1.address, 20);
  strncpy(c2.command, c1.command, 40);
  c2.bits = c1.bits;
  c2.raw = c1.raw;
  c2.timestamp = c1.timestamp;
  c2.valid = c1.valid;
}

void loop() {
  ArduinoOTA.handle();
  server->handleClient();
  decode_results  results;                                        // Somewhere to store the results

  if (irrecv.decode(&results) && !holdReceive) {                  // Grab an IR code
    Serial.println("Signal received:");
    fullCode(&results);                                           // Print the singleline value
    dumpCode(&results);                                           // Output the results as source code
    copyCode(last_recv_4, last_recv_5);                           // Pass
    copyCode(last_recv_3, last_recv_4);                           // Pass
    copyCode(last_recv_2, last_recv_3);                           // Pass
    copyCode(last_recv, last_recv_2);                             // Pass
    cvrtCode(last_recv, &results);                                // Store the results
    last_recv.timestamp = now();                                  // Set the new update time
    last_recv.valid = true;
    Serial.println("");                                           // Blank line between entries
    irrecv.resume();                                              // Prepare for the next value
    digitalWrite(ledpin, LOW);                                    // Turn on the LED for 0.5 seconds
    ticker.attach(0.5, disableLed);
  }
  delay(200);
}
