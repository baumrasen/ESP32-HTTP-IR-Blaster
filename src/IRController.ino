#include <FS.h>                                               // This needs to be first, or it all crashes and burns

#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <WiFi.h>
#include <WiFiManager.h>                                      // https://github.com/tzapu/WiFiManager WiFi Configuration Magic

#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
#include <ArduinoOTA.h>

#include <Ticker.h>                                           // For LED status
#include <TimeLib.h>

#include <LittleFS.h>
#include "esp_ota_ops.h" // Für esp_ota_get_running_partition()

// User settings are below here
//+=============================================================================
const bool getExternalIP = true;                               // Set to false to disable querying external IP

const bool getTime = true;                                     // Set to false to disable querying for the time
const int timeZone = -5;                                       // Timezone (-5 is EST)

const unsigned int captureBufSize = 1024;                      // Size of the IR capture buffer.

const bool toggleRC = true;                                    // Toggle RC signals every other transmission

const uint16_t  pinr1 = 15;                                          // Receiving pin
const uint16_t  pins1 = 13;                                           // Transmitting preset 1
const uint16_t  configpin = 10;                                      // Reset Pin
const uint16_t  pins2 = 5;                                           // Transmitting preset 2
const uint16_t  pins3 = 12;                                          // Transmitting preset 3
const uint16_t  pins4 = 4;                                          // Transmitting preset 4

//+=============================================================================
// User settings are above here

const int ledpin = LED_BUILTIN;                                // Built in LED defined for WEMOS people
const char *wifi_config_name = "IR Controller Configuration";
// const char serverName[] = "checkip.dyndns.org";
int port = 80;
char passcode[20] = "";
char host_name[20] = "";
char port_str[6] = "80";

// Do not modify these values with your own, they are placeholder values that WiFiManager will overwrite
char static_ip[16] = "10.0.1.10";
char static_gw[16] = "10.0.1.1";
char static_sn[16] = "255.255.255.0";
char static_dns[16] = "10.0.1.1";

DynamicJsonDocument deviceState(1024);

WiFiClient client;
AsyncWebServer *server = NULL;
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

bool externalIPError = false;
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
  bool configured = false; // Ist dieser Button-Slot konfiguriert?
};

// ButtonConfig buttonConfigs[MAX_BUTTONS]; // ALT
std::vector<ButtonConfig> buttonConfigs; // NEU
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
  buttonConfigs.clear(); // Vector vor dem Laden leeren
  
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

          // Optional: Limit prüfen, falls MAX_BUTTONS noch verwendet wird
          if (buttonConfigs.size() >= MAX_BUTTONS) {
             Serial.println("Maximum number of buttons reached, ignoring further entries.");
            break;
          }
          
          ButtonConfig newButton; // Temporäres Objekt erstellen

          strncpy(newButton.name, buttonJson["name"] | "", sizeof(newButton.name) - 1);
          strncpy(newButton.type, buttonJson["type"] | "", sizeof(newButton.type) - 1);
          strncpy(newButton.data, buttonJson["data"] | "", sizeof(newButton.data) - 1);
          newButton.length = buttonJson["length"] | 0;
          strncpy(newButton.address, buttonJson["address"] | "", sizeof(newButton.address) - 1);
          newButton.repeat = buttonJson["repeat"] | 1;
          newButton.out = buttonJson["out"] | 1;
          newButton.configured = buttonJson["configured"] | false;

          // Sicherstellen, dass Strings null-terminiert sind
          newButton.name[sizeof(newButton.name) - 1] = '\0';
          newButton.type[sizeof(newButton.type) - 1] = '\0';
          newButton.data[sizeof(newButton.data) - 1] = '\0';
          newButton.address[sizeof(newButton.address) - 1] = '\0';

          // Nur hinzufügen, wenn als konfiguriert markiert UND Name/Daten vorhanden
          if (newButton.configured && strlen(newButton.name) > 0 && strlen(newButton.data) > 0 && newButton.length > 0) {
             buttonConfigs.push_back(newButton); // Zum Vector hinzufügen
          }
        }
        Serial.printf("Loaded %d buttons successfully.\n", buttonConfigs.size());
      } else {
        Serial.print("Failed to parse buttons.json: ");
        Serial.println(error.c_str());
      }
    } else {
      Serial.println("Failed to open buttons.json for reading.");
    }
  } else {
    Serial.println("buttons.json not found. No buttons loaded.");
  }
}

void saveButtonConfig() {
  Serial.println("    ==> saveButtonConfig: Entered function."); // NEU
  if (!LittleFS.begin()) {
    Serial.println("        ERROR: Failed to mount LittleFS!"); // NEU
    return;
  }
  Serial.println("        LittleFS mounted."); // NEU

  DynamicJsonDocument jsonDoc(4096);
  JsonArray buttonArray = jsonDoc.to<JsonArray>();

  Serial.printf("        Serializing %d buttons...\n", buttonConfigs.size()); // NEU
  for (const auto& button : buttonConfigs) {
     Serial.println("          - Serializing: " + String(button.name)); // NEU (Optional)
     JsonObject buttonJson = buttonArray.createNestedObject();
     buttonJson["name"] = button.name;
    buttonJson["type"] = button.type;
    buttonJson["data"] = button.data;
    buttonJson["length"] = button.length;
    buttonJson["address"] = button.address;
    buttonJson["repeat"] = button.repeat;
    buttonJson["out"] = button.out;
    buttonJson["configured"] = true; // Alle im Vector sind konfiguriert
  }

  File configFile = LittleFS.open("/buttons.json", "w");
  if (!configFile) {
    Serial.println("        ERROR: Failed to open buttons.json for writing!"); // NEU
    return;
  }
  Serial.println("        buttons.json opened for writing."); // NEU

  size_t bytesWritten = serializeJson(jsonDoc, configFile); // NEU: Rückgabewert speichern
  if (bytesWritten == 0) {
    Serial.println("        ERROR: Failed to write to buttons.json (serializeJson returned 0)."); // NEU
  } else {
    Serial.printf("        %d bytes written to buttons.json.\n", bytesWritten); // NEU
    Serial.println("        Button config saved successfully.");
  }
  configFile.close();
  Serial.println("    <== saveButtonConfig: Leaving function."); // NEU
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


// //+=============================================================================
// // Get External IP Address
// //
// String externalIP()
// {
//   if (!getExternalIP) {
//     return "0.0.0.0"; // User doesn't want the external IP
//   }

//   if (strlen(_ip) > 0) {
//     unsigned long delta = millis() - lastupdate;
//     if (delta > resetfrequency || lastupdate == 0) {
//       Serial.println("Reseting cached external IP address");
//       strncpy(_ip, "", 16); // Reset the cached external IP every 72 hours
//     } else {
//       return String(_ip); // Return the cached external IP
//     }
//   }

//   HTTPClient http;
//   externalIPError = false;
//   unsigned long start = millis();
//   http.setTimeout(5000);
//   http.begin(serverName, 8245);
//   int httpCode = http.GET();

//   if (httpCode > 0 && httpCode == HTTP_CODE_OK) {
//     String payload = http.getString();
//     int pos_start = payload.indexOf("IP Address") + 12; // add 10 for "IP Address" and 2 for ":" + "space"
//     int pos_end = payload.indexOf("</body>", pos_start); // add nothing
//     strncpy(_ip, payload.substring(pos_start, pos_end).c_str(), 16);
//     Serial.print(F("External IP: "));
//     Serial.println(_ip);
//     lastupdate = millis();
//   } else {
//     Serial.println("Error retrieving external IP");
//     Serial.print("HTTP Code: ");
//     Serial.println(httpCode);
//     Serial.println(http.errorToString(httpCode));
//     externalIPError = true;
//   }

//   http.end();
//   Serial.print("External IP address request took ");
//   Serial.print(millis() - start);
//   Serial.println(" ms");

//   return _ip;
// }


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

  if (LittleFS.begin(true)) {
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
  port = atoi(port_str);

  // --- NEUE PRÜFUNG ---
  port = atoi(port_str);
  if (port <= 0 || port > 65535) { // Prüft auf Fehler bei atoi() oder ungültigen Portbereich
      Serial.print("Warning: Invalid port '");
      Serial.print(port_str);
      Serial.println("' detected. Defaulting to port 80.");
      port = 80; // Setze auf Standardwert 80
      strcpy(port_str, "80"); // Korrigiere auch den String für Konsistenz
  }
  // --- ENDE NEUE PRÜFUNG ---
  
  if (server != NULL) {
    delete server;
  }
  server = new AsyncWebServer(port);

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
// Hilfsfunktion zum Generieren des Type-Dropdowns
//+=============================================================================
String generateTypeDropdownHtml(const String& selectName, const String& selectedValue) {
  String html = "<select class='form-control' id='" + selectName + "' name='" + selectName + "'>\n";
  // Helper innerhalb der Funktion
  auto addSelected = [&](const String& val) { return val.equalsIgnoreCase(selectedValue) ? " selected" : ""; };

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
  // Füge hier weitere Typen hinzu, falls nötig. Sie gelten dann für beide Formulare.
  html += "</select>\n";
  return html;
}

// Optional: Globale Hilfsfunktion für Output-Dropdown (falls noch nicht geschehen)
String generateOutDropdownHtml(const String& selectName, int selectedValue) {
  String html = "<select class='form-control' id='" + selectName + "' name='" + selectName + "'>\n";
  auto addOutSelected = [&](int val) { return (val == selectedValue) ? " selected" : ""; };
  html += String("  <option value='1'") + addOutSelected(1) + ">1 (GPIO " + String(pins1) + ")</option>\n";
  html += String("  <option value='2'") + addOutSelected(2) + ">2 (GPIO " + String(pins2) + ")</option>\n";
  html += String("  <option value='3'") + addOutSelected(3) + ">3 (GPIO " + String(pins3) + ")</option>\n";
  html += String("  <option value='4'") + addOutSelected(4) + ">4 (GPIO " + String(pins4) + ")</option>\n";
  html += "</select>\n";
  return html;
}


// Hilfsfunktion zum Generieren des Formulars für einen Button
void generateButtonForm(AsyncResponseStream *response, const ButtonConfig& buttonData, int buttonId) {
  String prefix = "btn_"; // Einheitlicher Prefix
  String actionUrl = "/savebutton";
  String pageTitle = (buttonId == -1) ? "Add New Button" : "Edit Button: " + String(buttonData.name);

  response->print("      <div class='row'>\n");
  response->print("        <div class='col-md-12'>\n");
  response->print("          <h2>" + pageTitle + "</h2>\n");
  response->print("          <form class='form-horizontal' action='" + actionUrl + "' method='post'>\n");
  response->print("            <input type='hidden' name='button_id' value='" + String(buttonId) + "'>\n");

  // --- Formularfelder ---
  // Name
  response->print("            <div class='form-group'>\n");
  response->print("              <label for='" + prefix + "type' class='col-sm-2 control-label'>Type</label>\n");
  // --- KORREKTUR: Globale Funktion aufrufen ---
  response->print("              <div class='col-sm-10'>" + generateTypeDropdownHtml(prefix + "type", String(buttonData.type)) + "</div>\n");
  response->print("            </div>\n");

  // Type
  response->print("            <div class='form-group'>\n");
  response->print("              <label for='" + prefix + "out' class='col-sm-2 control-label'>Output Pin</label>\n");
  // --- KORREKTUR: Globale Funktion aufrufen ---
  response->print("              <div class='col-sm-10'>" + generateOutDropdownHtml(prefix + "out", buttonData.out) + "</div>\n");
  response->print("            </div>\n");

  // Data
  response->print("            <div class='form-group'>\n");
  response->print("              <label for='" + prefix + "data' class='col-sm-2 control-label'>Data (Hex)</label>\n");
  response->print("              <div class='col-sm-10'><input type='text' class='form-control' id='" + prefix + "data' name='" + prefix + "data' placeholder='e.g., FF02FD' value='" + String(buttonData.data) + "' required></div>\n");
  response->print("            </div>\n");

  // Length
  response->print("            <div class='form-group'>\n");
  response->print("              <label for='" + prefix + "length' class='col-sm-2 control-label'>Length (Bits)</label>\n");
  response->print("              <div class='col-sm-10'><input type='number' class='form-control' id='" + prefix + "length' name='" + prefix + "length' placeholder='e.g., 32' value='" + String(buttonData.length) + "' required min='1'></div>\n");
  response->print("            </div>\n");

  // Address
  response->print("            <div class='form-group'>\n");
  response->print("              <label for='" + prefix + "address' class='col-sm-2 control-label'>Address (Hex, opt.)</label>\n");
  response->print("              <div class='col-sm-10'><input type='text' class='form-control' id='" + prefix + "address' name='" + prefix + "address' placeholder='e.g., 0x404' value='" + String(buttonData.address) + "'></div>\n");
  response->print("            </div>\n");

  // Repeat
  response->print("            <div class='form-group'>\n");
  response->print("              <label for='" + prefix + "repeat' class='col-sm-2 control-label'>Repeat</label>\n");
  response->print("              <div class='col-sm-10'><input type='number' class='form-control' id='" + prefix + "repeat' name='" + prefix + "repeat' value='" + String(buttonData.repeat) + "' min='1'></div>\n");
  response->print("            </div>\n");

  // Output Pin
  response->print("            <div class='form-group'>\n");
  response->print("              <label for='" + prefix + "out' class='col-sm-2 control-label'>Output Pin</label>\n");
  // --- KORREKTUR: Globale Funktion aufrufen ---
  response->print("              <div class='col-sm-10'>" + generateOutDropdownHtml(prefix + "out", buttonData.out) + "</div>\n");
  response->print("            </div>\n");

  // --- Submit/Cancel Buttons ---
  response->print("            <div class='form-group'>\n");
  response->print("              <div class='col-sm-offset-2 col-sm-10'>\n");
  response->print("                <button type='submit' class='btn btn-success'>Save Button</button>\n");
  response->print("                <a href='/buttons' class='btn btn-default'>Cancel</a>\n");
  response->print("              </div>\n");
  response->print("            </div>\n");

  response->print("          </form>\n");
  response->print("        </div>\n");
  response->print("      </div>\n");
}


// Handler zum Anzeigen des "Add New Button"-Formulars
void handleAddButtonPage(AsyncWebServerRequest *request) {
Serial.println("Connection received endpoint '/addbutton' (GET)");
AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8", 200);
sendHeader(response);
ButtonConfig emptyButton; // Leeres Struct für leeres Formular
emptyButton.repeat = 1; // Standardwerte setzen
emptyButton.out = 1;
generateButtonForm(response, emptyButton, -1); // -1 signalisiert "neu"
sendFooter(response);
request->send(response);
}

// Handler zum Anzeigen des "Edit Button"-Formulars
void handleEditButtonPage(AsyncWebServerRequest *request) {
Serial.println("Connection received endpoint '/editbutton' (GET)");
if (!request->hasParam("id")) {
  request->redirect("/buttons?status=error_invalid_id");
  return;
}
int buttonId = request->getParam("id")->value().toInt();

// ID validieren (muss innerhalb der Vector-Grenzen liegen)
if (buttonId < 0 || buttonId >= buttonConfigs.size()) {
   Serial.printf("Error: Invalid button ID %d requested for edit.\n", buttonId);
   request->redirect("/buttons?status=error_invalid_id");
   return;
}

AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8", 200);
sendHeader(response);
generateButtonForm(response, buttonConfigs[buttonId], buttonId); // Daten des Buttons übergeben
sendFooter(response);
request->send(response);
}

// Handler zum Löschen eines Buttons
void handleDeleteButton(AsyncWebServerRequest *request) {
Serial.println("Connection received endpoint '/deletebutton' (GET)");
 if (!request->hasParam("id")) {
  request->redirect("/buttons?status=error_invalid_id");
  return;
}
int buttonId = request->getParam("id")->value().toInt();

// ID validieren
if (buttonId >= 0 && buttonId < buttonConfigs.size()) {
  Serial.printf("Deleting button ID %d ('%s').\n", buttonId, buttonConfigs[buttonId].name);
  buttonConfigs.erase(buttonConfigs.begin() + buttonId); // Element aus Vector löschen
  saveButtonConfig(); // Änderung speichern
  request->redirect("/buttons?status=deleted");
} else {
  Serial.printf("Error: Invalid button ID %d requested for deletion.\n", buttonId);
  request->redirect("/buttons?status=error_invalid_id");
}
}

void handleSaveButton(AsyncWebServerRequest *request) {
  Serial.println("==> handleSaveButton: Entered function.");

  // --- NEUER ANSATZ: Parameter manuell per Index suchen ---
  int params = request->params();
  Serial.printf("    Scanning %d parameters...\n", params);

  // Lokale Variablen für die gelesenen Werte initialisieren
  String buttonIdStr = ""; // ID als String lesen
  String name = "";
  String type = "";
  String data = "";
  String lengthStr = ""; // Länge als String lesen
  String address = "";
  String repeatStr = ""; // Repeat als String lesen
  String outStr = "";    // Out als String lesen
  bool buttonIdFound = false;

  for(int i=0; i<params; i++){
    const AsyncWebParameter* p = request->getParam(i);
    // Nur POST-Parameter berücksichtigen
    if(p->isPost()){
      String paramName = p->name(); // Namen holen
      String paramValue = p->value(); // Wert holen
      Serial.printf("      POST[%s]: %s\n", paramName.c_str(), paramValue.c_str()); // Debug

      // Werte basierend auf dem Namen zuweisen
      if (paramName.equals("button_id")) {
        buttonIdStr = paramValue;
        buttonIdFound = true;
      } else if (paramName.equals("btn_name")) {
        name = paramValue;
      } else if (paramName.equals("btn_type")) {
        type = paramValue;
      } else if (paramName.equals("btn_data")) {
        data = paramValue;
      } else if (paramName.equals("btn_length")) {
        lengthStr = paramValue;
      } else if (paramName.equals("btn_address")) {
        address = paramValue;
      } else if (paramName.equals("btn_repeat")) {
        repeatStr = paramValue;
      } else if (paramName.equals("btn_out")) {
        outStr = paramValue;
      }
    } else {
       Serial.printf("      Ignoring non-POST param[%s]: %s\n", p->name().c_str(), p->value().c_str());
    }
  }
  Serial.println("    Parameter scan complete.");
  // --- ENDE NEUER ANSATZ ---

  // --- Prüfung, ob button_id gefunden wurde ---
  if (!buttonIdFound) {
      Serial.println("    ERROR: Parameter 'button_id' not found during manual scan!");
      request->redirect("/buttons?status=error_save");
      return;
  }

  // --- Werte konvertieren ---
  int buttonId = buttonIdStr.toInt();
  int length = lengthStr.toInt();
  int repeat = repeatStr.toInt();
  int out = outStr.toInt();

  Serial.printf("    Button ID received: %d\n", buttonId);
  Serial.println("    Read Parameters (manually scanned):");
  Serial.println("      Name: '" + name + "'");
  Serial.println("      Type: '" + type + "'");
  Serial.println("      Data: '" + data + "'");
  Serial.println("      Length: " + String(length));
  Serial.println("      Address: '" + address + "'");
  Serial.println("      Repeat: " + String(repeat));
  Serial.println("      Out: " + String(out));

  name.trim();

  // --- Validierung ---
  if (name.length() == 0 || data.length() == 0 || length <= 0) {
      Serial.println("    ERROR: Validation failed! (name, data, or length invalid). Redirecting.");
      request->redirect("/buttons?status=error_invalid_data");
      return;
  }
  Serial.println("    Validation passed.");
  // Defaults setzen, falls Konvertierung fehlschlug oder Wert 0 war
  if (repeat <= 0) repeat = 1;
  if (out <= 0 || out > 4) out = 1; // Prüfe auch auf <=0

  // --- Entscheiden: Add oder Edit ---
  if (buttonId == -1) { // Neuer Button
    Serial.println("    -> Entering ADD logic.");
    if (buttonConfigs.size() >= MAX_BUTTONS) {
       Serial.println("      ERROR: Maximum number of buttons reached!");
       request->redirect("/buttons?status=error_max_buttons");
       return;
    }

    Serial.println("    Adding new button: " + name);
    ButtonConfig newButton;
    strncpy(newButton.name, name.c_str(), sizeof(newButton.name) - 1);
    newButton.name[sizeof(newButton.name) - 1] = '\0';
    strncpy(newButton.type, type.c_str(), sizeof(newButton.type) - 1);
    newButton.type[sizeof(newButton.type) - 1] = '\0';
    strncpy(newButton.data, data.c_str(), sizeof(newButton.data) - 1);
    newButton.data[sizeof(newButton.data) - 1] = '\0';
    newButton.length = length;
    strncpy(newButton.address, address.c_str(), sizeof(newButton.address) - 1);
    newButton.address[sizeof(newButton.address) - 1] = '\0';
    newButton.repeat = repeat;
    newButton.out = out;
    newButton.configured = true;

    buttonConfigs.push_back(newButton);
    Serial.printf("    Vector size after add: %d\n", buttonConfigs.size());

  } else if (buttonId >= 0 && buttonId < buttonConfigs.size()) { // Button bearbeiten
    Serial.println("    -> Entering EDIT logic for ID: " + String(buttonId));
    ButtonConfig& existingButton = buttonConfigs[buttonId];

    strncpy(existingButton.name, name.c_str(), sizeof(existingButton.name) - 1);
    existingButton.name[sizeof(existingButton.name) - 1] = '\0';
    strncpy(existingButton.type, type.c_str(), sizeof(existingButton.type) - 1);
    existingButton.type[sizeof(existingButton.type) - 1] = '\0';
    strncpy(existingButton.data, data.c_str(), sizeof(existingButton.data) - 1);
    existingButton.data[sizeof(existingButton.data) - 1] = '\0';
    existingButton.length = length;
    strncpy(existingButton.address, address.c_str(), sizeof(existingButton.address) - 1);
    existingButton.address[sizeof(existingButton.address) - 1] = '\0';
    existingButton.repeat = repeat;
    existingButton.out = out;
    existingButton.configured = true;

    Serial.println("    Button data updated in vector.");

  } else { // Ungültige ID
    Serial.println("    ERROR: Invalid button_id received: " + String(buttonId));
    request->redirect("/buttons?status=error_invalid_id");
    return;
  }

  // --- Speichern und Redirect ---
  Serial.println("    -> Calling saveButtonConfig()...");
  saveButtonConfig();
  Serial.println("    -> Redirecting to /buttons?status=saved");
  request->redirect("/buttons?status=saved");
}

//+=============================================================================
// Handler for Button Configuration Overview Page (NEU)
//
void handleButtonConfigPage(AsyncWebServerRequest *request) {
  Serial.println("Connection received endpoint '/buttons' (GET)");

  AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8", 200);
  sendHeader(response);

  response->print("      <div class='row'>\n");
  response->print("        <div class='col-md-12'>\n");
  response->print("          <h2>Configure Remote Buttons</h2>\n");

  // --- Feedback-Meldungen anzeigen ---
  if (request->hasParam("status")) {
      String status = request->getParam("status")->value();
      if (status == "saved") {
          response->print("<div class='alert alert-success'>Button saved successfully.</div>");
      } else if (status == "deleted") {
          response->print("<div class='alert alert-success'>Button deleted successfully.</div>");
      } else if (status == "error_invalid_id") {
          response->print("<div class='alert alert-danger'>Error: Invalid button ID specified.</div>");
      }
      // Weitere Status nach Bedarf
  }

  response->print("          <table class='table table-striped'>\n");
  response->print("            <thead><tr><th>Name</th><th>Type</th><th>Data</th><th>Actions</th></tr></thead>\n");
  response->print("            <tbody>\n");

  if (!buttonConfigs.empty()) {
    for (size_t i = 0; i < buttonConfigs.size(); ++i) {
      const auto& button = buttonConfigs[i];
      response->print("              <tr>\n");
      response->print("                <td>" + String(button.name) + "</td>\n");
      response->print("                <td><code>" + String(button.type) + "</code></td>\n");
      response->print("                <td><code>" + String(button.data) + " (" + String(button.length) + " bits)</code></td>\n");
      response->print("                <td>\n");
      // Edit Link mit ID (Index)
      response->print("                  <a href='/editbutton?id=" + String(i) + "' class='btn btn-xs btn-warning'>Edit</a>\n");
      // Delete Link mit ID (Index) und Bestätigung
      response->print("                  <a href='/deletebutton?id=" + String(i) + "' class='btn btn-xs btn-danger' onclick='return confirm(\"Are you sure you want to delete button \\'" + String(button.name) + "\\'?\");'>Delete</a>\n");
      response->print("                </td>\n");
      response->print("              </tr>\n");
    }
  } else {
    response->print("              <tr><td colspan='4' class='text-center'><em>No buttons configured.</em></td></tr>\n");
  }

  response->print("            </tbody>\n");
  response->print("          </table>\n");
  // Button zum Hinzufügen eines neuen Buttons
  response->print("          <a href='/addbutton' class='btn btn-success'>Add New Button</a>\n");
  response->print("          <a href='/' class='btn btn-default' style='margin-left: 10px;'>Back to Home</a>\n"); // Zurück zur Hauptseite
  response->print("        </div>\n");
  response->print("      </div>\n");

  sendFooter(response);
  request->send(response);
}

//+=============================================================================
// Handler to Save Button Configuration
//
void handleSaveButtons(AsyncWebServerRequest *request) {
  Serial.println("Connection received endpoint '/savebuttons' (POST)");

  // --- Security Check (optional) ---
  // if (!allowLocalBypass(request->client().remoteIP()) && !isPasscodeValid(request->arg("pass"))) { ... }

  bool changed = false;
  for (int i = 0; i < MAX_BUTTONS; ++i) {
    String prefix = "btn" + String(i) + "_";

    // --- Parameter mit request->hasParam / request->getParam abrufen ---
    String name = request->hasParam(prefix + "name") ? request->getParam(prefix + "name")->value() : "";
    String type = request->hasParam(prefix + "type") ? request->getParam(prefix + "type")->value() : "";
    String data = request->hasParam(prefix + "data") ? request->getParam(prefix + "data")->value() : "";
    int length = request->hasParam(prefix + "length") ? request->getParam(prefix + "length")->value().toInt() : 0; // Default 0 if missing
    String address = request->hasParam(prefix + "address") ? request->getParam(prefix + "address")->value() : "";
    int repeat = request->hasParam(prefix + "repeat") ? request->getParam(prefix + "repeat")->value().toInt() : 1; // Default 1 if missing
    int out = request->hasParam(prefix + "out") ? request->getParam(prefix + "out")->value().toInt() : 1;       // Default 1 if missing


    // Trim whitespace from name
    name.trim();

    // Grundlegende Validierung: Button ist konfiguriert, wenn Name, Daten und Länge vorhanden sind
    bool isConfigured = (name.length() > 0 && data.length() > 0 && length > 0);

    // Nur aktualisieren, wenn sich etwas geändert hat oder der Status sich ändert
    if (isConfigured != buttonConfigs[i].configured ||
        (isConfigured && (
          name != buttonConfigs[i].name || type != buttonConfigs[i].type || data != buttonConfigs[i].data ||
          length != buttonConfigs[i].length || address != buttonConfigs[i].address ||
          repeat != buttonConfigs[i].repeat || out != buttonConfigs[i].out)))
    {
        changed = true;
        strncpy(buttonConfigs[i].name, name.c_str(), sizeof(buttonConfigs[i].name) - 1);
        buttonConfigs[i].name[sizeof(buttonConfigs[i].name) - 1] = '\0'; // Null-terminieren

        if (isConfigured) {
            strncpy(buttonConfigs[i].type, type.c_str(), sizeof(buttonConfigs[i].type) - 1);
            strncpy(buttonConfigs[i].data, data.c_str(), sizeof(buttonConfigs[i].data) - 1);
            buttonConfigs[i].length = length;
            strncpy(buttonConfigs[i].address, address.c_str(), sizeof(buttonConfigs[i].address) - 1);
            buttonConfigs[i].repeat = (repeat > 0) ? repeat : 1;
            buttonConfigs[i].out = (out >= 1 && out <= 4) ? out : 1;
            buttonConfigs[i].configured = true;

            // Sicherstellen, dass Strings null-terminiert sind
            buttonConfigs[i].type[sizeof(buttonConfigs[i].type) - 1] = '\0';
            buttonConfigs[i].data[sizeof(buttonConfigs[i].data) - 1] = '\0';
            buttonConfigs[i].address[sizeof(buttonConfigs[i].address) - 1] = '\0';

        } else {
            // Button deaktivieren/leeren
            buttonConfigs[i].name[0] = '\0';
            buttonConfigs[i].type[0] = '\0';
            buttonConfigs[i].data[0] = '\0';
            buttonConfigs[i].length = 0;
            buttonConfigs[i].address[0] = '\0';
            buttonConfigs[i].repeat = 1;
            buttonConfigs[i].out = 1;
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

  // Redirect back to home page after saving using request->redirect()
  request->redirect("/?status=buttons_saved"); // 303 See Other wird implizit verwendet
}


//+=============================================================================
// Handler to Send IR Code from a Remote Button (AJAX - GET with URL Params)
//
void handleSendButton(AsyncWebServerRequest *request) {
  Serial.println("==> handleSendButton: Entered function.");

  // --- Argument Parsing (aus URL-Parametern) ---
  if (!request->hasParam("type") || !request->hasParam("data") || !request->hasParam("length")) {
    Serial.println("    ERROR: Missing required parameters!");
    request->send(400, "text/plain", "Bad Request: Missing required IR parameters (type, data, length).");
    return;
  }
  Serial.println("    Required parameters present.");

  String type = request->getParam("type")->value();
  String dataStr = request->getParam("data")->value();
  unsigned int len = request->getParam("length")->value().toInt();
  long address = 0;
  if (request->hasParam("address") && request->getParam("address")->value().length() > 0) {
      String addressStr = request->getParam("address")->value();
      if (addressStr.startsWith("0x")) {
          address = strtoul(addressStr.c_str(), 0, 0);
      } else {
          address = strtoul(("0x" + addressStr).c_str(), 0, 0);
      }
  }
  // --- KORREKTUR: Stelle sicher, dass 'repeat' hier deklariert wird ---
  int repeat = request->hasParam("repeat") ? request->getParam("repeat")->value().toInt() : 1;
  int out = request->hasParam("out") ? request->getParam("out")->value().toInt() : 1;

  // --- KORREKTUR: Deklaration für rdelay, pulse, pdelay hinzufügen ---
  int rdelay = 1000; // Default repeat delay
  int pulse = 1;     // Default pulse count
  int pdelay = 100;  // Default pulse delay

  // --- Parameter ausgeben ---
  Serial.println("    Read Parameters:");
  Serial.println("      Type: " + type);
  Serial.println("      Data: " + dataStr);
  Serial.println("      Length: " + String(len));
  Serial.println("      Address: " + String(address, HEX));
  Serial.println("      Repeat: " + String(repeat)); // Jetzt sollte 'repeat' bekannt sein
  Serial.println("      Out: " + String(out));
  // --- ENDE Parameter ausgeben ---


  // Validate inputs (basic)
  if (type.length() == 0 || dataStr.length() == 0 || len == 0) {
      Serial.println("    ERROR: Validation failed!");
      request->send(400, "text/plain", "Bad Request: Invalid IR parameters.");
      return;
  }
   Serial.println("    Validation passed.");
  if (repeat <= 0) repeat = 1; // Diese Zeile ist ok, da repeat jetzt deklariert ist
  if (out < 1 || out > 4) out = 1;

// --- Trigger IR Blast ---
Serial.println("    -> Calling irblast...");

Serial.println("      Setting LED LOW..."); // NEU
digitalWrite(ledpin, LOW);
Serial.println("      LED set LOW."); // NEU

Serial.println("      Attaching ticker..."); // NEU
ticker.attach(0.5, disableLed);
Serial.println("      Ticker attached."); // NEU

Serial.println("      Now calling irblast function..."); // NEU
irblast(type, dataStr, len, rdelay, pulse, pdelay, repeat, address, pickIRsend(out), out);

  // --- Send Success Response ---
  Serial.println("    <- Sending OK response.");
  request->send(200, "text/plain", "OK");
}

//+=============================================================================
// Handler for the IR sending form
//
void handleSendIr(AsyncWebServerRequest *request) {
  Serial.println("Connection received endpoint '/sendir' (POST)");

  // --- NEUER ANSATZ: Parameter manuell per Index suchen ---
  int params = request->params();
  Serial.printf("    Scanning %d parameters (handleSendIr)...\n", params);

  // Lokale Variablen für die gelesenen Werte initialisieren
  String type = "";
  String dataStr = "";
  String lengthStr = ""; // Länge als String lesen
  String addressStr = ""; // Adresse als String lesen
  String repeatStr = ""; // Repeat als String lesen
  String outStr = "";    // Out als String lesen
  bool typeFound = false, dataFound = false, lengthFound = false; // Flags für erforderliche Felder

  for(int i=0; i<params; i++){
    const AsyncWebParameter* p = request->getParam(i);
    // Nur POST-Parameter berücksichtigen
    if(p->isPost()){
      String paramName = p->name();
      String paramValue = p->value();
      Serial.printf("      POST[%s]: %s\n", paramName.c_str(), paramValue.c_str()); // Debug

      // Werte basierend auf dem Namen zuweisen
      if (paramName.equals("type")) {
        type = paramValue;
        typeFound = true;
      } else if (paramName.equals("data")) {
        dataStr = paramValue;
        dataFound = true;
      } else if (paramName.equals("length")) {
        lengthStr = paramValue;
        lengthFound = true;
      } else if (paramName.equals("address")) {
        addressStr = paramValue;
      } else if (paramName.equals("repeat")) {
        repeatStr = paramValue;
      } else if (paramName.equals("out")) {
        outStr = paramValue;
      }
    } else {
       Serial.printf("      Ignoring non-POST param[%s]: %s\n", p->name().c_str(), p->value().c_str());
    }
  }
  Serial.println("    Parameter scan complete (handleSendIr).");
  // --- ENDE NEUER ANSATZ ---

  // --- Prüfung, ob erforderliche Parameter gefunden wurden ---
  if (!typeFound || !dataFound || !lengthFound) {
      Serial.println("    ERROR: Missing required arguments (type, data, or length) during manual scan!");
      request->redirect("/?status=error_missing_args");
      return;
  }

  // --- Werte konvertieren ---
  unsigned int len = lengthStr.toInt();
  long address = 0;
  if (addressStr.length() > 0) {
      if (addressStr.startsWith("0x")) {
          address = strtoul(addressStr.c_str(), 0, 0);
      } else {
          address = strtoul(("0x" + addressStr).c_str(), 0, 0);
      }
  }
  int repeat = repeatStr.toInt();
  int out = outStr.toInt();

  // Default values for delays/pulse (werden nicht aus Formular gelesen)
  int rdelay = 1000;
  int pulse = 1;
  int pdelay = 100;

  // --- Parameter ausgeben (zum Debuggen) ---
  Serial.println("    Read Parameters (handleSendIr - manually scanned):");
  Serial.println("      Type: '" + type + "'");
  Serial.println("      Data: '" + dataStr + "'");
  Serial.println("      Length: " + String(len));
  Serial.println("      Address: " + String(address, HEX));
  Serial.println("      Repeat: " + String(repeat));
  Serial.println("      Out: " + String(out));
  // --- ENDE Parameter ausgeben ---

  // Validate inputs (basic)
  if (len == 0 || dataStr.length() == 0) { // type wurde schon geprüft
      Serial.println("    ERROR: Validation failed in handleSendIr (len=0 or data empty)!");
      request->redirect("/?status=error_invalid_args");
      return;
  }
   Serial.println("    Validation passed in handleSendIr.");
  // Defaults setzen, falls Konvertierung fehlschlug oder Wert 0 war
  if (repeat <= 0) repeat = 1;
  if (out <= 0 || out > 4) out = 1;


  // --- Trigger IR Blast ---
  Serial.println("    -> Calling irblast from form...");
  digitalWrite(ledpin, LOW);
  ticker.attach(0.5, disableLed);

  irblast(type, dataStr, len, rdelay, pulse, pdelay, repeat, address, pickIRsend(out), out);
  Serial.println("    <- irblast call returned (from form).");

  // --- Redirect back to home page with success message ---
  Serial.println("    -> Redirecting to /?status=success");
  request->redirect("/?status=success");
}


// Beispiel für einen Not Found Handler
void handleNotFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Seite nicht gefunden");
}


//+=============================================================================
// Setup web server and IR receiver/blaster
//
void setup() {
  // Initialize serial
  Serial.begin(115200);

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

  // --- TEMPORÄRER CODE ZUM FORMATIEREN ---
  // Diesen Block einkommentieren, EINMAL flashen & laufen lassen,
  // dann wieder auskommentieren und erneut flashen!
/* 
  Serial.println("Attempting to format LittleFS... THIS WILL ERASE ALL SAVED DATA (WiFi, Buttons)!");
  bool formatted = LittleFS.format();
  if (formatted) {
    LittleFS.begin(true);
    Serial.println("LittleFS formatted successfully.");
    while(1); // Anhalten
  } else {
    Serial.println("!!! LittleFS format failed. Halting. !!!");
    while(1); // Anhalten, wenn Formatierung fehlschlägt
  }
    */
  // --- ENDE TEMPORÄRER CODE ---

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

  // Server-Objekt erstellen (angenommen, es heißt 'server')
  server = new AsyncWebServer(port); // Oder AsyncWebServer server(port); wenn global deklariert

  // --- Globale CORS Header setzen ---
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS"); // OPTIONS hinzufügen ist oft gut für Preflight-Requests
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type, Authorization"); // Erlaube gängige Header

    // --- Handler registrieren ---

    server->on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("Connection received endpoint '/'");

    // Parameter mit request->getParam() abrufen (und auf Existenz prüfen)
    String signature = request->hasParam("auth") ? request->getParam("auth")->value() : "";
    String epid = request->hasParam("epid") ? request->getParam("epid")->value() : "";
    String mid = request->hasParam("mid") ? request->getParam("mid")->value() : "";
    String timestamp = request->hasParam("time") ? request->getParam("time")->value() : "";

    // Optional: Parameter loggen, falls vorhanden
    if (signature.length() > 0) Serial.println("Auth: " + signature);
    if (epid.length() > 0) Serial.println("EPID: " + epid);
    if (mid.length() > 0) Serial.println("MID: " + mid);
    if (timestamp.length() > 0) Serial.println("Time: " + timestamp);

    // sendHomePage mit dem request-Objekt aufrufen
    sendHomePage(request); // 200 wird innerhalb von sendHomePage/sendHeader gesetzt
  });

   // Configure the server
   server->on("/json", HTTP_POST, [](AsyncWebServerRequest *request) { // JSON handler
    Serial.println("Connection received endpoint '/json'");

    // --- Parameter mit request->hasParam / request->getParam abrufen ---
    int simple = 0;
    if (request->hasParam("simple")) simple = request->getParam("simple")->value().toInt();
    String signature = request->hasParam("auth") ? request->getParam("auth")->value() : "";
    String epid = request->hasParam("epid") ? request->getParam("epid")->value() : "";
    String mid = request->hasParam("mid") ? request->getParam("mid")->value() : "";
    String timestamp = request->hasParam("time") ? request->getParam("time")->value() : "";
    int out = (request->hasParam("out")) ? request->getParam("out")->value().toInt() : 1; // Default output pin

    // --- JSON Payload aus 'plain' Parameter holen ---
    if (!request->hasParam("plain")) {
        Serial.println("JSON parsing failed: Missing 'plain' parameter.");
        if (simple) {
            // sendCorsHeaders(); // ENTFERNT
            request->send(400, "text/plain", "JSON parsing failed: Missing 'plain' parameter.");
        } else {
            sendHomePage(request, "JSON parsing failed: Missing 'plain' parameter.", "Error", 3, 400); // Übergibt request
        }
        return; // Wichtig: Handler hier beenden
    }

    DynamicJsonDocument root(4096); // Größe ggf. anpassen
    DeserializationError error = deserializeJson(root, request->getParam("plain")->value());

    if (error) {
      Serial.println("JSON parsing failed");
      Serial.println(error.c_str());
      if (simple) {
        // sendCorsHeaders(); // ENTFERNT
        request->send(400, "text/plain", "JSON parsing failed, " + String(error.c_str()));
      } else {
        sendHomePage(request, "JSON parsing failed", "Error", 3, 400); // Übergibt request
      }
      root.clear();
      // return; // Kein return hier, da root.clear() schon passiert ist
    } else { // JSON erfolgreich geparst
      digitalWrite(ledpin, LOW);
      ticker.attach(0.5, disableLed);

      // Handle device state limitations for the global JSON command request
      if (request->hasParam("device")) {
        String device = request->getParam("device")->value();
        Serial.println("Device name detected " + device);
        int state = (request->hasParam("state")) ? request->getParam("state")->value().toInt() : 0;
        if (deviceState.containsKey(device)) {
          Serial.println("Contains the key!");
          Serial.println(state);
          int currentState = deviceState[device];
          Serial.println(currentState);
          if (state == currentState) {
            Serial.println("Not sending command to " + device + ", already in state " + state);
            if (simple) {
              // sendCorsHeaders(); // ENTFERNT
              request->send(200, "text/html", "Not sending command to " + device + ", already in state " + String(state)); // String() hinzugefügt
            } else {
              sendHomePage(request, "Not sending command to " + device + ", already in state " + String(state), "Warning", 2); // Übergibt request, String() hinzugefügt
            }
            // return; // Wichtig: Handler hier beenden, wenn Befehl nicht gesendet wird
            // Korrektur: Wenn nur *dieser* Teil des JSON übersprungen werden soll, darf hier kein return stehen,
            // aber wenn die *gesamte* Anfrage wegen des globalen device state ignoriert wird, dann return.
            // Aktuelle Logik: Die gesamte Anfrage wird ignoriert. Also return ist korrekt.
             root.clear(); // JSON leeren, bevor der Handler verlassen wird
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

      // Simple Success-Antwort *vor* dem Senden senden, wenn simple=1
      if (simple) {
        // sendCorsHeaders(); // ENTFERNT
        request->send(200, "text/html", "Success, processing codes..."); // Angepasste Nachricht
      }

      String message = "Code sent"; // Default message

      // --- IR-Befehle verarbeiten ---
      for (size_t x = 0; x < root.size(); x++) {
        String type = root[x]["type"].as<String>(); // .as<String>() ist sicherer
        String ip = root[x]["ip"].as<String>();
        int rdelay = root[x]["rdelay"] | 1000; // Default-Werte mit | Operator
        int pulse = root[x]["pulse"] | 1;
        int pdelay = root[x]["pdelay"] | 100;
        int repeat = root[x]["repeat"] | 1;
        int xout = root[x]["out"] | out; // Default auf globalen 'out' Parameter oder 1
        int duty = root[x]["duty"] | 50;

        // Handle device state limitations on a per JSON object basis
        String device = root[x]["device"].as<String>();
        if (device != "null" && device.length() > 0) { // Prüfe auch auf leeren String
          int state = root[x]["state"] | 0; // Default state 0
          if (deviceState.containsKey(device)) {
            int currentState = deviceState[device];
            if (state == currentState) {
              Serial.println("Not sending command component for " + device + ", already in state " + state);
              message = "Code sent. Some components were held because device was already in appropriate state";
              continue; // Nächsten Befehl im JSON Array verarbeiten
            } else {
              Serial.println("Setting device " + device + " to state " + state);
              deviceState[device] = state;
            }
          } else {
            Serial.println("Setting device " + device + " to state " + state);
            deviceState[device] = state;
          }
        }

        // --- IR Sende-Logik ---
        if (type == "delay") {
          delay(rdelay);
        } else if (type == "raw") {
          JsonArray raw = root[x]["data"].as<JsonArray>();
          if (!raw) { Serial.println("Error: 'data' is not an array for raw type."); continue; }
          int khz = root[x]["khz"] | 38;
          rawblast(raw, khz, rdelay, pulse, pdelay, repeat, pickIRsend(xout), duty, xout);
        } else if (type == "pronto") {
          JsonArray pdata = root[x]["data"].as<JsonArray>();
           if (!pdata) { Serial.println("Error: 'data' is not an array for pronto type."); continue; }
          pronto(pdata, rdelay, pulse, pdelay, repeat, pickIRsend(xout), xout);
        } else if (type == "roku") {
          String data = root[x]["data"].as<String>();
          if (data.length() == 0 || ip.length() == 0) { Serial.println("Error: Missing 'data' or 'ip' for roku type."); continue; }
          rokuCommand(ip, data, repeat, rdelay);
        } else { // Standard IR Codes
          String data = root[x]["data"].as<String>();
          if (data.length() == 0 || type.length() == 0) { Serial.println("Error: Missing 'data' or 'type'."); continue; }
          String addressString = root[x]["address"].as<String>();
          long address = 0;
          if (addressString.length() > 0) {
              address = strtoul(addressString.c_str(), 0, 0); // Base 0 erkennt 0x automatisch
          }
          int len = root[x]["length"] | 0; // Default 0, wird in irblast geprüft
          if (len == 0) { Serial.println("Error: Missing or invalid 'length'."); continue; }
          irblast(type, data, len, rdelay, pulse, pdelay, repeat, address, pickIRsend(xout), xout);
        }
      } // End for loop

      // --- Finale Antwort senden (nur wenn simple=0) ---
      if (!simple) {
        Serial.println("Sending home page after JSON processing");
        sendHomePage(request, message, "Success", 1); // Übergibt request
      }

      root.clear(); // JSON Speicher freigeben
    } // End else (JSON parsing successful)
  }); // End request->on("/json")


  // Setup simple msg server to mirror version 1.0 functionality
  server->on("/msg", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("Connection received endpoint '/msg'");

    // --- Parameter mit request->hasParam / request->getParam abrufen ---
    int simple = 0;
    if (request->hasParam("simple")) simple = request->getParam("simple")->value().toInt();
    String signature = request->hasParam("auth") ? request->getParam("auth")->value() : "";
    String epid = request->hasParam("epid") ? request->getParam("epid")->value() : "";
    String mid = request->hasParam("mid") ? request->getParam("mid")->value() : "";
    String timestamp = request->hasParam("time") ? request->getParam("time")->value() : "";

    // --- IR Parameter ---
    // Initialisiere mit Defaults oder leeren Strings, werden ggf. später überschrieben
    String type = request->hasParam("type") ? request->getParam("type")->value() : "";
    String data = request->hasParam("data") ? request->getParam("data")->value() : "";
    String ip = request->hasParam("ip") ? request->getParam("ip")->value() : ""; // Für Roku
    int len = request->hasParam("length") ? request->getParam("length")->value().toInt() : 0;
    long address = 0;
    if (request->hasParam("address")) {
      String addressString = request->getParam("address")->value();
      if (addressString.length() > 0) { // Nur parsen, wenn nicht leer
          address = strtoul(addressString.c_str(), 0, 0); // Base 0 erkennt 0x
      }
    }
    int rdelay = (request->hasParam("rdelay")) ? request->getParam("rdelay")->value().toInt() : 1000;
    int pulse = (request->hasParam("pulse")) ? request->getParam("pulse")->value().toInt() : 1;
    int pdelay = (request->hasParam("pdelay")) ? request->getParam("pdelay")->value().toInt() : 100;
    int repeat = (request->hasParam("repeat")) ? request->getParam("repeat")->value().toInt() : 1;
    int out = (request->hasParam("out")) ? request->getParam("out")->value().toInt() : 1;

    // --- Optionaler 'code' Parameter überschreibt data, type, len ---
    if (request->hasParam("code")) {
      String code = request->getParam("code")->value();
      char separator = ':';
      data = getValue(code, separator, 0);
      type = getValue(code, separator, 1);
      len = getValue(code, separator, 2).toInt();
      Serial.println("Parsed 'code' parameter: data=" + data + ", type=" + type + ", len=" + String(len));
    }

    // --- Grundlegende Validierung (NACH potenzieller 'code'-Überschreibung) ---
    if (type.length() == 0 || data.length() == 0 || (type != "roku" && len <= 0)) {
        Serial.println("Error: Missing or invalid required parameters (type, data, length).");
        if (simple) {
            request->send(400, "text/plain", "Bad Request: Missing or invalid required parameters (type, data, length).");
        } else {
            sendHomePage(request, "Missing or invalid required parameters (type, data, length).", "Error", 3, 400); // Übergibt request
        }
        return; // Wichtig: Handler hier beenden
    }
    // Validate repeat and out values
    if (repeat <= 0) repeat = 1;
    if (out < 1 || out > 4) out = 1;


    // --- LED einschalten ---
    digitalWrite(ledpin, LOW);
    ticker.attach(0.5, disableLed);

    // --- Handle device state limitations ---
    if (request->hasParam("device")) {
      String device = request->getParam("device")->value();
      Serial.println("Device name detected " + device);
      int state = (request->hasParam("state")) ? request->getParam("state")->value().toInt() : 0;
      if (deviceState.containsKey(device)) {
        Serial.println("Contains the key!");
        Serial.println(state);
        int currentState = deviceState[device];
        Serial.println(currentState);
        if (state == currentState) {
          Serial.println("Not sending command to " + device + ", already in state " + state);
          if (simple) {
            // sendCorsHeaders(); // ENTFERNT
            request->send(200, "text/html", "Not sending command to " + device + ", already in state " + String(state)); // String() hinzugefügt
          } else {
            sendHomePage(request, "Not sending command to " + device + ", already in state " + String(state), "Warning", 2); // Übergibt request, String() hinzugefügt
          }
          return; // Wichtig: Handler hier beenden
        } else {
          Serial.println("Setting device " + device + " to state " + state);
          deviceState[device] = state;
        }
      } else {
        Serial.println("Setting device " + device + " to state " + state);
        deviceState[device] = state;
      }
    }

    // --- Simple Success-Antwort *vor* dem Senden senden, wenn simple=1 ---
    if (simple) {
      // sendCorsHeaders(); // ENTFERNT
      request->send(200, "text/html", "Success, code sent");
    }

    // --- IR-Befehl senden ---
    if (type == "roku") {
      rokuCommand(ip, data, repeat, rdelay);
    } else {
      irblast(type, data, len, rdelay, pulse, pdelay, repeat, address, pickIRsend(out), out);
    }

    // --- Finale Antwort senden (nur wenn simple=0) ---
    if (!simple) {
      sendHomePage(request, "Code Sent", "Success", 1); // Übergibt request
    }
  }); // End request->on("/msg")


  server->on("/received", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("Connection received endpoint '/received'");

    // --- Optionale Parameter abrufen (mit Prüfung) ---
    String signature = request->hasParam("auth") ? request->getParam("auth")->value() : "";
    String epid = request->hasParam("epid") ? request->getParam("epid")->value() : "";
    String mid = request->hasParam("mid") ? request->getParam("mid")->value() : "";
    String timestamp = request->hasParam("time") ? request->getParam("time")->value() : "";

    // --- ID Parameter abrufen (WICHTIG: Prüfen!) ---
    int id = 0;
    if (request->hasParam("id")) {
        id = request->getParam("id")->value().toInt();
    } else {
        // ID ist erforderlich, Fehler senden, wenn sie fehlt
        Serial.println("Error: Missing 'id' parameter for /received");
        sendHomePage(request, "Missing required 'id' parameter.", "Error", 3, 400); // 400 Bad Request, request übergeben
        return; // Handler beenden
    }

    // --- Code-Seite basierend auf ID senden ---
    // String output; // 'output' wird nicht verwendet, kann entfernt werden

    Code selectedCode; // Temporäres Objekt für den ausgewählten Code
    bool codeFound = false;

    switch (id) {
        case 1:
            if (last_recv.valid) { selectedCode = last_recv; codeFound = true; }
            break;
        case 2:
            if (last_recv_2.valid) { selectedCode = last_recv_2; codeFound = true; }
            break;
        case 3:
            if (last_recv_3.valid) { selectedCode = last_recv_3; codeFound = true; }
            break;
        case 4:
            if (last_recv_4.valid) { selectedCode = last_recv_4; codeFound = true; }
            break;
        case 5:
            if (last_recv_5.valid) { selectedCode = last_recv_5; codeFound = true; }
            break;
        default:
            // Ungültige ID (sollte durch die Prüfung oben abgefangen werden, aber sicher ist sicher)
            codeFound = false;
            break;
    }

    if (codeFound) {
        sendCodePage(request, selectedCode); // request übergeben
    } else {
        // Code für die ID nicht gefunden oder ungültig
        Serial.println("Error: Code for id " + String(id) + " not found or invalid.");
        sendHomePage(request, "Code for id " + String(id) + " does not exist or is invalid", "Alert", 2, 404); // 404 Not Found, request übergeben
    }

  }); // End request->on("/received")


    // --- NEUE/GEÄNDERTE BUTTON HANDLER ---
    server->on("/buttons", HTTP_GET, handleButtonConfigPage);      // Zeigt die Übersicht
    server->on("/addbutton", HTTP_GET, handleAddButtonPage);       // Zeigt leeres Formular
    server->on("/editbutton", HTTP_GET, handleEditButtonPage);     // Zeigt befülltes Formular
    server->on("/deletebutton", HTTP_GET, handleDeleteButton);   // Löscht Button (GET für Einfachheit, POST wäre besser)
    server->on("/savebutton", HTTP_POST, handleSaveButton);
    server->on("/sendbutton", HTTP_GET, handleSendButton);
    server->on("/sendir", HTTP_POST, handleSendIr);

    server->begin();
  Serial.println("HTTP Server started on port " + String(port));

  

  Serial.println("Starting UDP");
  ntpUDP.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(localPort);
  Serial.println("Waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(300);
  
  // externalIP();

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
// Send header HTML (Async Version)
//
// Overload ohne httpcode
void sendHeader(AsyncWebServerRequest *request) {
  Serial.println("Warning: sendHeader(request) called in AsyncResponseStream context. Use sendHeader(response).");
}

// Overload für Kompatibilität (optional, wenn du immer httpcode brauchst)
void sendHeader(AsyncWebServerRequest *request, int httpcode) {
  // Diese Funktion wird mit AsyncResponseStream nicht mehr direkt so verwendet.
  // Der Stream wird in der Haupt-Handler-Funktion erstellt.
  // Man könnte hier eine Warnung ausgeben oder die Funktion entfernen.
  Serial.println("Warning: sendHeader(request, httpcode) called in AsyncResponseStream context. Use sendHeader(response).");
}

// Nimmt jetzt einen Zeiger auf den Response Stream entgegen
void sendHeader(AsyncResponseStream *response) {
  // KEIN beginResponse mehr hier!

  // Schreibe die HTML-Teile direkt in den Stream
  response->print("<!DOCTYPE html PUBLIC '-//W3C//DTD XHTML 1.0 Strict//EN' 'http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd'>\n");
  response->print("<html xmlns='http://www.w3.org/1999/xhtml' xml:lang='en'>\n");
  response->print("  <head>\n");
  response->print("    <meta name='viewport' content='width=device-width, initial-scale=.75' />\n");
  response->print("    <link rel='stylesheet' href='https://stackpath.bootstrapcdn.com/bootstrap/3.4.1/css/bootstrap.min.css' />\n");
  response->print("    <style>@media (max-width: 991px) {.nav-pills>li {float: none; margin-left: 0; margin-top: 5px; text-align: center;}}</style>\n");
  String title = "<title>ESP32 IR Controller (" + String(host_name) + ")</title>\n";
  response->print(title);
  response->print("  </head>\n");
  response->print("  <body>\n");
  response->print("    <div class='container'>\n");
  response->print("      <h1><a href='https://github.com/baumrasen/ESP8266-HTTP-IR-Blaster'>Extended ESP32 IR Controller</a></h1>\n");
  response->print("      <div class='row'>\n");
  response->print("        <div class='col-md-12'>\n");
  response->print("          <ul class='nav nav-pills'>\n");

  String hostLink = "            <li class='active'>\n              <a href='http://" + String(host_name) + ".local" + ":" + String(port_str) + "'>Hostname <span class='badge'>" + String(host_name) + ".local" + ":" + String(port_str) + "</span></a></li>\n";
  response->print(hostLink);
  String localLink = "            <li class='active'>\n              <a href='http://" + WiFi.localIP().toString() + ":" + String(port_str) + "'>Local <span class='badge'>" + WiFi.localIP().toString() + ":" + String(port_str) + "</span></a></li>\n";
  response->print(localLink);
  String dnsLink = "            <li class='active'>\n              <a href='http://" + WiFi.dnsIP().toString() + "'>DNS <span class='badge'>" + WiFi.dnsIP().toString() + "</span></a></li>\n";
  response->print(dnsLink);
  // String externalIpStr = externalIP(); // Auskommentiert lassen
  // String externalLink = ...
  // response->print(externalLink);
  String macLink = "            <li class='active'>\n              <a>MAC <span class='badge'>" + String(WiFi.macAddress()) + "</span></a></li>\n";
  response->print(macLink);

  response->print("          </ul>\n");
  response->print("        </div>\n");
  response->print("      </div><hr />\n");
}


//+=============================================================================
// Send footer HTML (AsyncResponseStream Version)
//
// Nimmt jetzt einen Zeiger auf den Response Stream entgegen
void sendFooter(AsyncResponseStream *response) {
  // --- Uptime and Epoch ---
  String uptimeEpochLine = "      <div class='row'><div class='col-md-12'><em>" + String(millis()) + "ms uptime; EPOCH " + String(now() - (timeZone * SECS_PER_HOUR)) + "</em> / <em id='jepoch'></em> ( <em id='jdiff'></em> )</div></div>\n";
  response->print(uptimeEpochLine);
  response->print("      <script>document.getElementById('jepoch').innerHTML = Math.round((new Date()).getTime() / 1000)</script>");
  response->print("      <script>document.getElementById('jdiff').innerHTML = Math.abs(Math.round((new Date()).getTime() / 1000) - " + String(now() - (timeZone * SECS_PER_HOUR)) + ")</script>");

  // // +++ Speicherbelegung als Tabelle +++
  // response->print("      <div class='row'>\n");
  // response->print("        <div class='col-md-12'>\n");
  // response->print("          <h4>Memory Usage</h4>\n");
  // response->print("          <table class='table table-condensed table-bordered' style='font-size: 0.9em; max-width: 600px;'>\n");
  // response->print("            <thead>\n");
  // response->print("              <tr><th>Type</th><th>Used</th><th>Partition Size</th><th>Usage (%)</th><th>Graph</th></tr>\n");
  // response->print("            </thead>\n");
  // response->print("            <tbody>\n");

  // char buffer[60];
  // int barWidth = 15;

  // // --- LittleFS ---
  // uint32_t totalBytesFS = 0;
  // uint32_t usedBytesFS = 0;
  // String fsStatus = "OK";
  // if (LittleFS.begin()) {
  //     totalBytesFS = LittleFS.totalBytes();
  //     usedBytesFS = LittleFS.usedBytes();
  // } else {
  //     fsStatus = "Mount Error";
  //     Serial.println("Error: LittleFS not mounted when trying to get size info for footer.");
  // }
  // response->print("              <tr>\n");
  // response->print("                <td>Filesystem</td>\n");
  // if (totalBytesFS > 0) {
  //     // ... (Berechnungen) ...
  //     snprintf(buffer, sizeof(buffer), "%.1f KB", usedKB_fs);
  //     response->print("                <td>" + String(buffer) + "</td>\n");
  //     // ... (Restliche FS-Zeile mit response->print) ...
  //     response->print("                <td><samp>" + bar_fs + "</samp></td>\n");
  // } else {
  //     response->print("                <td colspan='4' class='text-danger'>" + fsStatus + "</td>\n");
  // }
  // response->print("              </tr>\n");

  // // --- Flash (Sketch) ---
  // // ... (Berechnungen) ...
  // response->print("              <tr>\n");
  // response->print("                <td>Flash (App Partition)</td>\n");
  // if (totalSketchPartitionSize > 0) {
  //     // ... (Berechnungen) ...
  //     snprintf(buffer, sizeof(buffer), "%.1f KB", usedKB_flash);
  //     response->print("                <td>" + String(buffer) + "</td>\n");
  //     // ... (Restliche Flash-Zeile mit response->print) ...
  //     response->print("                <td><samp>" + bar_flash + "</samp></td>\n");
  // } else {
  //     response->print("                <td colspan='4' class='text-danger'>" + flashStatus + "</td>\n");
  // }
  // response->print("              </tr>\n");

  // // --- Heap (RAM) ---
  // // ... (Berechnungen) ...
  // response->print("              <tr>\n");
  // response->print("                <td>Heap (RAM)</td>\n");
  // if (totalHeap > 0) {
  //     // ... (Berechnungen) ...
  //     snprintf(buffer, sizeof(buffer), "%.1f KB", usedKB_heap);
  //     response->print("                <td>" + String(buffer) + "</td>\n");
  //     // ... (Restliche Heap-Zeile mit response->print) ...
  //     response->print("                <td><samp>" + bar_heap + "</samp></td>\n");
  // } else {
  //     response->print("                <td colspan='4' class='text-danger'>Unavailable</td>\n");
  // }
  // response->print("              </tr>\n");

  // // --- Tabelle beenden ---
  // response->print("            </tbody>\n");
  // response->print("          </table>\n");
  // response->print("        </div>\n");
  // response->print("      </div>\n");

  // --- Bestehende Fehler-/Statusmeldungen ---
  time_t timenow = now() - (timeZone * SECS_PER_HOUR);
  if (!validEPOCH(timenow))
    response->print("      <div class='row'><div class='col-md-12'><em>Error - EPOCH time is inappropriately low...</em></div></div>\n");
  if (ntpError)
    response->print("      <div class='row'><div class='col-md-12'><em>Error - last attempt to connect to the NTP server failed...</em></div></div>\n");

  // --- Closing HTML tags ---
  response->print("    </div>\n");
  response->print("  </body>\n");
  response->print("</html>\n");
}


//+=============================================================================
// Stream home page HTML (AsyncResponseStream Version)
//
// Overloads leiten den Request weiter
void sendHomePage(AsyncWebServerRequest *request) {
  sendHomePage(request, "", "");
}

void sendHomePage(AsyncWebServerRequest *request, String message, String header) {
  sendHomePage(request, message, header, 0);
}

void sendHomePage(AsyncWebServerRequest *request, String message, String header, int type) {
  sendHomePage(request, message, header, type, 200);
}

// Hauptfunktion, die die Arbeit macht (AsyncResponseStream Version)
void sendHomePage(AsyncWebServerRequest *request, String message, String header, int type, int httpcode) {

  // --- Erstelle den Response Stream ---
  AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8", httpcode);

  // --- Schreibe Header in den Stream ---
  sendHeader(response); // Übergibt den Stream

  // +++ FERNBEDIENUNGS-BUTTONS ANZEIGEN +++
  // +++ FERNBEDIENUNGS-BUTTONS ANZEIGEN (Vector Version) +++
  response->print("      <div class='row'>\n");
  response->print("        <div class='col-md-12'>\n");
  response->print("          <h3>Remote Buttons</h3>\n");
  response->print("          <div id='remote-buttons' class='text-center'>\n"); // Container für Buttons

  if (!buttonConfigs.empty()) { // Prüfen, ob Buttons vorhanden sind
    // Iteriere durch den Vector
    for (size_t i = 0; i < buttonConfigs.size(); ++i) {
      const auto& button = buttonConfigs[i]; // Referenz für Lesbarkeit
      // Baue den Button-String zusammen
      String buttonHtml = "            <button class='btn btn-primary btn-lg remote-button' style='margin: 5px;' ";
      buttonHtml += "data-type='" + String(button.type) + "' ";
      buttonHtml += "data-data='" + String(button.data) + "' ";
      buttonHtml += "data-length='" + String(button.length) + "' ";
      buttonHtml += "data-address='" + String(button.address) + "' ";
      buttonHtml += "data-repeat='" + String(button.repeat) + "' ";
      buttonHtml += "data-out='" + String(button.out) + "'>";
      buttonHtml += String(button.name); // Button-Beschriftung
      buttonHtml += "</button>\n";
      response->print(buttonHtml);
    }
  } else {
      response->print("            <p><em>No remote buttons configured yet.</em></p>\n");
  }

  // Link zur Konfigurationsübersicht (wo man hinzufügen/bearbeiten/löschen kann)
  response->print("            <a href='/buttons' class='btn btn-default' style='margin: 5px;'>Configure Buttons</a>\n");
  response->print("          </div>\n");
  response->print("        </div>\n");
  response->print("      </div><hr />\n");
  // +++ ENDE FERNBEDIENUNGS-BUTTONS +++

    // +++ Feedback vom Formular anzeigen +++
    if (request->hasParam("status")) {
      String status = request->getParam("status")->value();
      if (status == "success") {
        response->print("      <div class='row'><div class='col-md-12'><div class='alert alert-success'><strong>Success!</strong> IR code sent via form.</div></div></div>\n");
      } else if (status == "error_missing_args") {
        response->print("      <div class='row'><div class='col-md-12'><div class='alert alert-danger'><strong>Error!</strong> Missing required form fields (type, data, length).</div></div></div>\n");
      } else if (status == "error_invalid_args") {
        response->print("      <div class='row'><div class='col-md-12'><div class='alert alert-danger'><strong>Error!</strong> Invalid form data (e.g., length 0 or empty data).</div></div></div>\n");
      } else if (status == "buttons_saved") {
        response->print("      <div class='row'><div class='col-md-12'><div class='alert alert-success'><strong>Success!</strong> Button configuration saved.</div></div></div>\n");
      }
    }
    // +++ ENDE Feedback +++

  // --- Alert Messages ---
  if (type == 1)
    response->print("      <div class='row'><div class='col-md-12'><div class='alert alert-success'><strong>" + header + "!</strong> " + message + "</div></div></div>\n");
  if (type == 2)
    response->print("      <div class='row'><div class='col-md-12'><div class='alert alert-warning'><strong>" + header + "!</strong> " + message + "</div></div></div>\n");
  if (type == 3)
    response->print("      <div class='row'><div class='col-md-12'><div class='alert alert-danger'><strong>" + header + "!</strong> " + message + "</div></div></div>\n");

  // --- Codes Transmitted Table ---
  response->print("      <div class='row'>\n");
  response->print("        <div class='col-md-12'>\n");
  response->print("          <h3>Codes Transmitted</h3>\n");
  response->print("          <table class='table table-striped' style='table-layout: fixed;'>\n");
  response->print("            <thead><tr><th>Sent</th><th>Command</th><th>Type</th><th>Length</th><th>Address</th><th>Repeat</th><th>Out</th></tr></thead>\n");
  response->print("            <tbody>\n");
  auto generateSentRow = [&](const Code& code) {
      if (code.valid) {
          String rowHtml = "              <tr class='text-uppercase'><td>" + epochToString(code.timestamp) + "</td><td><code>" + String(code.data) + "</code></td><td><code>" + String(code.encoding) + "</code></td><td><code>" + String(code.bits) + "</code></td><td><code>" + String(code.address) + "</code></td><td><code>" + String(code.repeat) + "</code></td><td><code>" + String(code.out) + "</code></td></tr>\n";
          response->print(rowHtml);
      }
  };
  generateSentRow(last_send);
  generateSentRow(last_send_2);
  generateSentRow(last_send_3);
  generateSentRow(last_send_4);
  generateSentRow(last_send_5);
  if (!last_send.valid && !last_send_2.valid && !last_send_3.valid && !last_send_4.valid && !last_send_5.valid)
    response->print("              <tr><td colspan='7' class='text-center'><em>No codes sent</em></td></tr>");
  response->print("            </tbody></table>\n");
  response->print("          </div></div>\n");

  // --- Codes Received Table ---
  response->print("      <div class='row'>\n");
  response->print("        <div class='col-md-12'>\n");
  response->print("          <h3>Codes Received</h3>\n");
  response->print("          <table class='table table-striped' style='table-layout: fixed;'>\n");
  response->print("            <thead><tr><th>Received</th><th>Command</th><th>Type</th><th>Length</th><th>Address</th></tr></thead>\n");
  response->print("            <tbody>\n");
  auto generateReceivedRow = [&](const Code& code, int id) {
      if (code.valid) {
          String rowHtml = "              <tr class='text-uppercase'><td><a href='/received?id=" + String(id) + "'>" + epochToString(code.timestamp) + "</a></td><td><code>" + String(code.data) + "</code></td><td><code>" + String(code.encoding) + "</code></td><td><code>" + String(code.bits) + "</code></td><td><code>" + String(code.address) + "</code></td></tr>\n";
          response->print(rowHtml);
      }
  };
  generateReceivedRow(last_recv, 1);
  generateReceivedRow(last_recv_2, 2);
  generateReceivedRow(last_recv_3, 3);
  generateReceivedRow(last_recv_4, 4);
  generateReceivedRow(last_recv_5, 5);
  if (!last_recv.valid && !last_recv_2.valid && !last_recv_3.valid && !last_recv_4.valid && !last_recv_5.valid)
    response->print("              <tr><td colspan='5' class='text-center'><em>No codes received</em></td></tr>");
  response->print("            </tbody></table>\n");
  response->print("          </div></div><hr />\n");

  // +++ FORMULAR ZUM SENDEN +++
  response->print("      <div class='row'>\n");
  response->print("        <div class='col-md-12'>\n");
  response->print("          <h3>Send IR Code</h3>\n");
  response->print("          <form class='form-horizontal' action='/sendir' method='post'>\n");

  // --- Hilfsvariablen & Lambdas (bleiben gleich) ---
  String tempEncoding = "nec";
  if (last_send.valid) { tempEncoding = String(last_send.encoding); tempEncoding.toLowerCase(); }
  String lastEncoding = tempEncoding;
  String lastData = last_send.valid ? String(last_send.data) : "";
  String lastBits = last_send.valid ? String(last_send.bits) : "";
  String lastAddress = last_send.valid ? String(last_send.address) : "";
  String lastRepeat = last_send.valid ? String(last_send.repeat) : "1";
  String lastOut = last_send.valid ? String(last_send.out) : "1";
  auto addSelected = [&](const String& val) { return (val.equalsIgnoreCase(lastEncoding)) ? " selected" : ""; };
  auto addOutSelected = [&](const String& val) { return (val == lastOut) ? " selected" : ""; };

  // --- Encoding Type (Dropdown) ---
  response->print("            <div class='form-group'>\n");
  response->print("              <label for='type' class='col-sm-2 control-label'>Type</label>\n");
  response->print("              <div class='col-sm-10'>\n");
  // --- KORREKTUR: Globale Funktion aufrufen ---
  // Übergibt "type" als Namen des Select-Elements und lastEncoding als vorselektierten Wert
  response->print(generateTypeDropdownHtml("type", lastEncoding));
  response->print("              </div>\n");
  response->print("            </div>\n");

  // --- Data (Hex String) ---
  response->print("            <div class='form-group'>\n");
  response->print("              <label for='data' class='col-sm-2 control-label'>Data (Hex)</label>\n");
  response->print("              <div class='col-sm-10'><input type='text' class='form-control' id='data' name='data' placeholder='e.g., FF02FD' required value='" + lastData + "'></div>\n");
  response->print("            </div>\n");

  // --- Length (Bits) ---
  response->print("            <div class='form-group'>\n");
  response->print("              <label for='length' class='col-sm-2 control-label'>Length (Bits)</label>\n");
  response->print("              <div class='col-sm-10'><input type='number' class='form-control' id='length' name='length' placeholder='e.g., 32' required value='" + lastBits + "'></div>\n");
  response->print("            </div>\n");

  // --- Address (Hex String, optional) ---
  response->print("            <div class='form-group'>\n");
  response->print("              <label for='address' class='col-sm-2 control-label'>Address (Hex, optional)</label>\n");
  response->print("              <div class='col-sm-10'><input type='text' class='form-control' id='address' name='address' placeholder='e.g., 0x404 (for Panasonic)' value='" + lastAddress + "'></div>\n");
  response->print("            </div>\n");

  // --- Repeat ---
  response->print("            <div class='form-group'>\n");
  response->print("              <label for='repeat' class='col-sm-2 control-label'>Repeat</label>\n");
  response->print("              <div class='col-sm-10'><input type='number' class='form-control' id='repeat' name='repeat' value='" + lastRepeat + "' min='1'></div>\n");
  response->print("            </div>\n");

  // --- Output Pin ---
  response->print("            <div class='form-group'>\n");
  response->print("              <label for='out' class='col-sm-2 control-label'>Output Pin</label>\n");
  response->print("              <div class='col-sm-10'>\n");
  // --- KORREKTUR: Globale Funktion aufrufen ---
  // Übergibt "out" als Namen und lastOut (als int konvertiert) als vorselektierten Wert
  response->print(generateOutDropdownHtml("out", lastOut.toInt()));
  response->print("              </div>\n");
  response->print("            </div>\n");

  // --- Submit Button ---
  response->print("            <div class='form-group'>\n");
  response->print("              <div class='col-sm-offset-2 col-sm-10'>\n");
  response->print("                <button type='submit' class='btn btn-primary'>Send IR Code</button>\n");
  response->print("              </div>\n");
  response->print("            </div>\n");

  response->print("          </form>\n");
  response->print("        </div>\n");
  response->print("      </div><hr />\n");
  // +++ ENDE FORMULAR +++

  // --- Pin Information ---
  response->print("      <div class='row'>\n");
  response->print("        <div class='col-md-12'>\n");
  response->print("          <ul class='list-unstyled'>\n");
  response->print("            <li><span class='badge'>GPIO " + String(pinr1) + "</span> Receiving </li>\n");
  response->print("            <li><span class='badge'>GPIO " + String(pins1) + "</span> Transmitter 1 </li>\n");
  response->print("            <li><span class='badge'>GPIO " + String(pins2) + "</span> Transmitter 2 </li>\n");
  response->print("            <li><span class='badge'>GPIO " + String(pins3) + "</span> Transmitter 3 </li>\n");
  response->print("            <li><span class='badge'>GPIO " + String(pins4) + "</span> Transmitter 4 </li></ul>\n");
  response->print("        </div>\n");
  response->print("      </div>\n");

  // +++ JAVASCRIPT FÜR REMOTE BUTTONS +++
  response->print("      <script>\n");
  response->print("        document.getElementById('remote-buttons').addEventListener('click', function(event) {\n");
  response->print("          if (event.target.classList.contains('remote-button')) {\n");
  response->print("            event.preventDefault();\n");
  response->print("            const button = event.target;\n");
  response->print("            const irData = {\n");
  response->print("              type: button.dataset.type,\n");
  response->print("              data: button.dataset.data,\n");
  response->print("              length: parseInt(button.dataset.length, 10),\n");
  response->print("              address: button.dataset.address,\n");
  response->print("              repeat: parseInt(button.dataset.repeat, 10),\n");
  response->print("              out: parseInt(button.dataset.out, 10)\n");
  response->print("            };\n");
  response->print("            console.log('Sending IR:', irData);\n");
  response->print("            button.classList.add('btn-warning'); \n");
  response->print("            setTimeout(() => { button.classList.remove('btn-warning'); }, 500);\n");
  response->print("            const urlParams = new URLSearchParams({\n");
  response->print("              type: irData.type,\n");
  response->print("              data: irData.data,\n");
  response->print("              length: irData.length,\n");
  response->print("              address: irData.address,\n");
  response->print("              repeat: irData.repeat,\n");
  response->print("              out: irData.out\n");
  response->print("            }).toString();\n");
  response->print("            fetch(`/sendbutton?${urlParams}`, { method: 'GET' })\n");
  response->print("            .then(response => {\n");
  response->print("              if (!response.ok) {\n");
  response->print("                console.error('Error sending IR command via GET');\n");
  response->print("                button.classList.add('btn-danger');\n");
  response->print("                setTimeout(() => { button.classList.remove('btn-danger'); }, 1000);\n");
  response->print("              } else {\n");
  response->print("                button.classList.add('btn-success');\n");
  response->print("                setTimeout(() => { button.classList.remove('btn-success'); }, 500);\n");
  response->print("              }\n");
  response->print("              return response.text();\n");
  response->print("            })\n");
  response->print("            .then(data => console.log('Server response:', data))\n");
  response->print("            .catch(error => {\n");
  response->print("              console.error('Fetch error:', error);\n");
  response->print("              button.classList.add('btn-danger');\n");
  response->print("              setTimeout(() => { button.classList.remove('btn-danger'); }, 1000);\n");
  response->print("            });\n");
  response->print("          }\n");
  response->print("        });\n");
  response->print("      </script>\n");
  // +++ ENDE JAVASCRIPT +++

  // --- Schreibe Footer in den Stream ---
  sendFooter(response); // Übergibt den Stream

  // --- Sende den kompletten Stream ---
  request->send(response);
}



//+=============================================================================
// Stream code page HTML (AsyncResponseStream Version)
//
// Overload leitet den Request weiter
void sendCodePage(AsyncWebServerRequest *request, Code selCode) {
  sendCodePage(request, selCode, 200);
}

// Hauptfunktion, die die Arbeit macht (AsyncResponseStream Version)
void sendCodePage(AsyncWebServerRequest *request, Code selCode, int httpcode){

  // --- Erstelle den Response Stream ---
  AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8", httpcode);

  // --- Schreibe Header in den Stream ---
  sendHeader(response); // Übergibt den Stream

  // --- Code Details ---
  response->print("      <div class='row'>\n");
  response->print("        <div class='col-md-12'>\n");
  // Baue den Titel-String zusammen
  String codeTitle = "          <h2><span class='label label-success'>" + String(selCode.data) + ":" + String(selCode.encoding) + ":" + String(selCode.bits) + "</span></h2><br/>\n";
  response->print(codeTitle);
  response->print("          <dl class='dl-horizontal'>\n");
  response->print("            <dt>Data</dt>\n");
  response->print("            <dd><code>" + String(selCode.data)  + "</code></dd></dl>\n");
  response->print("          <dl class='dl-horizontal'>\n");
  response->print("            <dt>Type</dt>\n");
  response->print("            <dd><code>" + String(selCode.encoding)  + "</code></dd></dl>\n");
  response->print("          <dl class='dl-horizontal'>\n");
  response->print("            <dt>Length</dt>\n");
  response->print("            <dd><code>" + String(selCode.bits)  + "</code></dd></dl>\n");
  response->print("          <dl class='dl-horizontal'>\n");
  response->print("            <dt>Address</dt>\n");
  response->print("            <dd><code>" + String(selCode.address)  + "</code></dd></dl>\n");
  response->print("          <dl class='dl-horizontal'>\n");
  response->print("            <dt>Raw</dt>\n");
  String rawDisplay = selCode.raw;
  if (rawDisplay.length() > 200) {
      rawDisplay = rawDisplay.substring(0, 200) + "...";
  }
  response->print("            <dd><code>" + rawDisplay  + "</code></dd></dl>\n");
  response->print("          <dl class='dl-horizontal'>\n");
  response->print("            <dt>Timestamp</dt>\n");
  response->print("            <dd><code>" + epochToString(selCode.timestamp)  + "</code></dd></dl>\n");
  response->print("        </div></div>\n");

  // --- Passcode Warning ---
  response->print("      <div class='row'>\n");
  response->print("        <div class='col-md-12'>\n");
  response->print("          <div class='alert alert-warning'>Don't forget to add your passcode to the URLs below if you set one</div>\n");
  response->print("      </div></div>\n");

  // --- Example URLs ---
  String hostUrlBase = "http://" + String(host_name) + ".local:" + String(port_str);
  String localUrlBase = "http://" + WiFi.localIP().toString() + ":" + String(port_str);
  // String externalUrlBase = "http://" + externalIP() + ":" + String(port_str); // Auskommentiert lassen

  if (String(selCode.encoding) == "UNKNOWN") {
    String jsonPayload = "/json?plain=[{data:[" + String(selCode.raw) + "],type:'raw',khz:38}]";
    response->print("      <div class='row'>\n");
    response->print("        <div class='col-md-12'>\n");
    response->print("          <ul class='list-unstyled'>\n");
    response->print("            <li>Hostname <span class='label label-default'>JSON</span></li>\n");
    response->print("            <li><pre>" + hostUrlBase + jsonPayload + "</pre></li>\n");
    response->print("            <li>Local IP <span class='label label-default'>JSON</span></li>\n");
    response->print("            <li><pre>" + localUrlBase + jsonPayload + "</pre></li>\n");
    // response->print("            <li>External IP <span class='label label-default'>JSON</span></li>\n");
    // response->print("            <li><pre>" + externalUrlBase + jsonPayload + "</pre></li></ul>\n");
     response->print("          </ul>\n"); // Korrigiertes Ende
    response->print("        </div></div>\n");
  } else if (String(selCode.encoding) == "PANASONIC" || String(selCode.encoding) == "NEC") {
    String msgPayload = "/msg?code=" + String(selCode.data) + ":" + String(selCode.encoding) + ":" + String(selCode.bits) + "&address=" + String(selCode.address);
    String jsonPayload = "/json?plain=[{data:'" + String(selCode.data) + "',type:'" + String(selCode.encoding) + "',length:" + String(selCode.bits) + ",address:'" + String(selCode.address) + "'}]";
    response->print("      <div class='row'>\n");
    response->print("        <div class='col-md-12'>\n");
    response->print("          <ul class='list-unstyled'>\n");
    response->print("            <li>Hostname <span class='label label-default'>MSG</span></li>\n");
    response->print("            <li><pre>" + hostUrlBase + msgPayload + "</pre></li>\n");
    response->print("            <li>Local IP <span class='label label-default'>MSG</span></li>\n");
    response->print("            <li><pre>" + localUrlBase + msgPayload + "</pre></li>\n");
    // response->print("            <li>External IP <span class='label label-default'>MSG</span></li>\n");
    // response->print("            <li><pre>" + externalUrlBase + msgPayload + "</pre></li></ul>\n");
     response->print("          </ul>\n"); // Korrigiertes Ende
    response->print("          <ul class='list-unstyled'>\n");
    response->print("            <li>Hostname <span class='label label-default'>JSON</span></li>\n");
    response->print("            <li><pre>" + hostUrlBase + jsonPayload + "</pre></li>\n");
    response->print("            <li>Local IP <span class='label label-default'>JSON</span></li>\n");
    response->print("            <li><pre>" + localUrlBase + jsonPayload + "</pre></li>\n");
    // response->print("            <li>External IP <span class='label label-default'>JSON</span></li>\n");
    // response->print("            <li><pre>" + externalUrlBase + jsonPayload + "</pre></li></ul>\n");
     response->print("          </ul>\n"); // Korrigiertes Ende
    response->print("        </div></div>\n");
  } else { // Other known encodings
    String msgPayload = "/msg?code=" + String(selCode.data) + ":" + String(selCode.encoding) + ":" + String(selCode.bits);
    String jsonPayload = "/json?plain=[{data:'" + String(selCode.data) + "',type:'" + String(selCode.encoding) + "',length:" + String(selCode.bits) + "}]";
    response->print("      <div class='row'>\n");
    response->print("        <div class='col-md-12'>\n");
    response->print("          <ul class='list-unstyled'>\n");
    response->print("            <li>Hostname <span class='label label-default'>MSG</span></li>\n");
    response->print("            <li><pre>" + hostUrlBase + msgPayload + "</pre></li>\n");
    response->print("            <li>Local IP <span class='label label-default'>MSG</span></li>\n");
    response->print("            <li><pre>" + localUrlBase + msgPayload + "</pre></li>\n");
    // response->print("            <li>External IP <span class='label label-default'>MSG</span></li>\n");
    // response->print("            <li><pre>" + externalUrlBase + msgPayload + "</pre></li></ul>\n");
     response->print("          </ul>\n"); // Korrigiertes Ende
    response->print("          <ul class='list-unstyled'>\n");
    response->print("            <li>Hostname <span class='label label-default'>JSON</span></li>\n");
    response->print("            <li><pre>" + hostUrlBase + jsonPayload + "</pre></li>\n");
    response->print("            <li>Local IP <span class='label label-default'>JSON</span></li>\n");
    response->print("            <li><pre>" + localUrlBase + jsonPayload + "</pre></li>\n");
    // response->print("            <li>External IP <span class='label label-default'>JSON</span></li>\n");
    // response->print("            <li><pre>" + externalUrlBase + jsonPayload + "</pre></li></ul>\n");
     response->print("          </ul>\n"); // Korrigiertes Ende
    response->print("        </div></div>\n");
  }

  // --- JavaScript (wird hier nicht benötigt) ---
  // ... (auskommentiert lassen oder entfernen) ...

  // --- Schreibe Footer in den Stream ---
  sendFooter(response); // Übergibt den Stream

  // --- Sende den kompletten Stream ---
  request->send(response);
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
