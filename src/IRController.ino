#include <FS.h>                                               // This needs to be first, or it all crashes and burns
#include "credentials.h"                                      // Include der Zugangsdaten-Datei

#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <WiFi.h>
// #include <WiFiManager.h>                                      // https://github.com/tzapu/WiFiManager WiFi Configuration Magic

#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncEventSource.h>
#include <HTTPClient.h>
#include <ArduinoOTA.h>

#include <Ticker.h>                                           // For LED status
#include <TimeLib.h>

#include <LittleFS.h>
#include "esp_ota_ops.h" // Für esp_ota_get_running_partition()

// User settings are below here
//+=============================================================================

const int timeZone = 2;

const bool getExternalIP = false;                               // Set to false to disable querying external IP

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

// DynamicJsonDocument deviceState(1024);
DynamicJsonDocument deviceState(256);

WiFiClient client;
AsyncWebServer *server = NULL;
AsyncEventSource *events = nullptr;                             // Nur Zeiger deklarieren
Ticker ticker;

bool shouldSaveConfig = false;                                 // Flag for saving data
bool holdReceive = false;                                      // Flag to prevent IR receiving while transmitting

IRrecv irrecv(pinr1, captureBufSize, 35);
IRsend irsend1(pins1);
IRsend irsend2(pins2);
IRsend irsend3(pins3);
IRsend irsend4(pins4);

const unsigned long resetfrequency = 259200000;                // 72 hours in milliseconds for external IP reset

bool _rc5toggle = false;
bool _rc6toggle = false;

char _ip[16] = "";

unsigned long lastupdate = 0;

bool externalIPError = false;

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

// Add near other global variables
String buttonMacroJsStore = "const buttonMacroDataStore = {};";

// Globale Variable für Datei-Upload ---
File fsUploadFile;

//+=============================================================================
// Button Configuration
//+=============================================================================
const int MAX_BUTTONS = 24; // Maximale Anzahl an Buttons

struct ButtonConfig {
  char name[32] = "";      // Name des Buttons
  char type[14] = "";      // IR Protokoll (nec, sony, etc.)
  char data[40] = "";      // IR Daten (Hex String)
  int length = 0;          // Anzahl Bits
  char address[20] = "";   // Adresse (Hex String, optional)
  int repeat = 1;          // Wiederholungen
  int out = 1;             // Output Pin (1-4)
  bool isMacro = false;    //  Ist dieser Button ein Makro?
  char macroJson[512] = ""; //  JSON-String für das Makro (Größe ggf. anpassen)
  bool configured = false; // Ist dieser Button-Slot konfiguriert?
  int layoutRow = -1; // Zeile im Layout (-1 = nicht spezifiziert/Standardfluss)
  int layoutCol = -1; // Spalte im Layout (-1 = nicht spezifiziert/Standardfluss)
  char colorClass[16] = "btn-primary"; // Bootstrap-Klasse (z.B. "btn-success"), Default: primary
 
};

std::vector<ButtonConfig> buttonConfigs;
//+=============================================================================


// +++ HILFSFUNKTION ZUR NORMALISIERUNG +++
String normalizeHex(String hexStr) {
  hexStr.trim(); // Entferne Leerzeichen am Anfang/Ende
  if (hexStr.startsWith("0x")) {
    hexStr = hexStr.substring(2); // Entferne "0x"
  }
  hexStr.toUpperCase(); // Alles groß schreiben
  return hexStr;
}

// +++ findMatchingButtonName (mit numerischem Adressvergleich) +++
const ButtonConfig* findMatchingButton(const Code& codeToMatch) {
  if (!codeToMatch.valid || strlen(codeToMatch.encoding) == 0 || strlen(codeToMatch.data) == 0 || codeToMatch.bits <= 0) {
      return nullptr;
  }

  String codeTypeLower = String(codeToMatch.encoding);
  codeTypeLower.toLowerCase();
  String normCodeData = normalizeHex(String(codeToMatch.data));
  String normCodeAddressStr = normalizeHex(String(codeToMatch.address)); // Adresse als String

  // --- DEBUGGING: Gib die normalisierten Werte aus ---
  // Serial.printf("  Matching against: Type=%s, Data=%s, Bits=%d, AddressStr=%s\n",
  //               codeTypeLower.c_str(), normCodeData.c_str(), codeToMatch.bits, normCodeAddressStr.c_str());
  // --- ENDE DEBUGGING ---

  for (const auto& button : buttonConfigs) {
      if (!button.configured || button.isMacro) {
          continue;
      }

      String buttonTypeLower = String(button.type);
      buttonTypeLower.toLowerCase();
      String normButtonData = normalizeHex(String(button.data));
      String normButtonAddressStr = normalizeHex(String(button.address)); // Adresse als String

      // --- DEBUGGING: Gib die normalisierten Button-Werte aus ---
      // Serial.printf("    Comparing with Button '%s': Type=%s, Data=%s, Len=%d, AddressStr=%s\n",
      //               button.name, buttonTypeLower.c_str(), normButtonData.c_str(), button.length, normButtonAddressStr.c_str());
      // --- ENDE DEBUGGING ---

      // --- Kernvergleich (Typ, Daten, Länge) ---
      if (codeTypeLower == buttonTypeLower &&
          normCodeData == normButtonData &&
          codeToMatch.bits == button.length) {

          // --- KORRIGIERTER Adress-Vergleich (numerisch) ---
          bool addressMatch = true;
          // Wandle normalisierte Adress-Strings in Zahlen um (Basis 16)
          // Wichtig: strtoul gibt 0 zurück, wenn der String leer ist oder ungültig. Das passt für uns.
          unsigned long codeAddrNum = strtoul(normCodeAddressStr.c_str(), NULL, 16);
          unsigned long buttonAddrNum = strtoul(normButtonAddressStr.c_str(), NULL, 16);

          // Serial.printf("      Address Check: CodeNum=%lu, ButtonNum=%lu\n", codeAddrNum, buttonAddrNum); // Debug

          // Vergleiche die numerischen Werte
          if (buttonAddrNum != 0) { // Nur wenn der Button eine Adresse != 0 hat...
              if (codeAddrNum != buttonAddrNum) { // ...muss die Code-Adresse exakt übereinstimmen.
                  addressMatch = false;
                  // Serial.println("      Address mismatch (Button requires specific address)!"); // Debug
              }
          }
          // Wenn buttonAddrNum == 0 ist, ist addressMatch standardmäßig true (wir ignorieren die Adresse)
          // --- ENDE KORRIGIERTER Adress-Vergleich ---

          if (addressMatch) {
              // Serial.printf("      MATCH FOUND! Button: %s\n", button.name); // Debug
              return &button; // Treffer gefunden!
          } else {
              // Serial.println("      Address mismatch prevented match."); // Debug
          }
      } // Ende Kernvergleich
  } // Ende for-Schleife

  // Serial.println("  No match found for this code."); // Debug
  return nullptr; // Kein passender Button gefunden
}


// Funktion: Speichert die aktuelle Konfiguration in eine spezifische Datei
bool saveConfigToFile(const char* filePath) {
  Serial.printf("==> saveConfigToFile: Saving current config to '%s'\n", filePath);

  // Optional: Prüfen, ob das Root-Verzeichnis existiert
  if (!LittleFS.exists("/")) {
    Serial.println("    ERROR: LittleFS root directory not found (FS likely not mounted).");
    return false;
  }

  // Stelle sicher, dass das Verzeichnis existiert (falls filePath einen Pfad enthält)
  String pathStr = String(filePath);
  int lastSlash = pathStr.lastIndexOf('/');
  if (lastSlash > 0) {
      String dirPath = pathStr.substring(0, lastSlash);
      if (!LittleFS.exists(dirPath)) {
          Serial.printf("    Directory '%s' does not exist. Creating...\n", dirPath.c_str());
          if (!LittleFS.mkdir(dirPath)) {
              Serial.println("    ERROR: Failed to create directory!");
              return false;
          }
      }
  }


  DynamicJsonDocument jsonDoc(8192); // Größe ggf. erhöhen, falls viele große Makros
  JsonArray buttonArray = jsonDoc.to<JsonArray>();

  Serial.printf("    Serializing %d buttons...\n", buttonConfigs.size());
  for (const auto& button : buttonConfigs) {
     if (!button.configured) continue; // Nur konfigurierte speichern

     JsonObject buttonJson = buttonArray.createNestedObject();
     buttonJson["name"] = button.name;
     buttonJson["configured"] = button.configured;
     buttonJson["isMacro"] = button.isMacro;
     if (button.isMacro) {
        buttonJson["macroJson"] = button.macroJson;
        buttonJson["type"] = ""; buttonJson["data"] = ""; buttonJson["length"] = 0;
        buttonJson["address"] = ""; buttonJson["repeat"] = 1; buttonJson["out"] = 1;
     } else {
        buttonJson["type"] = button.type; buttonJson["data"] = button.data;
        buttonJson["length"] = button.length; buttonJson["address"] = button.address;
        buttonJson["repeat"] = button.repeat; buttonJson["out"] = button.out;
        buttonJson["macroJson"] = "";
     }
     buttonJson["layoutRow"] = button.layoutRow;
     buttonJson["layoutCol"] = button.layoutCol;
     buttonJson["colorClass"] = button.colorClass;
  }

  File configFile = LittleFS.open(filePath, "w");
  if (!configFile) {
    Serial.printf("    ERROR: Failed to open '%s' for writing!\n", filePath);
    return false;
  }
  Serial.printf("    '%s' opened for writing.\n", filePath);

  size_t bytesWritten = serializeJson(jsonDoc, configFile);
  bool success = false;
  if (bytesWritten == 0 && jsonDoc.size() > 0) {
    Serial.printf("    ERROR: Failed to write to '%s' (serializeJson returned 0).\n", filePath);
  } else {
    Serial.printf("    %d bytes written to '%s'.\n", bytesWritten, filePath);
    success = true;
  }
  configFile.close();
  Serial.printf("<== saveConfigToFile: Leaving function for '%s'. Success: %d\n", filePath, success);
  return success;
}


// Handler: Speichert die aktuelle Konfiguration als benanntes Backup
void handleSaveNamedBackup(AsyncWebServerRequest *request) {
  Serial.println("==> handleSaveNamedBackup: Entered function.");
  if (!request->hasParam("backup_name", true)) { // true = check POST body
    request->redirect("/buttons?status=error_missing_backup_name");
    return;
  }
  String backupName = request->getParam("backup_name", true)->value();
  backupName.trim();

  if (backupName.length() == 0) {
    request->redirect("/buttons?status=error_empty_backup_name");
    return;
  }

  // --- Namen bereinigen (einfache Version) ---
  String sanitizedName = "";
  for (char c : backupName) {
    if (isalnum(c) || c == '_' || c == '-') { // Erlaube Buchstaben, Zahlen, Unterstrich, Bindestrich
      sanitizedName += c;
    }
  }
  if (sanitizedName.length() == 0) { // Falls nach Bereinigung leer
      request->redirect("/buttons?status=error_invalid_backup_name");
      return;
  }
  // --- Ende Bereinigung ---

  String filePath = "/backups/" + sanitizedName + ".json";

  Serial.printf("    Attempting to save backup to: %s\n", filePath.c_str());

  if (saveConfigToFile(filePath.c_str())) {
    request->redirect("/buttons?status=backup_saved");
  } else {
    request->redirect("/buttons?status=error_backup_save");
  }
}


// Handler: Lädt ein benanntes Backup und macht es zur aktiven Konfiguration
void handleLoadNamedBackup(AsyncWebServerRequest *request) {
  Serial.println("==> handleLoadNamedBackup: Entered function.");
  if (!request->hasParam("name")) { // name kommt aus URL-Parameter
    request->redirect("/buttons?status=error_missing_backup_name");
    return;
  }
  String backupFilename = request->getParam("name")->value();

  // Sicherheitscheck: Ist der Name plausibel? (z.B. keine Pfadtrenner)
  if (backupFilename.indexOf('/') != -1 || backupFilename.indexOf('\\') != -1 || backupFilename == "." || backupFilename == "..") {
      request->redirect("/buttons?status=error_invalid_backup_name");
      return;
  }

  String backupFilePath = "/backups/" + backupFilename;
  String activeFilePath = "/buttons.json";

  Serial.printf("    Attempting to load backup from: %s\n", backupFilePath.c_str());

  if (!LittleFS.exists(backupFilePath)) {
    Serial.println("    ERROR: Backup file not found.");
    request->redirect("/buttons?status=error_backup_not_found");
    return;
  }

  // --- Datei kopieren (Backup -> Aktiv) ---
  File sourceFile = LittleFS.open(backupFilePath, "r");
  File destFile = LittleFS.open(activeFilePath, "w");

  if (!sourceFile || !destFile) {
    Serial.println("    ERROR: Failed to open source or destination file for copying.");
    if (sourceFile) sourceFile.close();
    if (destFile) destFile.close();
    request->redirect("/buttons?status=error_backup_load");
    return;
  }

  Serial.println("    Copying backup file to active configuration...");
  size_t bufferSize = 512;
  uint8_t buffer[bufferSize];
  size_t bytesRead = 0;
  size_t totalBytesCopied = 0;
  while ((bytesRead = sourceFile.read(buffer, bufferSize)) > 0) {
    size_t bytesWritten = destFile.write(buffer, bytesRead);
    if (bytesWritten != bytesRead) {
        Serial.println("    ERROR: Failed during file copy!");
        sourceFile.close();
        destFile.close();
        request->redirect("/buttons?status=error_backup_load");
        return;
    }
    totalBytesCopied += bytesWritten;
    yield(); // Wichtig bei größeren Dateien
  }

  sourceFile.close();
  destFile.close();
  Serial.printf("    Successfully copied %d bytes.\n", totalBytesCopied);
  // --- Ende Datei kopieren ---

  // --- Konfiguration neu laden ---
  Serial.println("    Reloading configuration from newly copied file...");
  loadButtonConfig(); // Lädt /buttons.json neu in den buttonConfigs Vektor
  // updateButtonMacroJsStore() wird innerhalb von loadButtonConfig aufgerufen

  request->redirect("/buttons?status=backup_loaded");
}


// Handler: Löscht ein benanntes Backup
void handleDeleteNamedBackup(AsyncWebServerRequest *request) {
  Serial.println("==> handleDeleteNamedBackup: Entered function.");
  if (!request->hasParam("name")) {
    request->redirect("/buttons?status=error_missing_backup_name");
    return;
  }
  String backupFilename = request->getParam("name")->value();

  // Sicherheitscheck
  if (backupFilename.indexOf('/') != -1 || backupFilename.indexOf('\\') != -1 || backupFilename == "." || backupFilename == "..") {
      request->redirect("/buttons?status=error_invalid_backup_name");
      return;
  }

  String filePath = "/backups/" + backupFilename;

  Serial.printf("    Attempting to delete backup: %s\n", filePath.c_str());

  if (!LittleFS.exists(filePath)) {
    Serial.println("    ERROR: Backup file not found for deletion.");
    request->redirect("/buttons?status=error_backup_not_found");
    return;
  }

  if (LittleFS.remove(filePath)) {
    Serial.println("    Backup deleted successfully.");
    request->redirect("/buttons?status=backup_deleted");
  } else {
    Serial.println("    ERROR: Failed to delete backup file.");
    request->redirect("/buttons?status=error_backup_delete");
  }
}


// Function to update the JS store string (Optimized with reserve)
// Version OHNE deserializeJson-Validierung
void updateButtonMacroJsStore() {

  String tempJs;
  size_t estimatedSize = 8192;
  if (!tempJs.reserve(estimatedSize)) {
      Serial.println("  !!! WARNING: Failed to reserve memory for tempJs!");
  }

  tempJs = "const buttonMacroDataStore = {";
  bool firstEntry = true;

  for (size_t i = 0; i < buttonConfigs.size(); ++i) {
      // Prüfe nur noch, ob es ein konfigurierter Makro-Button ist
      if (buttonConfigs[i].configured && buttonConfigs[i].isMacro) {

          // --- KEINE KOPIE, KEIN deserializeJson MEHR ---

          // Direkter Aufbau von tempJs mit dem Original
          if (!firstEntry) {
              tempJs += ",";
          }
          tempJs += "\n  '";
          tempJs += "btn_";
          tempJs += String(i);
          tempJs += "': `";
          // --- VERWENDE DAS ORIGINAL zum Aufbau des JS-Strings ---
          tempJs += String(buttonConfigs[i].macroJson); // Hier das Original verwenden!
          tempJs += "`";
          firstEntry = false;
      }
  }
  tempJs += "\n};";

  buttonMacroJsStore = tempJs;
  // Serial.printf("    buttonMacroJsStore updated. Length: %d\n", buttonMacroJsStore.length());
}


// --- Hilfsfunktion zum Senden von Code-Updates als SSE ---
void sendCodeUpdateEvent(const char* eventName, const Code& code) {
  if (events != nullptr && events->count() > 0) {
    DynamicJsonDocument jsonDoc(512); // Ggf. Größe leicht erhöhen
    jsonDoc["encoding"] = code.encoding;
    jsonDoc["data"] = code.data;
    jsonDoc["bits"] = code.bits;
    jsonDoc["address"] = code.address;
    jsonDoc["repeat"] = code.repeat;
    jsonDoc["out"] = code.out;
    jsonDoc["timestamp"] = epochToString(code.timestamp);

    const ButtonConfig* matchedButton = findMatchingButton(code); // <-- Aufruf der neuen Funktion
    if (matchedButton != nullptr) {
        jsonDoc["matchedButtonName"] = matchedButton->name;
        jsonDoc["matchedButtonColor"] = matchedButton->colorClass; // Die Original-Klasse (z.B. btn-primary)
    } else {
        jsonDoc["matchedButtonName"] = ""; // Leerer String, wenn kein Match
        jsonDoc["matchedButtonColor"] = ""; // Leerer String, wenn kein Match
    }

    String jsonString;
    serializeJson(jsonDoc, jsonString);
    events->send(jsonString.c_str(), eventName, millis());
  }
}


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
  
  // Optional: Prüfen, ob das Root-Verzeichnis existiert (Indikator für gemountetes FS)
  if (!LittleFS.exists("/")) {
    Serial.println("ERROR in loadButtonConfig: LittleFS root directory not found (FS likely not mounted).");
    return; // Funktion verlassen, da FS nicht verfügbar ist
 }

  if (LittleFS.exists("/buttons.json")) {
    File configFile = LittleFS.open("/buttons.json", "r");
    if (configFile) {
      DynamicJsonDocument jsonDoc(8192); // Größe ggf. anpassen (9 Buttons * ~150 Zeichen)
      DeserializationError error = deserializeJson(jsonDoc, configFile);
      configFile.close(); // Datei schließen, sobald gelesen

      if (!error) {
        JsonArray buttonArray = jsonDoc.as<JsonArray>();
        int count = 0;
        for (JsonObject buttonJson : buttonArray) {

          // Optional: Limit prüfen, falls MAX_BUTTONS noch verwendet wird
          if (buttonConfigs.size() >= MAX_BUTTONS) {
            break;
          }
          
          ButtonConfig newButton; // Temporäres Objekt erstellen

          strncpy(newButton.name, buttonJson["name"] | "", sizeof(newButton.name) - 1);
          // --- Makro-Felder lesen ---
          newButton.isMacro = buttonJson["isMacro"] | false;
          strncpy(newButton.macroJson, buttonJson["macroJson"] | "", sizeof(newButton.macroJson) - 1);

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
          newButton.macroJson[sizeof(newButton.macroJson) - 1] = '\0';
          newButton.layoutRow = buttonJson["layoutRow"] | -1; // Default -1, falls nicht in JSON
          newButton.layoutCol = buttonJson["layoutCol"] | -1; // Default -1, falls nicht in JSON
          strncpy(newButton.colorClass, buttonJson["colorClass"] | "btn-primary", sizeof(newButton.colorClass) - 1); // Default "btn-primary"
          newButton.colorClass[sizeof(newButton.colorClass) - 1] = '\0'; // Null-terminieren
          

          // Gültigkeitsprüfung anpassen:
          // Ein Button ist gültig, wenn er einen Namen hat UND
          // (entweder KEIN Makro ist UND gültige Einzelbefehl-Daten hat) ODER (ein Makro ist UND einen JSON-String hat)
          bool isValidSingle = !newButton.isMacro && strlen(newButton.name) > 0 && strlen(newButton.data) > 0 && newButton.length > 0;
          bool isValidMacro = newButton.isMacro && strlen(newButton.name) > 0 && strlen(newButton.macroJson) > 0;

          // Setze 'configured' basierend auf der Gültigkeit
          newButton.configured = (isValidSingle || isValidMacro);

          if (newButton.configured) {
             buttonConfigs.push_back(newButton); // Zum Vector hinzufügen
          }

        }
        Serial.printf("Loaded %d buttons successfully.\n", buttonConfigs.size());
        updateButtonMacroJsStore(); // <-- ADD THIS CALL
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


// Die alte saveButtonConfig ruft jetzt die neue Funktion auf
void saveButtonConfig() {
  if (saveConfigToFile("/buttons.json")) {
      updateButtonMacroJsStore(); // Nur JS Store aktualisieren, wenn die *aktive* Config gespeichert wurde
  }
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


// //+=============================================================================
// // Gets called when WiFiManager enters configuration mode
// //
// void configModeCallback (WiFiManager *myWiFiManager) {
//   Serial.println("Entered config mode");
//   Serial.println(WiFi.softAPIP());
//   //if you used auto generated SSID, print it
//   Serial.println(myWiFiManager->getConfigPortalSSID());
//   //entered config mode, make led toggle faster
//   ticker.attach(0.2, tick);
// }


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
  
  // // WiFiManager
  // // Local intialization. Once its business is done, there is no need to keep it around
  // WiFiManager wifiManager;

  // // set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  // wifiManager.setAPCallback(configModeCallback);
  // // set config save notify callback
  // wifiManager.setSaveConfigCallback(saveConfigCallback);

  // // Reset device if on config portal for greater than 3 minutes
  // wifiManager.setConfigPortalTimeout(180);

  // if (LittleFS.begin(true)) {
  //   Serial.println("mounted file system");
  //   // if (LittleFS.exists("/config.json")) {
  //   //   //file exists, reading and loading
  //   //   Serial.println("reading config file");
  //   //   File configFile = LittleFS.open("/config.json", "r");
  //   //   if (configFile) {
  //   //     Serial.println("opened config file");
  //   //     size_t size = configFile.size();
  //   //     // Allocate a buffer to store contents of the file.
  //   //     std::unique_ptr<char[]> buf(new char[size]);

  //   //     configFile.readBytes(buf.get(), size);
  //   //     DynamicJsonDocument json(1024);
  //   //     DeserializationError error = deserializeJson(json, buf.get());
  //   //     serializeJson(json, Serial);
  //   //     if (!error) {
  //   //       Serial.println("\nparsed json");

  //   //       if (json.containsKey("hostname")) strncpy(host_name, json["hostname"], 20);
  //   //       if (json.containsKey("passcode")) strncpy(passcode, json["passcode"], 20);
  //   //       if (json.containsKey("port_str")) {
  //   //         strncpy(port_str, json["port_str"], 6);
  //   //         port = atoi(json["port_str"]);
  //   //       }
  //   //       if (json.containsKey("ip")) strncpy(static_ip, json["ip"], 16);
  //   //       if (json.containsKey("gw")) strncpy(static_gw, json["gw"], 16);
  //   //       if (json.containsKey("sn")) strncpy(static_sn, json["sn"], 16);
  //   //       if (json.containsKey("dns")) strncpy(static_dns, json["dns"], 16);
  //   //     } else {
  //   //       Serial.println("failed to load json config");
  //   //     }
  //   //   }
  //   // }
  // } else {
  //   Serial.println("failed to mount FS");
  // }

  // WiFiManagerParameter custom_hostname("hostname", "Choose a hostname to this IR Controller", host_name, 20);
  // wifiManager.addParameter(&custom_hostname);
  // WiFiManagerParameter custom_passcode("passcode", "Choose a passcode", passcode, 20);
  // wifiManager.addParameter(&custom_passcode);
  // WiFiManagerParameter custom_port("port_str", "Choose a port", port_str, 6);
  // wifiManager.addParameter(&custom_port);

  // wifiManager.setShowStaticFields(true);
  // wifiManager.setShowDnsFields(true);

  // IPAddress sip, sgw, ssn, dns;
  // sip.fromString(static_ip);
  // sgw.fromString(static_gw);
  // ssn.fromString(static_sn);
  // dns.fromString(static_dns);

  // if (resetConf) {
  //   Serial.println("Reset triggered, launching in AP mode");
  //   wifiManager.startConfigPortal(wifi_config_name);
  // } else {
  //   Serial.println("Setting static WiFi data from config");
  //   wifiManager.setSTAStaticIPConfig(sip, sgw, ssn, dns);
  // }

  // fetches ssid and pass and tries to connect
  // if it does not connect it starts an access point with the specified name
  // and goes into a blocking loop awaiting configuration
  // if (!wifiManager.autoConnect(wifi_config_name)) {
  //   Serial.println("Failed to connect and hit timeout");
  //   // reset and try again, or maybe put it to deep sleep
  //   ESP.restart();
  //   delay(1000);
  // }

  // if you get here you have connected to the WiFi
  // strncpy(host_name, custom_hostname.getValue(), 20);
  // strncpy(passcode, custom_passcode.getValue(), 20);
  // strncpy(port_str, custom_port.getValue(), 6);
  strncpy(host_name, custom_hostname, 20);
  strncpy(passcode, custom_passcode, 20);
  strncpy(port_str, custom_port, 6);
  port = atoi(port_str);

  // --- PRÜFUNG ---
  port = atoi(port_str);
  if (port <= 0 || port > 65535) { // Prüft auf Fehler bei atoi() oder ungültigen Portbereich
      Serial.print("Warning: Invalid port '");
      Serial.print(port_str);
      Serial.println("' detected. Defaulting to port 80.");
      port = 80; // Setze auf Standardwert 80
      strcpy(port_str, "80"); // Korrigiere auch den String für Konsistenz
  }
  // --- ENDE PRÜFUNG ---
  
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


void flushStep(String step, File jsf, size_t wc) {

  // --- Zusätzlicher Check ---
size_t currentSize = jsf.size();
int currentError = jsf.getWriteError();
// Serial.printf("  Checkpoint: Size before flush: %d, Error: %d, Heap: %u\n", currentSize, currentError, ESP.getFreeHeap());
jsf.flush(); // Versuch, hier schon zu flushen
delay(50); // <-- Kleine Verzögerung
currentSize = jsf.size();
currentError = jsf.getWriteError();
// Serial.printf("  Checkpoint 1: Size after flush: %d, Error: %d, Heap: %u\n", currentSize, currentError, ESP.getFreeHeap());
if (currentSize == 0 && wc > 0) { // written_chunk vom ersten print
    Serial.println("  !!! ERROR DETECTED: Size reset to 0 after Checkpoint 1 flush!");
    // Hier könnte man ggf. abbrechen
}

}


// Überarbeitete Funktion zum Generieren und Schreiben von JavaScript
void generateAndWriteJavaScript() {
  Serial.println("Checking/Generating JavaScript (/js/scripts.js)...");

  // 1. Generiere den JavaScript-Inhalt in einen String im RAM
  String newJsContent;
  size_t estimatedSize = 8192; // Passe die Größe basierend auf der erwarteten JS-Größe an
  if (!newJsContent.reserve(estimatedSize)) {
      Serial.println("  !!! WARNING: Failed to reserve memory for newJsContent!");
      // Optional: Abbruch oder weitermachen mit Risiko
  } else {
      Serial.printf("  Reserved %d bytes for newJsContent.\n", estimatedSize);
  }

  // --- Baue den newJsContent String auf ---
  // Verwende newJsContent += F("...") oder newJsContent += String(...)
  // Beispiel (ersetze alle jsFile.print durch +=):
  newJsContent += F("/* --- Helper Functions --- */\n");
  newJsContent += F("function setButtonState(button, state, resetDelay = 750) {\n");
  newJsContent += F("  if (!button) return;\n");
  newJsContent += F("  button.classList.remove('btn-warning', 'btn-success', 'btn-danger');\n");
  newJsContent += F("  button.disabled = (state === 'sending');\n");
  newJsContent += F("  if (state === 'sending') button.classList.add('btn-warning');\n");
  newJsContent += F("  if (state === 'success') button.classList.add('btn-success');\n");
  newJsContent += F("  if (state === 'error') button.classList.add('btn-danger');\n");
  newJsContent += F("  if (state === 'success' || state === 'error') {\n");
  newJsContent += F("    setTimeout(() => {\n");
  newJsContent += F("      button.classList.remove('btn-success', 'btn-danger');\n");
  newJsContent += F("      button.disabled = false;\n");
  newJsContent += F("    }, resetDelay);\n");
  newJsContent += F("  } else if (state === 'reset') {\n");
  newJsContent += F("     button.disabled = false;\n");
  newJsContent += F("  }\n");
  newJsContent += F("}\n\n");

  newJsContent += F("function toggleButtonFields(isMacro) {\n");
  newJsContent += F("  const singleFieldsContainer = document.getElementById('single-ir-fields');\n");
  newJsContent += F("  const macroFieldContainer = document.getElementById('macro-json-field');\n");
  newJsContent += F("  const singleFieldIds = ['btn_type', 'btn_data', 'btn_length', 'btn_address', 'btn_repeat', 'btn_out'];\n");
  newJsContent += F("  const macroFieldIds = ['btn_macroJson'];\n");
  newJsContent += F("  const requiredSingleIds = ['btn_type', 'btn_data', 'btn_length'];\n");
  newJsContent += F("  const requiredMacroIds = ['btn_macroJson'];\n");
  newJsContent += F("\n");
  newJsContent += F("  if (isMacro) {\n");
  newJsContent += F("    if (singleFieldsContainer) singleFieldsContainer.style.display = 'none';\n");
  newJsContent += F("    if (macroFieldContainer) macroFieldContainer.style.display = 'block';\n");
  newJsContent += F("\n");
  newJsContent += F("    singleFieldIds.forEach(id => {\n");
  newJsContent += F("      const el = document.getElementById(id);\n");
  newJsContent += F("      if (el) { el.required = false; el.disabled = true; }\n");
  newJsContent += F("    });\n");
  newJsContent += F("    macroFieldIds.forEach(id => {\n");
  newJsContent += F("      const el = document.getElementById(id);\n");
  newJsContent += F("      if (el) {\n");
  newJsContent += F("        el.disabled = false; \n");
  newJsContent += F("        el.required = requiredMacroIds.includes(id);\n");
  newJsContent += F("      }\n");
  newJsContent += F("    });\n");
  newJsContent += F("  } else {\n");
  newJsContent += F("    if (singleFieldsContainer) singleFieldsContainer.style.display = 'block';\n");
  newJsContent += F("    if (macroFieldContainer) macroFieldContainer.style.display = 'none';\n");
  newJsContent += F("\n");
  newJsContent += F("    macroFieldIds.forEach(id => {\n");
  newJsContent += F("      const el = document.getElementById(id);\n");
  newJsContent += F("      if (el) { el.required = false; el.disabled = true; }\n");
  newJsContent += F("    });\n");
  newJsContent += F("    singleFieldIds.forEach(id => {\n");
  newJsContent += F("      const el = document.getElementById(id);\n");
  newJsContent += F("      if (el) {\n");
  newJsContent += F("        el.disabled = false; \n");
  newJsContent += F("        el.required = requiredSingleIds.includes(id);\n");
  newJsContent += F("      }\n");
  newJsContent += F("    });\n");
  newJsContent += F("  }\n");
  newJsContent += F("}\n\n");

  newJsContent += F("/* --- SSE Logic --- */\n");
  newJsContent += F("console.log('Setting up EventSource...');\n");
  newJsContent += F("const evtSource = new EventSource('/events');\n");
  newJsContent += F("const MAX_TABLE_ROWS = 5;\n");
  
  newJsContent += F("function addTableRow(tableBodyId, codeData, isSentTable) {\n");
  newJsContent += F("  const tableBody = document.getElementById(tableBodyId);\n");
  newJsContent += F("  if (!tableBody) return;\n");
  newJsContent += F("\n");
  newJsContent += F("  // Platzhalter entfernen\n");
  newJsContent += F("  const placeholderId = isSentTable ? 'no-sent-codes' : 'no-received-codes';\n");
  newJsContent += F("  const placeholderRow = document.getElementById(placeholderId);\n");
  newJsContent += F("  if (placeholderRow) placeholderRow.remove();\n");
  newJsContent += F("\n");
  newJsContent += F("  // Neue Zeile erstellen\n");
  newJsContent += F("  let newRowHtml = `<tr class='text-uppercase'>`;\n");
  newJsContent += F("  newRowHtml += `<td>${codeData.timestamp}</td>`;\n");
  newJsContent += F("  newRowHtml += `<td><code>${codeData.data}</code></td>`;\n");
  newJsContent += F("  newRowHtml += `<td><code>${codeData.encoding}</code></td>`;\n");
  newJsContent += F("  newRowHtml += `<td><code>${codeData.bits}</code></td>`;\n");
  newJsContent += F("  newRowHtml += `<td><code>${codeData.address || '-'}</code></td>`;\n");
  newJsContent += F("\n");
  newJsContent += F("  // Zusätzliche Spalten für 'Sent' Tabelle\n");
  newJsContent += F("  if (isSentTable) {\n");
  newJsContent += F("    newRowHtml += `<td><code>${codeData.repeat}</code></td>`;\n");
  newJsContent += F("    newRowHtml += `<td><code>${codeData.out}</code></td>`;\n");
  newJsContent += F("  }\n");
  newJsContent += F("\n");
  newJsContent += F("  // +++ ANGEPASSTE ZELLE FÜR BUTTON MATCH +++\n");
  newJsContent += F("  let matchCell = '<td>-</td>'; // Default: Kein Match\n");

  newJsContent += F("  // Prüfe, ob Name UND Farbe vorhanden sind (Match gefunden)\n");
  newJsContent += F("  if (codeData.matchedButtonName && codeData.matchedButtonName.length > 0 && codeData.matchedButtonColor && codeData.matchedButtonColor.length > 0) {\n");
  newJsContent += F("      const labelClass = codeData.matchedButtonColor.replace('btn-', 'label-');\n");
  newJsContent += F("      matchCell = `<td><span class='label ${labelClass}'>${codeData.matchedButtonName}</span></td>`;\n");
  newJsContent += F("  }\n");
  // --- NEU: Wenn kein Match UND es die Received-Tabelle ist -> Create Button Link ---
  newJsContent += F("  else if (!isSentTable) { // Nur für Received-Tabelle\n");
  newJsContent += F("      let createUrl = `/addbutton?prefill_type=${encodeURIComponent(codeData.encoding || '')}`;\n"); // URL Encoding sicherheitshalber
  newJsContent += F("      createUrl += `&prefill_data=${encodeURIComponent(codeData.data || '')}`;\n");
  newJsContent += F("      createUrl += `&prefill_length=${codeData.bits || ''}`;\n");
  newJsContent += F("      // Adresse nur hinzufügen, wenn vorhanden und nicht '0x0' (vereinfachte Prüfung)\n");
  newJsContent += F("      if (codeData.address && codeData.address !== '0x0' && codeData.address !== '0') {\n");
  newJsContent += F("          createUrl += `&prefill_address=${encodeURIComponent(codeData.address)}`;\n");
  newJsContent += F("      }\n");
  newJsContent += F("      matchCell = `<td><a href='${createUrl}' class='btn btn-xs btn-success' title='Create button from this code'>🆕 Create Button</a></td>`;\n");
  newJsContent += F("  }\n");
  newJsContent += F("  newRowHtml += matchCell; // Füge die Match-Zelle hinzu\n");
  newJsContent += F("  // +++ ENDE ANGEPASSTE ZELLE +++\n");
  newJsContent += F("\n");
  newJsContent += F("  newRowHtml += `</tr>`;\n");
  newJsContent += F("\n");
  newJsContent += F("  // Zeile am Anfang einfügen\n");
  newJsContent += F("  tableBody.insertAdjacentHTML('afterbegin', newRowHtml);\n");
  newJsContent += F("\n");
  newJsContent += F("  // Alte Zeilen entfernen, wenn Limit überschritten\n");
  newJsContent += F("  while (tableBody.rows.length > MAX_TABLE_ROWS) {\n");
  newJsContent += F("    tableBody.deleteRow(-1); // Letzte Zeile löschen\n");
  newJsContent += F("  }\n");
  newJsContent += F("}\n"); // Ende der addTableRow Funktion
  
  newJsContent += F("evtSource.addEventListener('codeSent', function(event) {\n");
  newJsContent += F("  console.log('SSE codeSent:', event.data);\n");
  newJsContent += F("  try { const codeData = JSON.parse(event.data); addTableRow('sent-codes-body', codeData, true); } catch (e) { console.error('Error parsing codeSent data:', e); }\n");
  newJsContent += F("});\n");
  newJsContent += F("evtSource.addEventListener('codeReceived', function(event) {\n");
  newJsContent += F("  console.log('SSE codeReceived:', event.data);\n");
  newJsContent += F("  try { const codeData = JSON.parse(event.data); addTableRow('received-codes-body', codeData, false); } catch (e) { console.error('Error parsing codeReceived data:', e); }\n");
  newJsContent += F("});\n");
  newJsContent += F("evtSource.onerror = function(err) { console.error('EventSource failed:', err); };\n\n");

  newJsContent += F("/* --- Initializations & Listeners --- */\n");
  newJsContent += F("  /* Remote Button Handler */\n");
  newJsContent += F("  console.log('Attempting to find #remote-buttons...');\n");
  newJsContent += F("  const remoteButtonsContainer = document.getElementById('remote-buttons');\n");
  newJsContent += F("  if (remoteButtonsContainer) {\n");
  newJsContent += F("    console.log('#remote-buttons found. Attaching listener...');\n");
  newJsContent += F("    remoteButtonsContainer.addEventListener('click', function(event) {\n");
  newJsContent += F("      console.log('Click detected inside container.');\n");
  newJsContent += F("      if (event.target.classList.contains('remote-button')) {\n");
  newJsContent += F("        event.preventDefault();\n");
  newJsContent += F("        const button = event.target;\n");
  newJsContent += F("        const isMacro = button.dataset.ismacro === 'true';\n");
  newJsContent += F("        const buttonId = button.id;\n");
  newJsContent += F("        console.log('Button clicked:', button.textContent, 'Is Macro:', isMacro, 'ID:', buttonId);\n");
  newJsContent += F("        if (typeof setButtonState !== 'function') { console.error('setButtonState missing!'); alert('Internal Error'); return; }\n");
  newJsContent += F("        setButtonState(button, 'sending');\n");
  newJsContent += F("        if (isMacro) {\n");
  newJsContent += F("          if (typeof buttonMacroDataStore === 'undefined' || !buttonMacroDataStore.hasOwnProperty(buttonId)) { console.error('Macro data missing for', buttonId); setButtonState(button, 'error'); alert('Error: Macro data missing.'); return; }\n");
  newJsContent += F("          const macroJsonString = buttonMacroDataStore[buttonId];\n");
  newJsContent += F("          console.log('Sending Macro JSON:', macroJsonString);\n");
  newJsContent += F("          const formData = new URLSearchParams(); formData.append('plain', macroJsonString);\n");
  newJsContent += F("          fetch('/json', { method: 'POST', headers: {'Content-Type': 'application/x-www-form-urlencoded'}, body: formData })\n");
  newJsContent += F("          .then(response => { setButtonState(button, response.ok ? 'success' : 'error'); if (!response.ok) console.error('Macro POST Error:', response.status); else console.log('Macro OK'); return response.text(); })\n");
  newJsContent += F("          .then(data => console.log('Server response macro:', data))\n");
  newJsContent += F("          .catch(error => { console.error('Fetch error macro:', error); setButtonState(button, 'error'); alert('Network Error (Macro).'); });\n");
  newJsContent += F("        } else {\n");
  newJsContent += F("          const irData = { type: button.dataset.type, data: button.dataset.data, length: parseInt(button.dataset.length, 10), address: button.dataset.address, repeat: parseInt(button.dataset.repeat, 10), out: parseInt(button.dataset.out, 10) };\n");
  newJsContent += F("          console.log('Sending Single IR:', irData);\n");
  newJsContent += F("          if (!irData.type || !irData.data || !irData.length || irData.length <= 0) { console.error('Invalid data attributes:', buttonId, irData); setButtonState(button, 'error'); alert('Error: Invalid button data.'); return; }\n");
  newJsContent += F("          const urlParams = new URLSearchParams(irData).toString();\n");
  newJsContent += F("          fetch(`/sendbutton?${urlParams}`, { method: 'GET' })\n");
  newJsContent += F("          .then(response => { setButtonState(button, response.ok ? 'success' : 'error'); if (!response.ok) console.error('Single IR GET Error:', response.status); else console.log('Single IR OK'); return response.text(); })\n");
  newJsContent += F("          .then(data => console.log('Server response single IR:', data))\n");
  newJsContent += F("          .catch(error => { console.error('Fetch error single IR:', error); setButtonState(button, 'error'); alert('Network Error (Single IR).'); });\n");
  newJsContent += F("        }\n");
  newJsContent += F("      }\n");
  newJsContent += F("    });\n");
  newJsContent += F("  }\n\n");

  newJsContent += F("  /* Test Send Button Handler */\n");
  newJsContent += F("  const testSendButton = document.getElementById('test-send-button');\n");
  newJsContent += F("  if (testSendButton) {\n");
  newJsContent += F("    const originalTestButtonText = testSendButton.textContent;\n");
  newJsContent += F("    testSendButton.addEventListener('click', function(event) {\n");
  newJsContent += F("      console.log('Test Send button clicked.');\n");
  newJsContent += F("      const prefix = 'btn_'; let irData = {}; let macroJsonString = ''; let isMacroTest = false;\n");
  newJsContent += F("      try {\n");
  newJsContent += F("        isMacroTest = document.querySelector('input[name=\"btn_isMacro\"]:checked').value === '1';\n");
  newJsContent += F("        if (isMacroTest) {\n");
  newJsContent += F("          macroJsonString = document.getElementById(prefix + 'macroJson').value;\n");
  newJsContent += F("          JSON.parse(macroJsonString);\n");
  newJsContent += F("        } else {\n");
  newJsContent += F("          irData.type = document.getElementById(prefix + 'type').value; irData.data = document.getElementById(prefix + 'data').value; irData.length = parseInt(document.getElementById(prefix + 'length').value, 10); irData.address = document.getElementById(prefix + 'address').value; irData.repeat = parseInt(document.getElementById(prefix + 'repeat').value, 10); irData.out = parseInt(document.getElementById(prefix + 'out').value, 10);\n");
  newJsContent += F("          if (!irData.type || !irData.data || !irData.length || irData.length <= 0) throw new Error('Missing Type, Data, or valid Length for Single IR.');\n");
  newJsContent += F("          if (!irData.repeat || irData.repeat <= 0) irData.repeat = 1;\n");
  newJsContent += F("          if (!irData.out || irData.out <= 0 || irData.out > 4) irData.out = 1;\n");
  newJsContent += F("        }\n");
  newJsContent += F("      } catch (e) { console.error('Error reading/validating form:', e); alert('Error: ' + e.message); return; }\n");
  newJsContent += F("      testSendButton.textContent = 'Sending...'; setButtonState(testSendButton, 'sending');\n");
  newJsContent += F("      if (isMacroTest) {\n");
  newJsContent += F("        console.log('Sending Test Macro JSON:', macroJsonString);\n");
  newJsContent += F("        const formData = new URLSearchParams(); formData.append('plain', macroJsonString);\n");
  newJsContent += F("        fetch('/json', { method: 'POST', headers: {'Content-Type': 'application/x-www-form-urlencoded'}, body: formData })\n");
  newJsContent += F("        .then(response => { setButtonState(testSendButton, response.ok ? 'success' : 'error'); if (!response.ok) console.error('Test Macro POST Error:', response.status); return response.text(); })\n");
  newJsContent += F("        .then(data => console.log('Server response test macro:', data))\n");
  newJsContent += F("        .catch(error => { console.error('Fetch error test macro:', error); setButtonState(testSendButton, 'error'); alert('Network Error (Test Macro).'); })\n");
  newJsContent += F("        .finally(() => { testSendButton.textContent = originalTestButtonText; });\n");
  newJsContent += F("      } else {\n");
  newJsContent += F("        console.log('Sending Test Single IR:', irData);\n");
  newJsContent += F("        const urlParams = new URLSearchParams(irData).toString();\n");
  newJsContent += F("        fetch(`/sendbutton?${urlParams}`, { method: 'GET' })\n");
  newJsContent += F("        .then(response => { setButtonState(testSendButton, response.ok ? 'success' : 'error'); if (!response.ok) console.error('Test Single IR GET Error:', response.status); return response.text(); })\n");
  newJsContent += F("        .then(data => console.log('Server response test single IR:', data))\n");
  newJsContent += F("        .catch(error => { console.error('Fetch error test single IR:', error); setButtonState(testSendButton, 'error'); alert('Network Error (Test Single IR).'); })\n");
  newJsContent += F("        .finally(() => { testSendButton.textContent = originalTestButtonText; });\n");
  newJsContent += F("      }\n");
  newJsContent += F("    });\n");
  newJsContent += F("  }\n\n");

  newJsContent += F("  /* Initial Button Form Toggle */\n");
  newJsContent += F("  const macroRadio = document.querySelector('input[name=\"btn_isMacro\"][value=\"1\"]');\n");
  newJsContent += F("  if (macroRadio) {\n");
  newJsContent += F("    if (macroRadio.checked) { toggleButtonFields(true); }\n");
  newJsContent += F("    else { const singleRadio = document.querySelector('input[name=\"btn_isMacro\"][value=\"0\"]'); if (singleRadio && singleRadio.checked) { toggleButtonFields(false); } else { toggleButtonFields(false); } }\n");
  newJsContent += F("  }\n\n");
  // --- Ende Aufbau newJsContent ---

  // 2. Lese den VORHANDENEN Inhalt (falls Datei existiert)
  String existingJsContent = "";
  bool fileExists = LittleFS.exists("/js/scripts.js");
  bool readSuccess = false;

  if (fileExists) {
    Serial.println("  Existing /js/scripts.js found. Reading content...");
    File existingJsFile = LittleFS.open("/js/scripts.js", "r");
    if (existingJsFile && !existingJsFile.isDirectory()) {
      // Lese den gesamten Inhalt. Vorsicht bei sehr großen Dateien!
      existingJsContent = existingJsFile.readString();
      existingJsFile.close();
      // Prüfe, ob das Lesen erfolgreich war (readString gibt leeren String bei Fehler)
      // Wir erlauben auch eine leere Datei (size 0), falls das gewollt ist.
      if (existingJsContent.length() > 0 || existingJsFile.size() == 0) {
         Serial.printf("  Read %d bytes from existing file.\n", existingJsContent.length());
         readSuccess = true;
      } else {
         Serial.println("  ERROR: Failed to read content from existing file (readString failed?).");
      }
    } else {
      Serial.println("  ERROR: Failed to open existing file for reading!");
      if (existingJsFile) existingJsFile.close(); // Schließen, falls es ein Verzeichnis war
    }
  } else {
    Serial.println("  Existing /js/scripts.js not found.");
  }

  // 3. Vergleiche und schreibe nur, wenn nötig
  bool needsWrite = true; // Standardmäßig schreiben
  if (fileExists && readSuccess) {
    // Vergleiche den neu generierten String mit dem gelesenen String
    if (newJsContent == existingJsContent) {
      needsWrite = false; // Inhalte sind identisch, kein Schreiben nötig
    } else {
       Serial.println("  Content differs. Update needed.");
       // Optional: Logge Längenunterschiede oder erste paar Zeichen für Debugging
       // Serial.printf("  New length: %d, Existing length: %d\n", newJsContent.length(), existingJsContent.length());
    }
  } else if (!fileExists) {
     Serial.println("  File does not exist. Writing needed.");
  } else { // File existed but read failed
     Serial.println("  File existed but read failed. Overwriting needed.");
  }

  if (needsWrite) {
    Serial.println("  Writing new content to /js/scripts.js...");

    // Stelle sicher, dass das /js Verzeichnis existiert (wichtig VOR dem Schreiben)
    if (!LittleFS.exists("/js")) {
      if (LittleFS.mkdir("/js")) {
        Serial.println("  Created /js directory.");
        delay(50); // Kurze Pause kann manchmal helfen
      } else {
        Serial.println("  ERROR: Failed to create /js directory! Aborting write.");
        return; // Abbrechen, wenn Verzeichnis nicht erstellt werden kann
      }
    }

    // Öffne Datei im Schreibmodus (überschreibt vorhandene)
    File jsFile = LittleFS.open("/js/scripts.js", "w");
    if (!jsFile || jsFile.isDirectory()) {
      Serial.println("  ERROR: Failed to open /js/scripts.js for writing!");
      if (jsFile) jsFile.close();
      return; // Abbrechen bei Fehler
    }

    // Schreibe den gesamten Inhalt
    size_t bytesWritten = jsFile.print(newJsContent);
    int writeError = jsFile.getWriteError(); // Fehlerstatus holen
    jsFile.close(); // Datei schließen

    // Erfolg prüfen
    if (writeError == 0 && bytesWritten == newJsContent.length()) {
      Serial.printf("  Successfully wrote %d bytes.\n", bytesWritten);
      // Optional: Verifizierung durch erneutes Lesen der Größe
      File checkFile = LittleFS.open("/js/scripts.js", "r");
      if (checkFile) {
          Serial.printf("  Verification: File size after write: %d\n", checkFile.size());
          checkFile.close();
      }
    } else {
      Serial.println("  ERROR writing to /js/scripts.js!");
      Serial.printf("    Bytes written: %d (expected %d), Error code: %d\n", bytesWritten, newJsContent.length(), writeError);
      // Versuch, die potenziell korrupte Datei zu löschen
      if (LittleFS.exists("/js/scripts.js")) {
          LittleFS.remove("/js/scripts.js");
      }
    }
  } else {
    Serial.println("  JavaScript file is up-to-date. Skipping write.");
  }

  // Heap-Status nach der Operation (optional)
  // Serial.printf("  Heap after JS check/write: %u\n", ESP.getFreeHeap());
}


// --- Hilfsfunktion für Farb-Dropdown (kann global oder innerhalb von generateButtonForm platziert werden) ---
String generateColorDropdownHtml(const String& selectName, const String& selectedValue) {
  String html = "<select class='form-control' id='" + selectName + "' name='" + selectName + "'>\n";
  const char* colors[][2] = { // Array von Paaren: [Klasse, Angezeigter Name]
      {"btn-primary", "Primary (Blue)"},
      {"btn-success", "Success (Green)"},
      {"btn-info",    "Info (Light Blue)"},
      {"btn-warning", "Warning (Orange)"},
      {"btn-danger",  "Danger (Red)"},
      {"btn-default", "Default (Gray)"}
      // Füge hier bei Bedarf weitere hinzu (z.B. btn-link)
  };
  for (const auto& colorPair : colors) {
      html += "  <option value='" + String(colorPair[0]) + "'";
      if (String(colorPair[0]).equalsIgnoreCase(selectedValue)) {
          html += " selected";
      }
      html += ">" + String(colorPair[1]) + "</option>\n";
  }
  html += "</select>\n";
  return html;
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
  response->print("          <form class='form-horizontal' action='" + actionUrl + "' method='post'>\n"); // <-- FORM BEGINNT
  response->print("            <input type='hidden' name='button_id' value='" + String(buttonId) + "'>\n");

  // --- Name ---
  response->print("            <div class='form-group'>\n");
  response->print("              <label for='" + prefix + "name' class='col-sm-2 control-label'>Name</label>\n");
  response->print("              <div class='col-sm-10'><input type='text' class='form-control' id='" + prefix + "name' name='" + prefix + "name' placeholder='Button Label' value='" + String(buttonData.name) + "' required></div>\n");
  response->print("            </div>\n");

  // --- Layout Row ---
  response->print("            <div class='form-group'>\n");
  response->print("              <label for='" + prefix + "layoutRow' class='col-sm-2 control-label'>Layout Row</label>\n");
  response->print("              <div class='col-sm-4'><input type='number' class='form-control' id='" + prefix + "layoutRow' name='" + prefix + "layoutRow' placeholder='Row (e.g., 0)' value='" + String(buttonData.layoutRow == -1 ? "" : String(buttonData.layoutRow)) + "' min='0'></div>\n");
  // --- Layout Column ---
  response->print("              <label for='" + prefix + "layoutCol' class='col-sm-2 control-label'>Layout Column</label>\n");
  response->print("              <div class='col-sm-4'><input type='number' class='form-control' id='" + prefix + "layoutCol' name='" + prefix + "layoutCol' placeholder='Column (e.g., 0)' value='" + String(buttonData.layoutCol == -1 ? "" : String(buttonData.layoutCol)) + "' min='0'></div>\n");
  response->print("              <span class='help-block col-sm-offset-2 col-sm-10'>Optional: Specify row/column for grid layout (starting from 0). Leave blank for default flow.</span>\n");
  response->print("            </div>\n");

      // --- Button Color ---
  response->print("            <div class='form-group'>\n");
  response->print("              <label for='" + prefix + "colorClass' class='col-sm-2 control-label'>Button Color</label>\n");
  response->print("              <div class='col-sm-10'>" + generateColorDropdownHtml(prefix + "colorClass", String(buttonData.colorClass)) + "</div>\n");
  response->print("            </div>\n");

  // --- Button Type Selector (Radios) ---
  response->print("            <div class='form-group'>\n");
  response->print("              <label class='col-sm-2 control-label'>Button Type</label>\n");
  response->print("              <div class='col-sm-10'>\n");
  response->print("                <label class='radio-inline'><input type='radio' name='btn_isMacro' value='0' ");
  if (!buttonData.isMacro) response->print("checked ");
  response->print("onclick='toggleButtonFields(false)'> Single IR Command</label>\n");
  response->print("                <label class='radio-inline'><input type='radio' name='btn_isMacro' value='1' ");
  if (buttonData.isMacro) response->print("checked ");
  response->print("onclick='toggleButtonFields(true)'> Macro (JSON)</label>\n");
  response->print("              </div>\n");
  response->print("            </div>\n");

  // --- Container für Single IR Felder ---
  response->print("            <div id='single-ir-fields' style='display: " + String(!buttonData.isMacro ? "block" : "none") + ";'>\n"); // <-- OPEN single-ir-fields

  // Type Dropdown
  response->print("            <div class='form-group'>\n");
  response->print("              <label for='" + prefix + "type' class='col-sm-2 control-label'>Type</label>\n");
  response->print("              <div class='col-sm-10'>" + generateTypeDropdownHtml(prefix + "type", String(buttonData.type)) + "</div>\n");
  response->print("            </div>\n");

  // Data
  response->print("            <div class='form-group'>\n");
  response->print("              <label for='" + prefix + "data' class='col-sm-2 control-label'>Data (Hex)</label>\n");
  response->print("              <div class='col-sm-10'><input type='text' class='form-control' id='" + prefix + "data' name='" + prefix + "data' placeholder='e.g., FF02FD' value='" + String(buttonData.data) + "'></div>\n"); // required wird durch JS gesetzt
  response->print("            </div>\n");

  // Length
  response->print("            <div class='form-group'>\n");
  response->print("              <label for='" + prefix + "length' class='col-sm-2 control-label'>Length (Bits)</label>\n");
  response->print("              <div class='col-sm-10'><input type='number' class='form-control' id='" + prefix + "length' name='" + prefix + "length' placeholder='e.g., 32' value='" + String(buttonData.length) + "' min='1'></div>\n"); // required wird durch JS gesetzt
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
  response->print("              <div class='col-sm-10'>" + generateOutDropdownHtml(prefix + "out", buttonData.out) + "</div>\n");
  response->print("            </div>\n");

  response->print("            </div>\n"); // <-- KORREKTES ENDE von 'single-ir-fields'

  // --- Container und Feld für Macro JSON ---
  response->print("            <div id='macro-json-field' style='display: " + String(buttonData.isMacro ? "block" : "none") + ";'>\n"); // <-- OPEN macro-json-field
  response->print("              <div class='form-group'>\n");
  response->print("                <label for='" + prefix + "macroJson' class='col-sm-2 control-label'>Macro JSON</label>\n");
  response->print("                <div class='col-sm-10'>\n");
  response->print("                  <textarea class='form-control' id='" + prefix + "macroJson' name='" + prefix + "macroJson' rows='10' placeholder='[{\"type\":\"nec\",\"data\":\"FF02FD\",\"length\":32}, {\"type\":\"delay\",\"rdelay\":500}, {\"type\":\"sony\",\"data\":\"A90\",\"length\":12}]'>" + String(buttonData.macroJson) + "</textarea>\n"); // required wird durch JS gesetzt
  response->print("                  <span class='help-block'>Enter a JSON array defining the sequence of actions (like the payload for the /json endpoint). Use double quotes for keys and string values.</span>\n");
  response->print("                </div>\n");
  response->print("              </div>\n");
  response->print("            </div>\n"); // <-- KORREKTES ENDE von 'macro-json-field'

  // --- Submit/Cancel/Test Buttons (JETZT NACH BEIDEN FELD-CONTAINERN) ---
  response->print("            <div class='form-group'>\n");
  response->print("              <div class='col-sm-offset-2 col-sm-10'>\n");
  response->print("                <button type='submit' class='btn btn-success'>Save Button</button>\n"); // <-- Der Button
  response->print("                <button type='button' id='test-send-button' class='btn btn-info' style='margin-left: 10px;'>Test Send</button>\n");
  response->print("                <a href='/buttons' class='btn btn-default' style='margin-left: 10px;'>Cancel</a>\n");
  response->print("              </div>\n");
  response->print("            </div>\n");

  response->print("          </form>\n"); // <-- FORM ENDET HIER KORREKT
  response->print("        </div>\n"); // Ende col-md-12
  response->print("      </div>\n");   // Ende row
}


// --- Handler für Backup (Download) ---
void handleBackup(AsyncWebServerRequest *request) {
  Serial.println("Handling /backup request...");
  if (LittleFS.exists("/buttons.json")) {
    // Setze Header für Download
    AsyncWebServerResponse *response = request->beginResponse(LittleFS, "/buttons.json", "application/json", true);
    // Der letzte Parameter 'true' setzt Content-Disposition: attachment
    response->addHeader("Content-Disposition", "attachment; filename=\"buttons.json\"");
    request->send(response);
  } else {
    Serial.println("  ERROR: /buttons.json not found for backup.");
    request->send(404, "text/plain", "ERROR: buttons.json not found.");
  }
}

// --- Handler für Datei-Upload (wird während des Uploads aufgerufen) ---
void handleRestoreUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  if (!index) { // Erster Chunk der Datei
    Serial.printf("Upload Start: %s\n", filename.c_str());
    // Sicherheitscheck: Nur .json Dateien erlauben (optional aber empfohlen)
    if (!filename.endsWith(".json")) {
        Serial.println("  ERROR: Invalid file type uploaded (not .json). Aborting.");
        // Hier könnten wir die Verbindung schließen oder eine Fehlermeldung vorbereiten
        // Fürs Erste brechen wir nur das Schreiben ab.
        if(fsUploadFile) fsUploadFile.close(); // Sicherstellen, dass nichts offen bleibt
        // Man könnte hier auch einen Fehlerstatus für den finalen Handler setzen
        return; // Verhindert das Öffnen/Schreiben
    }

    // Datei im LittleFS zum Schreiben öffnen (überschreibt vorhandene)
    fsUploadFile = LittleFS.open("/buttons.json", "w");
    if (!fsUploadFile) {
        Serial.println("  ERROR: Could not open /buttons.json for writing!");
        return; // Verhindert das Schreiben
    }
    Serial.println("  Opened /buttons.json for writing.");
  }

  if (fsUploadFile) { // Nur schreiben, wenn Datei erfolgreich geöffnet wurde
      // Daten-Chunk in die Datei schreiben
      if (len) {
          size_t written = fsUploadFile.write(data, len);
          if (written != len) {
              Serial.printf("  ERROR: Writing to file failed! Expected %d, wrote %d\n", len, written);
              // Hier könnte man den Upload abbrechen
          }
      }
  }


  if (final) { // Letzter Chunk der Datei
    if (fsUploadFile) {
        fsUploadFile.close();
        Serial.printf("Upload Finished: %s, Size: %u\n", filename.c_str(), index + len);
        // WICHTIG: Konfiguration neu laden, NACHDEM die Datei geschlossen wurde!
        Serial.println("  Reloading button configuration from restored file...");
        loadButtonConfig(); // Lädt die gerade hochgeladene Konfiguration
        updateButtonMacroJsStore(); // JS Store aktualisieren
    } else {
        Serial.println("  ERROR: Upload finished, but file was not open/valid.");
    }
  }
}

// --- Handler für die POST-Anfrage nach dem Upload ---
void handleRestoreRequest(AsyncWebServerRequest *request) {
    // Dieser Handler wird aufgerufen, NACHDEM handleRestoreUpload fertig ist.
    // Wir senden hier nur eine Bestätigung und leiten zurück.
    // Die eigentliche Arbeit (Datei speichern, Konfig laden) passiert in handleRestoreUpload.

    // Sende eine einfache Bestätigungsseite mit Weiterleitung
    AsyncWebServerResponse *response = request->beginResponse(200, "text/html",
        "<!DOCTYPE html><html><head><title>Restore Complete</title>"
        "<meta http-equiv='refresh' content='3;url=/buttons'>" // Leitet nach 3s weiter
        "</head><body>"
        "<h2>Restore Successful!</h2>"
        "<p>Button configuration has been updated from the uploaded file.</p>"
        "<p>Reloading configuration and redirecting back to the button page in 3 seconds...</p>"
        "<a href='/buttons'>Go back now</a>"
        "</body></html>");
    request->send(response);
    Serial.println("Sent restore confirmation page.");
}

// Handler zum Anzeigen des "Add New Button"-Formulars
// Handler zum Anzeigen des "Add New Button"-Formulars
void handleAddButtonPage(AsyncWebServerRequest *request) {
  Serial.println("Connection received endpoint '/addbutton' (GET)");
  AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8", 200);
  sendHeader(response);

  ButtonConfig prefilledButton; // Leeres Struct für Formular
  prefilledButton.repeat = 1;   // Standardwerte setzen
  prefilledButton.out = 1;
  strncpy(prefilledButton.colorClass, "btn-primary", sizeof(prefilledButton.colorClass) -1); // Default Farbe
  prefilledButton.colorClass[sizeof(prefilledButton.colorClass) - 1] = '\0';
  prefilledButton.layoutRow = -1; // Default Layout
  prefilledButton.layoutCol = -1;
  prefilledButton.isMacro = false; // Default: Single IR

  // --- NEU: Prüfe auf Prefill-Parameter ---
  bool prefilled = false;
  if (request->hasParam("prefill_type")) {
    strncpy(prefilledButton.type, request->getParam("prefill_type")->value().c_str(), sizeof(prefilledButton.type) - 1);
    prefilledButton.type[sizeof(prefilledButton.type) - 1] = '\0';
    prefilled = true;
  }
  if (request->hasParam("prefill_data")) {
    strncpy(prefilledButton.data, request->getParam("prefill_data")->value().c_str(), sizeof(prefilledButton.data) - 1);
    prefilledButton.data[sizeof(prefilledButton.data) - 1] = '\0';
    prefilled = true;
  }
  if (request->hasParam("prefill_length")) {
    prefilledButton.length = request->getParam("prefill_length")->value().toInt();
    if (prefilledButton.length <= 0) prefilledButton.length = 0; // Korrektur bei ungültiger Zahl
    prefilled = true;
  }
  if (request->hasParam("prefill_address")) {
    strncpy(prefilledButton.address, request->getParam("prefill_address")->value().c_str(), sizeof(prefilledButton.address) - 1);
    prefilledButton.address[sizeof(prefilledButton.address) - 1] = '\0';
    prefilled = true;
  }
  if (prefilled) {
      Serial.println("  Prefilling 'Add Button' form from URL parameters.");
      // Optional: Einen Standardnamen vorschlagen
      String suggestedName = "New_" + String(prefilledButton.type) + "_" + String(prefilledButton.data);
      suggestedName.toUpperCase();
      strncpy(prefilledButton.name, suggestedName.c_str(), sizeof(prefilledButton.name) - 1);
      prefilledButton.name[sizeof(prefilledButton.name) - 1] = '\0';
  }
  // --- ENDE NEU ---

  generateButtonForm(response, prefilledButton, -1); // -1 signalisiert "neu", prefilledButton enthält ggf. Daten
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
  buttonConfigs.erase(buttonConfigs.begin() + buttonId);
  saveButtonConfig(); // This already calls updateButtonMacroJsStore() now
  request->redirect("/buttons?status=deleted");
} else {
  Serial.printf("Error: Invalid button ID %d requested for deletion.\n", buttonId);
  request->redirect("/buttons?status=error_invalid_id");
}
}

void handleSaveButton(AsyncWebServerRequest *request) {
  Serial.println("==> handleSaveButton: Entered function.");

  // -- manuell per Index suchen ---
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
  String isMacroStr = "";
  String macroJson = "";
  String layoutRowStr = "";
  String layoutColStr = "";
  String colorClassStr = ""; // Für die Farbklasse
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
      } 
      // --- Makro-Felder lesen ---
      else if (paramName.equals("btn_isMacro")) { isMacroStr = paramValue; }
      else if (paramName.equals("btn_macroJson")) { macroJson = paramValue; }
      
      else if (paramName.equals("btn_type")) {
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
      } else if (paramName.equals("btn_layoutRow")) {
        layoutRowStr = paramValue;
      } else if (paramName.equals("btn_layoutCol")) {
        layoutColStr = paramValue;
      } else if (paramName.equals("btn_colorClass")) {
        colorClassStr = paramValue;
      }
    } else {
       Serial.printf("      Ignoring non-POST param[%s]: %s\n", p->name().c_str(), p->value().c_str());
    }
  }
  Serial.println("    Parameter scan complete.");

  // --- Prüfung, ob button_id gefunden wurde ---
  if (!buttonIdFound) {
      Serial.println("    ERROR: Parameter 'button_id' not found during manual scan!");
      request->redirect("/buttons?status=error_save");
      return;
  }

  // --- Werte konvertieren ---
  int buttonId = buttonIdStr.toInt();
  bool isMacro = (isMacroStr == "1"); // Prüfe, ob der Wert "1" ist
  int length = lengthStr.toInt();
  int repeat = repeatStr.toInt();
  int out = outStr.toInt();
  // ... (andere Konvertierungen) ...
  int layoutRow = layoutRowStr.toInt();
  int layoutCol = layoutColStr.toInt();
  // Konvertiere leere Eingaben oder 0 zu -1 für "nicht gesetzt"
  if (layoutRowStr.length() == 0 || layoutRow < 0) layoutRow = -1;
  if (layoutColStr.length() == 0 || layoutCol < 0) layoutCol = -1;
  
  name.trim();  // Trimme den Namen

  Serial.println("--- Parameter Values ---");
  Serial.println("buttonIdStr: " + buttonIdStr);
  Serial.println("name: " + name);
  Serial.println("isMacroStr: " + isMacroStr);
  Serial.println("macroJson (String): " + macroJson); // <-- WICHTIG: Inhalt der String-Variable
  Serial.println("type (String): " + type);
  Serial.println("data (String): " + data);
  Serial.println("lengthStr: " + lengthStr);
  Serial.println("address (String): " + address);
  Serial.println("repeatStr: " + repeatStr);
  Serial.println("outStr: " + outStr);
  Serial.println("layoutRowStr: " + layoutRowStr + " -> " + String(layoutRow));
  Serial.println("layoutColStr: " + layoutColStr + " -> " + String(layoutCol));
  Serial.println("colorClassStr: " + colorClassStr);
  Serial.println("------------------------");

  // --- Validierung ---
  // 1. Name ist immer erforderlich
  if (name.length() == 0) {
    Serial.println("    ERROR: Validation failed! (Name is missing). Redirecting.");
    request->redirect("/buttons?status=error_invalid_data"); // Oder spezifischerer Status?
    return;
}

// 2. Spezifische Validierung basierend auf isMacro
if (isMacro) {
  Serial.println("    DEBUG: Vor strncpy für macroJson. Lokaler Wert:"); // NEU
  Serial.println("    >>>> " + macroJson + " <<<<"); // NEU
  // Validierung für Makro
  if (macroJson.length() == 0) {
      Serial.println("    ERROR: Validation failed! (Macro JSON missing). Redirecting.");
      request->redirect("/buttons?status=error_invalid_macro_data"); // Status für Makro-Fehler
      return;
  }
  // JSON Validierung (wie gehabt)
  DynamicJsonDocument tempDoc(1024);
  DeserializationError error = deserializeJson(tempDoc, macroJson);
  if (error) {
      Serial.print("    ERROR: Macro JSON validation failed: ");
      Serial.println(error.c_str());
      request->redirect("/buttons?status=error_invalid_json");
      return;
  }
  if (!tempDoc.is<JsonArray>()) {
      Serial.println("    ERROR: Macro JSON is not a valid JSON array.");
      request->redirect("/buttons?status=error_json_not_array");
      return;
  }
  Serial.println("    Macro JSON validation passed.");
  // Single-IR Felder werden später ignoriert/geleert
  type = ""; data = ""; length = 0; address = ""; repeat = 1; out = 1; // Setze hier schon Defaults/Leerwerte

  } else {
      // Validierung für Single IR (jetzt hier)
      if (data.length() == 0 || length <= 0) {
          Serial.println("    ERROR: Validation failed! (data or length invalid for single IR). Redirecting.");
          request->redirect("/buttons?status=error_invalid_data"); // Status passt hier
          return;
      }
      // Defaults für Single IR (wie gehabt)
      if (repeat <= 0) repeat = 1;
      if (out <= 0 || out > 4) out = 1;
      // Makro-Feld wird später ignoriert/geleert
      macroJson = ""; // Setze hier schon Leerwert
      Serial.println("    Single IR validation passed.");
  }

  // --- Farbklasse validieren ---
  const char* validColors[] = {"btn-primary", "btn-success", "btn-info", "btn-warning", "btn-danger", "btn-default"};
  bool colorIsValid = false;
  for (const char* validColor : validColors) {
      if (colorClassStr.equalsIgnoreCase(validColor)) {
          colorClassStr = validColor; // Stelle korrekte Schreibweise sicher
          colorIsValid = true;
          break;
      }
  }
  if (!colorIsValid) {
      Serial.println("    Warning: Invalid color class received ('" + colorClassStr + "'). Defaulting to btn-primary.");
      colorClassStr = "btn-primary"; // Fallback auf Default
  }
  // --- Ende Farbklasse validieren ---

  
  // --- ENDE VALIDIERUNG ---

  // Defaults setzen (redundant, da oben schon erledigt, aber schadet nicht)
  if (repeat <= 0) repeat = 1;
  if (out <= 0 || out > 4) out = 1;

 // --- Button-Daten vorbereiten ---
 ButtonConfig tempButton; // Temporäres Struct zum Befüllen

 // Gemeinsame Felder
 tempButton.configured = true;
 strncpy(tempButton.name, name.c_str(), sizeof(tempButton.name) - 1);
 tempButton.name[sizeof(tempButton.name) - 1] = '\0';
 tempButton.isMacro = isMacro;
 tempButton.layoutRow = layoutRow;
 tempButton.layoutCol = layoutCol;
 strncpy(tempButton.colorClass, colorClassStr.c_str(), sizeof(tempButton.colorClass) - 1);
 tempButton.colorClass[sizeof(tempButton.colorClass) - 1] = '\0';


 // Modus-spezifische Felder
 if (isMacro) {
     // Makro-Daten kopieren
     strncpy(tempButton.macroJson, macroJson.c_str(), sizeof(tempButton.macroJson) - 1);
     tempButton.macroJson[sizeof(tempButton.macroJson) - 1] = '\0';
     // Single-IR Felder leeren
     tempButton.type[0] = '\0';
     tempButton.data[0] = '\0';
     tempButton.length = 0;
     tempButton.address[0] = '\0';
     tempButton.repeat = 1; // Default
     tempButton.out = 1;    // Default
     Serial.println("    Prepared tempButton for MACRO."); // Debug
 } else {
     // Single-IR Daten kopieren
     strncpy(tempButton.type, type.c_str(), sizeof(tempButton.type) - 1);
     tempButton.type[sizeof(tempButton.type) - 1] = '\0';
     strncpy(tempButton.data, data.c_str(), sizeof(tempButton.data) - 1);
     tempButton.data[sizeof(tempButton.data) - 1] = '\0';
     tempButton.length = length;
     strncpy(tempButton.address, address.c_str(), sizeof(tempButton.address) - 1);
     tempButton.address[sizeof(tempButton.address) - 1] = '\0';
     tempButton.repeat = repeat;
     tempButton.out = out;
     // Makro-Feld leeren
     tempButton.macroJson[0] = '\0';
     Serial.println("    Prepared tempButton for SINGLE IR."); // Debug
 }

 // --- Entscheiden: Add oder Edit ---
 if (buttonId == -1) { // Neuer Button
   Serial.println("    -> Entering ADD logic.");
   if (buttonConfigs.size() >= MAX_BUTTONS) {
      Serial.println("      ERROR: Maximum number of buttons reached!");
      request->redirect("/buttons?status=error_max_buttons");
      return;
   }
   buttonConfigs.push_back(tempButton); // Füge das vorbereitete Objekt hinzu
   Serial.printf("    Added new button '%s'. Vector size: %d\n", tempButton.name, buttonConfigs.size());

 } else if (buttonId >= 0 && buttonId < buttonConfigs.size()) { // Button bearbeiten
   Serial.println("    -> Entering EDIT logic for ID: " + String(buttonId));
   buttonConfigs[buttonId] = tempButton; // Überschreibe das vorhandene Objekt
   Serial.printf("    Edited button ID %d ('%s').\n", buttonId, tempButton.name);

 } else { // Ungültige ID
   Serial.println("    ERROR: Invalid button_id received: " + String(buttonId));
   request->redirect("/buttons?status=error_invalid_id");
   return;
 }

 // --- Speichern und Redirect ---
 Serial.println("    -> Calling saveButtonConfig()...");
 saveButtonConfig(); // Sollte jetzt die korrekten Daten aus dem Vector lesen
 Serial.println("    -> Redirecting to /buttons?status=saved");
 request->redirect("/buttons?status=saved");
}

//+=============================================================================
// Handler for Button Configuration Overview Page
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
      } else if (status == "error_invalid_data") { // Fehler von handleSaveButton
          response->print("<div class='alert alert-danger'>Error: Invalid data submitted for button.</div>");
      } else if (status == "error_save") { // Allgemeiner Speicherfehler
          response->print("<div class='alert alert-danger'>Error: Could not save button configuration.</div>");
      } else if (status == "error_max_buttons") { // Max Buttons erreicht
          response->print("<div class='alert alert-warning'>Warning: Maximum number of buttons reached. Could not add new button.</div>");
      }
  }

  response->print("          <table class='table table-striped table-condensed' style='font-size: 0.9em;'>\n");
  response->print("            <thead><tr><th>Name</th><th>Mode</th><th>Type</th><th>Data/Macro</th><th>Length</th><th>Address</th><th>Repeat</th><th>Out</th><th>Actions</th></tr></thead>\n");
 response->print("            <tbody>\n");

 if (!buttonConfigs.empty()) {
  for (size_t i = 0; i < buttonConfigs.size(); ++i) {
    const auto& button = buttonConfigs[i];

    response->print("              <tr>\n");
    response->print("                <td>" + String(button.name) + "</td>\n"); // Name
    response->print("                <td>" + String(button.isMacro ? "Macro" : "Single") + "</td>\n");


    // Jetzt die restlichen Spalten basierend auf dem Modus
    if (button.isMacro) {
        response->print("                <td><code>-</code></td>\n"); // Type N/A
        // Macro JSON anzeigen (gekürzt)
        String macroSnippet = String(button.macroJson);
        if (macroSnippet.length() > 30) {
            macroSnippet = macroSnippet.substring(0, 27) + "...";
        }
        response->print("                <td><code style='font-size: 0.8em;'>" + macroSnippet + "</code></td>\n"); // Macro Snippet
        response->print("                <td><code>-</code></td>\n"); // Length N/A
        response->print("                <td><code>-</code></td>\n"); // Address N/A
        response->print("                <td><code>-</code></td>\n"); // Repeat N/A
        response->print("                <td><code>-</code></td>\n"); // Out N/A
    } else {
        response->print("                <td><code>" + String(button.type) + "</code></td>\n"); // Type
        response->print("                <td><code>" + String(button.data) + "</code></td>\n"); // Data
        response->print("                <td><code>" + String(button.length) + "</code></td>\n"); // Length
        response->print("                <td><code>" + (String(button.address).length() > 0 ? String(button.address) : "-") + "</code></td>\n"); // Address
        response->print("                <td><code>" + String(button.repeat) + "</code></td>\n"); // Repeat
        // Output Pin mit GPIO Info
        String outText = String(button.out) + " (GPIO "; // Start building the string
        switch(button.out) {
            case 1: outText += String(pins1); break;
            case 2: outText += String(pins2); break;
            case 3: outText += String(pins3); break;
            case 4: outText += String(pins4); break;
            default: outText += "?"; break; // Fallback
        }
        outText += ")"; // Close parenthesis
        response->print("                <td><code>" + outText + "</code></td>\n"); // Out
    }

    // Actions Spalte
    response->print("                <td>\n");
    response->print("                  <a href='/editbutton?id=" + String(i) + "' class='btn btn-xs btn-warning' style='margin-right: 3px;'>Edit</a>\n");
    response->print("                  <a href='/deletebutton?id=" + String(i) + "' class='btn btn-xs btn-danger' onclick='return confirm(\"Are you sure you want to delete button \\'" + String(button.name) + "\\'?\");'>Delete</a>\n");
    response->print("                </td>\n");
    response->print("              </tr>\n");
  }
} else {
  response->print("              <tr><td colspan='9' class='text-center'><em>No buttons configured.</em></td></tr>\n");
}

  response->print("            </tbody>\n");
  response->print("          </table>\n");
  // Button zum Hinzufügen eines neuen Buttons
  response->print("          <a href='/addbutton' class='btn btn-success'>Add New Button</a>\n");
  response->print("          <a href='/' class='btn btn-default' style='margin-left: 10px;'>Back to Home</a>\n");
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

  int repeat = request->hasParam("repeat") ? request->getParam("repeat")->value().toInt() : 1;
  int out = request->hasParam("out") ? request->getParam("out")->value().toInt() : 1;

  int rdelay = 1000; // Default repeat delay
  int pulse = 1;     // Default pulse count
  int pdelay = 100;  // Default pulse delay

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

Serial.println("      Setting LED LOW...");
digitalWrite(ledpin, LOW);
Serial.println("      LED set LOW.");

Serial.println("      Attaching ticker...");
ticker.attach(0.5, disableLed);
Serial.println("      Ticker attached.");

Serial.println("      Now calling irblast function...");
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
  Serial.println("    -> Redirecting to /buttons?status=manual_send_success"); // <-- Geänderte Log-Meldung
  request->redirect("/buttons?status=manual_send_success"); // <-- ***** HIER DIE ÄNDERUNG *****
}


// Not Found Handler
void handleNotFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Seite nicht gefunden");
}


//+=============================================================================
// Setup web server and IR receiver/blaster
//
void setup() {
  // Initialize serial
  Serial.begin(115200);
  Serial.println("\n\nBooting..."); // Frühe Meldung
  
  // --- TEMPORÄRER CODE ZUM FORMATIEREN ---
// Diesen Block einkommentieren, EINMAL flashen & laufen lassen,
// dann ESP manuell stoppen, Block wieder auskommentieren und erneut flashen!
/* 
Serial.println("Attempting to unmount and format LittleFS... THIS WILL ERASE ALL SAVED DATA!");
LittleFS.end(); // Sicherstellen, dass es unmounted ist
delay(100);     // Kurze Pause
bool formatted = LittleFS.format();
if (formatted) {
  Serial.println("LittleFS formatted successfully. Halting now. Please stop/reset manually.");
  delay(1000); // Gib der Meldung Zeit
  while(1) { delay(100); } // Anhalten
} else {
  Serial.println("!!! LittleFS format failed. Halting. !!!");
  while(1) { delay(100); } // Anhalten, wenn Formatierung fehlschlägt
}
 */
// --- ENDE TEMPORÄRER CODE ---

  if (!LittleFS.begin(true)) { // Mount FS (true = format if mount failed)
    Serial.println("LittleFS Mount Failed! Halting.");
    while(1); // Anhalten, da JS nicht generiert werden kann
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

  // NEU: Einfache WiFi Verbindung (wie im Minimaltest)
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); // Verwende die globalen ssid/password Variablen
  Serial.print("Connecting to WiFi (direct)...");
  unsigned long startMillis = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - startMillis > 20000) {
        Serial.println("\nWiFi Connection Failed! Restarting...");
        ESP.restart();
    }
  }
  Serial.println("\nWiFi connected.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  // --- ENDE NEU ---

  Serial.println("WiFi configuration complete");

  // --- NEU: JS generieren und schreiben ---
  generateAndWriteJavaScript();
  // --- ENDE NEU ---

  loadButtonConfig(); // Lade Buttons NACHDEM FS gemountet ist

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

  // NEU: Explizite Header für SSE (oft von DefaultHeaders abgedeckt, aber sicher ist sicher)
  DefaultHeaders::Instance().addHeader("Cache-Control", "no-cache");

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

// --- WICHTIG: JS Handler registrieren ---
// --- ALT (Regex-Handler) ---
/*
server->on("^\\/js\\/(.+)$", HTTP_GET, [](AsyncWebServerRequest *request){
  String filename = "/js/" + request->pathArg(0); // Verwendet Regex und pathArg
  Serial.println("Request for JS file: " + filename);
  if (LittleFS.exists(filename)) {
    request->send(LittleFS, filename, "application/javascript");
    Serial.println("Sent JS file: " + filename);
  } else {
    Serial.println("JS file not found: " + filename);
    request->send(404, "text/plain", "Not Found");
  }
});
*/

// --- NEU (Statischer Handler für die spezifische Datei) ---
server->on("/js/scripts.js", HTTP_GET, [](AsyncWebServerRequest *request){
  String filename = "/js/scripts.js"; // Der Dateiname ist jetzt fest
  Serial.println("Request for specific JS file: " + filename); // Angepasste Log-Meldung

  if (LittleFS.exists(filename)) {
    // Datei existiert, sende sie mit korrektem Content-Type
    request->send(LittleFS, filename, "application/javascript");
  } else {
    // Datei nicht gefunden (sollte nach setup() nicht passieren, aber sicher ist sicher)
    Serial.println("JS file not found: " + filename);
    request->send(404, "text/plain", "Not Found");
  }
});

   // Configure the server
   server->on("/json", HTTP_POST, [](AsyncWebServerRequest *request) { // JSON handler
    Serial.println("Connection received endpoint '/json'");

    // --- Parameter mit request->hasParam / request->getParam abrufen ---
    int simple = 0;
    if (request->hasParam("simple")) simple = request->getParam("simple")->value().toInt();
    String signature = request->hasParam("auth") ? request->getParam("auth")->value() : "";
    int out = (request->hasParam("out")) ? request->getParam("out")->value().toInt() : 1;

    // --- JSON Payload aus 'plain' POST-Parameter holen ---
    if (!request->hasParam("plain", true)) {
        Serial.println("JSON parsing failed: Missing 'plain' POST parameter.");
        request->send(400, "text/plain", "Bad Request: Missing 'plain' parameter in POST body.");
        return;
    }
    String plainJson = request->getParam("plain", true)->value();

    DynamicJsonDocument root(1024);
    DeserializationError error = deserializeJson(root, plainJson);

    if (error) {
      Serial.println("JSON parsing failed");
      Serial.println(error.c_str());
      request->send(400, "text/plain", "Bad Request: JSON parsing failed, " + String(error.c_str()));
      root.clear();
      return;
    }

    Serial.println("JSON parsed successfully.");
    digitalWrite(ledpin, LOW);
    ticker.attach(0.5, disableLed);

    // --- Device State Logik ---
    // --- HIER WIEDER EINGEFÜGT ---
    if (request->hasParam("device")) {
      String device = request->getParam("device")->value(); // Variable 'device' deklarieren und zuweisen
      int state = 0; // Default state
      if (request->hasParam("state")) { // Prüfen, ob 'state' Parameter existiert
          state = request->getParam("state")->value().toInt(); // Variable 'state' deklarieren und zuweisen
      }
      Serial.println("Device name detected " + device + ", requested state " + String(state)); // Loggen

      // Jetzt prüfen, ob der Zustand bereits erreicht ist
      if (deviceState.containsKey(device)) {
        Serial.println("Contains the key!");
        int currentState = deviceState[device];
        Serial.println("Current stored state: " + String(currentState));
        // --- KORRIGIERTE BEDINGUNG ---
        if (state == currentState) { // Die tatsächliche Bedingung
          Serial.println("Not sending command to " + device + ", already in state " + state);
          request->send(200, "text/plain", "OK: Command held, device already in state " + String(state));
          root.clear();
          return; // Wichtig: Handler hier beenden
        } else {
           Serial.println("Updating device " + device + " to state " + state);
           deviceState[device] = state; // Zustand aktualisieren, wenn er sich ändert
        }
      } else {
        Serial.println("Setting initial device " + device + " to state " + state);
        deviceState[device] = state; // Initialen Zustand setzen
      }
    }
    // --- ENDE WIEDER EINGEFÜGT ---


    // --- IR-Befehle verarbeiten ---
    String message = "OK: Code(s) sent.";
    for (size_t x = 0; x < root.size(); x++) {
        JsonObject item = root[x].as<JsonObject>(); // Das aktuelle Objekt im Array

        // --- HIER WIEDER EINGEFÜGT: Device State Prüfung PRO BEFEHL ---
        if (item.containsKey("device")) {
            String itemDevice = item["device"].as<String>();
            int itemState = item["state"] | 0; // Default 0, falls "state" fehlt

            if (deviceState.containsKey(itemDevice)) {
                int currentItemState = deviceState[itemDevice];
                // --- KORRIGIERTE BEDINGUNG ---
                if (itemState == currentItemState) { // Die tatsächliche Bedingung
                    Serial.printf("  Skipping command %d for device %s, already in state %d\n", x, itemDevice.c_str(), itemState);
                    message = "OK: Code(s) sent, but some components held.";
                    continue; // Zum nächsten Befehl im Array springen
                } else {
                     deviceState[itemDevice] = itemState; // Zustand für dieses Gerät aktualisieren
                }
            } else {
                 deviceState[itemDevice] = itemState; // Initialen Zustand für dieses Gerät setzen
            }
        }
        // --- ENDE WIEDER EINGEFÜGT ---

        // --- IR Sende-Logik (delay, raw, pronto, standard) ---
        if (item.containsKey("type")) {
            String type = item["type"].as<String>();
            type.toLowerCase();

            if (type == "delay" || type == "wait") {
                int rdelay = item["rdelay"] | 500; // Default 500ms
                Serial.printf("  Delaying for %d ms...\n", rdelay);
                delay(rdelay);
                continue; // Zum nächsten Befehl springen
            }

            // Standard-Parameter für IR-Befehle
            int rdelay = item["rdelay"] | 1000;
            int pulse = item["pulse"] | 1;
            int pdelay = item["pdelay"] | 100;
            int repeat = item["repeat"] | 1;
            int item_out = item["out"] | out; // Nimm 'out' vom Item oder globalen Default
            if (item_out < 1 || item_out > 4) item_out = out; // Fallback auf globalen Default bei ungültigem Item-Out

            if (type == "raw") {
                if (item.containsKey("data") && item["data"].is<JsonArray>()) {
                    JsonArray raw = item["data"].as<JsonArray>();
                    int khz = item["khz"] | 38;
                    int duty = item["duty"] | 50;
                    rawblast(raw, khz, rdelay, pulse, pdelay, repeat, pickIRsend(item_out), duty, item_out);
                } else { Serial.println("  ERROR: Raw type missing 'data' array."); message = "ERROR: Invalid raw command."; }
            } else if (type == "pronto") {
                 if (item.containsKey("data") && item["data"].is<JsonArray>()) {
                    JsonArray data = item["data"].as<JsonArray>();
                    pronto(data, rdelay, pulse, pdelay, repeat, pickIRsend(item_out), item_out);
                 } else { Serial.println("  ERROR: Pronto type missing 'data' array."); message = "ERROR: Invalid pronto command."; }
            } else if (type == "roku") {
                 String ip = item["ip"] | "";
                 String data = item["data"] | "";
                 if (ip.length() > 0 && data.length() > 0) {
                    rokuCommand(ip, data, repeat, rdelay);
                 } else { Serial.println("  ERROR: Roku type missing 'ip' or 'data'."); message = "ERROR: Invalid roku command."; }
            } else { // Standard IR command
                String dataStr = item["data"] | "";
                int len = item["length"] | 0;
                long address = 0;
                if (item.containsKey("address")) {
                    String addressStr = item["address"].as<String>();
                    if (addressStr.length() > 0) address = strtoul(addressStr.c_str(), 0, 0);
                }

                if (dataStr.length() > 0 && len > 0) {
                    irblast(type, dataStr, len, rdelay, pulse, pdelay, repeat, address, pickIRsend(item_out), item_out);
                } else { Serial.println("  ERROR: Standard IR type missing 'data' or 'length'."); message = "ERROR: Invalid standard IR command."; }
            }
        } else {
            Serial.printf("  ERROR: Command %d missing 'type'.\n", x);
            message = "ERROR: Command missing 'type'.";
        }
        // Kurze Pause nach jedem Befehl im Makro (optional)
        delay(50);
    } // End for loop

    Serial.println("Finished processing JSON commands.");
    request->send(200, "text/plain", message);

    root.clear();

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
            // sendCorsHeaders();
            request->send(200, "text/html", "Not sending command to " + device + ", already in state " + String(state));
          } else {
            sendHomePage(request, "Not sending command to " + device + ", already in state " + String(state), "Warning", 2);
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
      // sendCorsHeaders();
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


  // --- Routen für Backup und Restore hinzufügen ---
  server->on("/backup", HTTP_GET, handleBackup);

  // Handler für die Seite, die das Upload-Formular anzeigt (angenommen /buttons)
  server->on("/buttons", HTTP_GET, sendButtonConfigPage); // Stelle sicher, dass diese Funktion die UI hinzufügt (siehe Schritt 4)

  // Handler für den eigentlichen Upload-Vorgang
  // Der erste Handler (handleRestoreRequest) wird nach Abschluss des Uploads aufgerufen.
  // Der zweite Handler (handleRestoreUpload) wird während des Uploads für jeden Datenchunk aufgerufen.
  server->on("/restore", HTTP_POST, handleRestoreRequest, handleRestoreUpload);

  server->on("/savebackup", HTTP_POST, handleSaveNamedBackup);
  server->on("/loadbackup", HTTP_GET, handleLoadNamedBackup);
  server->on("/deletebackup", HTTP_GET, handleDeleteNamedBackup);

  // --- BUTTON HANDLER ---
  server->on("/buttons", HTTP_GET, handleButtonConfigPage);      // Zeigt die Übersicht
  server->on("/addbutton", HTTP_GET, handleAddButtonPage);       // Zeigt leeres Formular
  server->on("/editbutton", HTTP_GET, handleEditButtonPage);     // Zeigt befülltes Formular
  server->on("/deletebutton", HTTP_GET, handleDeleteButton);   // Löscht Button (GET für Einfachheit, POST wäre besser)
  server->on("/savebutton", HTTP_POST, handleSaveButton);
  server->on("/sendbutton", HTTP_GET, handleSendButton);
  server->on("/sendir", HTTP_POST, handleSendIr);

  server->begin();
  Serial.println("HTTP Server started on port " + String(port));

  irsend1.begin();
  irsend2.begin();
  irsend3.begin();
  irsend4.begin();
  irrecv.enableIRIn();
  Serial.println("Ready to send and receive IR signals");

  // --- SSE Initialisierung ---
  if (server != nullptr) {
    Serial.println("Creating AsyncEventSource at end of setup...");
    events = new AsyncEventSource("/events");
    if (events == nullptr) {
        Serial.println("FATAL ERROR: Failed to allocate memory for AsyncEventSource!");
        ESP.restart();
    } else {
        Serial.println("AsyncEventSource object created.");
        Serial.printf("Free Heap after AsyncEventSource (end of setup): %u\n", ESP.getFreeHeap());

        // onConnect und addHandler jetzt auch aktivieren
        events->onConnect([](AsyncEventSourceClient *client){ 
          if(client->lastId()){
            Serial.printf("EventSource Client Reconnected! Last ID: %u\n", client->lastId());
          } else {
            Serial.println("EventSource Client Connected!");
          }
        });

        server->addHandler(events);
        Serial.println("EventSource handler registered at /events");
    }
} else {
    Serial.println("FATAL ERROR: Server object is null, cannot create EventSource!");
    ESP.restart();
}

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

  // Event senden (nachdem last_send aktualisiert wurde)
  sendCodeUpdateEvent("codeSent", last_send);

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


void sendButtonConfigPage(AsyncWebServerRequest *request) {
  Serial.println("Connection received endpoint '/buttons' (GET)");

  AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8", 200);
  sendHeader(response);

  response->print("      <div class='row'>\n");
  response->print("        <div class='col-md-12'>\n");

  // --- Feedback-Meldungen anzeigen (ERWEITERT) ---
  if (request->hasParam("status")) {
      String status = request->getParam("status")->value();
      // Bestehende Status...
      if (status == "saved") response->print("<div class='alert alert-success'>Button saved successfully.</div>");
      else if (status == "deleted") response->print("<div class='alert alert-success'>Button deleted successfully.</div>");
      else if (status == "error_invalid_id") response->print("<div class='alert alert-danger'>Error: Invalid button ID specified.</div>");
      else if (status == "error_invalid_data") response->print("<div class='alert alert-danger'>Error: Invalid data submitted for button.</div>");
      else if (status == "error_save") response->print("<div class='alert alert-danger'>Error: Could not save button configuration.</div>");
      else if (status == "error_max_buttons") response->print("<div class='alert alert-warning'>Warning: Maximum number of buttons reached. Could not add new button.</div>");
      else if (status == "error_invalid_json") response->print("<div class='alert alert-danger'>Error: Invalid JSON format for Macro.</div>");
      else if (status == "error_json_not_array") response->print("<div class='alert alert-danger'>Error: Macro JSON must be a valid JSON array.</div>");
      else if (status == "error_invalid_macro_data") response->print("<div class='alert alert-danger'>Error: Macro JSON cannot be empty.</div>");
      // Status für Backup/Restore
      else if (status == "backup_saved") response->print("<div class='alert alert-success'>Configuration backup saved successfully.</div>");
      else if (status == "backup_loaded") response->print("<div class='alert alert-success'>Configuration restored successfully from backup.</div>");
      else if (status == "backup_deleted") response->print("<div class='alert alert-success'>Configuration backup deleted successfully.</div>");
      else if (status == "error_missing_backup_name") response->print("<div class='alert alert-danger'>Error: Backup name was missing.</div>");
      else if (status == "error_empty_backup_name") response->print("<div class='alert alert-danger'>Error: Backup name cannot be empty.</div>");
      else if (status == "error_invalid_backup_name") response->print("<div class='alert alert-danger'>Error: Invalid characters in backup name. Use only letters, numbers, underscore, or hyphen.</div>");
      else if (status == "error_backup_save") response->print("<div class='alert alert-danger'>Error: Could not save configuration backup. Check logs.</div>");
      else if (status == "error_backup_not_found") response->print("<div class='alert alert-danger'>Error: Specified backup file not found.</div>");
      else if (status == "error_backup_load") response->print("<div class='alert alert-danger'>Error: Could not load configuration from backup. Check logs.</div>");
      else if (status == "error_backup_delete") response->print("<div class='alert alert-danger'>Error: Could not delete configuration backup. Check logs.</div>");
      else if (status == "manual_send_success") response->print("<div class='alert alert-success'>Manual IR code sent successfully.</div>");
      else if (status == "error_missing_args") response->print("<div class='alert alert-danger'><strong>Error!</strong> Missing required form fields (type, data, length) for manual send.</div>");
      else if (status == "error_invalid_args") response->print("<div class='alert alert-danger'><strong>Error!</strong> Invalid form data (e.g., length 0 or empty data) for manual send.</div>");
    }

  // --- Backup/Restore Sektion ---
  response->print("          <hr><h2>Manage Configuration Backups</h2>");
  response->print("          <div style='margin-bottom: 20px;'>");
  response->print("            <h4>Save Current Configuration As:</h4>");
  response->print("            <form method='POST' action='/savebackup' class='form-inline'>");
  response->print("              <div class='form-group'>");
  response->print("                <label for='backup_name' class='sr-only'>Backup Name</label>");
  response->print("                <input type='text' class='form-control' id='backup_name' name='backup_name' placeholder='e.g., living_room_setup' required>");
  response->print("                <button type='submit' class='btn btn-primary'>Save Backup</button>");
  response->print("                <a href='/backup' class='btn btn-info'>Download Active Config (buttons.json)</a>");
  response->print("              </div>");
  response->print("            </form>");
  response->print("          </div>");
  response->print("          <h4>Available Backups:</h4>");
  response->print("          <ul class='list-group'>");
  File backupDir = LittleFS.open("/backups");
  if (!backupDir) {
      response->print("<li class='list-group-item list-group-item-warning'>Could not open backups directory.</li>");
  } else if (!backupDir.isDirectory()) {
      response->print("<li class='list-group-item list-group-item-danger'>Error: /backups is not a directory!</li>");
  } else {
      File file = backupDir.openNextFile();
      bool foundFiles = false;
      while(file){
          if (!file.isDirectory() && String(file.name()).endsWith(".json")) {
              foundFiles = true;
              String filename = String(file.name());
              String displayName = filename;
              if (displayName.startsWith("/backups/")) {
                  displayName = displayName.substring(9);
              }
              response->print("<li class='list-group-item'>");
              response->print(displayName);
              response->print("<div style='float: right;'>");
              response->print("<a href='/loadbackup?name=" + filename + "' class='btn btn-xs btn-success' style='margin-left: 10px;' onclick='return confirm(\"Load backup \\'" + displayName + "\\'? This will overwrite the current active configuration.\");'>Load</a>");
              response->print("<a href='/deletebackup?name=" + filename + "' class='btn btn-xs btn-danger' style='margin-left: 5px;' onclick='return confirm(\"Delete backup \\'" + displayName + "\\'?\");'>Delete</a>");
              response->print("</div>");
              response->print("</li>\n");
          }
          file = backupDir.openNextFile();
          yield();
      }
      if (!foundFiles) {
          response->print("<li class='list-group-item'><em>No backups found.</em></li>");
      }
  }
  if (backupDir) backupDir.close();
  response->print("          </ul><hr>");
  // --- ENDE Backup/Restore Sektion ---

  // --- ANZEIGE DER BUTTONS ---
  response->print("          <h2>Current Active Buttons</h2>\n");
  response->print("          <table class='table table-striped table-condensed' style='font-size: 0.9em;'>\n");
  // --- Tabellenkopf ---
  response->print("            <thead><tr><th>ID</th><th>Name</th><th>Row</th><th>Col</th><th>Color</th><th>Type</th><th>Data/Macro Preview</th><th>Mode</th><th>Actions</th></tr></thead>\n"); // <-- NEUE SPALTE: Color
  response->print("            <tbody>\n");

  if (!buttonConfigs.empty()) {
    for (size_t i = 0; i < buttonConfigs.size(); ++i) {
      const auto& button = buttonConfigs[i];

      // Nur konfigurierte Buttons anzeigen
      if (!button.configured) continue;

      response->print("              <tr>\n");
      // --- KORRIGIERTE Reihenfolge und Inhalt der Zellen ---
      response->print("                <td>" + String(i) + "</td>\n"); // ID
      response->print("                <td>" + String(button.name) + "</td>\n"); // Name
      response->print("                <td>" + String(button.layoutRow) + "</td>\n"); // Row
      response->print("                <td>" + String(button.layoutCol) + "</td>\n"); // Col
      response->print("                <td><span class='label " + String(button.colorClass) + "'>" + String(button.colorClass) + "</span></td>\n"); // Zeigt Klasse mit Farb-Badge


      // Conditional columns based on mode
      if (button.isMacro) {
          response->print("                <td><code>-</code></td>\n"); // Type (N/A for Macro)
          // Macro Preview
          String macroSnippet = String(button.macroJson);
          if (macroSnippet.length() > 50) { // Gekürzte Vorschau
              macroSnippet = macroSnippet.substring(0, 47) + "...";
          }
          response->print("                <td><pre style='margin:0; padding: 2px; font-size: 0.9em;'>" + macroSnippet + "</pre></td>\n"); // Data/Macro Preview
          response->print("                <td>Macro</td>\n"); // Mode
      } else {
          response->print("                <td><code>" + String(button.type) + "</code></td>\n"); // Type
          // Single IR Preview (kombiniert)
          String singlePreview = "D:<code>" + String(button.data) + "</code> L:" + String(button.length) + " A:<code>" + (String(button.address).length() > 0 ? String(button.address) : "-") + "</code> R:" + String(button.repeat) + " O:" + String(button.out);
          response->print("                <td>" + singlePreview + "</td>\n"); // Data/Macro Preview
          response->print("                <td>Single IR</td>\n"); // Mode
      }

      // Actions Spalte (bleibt gleich)
      response->print("                <td>\n");
      response->print("                  <a href='/editbutton?id=" + String(i) + "' class='btn btn-xs btn-warning' style='margin-right: 3px;'>Edit</a>\n");
      // Verwende POST für Delete, wenn möglich, aber behalte GET für jetzt bei, wie im bestehenden Code
      response->print("                  <a href='/deletebutton?id=" + String(i) + "' class='btn btn-xs btn-danger' onclick='return confirm(\"Are you sure you want to delete button \\'" + String(button.name) + "\\'?\");'>Delete</a>\n");
      response->print("                </td>\n");
      response->print("              </tr>\n");
      yield(); // Wichtig bei vielen Buttons
    }
  } else {
    // --- colspan ---
    response->print("              <tr><td colspan='9' class='text-center'><em>No active buttons configured.</em></td></tr>\n"); // 9 Spalten jetzt
  }

  response->print("            </tbody>\n");
  response->print("          </table>\n");
  // --- ENDE KORRIGIERTE BUTTON-TABELLE ---

  // --- AKTIONEN (wie gehabt) ---
  response->print("          <a href='/addbutton' class='btn btn-success'>Add New Button</a>\n");
  response->print("          <a href='/' class='btn btn-default' style='margin-left: 10px;'>Back to Home</a>\n");

  response->print("        </div>\n"); // Ende col-md-12
  response->print("      </div>\n");   // Ende row

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
  // --- Globale Funktion aufrufen ---
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
  // --- Globale Funktion aufrufen ---
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

  yield(); // Keep the existing yield after the Received table

  sendFooter(response);
  request->send(response);
}


//+=============================================================================
// Send header HTML (Async Version)
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

  response->print("      <hr />\n");
}


//+=============================================================================
// Send footer HTML (AsyncResponseStream Version) - DREI SPALTEN NEBENEINANDER
void sendFooter(AsyncResponseStream *response) {

  // --- Reihe 1: Device Info, Pin Config, Memory Usage (nebeneinander) ---
  response->print("      <div class='row'>\n"); // <-- Die EINE Reihe für alle drei

  // --- Spalte 1: Device Information ---
  response->print("        <div class='col-md-4'>\n"); // <-- Spalte 1 (1/3 Breite)
  response->print("          <h4>Device Information</h4>\n"); // Kleinere Überschrift (h4)
  response->print("          <ul class='list-unstyled' style='font-size: 0.9em;'>\n"); // Kleinere Schrift

  // Hostname Info
  String hostInfo = "            <li><strong>Hostname:</strong> <a href='http://" + String(host_name) + ".local" + ":" + String(port_str) + "'>" + String(host_name) + ".local" + ":" + String(port_str) + "</a></li>\n";
  response->print(hostInfo);

  // Local IP Info
  String localInfo = "            <li><strong>Local IP:</strong> <a href='http://" + WiFi.localIP().toString() + ":" + String(port_str) + "'>" + WiFi.localIP().toString() + ":" + String(port_str) + "</a></li>\n";
  response->print(localInfo);

  // DNS IP Info
  String dnsInfo = "            <li><strong>DNS IP:</strong> <a href='http://" + WiFi.dnsIP().toString() + "'>" + WiFi.dnsIP().toString() + "</a></li>\n";
  response->print(dnsInfo);

  // MAC Address Info
  String macInfo = "            <li><strong>MAC Address:</strong> <code>" + String(WiFi.macAddress()) + "</code></li>\n";
  response->print(macInfo);

  response->print("          </ul>\n");
  response->print("        </div>\n"); // <-- Ende Spalte 1 (Device Info)

  // --- Spalte 2: Pin Configuration ---
  response->print("        <div class='col-md-4'>\n"); // <-- Spalte 2 (1/3 Breite)
  response->print("          <h4>Pin Configuration</h4>\n"); // Kleinere Überschrift (h4)
  response->print("          <ul class='list-unstyled' style='font-size: 0.9em;'>\n"); // Kleinere Schrift
  response->print("            <li><span class='badge'>GPIO " + String(pinr1) + "</span> Receiving </li>\n");
  response->print("            <li><span class='badge'>GPIO " + String(pins1) + "</span> Transmitter 1 </li>\n");
  response->print("            <li><span class='badge'>GPIO " + String(pins2) + "</span> Transmitter 2 </li>\n");
  response->print("            <li><span class='badge'>GPIO " + String(pins3) + "</span> Transmitter 3 </li>\n");
  response->print("            <li><span class='badge'>GPIO " + String(pins4) + "</span> Transmitter 4 </li></ul>\n");
  response->print("        </div>\n"); // <-- Ende Spalte 2 (Pin Config)

  // --- Spalte 3: Memory Usage ---
  response->print("        <div class='col-md-4'>\n"); // <-- Spalte 3 (1/3 Breite)
  response->print("          <h4>Memory Usage</h4>\n"); // Kleinere Überschrift (h4)

  // --- Variablen und Berechnungen für Memory Usage (innerhalb der Spalte) ---
  char buffer[60];
  int barWidth = 10; // Schmalere Balken für weniger Platz

  // --- LittleFS ---
  uint32_t totalBytesFS = 0;
  uint32_t usedBytesFS = 0;
  float percentFS = 0;
  float usedKB_fs = 0;
  String bar_fs = "";
  String fsStatus = "OK";
  totalBytesFS = LittleFS.totalBytes();
  usedBytesFS = LittleFS.usedBytes();
  if (totalBytesFS > 0) {
      percentFS = (float)usedBytesFS / totalBytesFS * 100.0;
      usedKB_fs = (float)usedBytesFS / 1024.0;
      int filled = round(percentFS / 100.0 * barWidth);
      for (int i = 0; i < barWidth; i++) { bar_fs += (i < filled) ? "=" : "-"; }
  } else { fsStatus = "Size Error"; }

  // --- Flash (Sketch) ---
  uint32_t sketchSize = ESP.getSketchSize();
  const esp_partition_t* running = esp_ota_get_running_partition();
  uint32_t totalSketchPartitionSize = 0;
  float percentFlash = 0;
  float usedKB_flash = 0;
  String bar_flash = "";
  String flashStatus = "OK";
  if (running != NULL) {
      totalSketchPartitionSize = running->size;
      if (totalSketchPartitionSize > 0) {
          percentFlash = (float)sketchSize / totalSketchPartitionSize * 100.0;
          usedKB_flash = (float)sketchSize / 1024.0;
          int filled = round(percentFlash / 100.0 * barWidth);
          for (int i = 0; i < barWidth; i++) { bar_flash += (i < filled) ? "=" : "-"; }
      } else { flashStatus = "Size Error"; }
  } else { flashStatus = "Partition Error"; }

  // --- Heap (RAM) ---
  uint32_t totalHeap = ESP.getHeapSize();
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t usedHeap = totalHeap - freeHeap;
  float percentHeap = 0;
  float usedKB_heap = 0;
  String bar_heap = "";
  if (totalHeap > 0) {
      percentHeap = (float)usedHeap / totalHeap * 100.0;
      usedKB_heap = (float)usedHeap / 1024.0;
      int filled = round(percentHeap / 100.0 * barWidth);
      for (int i = 0; i < barWidth; i++) { bar_heap += (i < filled) ? "=" : "-"; }
  }
  // --- ENDE Memory Usage Variablen & Berechnungen ---

  // --- Memory Usage Tabelle HTML (innerhalb der Spalte, noch kompakter) ---
  response->print("          <table class='table table-condensed table-bordered' style='font-size: 0.8em;'>\n"); // Noch kleinere Schrift
  response->print("            <thead>\n");
  response->print("              <tr><th>T</th><th>Used</th><th>Total</th><th>%</th><th>Graph</th></tr>\n"); // Sehr kurze Header
  response->print("            </thead>\n");
  response->print("            <tbody>\n");

  // --- LittleFS Zeile ---
  response->print("              <tr>\n");
  response->print("                <td>FS</td>\n");
  if (fsStatus == "OK" && totalBytesFS > 0) {
      snprintf(buffer, sizeof(buffer), "%.0fK", usedKB_fs); // Keine Nachkommastelle
      response->print("                <td>" + String(buffer) + "</td>\n");
      snprintf(buffer, sizeof(buffer), "%.0fK", (float)totalBytesFS / 1024.0); // Keine Nachkommastelle
      response->print("                <td>" + String(buffer) + "</td>\n");
      snprintf(buffer, sizeof(buffer), "%.0f", percentFS); // Keine Nachkommastelle, kein %
      response->print("                <td>" + String(buffer) + "</td>\n");
      response->print("                <td><samp>" + bar_fs + "</samp></td>\n");
  } else { response->print("                <td colspan='4' class='text-danger'>" + fsStatus + "</td>\n"); }
  response->print("              </tr>\n");

  // --- Flash (Sketch) Zeile ---
  response->print("              <tr>\n");
  response->print("                <td>App</td>\n");
  if (flashStatus == "OK" && totalSketchPartitionSize > 0) {
      snprintf(buffer, sizeof(buffer), "%.0fK", usedKB_flash); // Keine Nachkommastelle
      response->print("                <td>" + String(buffer) + "</td>\n");
      snprintf(buffer, sizeof(buffer), "%.0fK", (float)totalSketchPartitionSize / 1024.0); // Keine Nachkommastelle
      response->print("                <td>" + String(buffer) + "</td>\n");
      snprintf(buffer, sizeof(buffer), "%.0f", percentFlash); // Keine Nachkommastelle, kein %
      response->print("                <td>" + String(buffer) + "</td>\n");
      response->print("                <td><samp>" + bar_flash + "</samp></td>\n");
  } else { response->print("                <td colspan='4' class='text-danger'>" + flashStatus + "</td>\n"); }
  response->print("              </tr>\n");

  // --- Heap (RAM) Zeile ---
  response->print("              <tr>\n");
  response->print("                <td>RAM</td>\n");
  if (totalHeap > 0) {
      snprintf(buffer, sizeof(buffer), "%.0fK", usedKB_heap); // Keine Nachkommastelle
      response->print("                <td>" + String(buffer) + "</td>\n");
      snprintf(buffer, sizeof(buffer), "%.0fK", (float)totalHeap / 1024.0); // Keine Nachkommastelle
      response->print("                <td>" + String(buffer) + "</td>\n");
      snprintf(buffer, sizeof(buffer), "%.0f", percentHeap); // Keine Nachkommastelle, kein %
      response->print("                <td>" + String(buffer) + "</td>\n");
      response->print("                <td><samp>" + bar_heap + "</samp></td>\n");
  } else { response->print("                <td colspan='4' class='text-danger'>N/A</td>\n"); }
  response->print("              </tr>\n");

  // --- Tabelle beenden ---
  response->print("            </tbody>\n");
  response->print("          </table>\n");
  // --- ENDE Memory Usage Tabelle HTML ---

  response->print("        </div>\n"); // <-- Ende Spalte 3 (Memory Usage)
  response->print("      </div>\n");   // <-- Ende der EINEN Reihe
  response->print("      <hr />\n");   // <-- Trennlinie NACH der 3-Spalten-Reihe

  // --- Uptime and Epoch (bleibt darunter in eigener Reihe) ---
  yield();
  response->print("      <div class='row'>\n"); // <-- Eigene Reihe für Uptime/Epoch
  response->print("        <div class='col-md-12'>\n"); // Volle Breite
  String uptimeEpochLine = "<em>" + String(millis()) + "ms uptime</em> / Client Epoch: <em id='jepoch'></em> / Diff: <em id='jdiff'></em>\n";
  response->print(uptimeEpochLine);
  response->print("        </div>\n");
  response->print("      </div>\n"); // <-- Ende Uptime/Epoch Reihe

  // --- Die Scripts für Epoch bleiben hier ---
  response->print("      <script>document.getElementById('jepoch').innerHTML = Math.round((new Date()).getTime() / 1000)</script>");
  response->print("      <script>document.getElementById('jdiff').innerHTML = Math.abs(Math.round((new Date()).getTime() / 1000) - " + String(now() - (timeZone * SECS_PER_HOUR)) + ")</script>");

  yield(); // <-- ADD YIELD after initial scripts

  // --- Restlicher Footer (Container Ende, JS Link, Body/HTML Ende) ---
  response->print("    </div>\n"); // Ende .container

  // --- Lade das generierte Skript am Ende des Body ---
  response->print("    <script src='/js/scripts.js'></script>\n");

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

  yield(); // <--- Yield IMMEDIATELY upon entry

  // --- Erstelle den Response Stream ---
  AsyncResponseStream *response = request->beginResponseStream("text/html; charset=utf-8", httpcode);
  if (!response) { // Check if stream creation failed (unlikely but possible)
      Serial.println("!!! ERROR: Failed to beginResponseStream!");
      request->send(500, "text/plain", "Internal Server Error");
      return;
  }

  sendHeader(response);

  yield();

  // --- Chunked Printing ---
  response->print("          <script>\n");

  const size_t chunkSize = 512; // Print in 512-byte chunks (adjust if needed)
  size_t totalLength = buttonMacroJsStore.length();
  for (size_t i = 0; i < totalLength; i += chunkSize) {
    size_t currentChunkSize = std::min(chunkSize, totalLength - i);
    // Use substring or direct pointer access if comfortable
    response->print(buttonMacroJsStore.substring(i, i + currentChunkSize));
    // Serial.printf("      Printed chunk %d/%d (%d bytes)\n", (i / chunkSize) + 1, (totalLength + chunkSize - 1) / chunkSize, currentChunkSize); // Log chunk progress
    yield(); // <--- Yield AFTER printing each chunk
  }
  // response->print(buttonMacroJsStore); // OLD: Print all at once (Remove/Comment this)

  response->print("          </script>\n");
  // yield(); // The yield inside the loop makes this one potentially redundant, but keep it for safety for now.

  // +++ FERNBEDIENUNGS-BUTTONS ANZEIGEN (GRID LAYOUT) +++
  response->print("      <div class='row'>\n");
  response->print("        <div class='col-md-12' id='remote-buttons'>\n"); // Container behalten
  response->print("          <h3>Remote Buttons</h3>\n");

  // --- Grid-Logik ---
  int maxRow = -1;
  int maxCol = -1;
  std::vector<ButtonConfig*> buttonsWithLayout;
  std::vector<ButtonConfig*> buttonsWithoutLayout;

  // 1. Buttons sortieren und maxRow/maxCol finden
  for (auto& button : buttonConfigs) { // Referenz verwenden!
    if (button.configured) {
        if (button.layoutRow >= 0 && button.layoutCol >= 0) {
            buttonsWithLayout.push_back(&button); // Zeiger speichern
            if (button.layoutRow > maxRow) maxRow = button.layoutRow;
            if (button.layoutCol > maxCol) maxCol = button.layoutCol;
        } else {
            buttonsWithoutLayout.push_back(&button); // Zeiger speichern
        }
    }
}
Serial.printf("    Grid Layout: Max Row = %d, Max Col = %d\n", maxRow, maxCol);

// 2. Grid generieren (wenn Layout-Buttons vorhanden sind)
if (maxRow >= 0 && maxCol >= 0) {
    // Bestimme die Spaltenbreite (z.B. 12 / (maxCol + 1))
    // Hier ein Beispiel für max. 4 Spalten (col-xs-3), anpassbar!
    int colsPerButton = 3; // 12 / 4 = 3 -> 4 Spalten auf kleinsten Screens
    if (maxCol + 1 > 4) colsPerButton = 2; // Bei mehr als 4 Spalten -> 6 Spalten (col-xs-2)
    if (maxCol + 1 > 6) colsPerButton = 1; // Bei mehr als 6 Spalten -> 12 Spalten (col-xs-1)
    String colClass = "col-xs-" + String(colsPerButton) + " text-center"; // Zentriert den Inhalt

    Serial.printf("    Using column class: %s\n", colClass.c_str());

    for (int r = 0; r <= maxRow; ++r) {
        response->print("          <div class='row' style='margin-bottom: 10px;'>\n"); // Eine Bootstrap-Reihe pro Layout-Zeile
        for (int c = 0; c <= maxCol; ++c) {
            ButtonConfig* btnPtr = nullptr;
            // Finde den Button für diese Zelle (r, c)
            for (auto* p : buttonsWithLayout) {
                if (p->layoutRow == r && p->layoutCol == c) {
                    btnPtr = p;
                    break;
                }
            }

            response->print("            <div class='" + colClass + "'>\n"); // Spalte öffnen
            if (btnPtr != nullptr) {
                // Button gefunden -> HTML generieren
                const ButtonConfig& button = *btnPtr; // Dereferenzieren
                String buttonId = "btn_" + String(std::distance(buttonConfigs.data(), btnPtr)); // Index im Originalvektor finden

                String buttonHtml = "<button id='" + buttonId + "' class='btn " + String(button.colorClass) + " btn-lg remote-button' style='margin: 2px;' "; // <-- HIER ANPASSEN
                buttonHtml += "data-ismacro='" + String(button.isMacro ? "true" : "false") + "' ";
                if (!button.isMacro) {
                    buttonHtml += "data-type='" + String(button.type) + "' ";
                    buttonHtml += "data-data='" + String(button.data) + "' ";
                    buttonHtml += "data-length='" + String(button.length) + "' ";
                    buttonHtml += "data-address='" + String(button.address) + "' ";
                    buttonHtml += "data-repeat='" + String(button.repeat) + "' ";
                    buttonHtml += "data-out='" + String(button.out) + "'";
                }
                buttonHtml += ">";
                buttonHtml += String(button.name);
                buttonHtml += "</button>\n";
                response->print(buttonHtml);
            } else {
                // Kein Button für diese Zelle -> Platzhalter
                response->print("&nbsp;"); // Leeres Leerzeichen für Höhe oder leeres Div
            }
            response->print("            </div>\n"); // Spalte schließen
            yield(); // Innerhalb der Spaltenschleife
        }
        response->print("          </div>\n"); // Bootstrap-Reihe schließen
        yield(); // Nach jeder Reihe
    }
    response->print("<hr/>"); // Trennlinie nach dem Grid
} // Ende if (maxRow >= 0)

// 3. Buttons ohne Layout-Info anhängen (Standardfluss)
if (!buttonsWithoutLayout.empty()) {
    response->print("          <div class='row'>\n"); // Eigene Reihe für Buttons ohne Layout
    response->print("            <div class='col-xs-12'>\n"); // Volle Breite
    response->print("              <h4>Other Buttons:</h4>\n");
    for (auto* btnPtr : buttonsWithoutLayout) {
        const ButtonConfig& button = *btnPtr;
        String buttonId = "btn_" + String(std::distance(buttonConfigs.data(), btnPtr));

        // Button HTML (wie oben, aber mit Standard-Margin)
        String buttonHtml = "<button id='" + buttonId + "' class='btn " + String(button.colorClass) + " btn-lg remote-button' style='margin: 5px;' "; // <-- HIER ANPASSEN
        buttonHtml += "data-ismacro='" + String(button.isMacro ? "true" : "false") + "' ";
        if (!button.isMacro) {
            buttonHtml += "data-type='" + String(button.type) + "' ";
            buttonHtml += "data-data='" + String(button.data) + "' ";
            buttonHtml += "data-length='" + String(button.length) + "' ";
            buttonHtml += "data-address='" + String(button.address) + "' ";
            buttonHtml += "data-repeat='" + String(button.repeat) + "' ";
            buttonHtml += "data-out='" + String(button.out) + "'";
        }
        buttonHtml += ">";
        buttonHtml += String(button.name);
        buttonHtml += "</button>\n";
        response->print(buttonHtml);
        yield();
    }
    response->print("            </div>\n");
    response->print("          </div><hr/>\n"); // Trennlinie nach den "anderen" Buttons
}

// Link zum Konfigurieren (jetzt nach allen Buttons platziert)
response->print("          <a href='/buttons' class='btn btn-default' style='margin: 5px;'>Configure Buttons / Send IR Code for testing</a>\n");

response->print("        </div>\n"); // Ende col-md-12 (remote-buttons container)
response->print("      </div><hr />\n"); // Ende row (remote-buttons container)
// +++ ENDE FERNBEDIENUNGS-BUTTONS (GRID LAYOUT) +++

  yield();

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
  response->print("            <thead><tr><th>Sent</th><th>Command</th><th>Type</th><th>Length</th><th>Address</th><th>Repeat</th><th>Out</th><th>Button Match</th></tr></thead>\n");
  response->print("            <tbody id='sent-codes-body'>\n");
  auto generateSentRow = [&](const Code& code) {
      if (code.valid) {
          const ButtonConfig* matchedButton = findMatchingButton(code); // <-- Aufruf der neuen Funktion
          String matchCell = "<td>-</td>"; // Default
          if (matchedButton != nullptr) {
              // --- Bootstrap Label-Klasse aus Button-Klasse ableiten ---
              // Bootstrap Labels verwenden label-primary, label-success etc.
              String labelClass = String(matchedButton->colorClass);
              labelClass.replace("btn-", "label-"); // Ersetze "btn-" durch "label-"
              // --- Ende Ableitung ---
              matchCell = "<td><span class='label " + labelClass + "'>" + String(matchedButton->name) + "</span></td>"; // <-- Verwende Name und abgeleitete Klasse
          }
          // Generiere die Zeile mit der (ggf. aktualisierten) matchCell
          String rowHtml = "              <tr class='text-uppercase'><td>" + epochToString(code.timestamp) + "</td><td><code>" + String(code.data) + "</code></td><td><code>" + String(code.encoding) + "</code></td><td><code>" + String(code.bits) + "</code></td><td><code>" + String(code.address) + "</code></td><td><code>" + String(code.repeat) + "</code></td><td><code>" + String(code.out) + "</code></td>" + matchCell + "</tr>\n"; // <-- matchCell am Ende
          response->print(rowHtml);
      }
  };


  generateSentRow(last_send);
  yield();
  generateSentRow(last_send_2);
  yield(); 
  generateSentRow(last_send_3);
  yield();
  generateSentRow(last_send_4);
  yield();
  generateSentRow(last_send_5);
  yield(); 

  // --- Add Log after lambda calls ---
  // Serial.println("    Finished generating Sent Codes rows.");

  // Platzhalterzeile mit ID versehen
  if (!last_send.valid && !last_send_2.valid && !last_send_3.valid && !last_send_4.valid && !last_send_5.valid)
  response->print("              <tr id='no-sent-codes'><td colspan='8' class='text-center'><em>No codes sent</em></td></tr>"); // <-- colspan="8"
  response->print("            </tbody></table>\n");
  response->print("          </div></div>\n");

  yield(); // Keep the existing yield after the Sent table

  // --- Codes Received Table ---
  response->print("      <div class='row'>\n");
  response->print("        <div class='col-md-12'>\n");
  response->print("          <h3>Codes Received</h3>\n");
  response->print("          <table class='table table-striped' style='table-layout: fixed;'>\n");
  response->print("            <thead><tr><th>Received</th><th>Command</th><th>Type</th><th>Length</th><th>Address</th><th>Button Match</th></tr></thead>\n");
  response->print("            <tbody id='received-codes-body'>\n");
  auto generateReceivedRow = [&](const Code& code, int id) {
      if (code.valid) {
          const ButtonConfig* matchedButton = findMatchingButton(code);
          String matchCell = ""; // Leeren String initialisieren

          if (matchedButton != nullptr) {
              // --- Match gefunden: Farbigen Label anzeigen ---
              String labelClass = String(matchedButton->colorClass);
              labelClass.replace("btn-", "label-");
              matchCell = "<td><span class='label " + labelClass + "'>" + String(matchedButton->name) + "</span></td>";
          } else {
              // --- KEIN Match gefunden: "Create Button"-Link anzeigen ---
              // Baue die URL mit Prefill-Parametern
              String createUrl = "/addbutton?";
              createUrl += "prefill_type=" + String(code.encoding);
              createUrl += "&prefill_data=" + String(code.data);
              createUrl += "&prefill_length=" + String(code.bits);
              // Adresse nur hinzufügen, wenn sie nicht "0x0" oder leer ist (optional, aber sauberer)
              String normAddr = normalizeHex(String(code.address));
              if (normAddr.length() > 0 && normAddr != "0") {
                 createUrl += "&prefill_address=" + String(code.address); // Originalformat beibehalten
              }
              // Optional: Weitere Defaults wie repeat=1, out=1 könnten hier auch gesetzt werden

              matchCell = "<td><a href='" + createUrl + "' class='btn btn-xs btn-success' title='Create button from this code'>Create Button</a></td>";
          }

          // Generiere die Zeile mit der matchCell
          String rowHtml = "              <tr class='text-uppercase'><td><a href='/received?id=" + String(id) + "'>" + epochToString(code.timestamp) + "</a></td><td><code>" + String(code.data) + "</code></td><td><code>" + String(code.encoding) + "</code></td><td><code>" + String(code.bits) + "</code></td><td><code>" + String(code.address) + "</code></td>" + matchCell + "</tr>\n";
          response->print(rowHtml);
      }
  };
  generateReceivedRow(last_recv, 1);
  yield();
  generateReceivedRow(last_recv_2, 2);
  yield();
  generateReceivedRow(last_recv_3, 3);
  yield();
  generateReceivedRow(last_recv_4, 4);
  yield();
  generateReceivedRow(last_recv_5, 5);
  yield();

  // Platzhalterzeile mit ID versehen
  if (!last_recv.valid && !last_recv_2.valid && !last_recv_3.valid && !last_recv_4.valid && !last_recv_5.valid)
  response->print("              <tr id='no-received-codes'><td colspan='6' class='text-center'><em>No codes received</em></td></tr>"); // <-- colspan="6"
  response->print("            </tbody></table>\n");
  response->print("          </div></div><hr />\n");
  
  yield(); // yield after the Received table

  // --- Schreibe Footer in den Stream ---
  sendFooter(response); // Übergibt den Stream

  // --- Sende den kompletten Stream ---
  request->send(response);

  // --- Stack Check at End ---
  UBaseType_t stackHighWaterMarkEnd = uxTaskGetStackHighWaterMark(NULL);
  // Serial.printf("<-- Leaving sendHomePage (after send) (Stack HWM: %u bytes, Min Free: %u)\n", stackHighWaterMarkEnd, stackHighWaterMarkEnd); // HWM is minimum free stack

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

  Serial.println("    Calling sendHeader...");
  sendHeader(response);

   // --- Check if stream creation failed ---
   if (response == nullptr) {
    Serial.println("!!! ERROR: Failed to beginResponseStream in sendCodePage!");
    request->send(500, "text/plain", "Internal Server Error");
    return;
  }

  sendHeader(response);
  // --- END CHECK ---

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

  last_send.repeat = repeat;
  last_send.out = out_pin;

  resetReceive();
  // Event senden
  sendCodeUpdateEvent("codeSent", last_send);
  Serial.println("  <== irblast: Leaving function.");
}

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

  last_send.repeat = repeat;
  last_send.out = out_pin;

  resetReceive();

  // Event senden
  sendCodeUpdateEvent("codeSent", last_send);

}

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

  last_send.repeat = repeat;
  last_send.out = out_pin;

  resetReceive();

  // Event senden
  sendCodeUpdateEvent("codeSent", last_send);
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

    // Event senden
    sendCodeUpdateEvent("codeReceived", last_recv);
    
    Serial.println("");                                           // Blank line between entries
    irrecv.resume();                                              // Prepare for the next value
    digitalWrite(ledpin, LOW);                                    // Turn on the LED for 0.5 seconds
    ticker.attach(0.5, disableLed);
  }
  delay(50);
}
