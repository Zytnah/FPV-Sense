// Waveshare D500 LiDAR UART Parser für ESP32
// LiDAR TX an ESP32 RX_PIN, LiDAR RX an ESP32 TX_PIN
// Baudrate: 230400

// Passen Sie die Pins an Ihr Board/Setup an:
#define RX_PIN 16   // GPIO16 (z.B. für UART2)
#define TX_PIN 17   // GPIO17

#define BF_RX_PIN 4   
#define BF_TX_PIN 5

#define LEN1 170
#define LEN2 250

#define SERVICE_UUID        "91bad492-b950-4226-aa2b-4ede9fa42f59"
#define CHARACTERISTIC_UUID "cba1d466-344c-4be3-ab3f-189f80dd7518"

#define MSP_SET_RAW_RC 200

#define LED_PIN 2

#include <HardwareSerial.h>
#include <cmath>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

HardwareSerial lidarSerial(2);  // UART2 auf ESP32
HardwareSerial BetaflightSerial(1); //UART1 auf EPS32

//LIDAR Parsing
const int PACKET_SIZE = 9;
uint8_t packet[PACKET_SIZE];

int packetIndex = 0;
bool packetStarted = false;

//Näherung 
float currentAngles1[LEN1];
int currentDistances1[LEN1];
float prevAngles1[LEN2];
int prevDistances1[LEN2];

//Ausweichen
float evasionAngle;

// BLE Switch
bool mainSwitch = false;

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = String(pCharacteristic->getValue().c_str());
    if (value.length() > 0) {
      if (value[0] == '1') {
        mainSwitch = true;
      } else {
        mainSwitch = false;
      }
      Serial.print("mainSwitch wurde gesetzt auf: ");
      Serial.println(mainSwitch ? "true" : "false");
    }
  }
};

// Server-Callback für Verbindungsabbruch
class SenseServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.println("Client verbunden.");
  }

  void onDisconnect(BLEServer* pServer) {
    mainSwitch = false;
    Serial.println("Client getrennt! mainSwitch wurde auf false gesetzt.");
  }
};
void evadeDirection(HardwareSerial &serial, float angle_deg, int duration_ms = 200, int16_t offset = 100);


void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);               // Für Ausgabe an den PC
  lidarSerial.begin(230400, SERIAL_8N1, RX_PIN, TX_PIN);  // LiDAR
  BetaflightSerial.begin(115200, SERIAL_8N1, BF_RX_PIN, BF_TX_PIN); //Betaflight
  Serial.println("D500 LiDAR Parser (ESP32) gestartet");
  Serial.println("Betaflight Serial - (ESP32) gestartet");

  //Blootooth Server aufbau
  BLEDevice::init("ESP32_BLE_Switch");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new SenseServerCallbacks()); // <- Server-Callback setzen

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->setValue("0"); // Initialwert
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();

  Serial.println("Warte auf Verbindung vom Handy...");


  /////// TEST TEST //////////////////////////
 // Arrays generieren zum testen des Annähern
  for (int i = 0; i < LEN1; i++) {
    // Winkel zwischen 0 und 169
    currentAngles1[i] = i;
    prevAngles1[i] = i;

    // Objekt im Bereich 30° bis 45°
    if (i >= 30 && i <= 45) {
      prevDistances1[i] = 480 + random(0, 20);         // vorher knapp unter 500mm
      currentDistances1[i] = prevDistances1[i] - 20;   // jetzt noch kleiner, z.B. Annäherung
    } else {
      prevDistances1[i] = 600 + random(0, 200);        // alles über 500mm
      currentDistances1[i] = prevDistances1[i] + random(0, 10);
    }
  }


  float resultAngles[LEN1]; // Maximale Länge = currentLen
  int resultCount;

  detectApproachingObjects(currentAngles1, currentDistances1, LEN1,
                          prevAngles1, prevDistances1, LEN2,
                          resultAngles, resultCount);

  // for (int i = 0; i < resultCount; i++) {
  //     Serial.print("Annäherndes Objekt bei Winkel: ");
  //     Serial.println(resultAngles[i]);
  // }
  
}




void loop() {
  digitalWrite(LED_PIN, mainSwitch ? HIGH : LOW);
  if(mainSwitch){
    while (lidarSerial.available()) {
      uint8_t b = lidarSerial.read();

      // Paket-Synchronisation: Paket beginnt mit 0x54 0x2C
      if (!packetStarted) {
        if (b == 0x54) {
          packet[0] = b;
          packetIndex = 1;
          packetStarted = true;
        }
      } else {
        packet[packetIndex++] = b;
        if (packetIndex == 2 && packet[1] != 0x2C) {
          // Falsches Header, zurücksetzen
          packetStarted = false;
          packetIndex = 0;
        } else if (packetIndex >= PACKET_SIZE) {
          // Paket komplett, verarbeiten
          if (checkChecksum(packet)) {
            parsePacket(packet);
          } else {
            Serial.println("Ungültige Checksumme");
          }
          packetStarted = false;
          packetIndex = 0;
        }
      }
    }

    float resultAngles[] = {10, 12, 15, 19, 24, 40};
    int numAngles = 6;
    float resultMeans[numAngles];
    int numMeans = 0;

    calculateMeanDirections(resultAngles, numAngles, resultMeans, numMeans);
    //CheckManeuverPossibility
    //Wenn es 1 Mittel wert ist => gib einen Steuerimpuls in die entgegengesetze Richtung
    //Wenn es 2 oder Mehr Mittelwerte sind => Falls sich der winkel zwischen den Winkeln unter 160° liegt wird der Mittel wert dieser zusammen gerechnet und in die entgegengesetzte Richtung dieses Mittelwertes gesteuert
    //                                     => Falls sich der Winkel zwischen den Winkeln über 160° so wird kein Ausweichmanöver ausgelöst
    int resultMeanslength = sizeof(resultMeans) / sizeof(resultMeans[0]);

    if(resultMeanslength == 0){
      Serial.println("Kein Manöver vollzogen");
    }else if(resultMeanslength == 1){
      evasionAngle = fmod((resultMeans[0] + 180.0f), 360.0f);
      evadeDirection(BetaflightSerial, evasionAngle);
    }else if (resultMeanslength >= 2){
      //Sort Array
      for (int i = 0; i < resultMeanslength - 1; i++) {
        for (int j = 0; j < resultMeanslength - i - 1; j++) {
          if (resultMeans[j] > resultMeans[j + 1]) {
            float temp = resultMeans[j];
            resultMeans[j] = resultMeans[j + 1];
            resultMeans[j + 1] = temp;
          }
        }
      }
      float diff = resultMeans[resultMeanslength - 1] - resultMeans[0];
      if(diff <= 160){
        float sum = 0.0;
        for (int i = 0; i < resultMeanslength; i++) {
          sum += resultMeans[i];
        }
        evasionAngle = fmod(((sum / resultMeanslength) + 180.0f),  360.0f);
        evadeDirection(BetaflightSerial, evasionAngle);
      }else{
        Serial.println("Kein Manöver vollzogen");
      }
      
    }else{
      Serial.println("Kein Manöver vollzogen");
    }
  }else{
    delay(500);
  }
}
/////// FLIGHT INFO FROM BETAFLIGHT ///////////////////////////////////////////
// TODO: Noch werden diese Daten nicht verwendet
// Command Liste
// MSP_RC	      105	    RC-Werte
// MSP_ATTITUDE	108	    Neigung (Roll, Pitch, Yaw)
// MSP_RAW_GPS	106	    GPS-Daten
// MSP_ANALOG   110	    Batteriespannung, RSSI
// MSP_SET_RAW_RC 200   8 rc chan
//https://github.com/betaflight/betaflight/blob/master/src/main/msp/msp_protocol.h Alle MSP befehle
void sendMSPPacket(HardwareSerial &serial, uint8_t msp_code, uint8_t *payload, uint8_t payload_size) {
  uint8_t checksum = 0;
  serial.write('$');
  serial.write('M');
  serial.write('<');
  serial.write(payload_size);
  checksum ^= payload_size;
  serial.write(msp_code);
  checksum ^= msp_code;
  for (int i = 0; i < payload_size; i++) {
    serial.write(payload[i]);
    checksum ^= payload[i];
  }
  serial.write(checksum);
}


void sendRCOverride(HardwareSerial &serial, uint16_t roll, uint16_t pitch, uint16_t yaw, uint16_t throttle) {
  uint8_t payload[8];
  payload[0] = roll & 0xFF;
  payload[1] = (roll >> 8) & 0xFF;
  payload[2] = pitch & 0xFF;
  payload[3] = (pitch >> 8) & 0xFF;
  payload[4] = yaw & 0xFF;
  payload[5] = (yaw >> 8) & 0xFF;
  payload[6] = throttle & 0xFF;
  payload[7] = (throttle >> 8) & 0xFF;
  sendMSPPacket(serial, MSP_SET_RAW_RC, payload, 8);
}
// Evade hält den Restlichen Code während der Dauer des Ausweichmanövers blockiert. 
// Optimal wäre ein nicht blockierender Code, jedoch bräuchte dies mehr Rechenleistung und wäre um einiges komplexer
void evadeDirection(HardwareSerial &serial, float angle_deg, int duration_ms, int16_t offset) {
  // Neutrale Werte (anpassen an Ihren Sender meist 1500 für Roll,Pitch,Yaw, 1000-2000 für Throttle)
  uint16_t roll = 1500;
  uint16_t pitch = 1500;
  uint16_t yaw = 1500;
  uint16_t throttle = 1400; //Ist von Drohne zu Drohne unterschiedlich

  // offset = stärke des manövers
  // Richtung aufteilen
  roll += (int16_t)(offset * cos(angle_deg * DEG_TO_RAD));
  pitch += (int16_t)(offset * sin(angle_deg * DEG_TO_RAD));
  
  // RC-Override send
  sendRCOverride(serial, roll, pitch, yaw, throttle);
  delay(duration_ms); // Solange beibehalten

  //Bevor Resert kurz gegenlenken
  angle_deg = fmod(angle_deg + 180.0f, 360.0f);
  roll = 1500;
  pitch = 1500;
  roll += (int16_t)(offset * 2 * cos(angle_deg * DEG_TO_RAD));
  pitch += (int16_t)(offset * 2 * sin(angle_deg * DEG_TO_RAD));
  sendRCOverride(serial, roll, pitch, yaw, throttle);
  delay(duration_ms/5);
  // Reset
  sendRCOverride(serial, 1500, 1500, 1500, throttle);
}

void detectApproachingObjects(float* currentAngles, int* currentDistances, int currentLen, float* prevAngles, int* prevDistances, int prevLen,float* resultAngles, int& resultCount) {
    resultCount = 0;
    for (int i = 0; i < currentLen; i++) {
        float angle = currentAngles[i];
        int dist = currentDistances[i];

        // Prüfen, ob aktuelle Distanz unter 500 mm 
        if (dist < 500) {
            bool isCloser = false;
            // Suche in prevAngles alle Winkel im Bereich ±5°
            for (int j = 0; j < prevLen; j++) {
                if (fabs(angle - prevAngles[j]) <= 5.0) {
                    if (dist < prevDistances[j]) {
                        isCloser = true;
                        break;
                    }
                }
            }
            if (isCloser) {
                resultAngles[resultCount++] = angle;
            }
        }
    }
}
//Winkeldifferenz mit berücksichtîgung von 360 -> 0
int angleDiff(int a, int b) {
  int diff = abs(a - b);
  return min(diff, 360 - diff);
}

void calculateMeanDirections(float* angles, int numAngles, float* resultMeans, int& numMeans) {
  // Array sortieren (Bubble Sort)
  for (int i = 0; i < numAngles - 1; i++) {
    for (int j = 0; j < numAngles - i - 1; j++) {
      if (angles[j] > angles[j + 1]) {
        float temp = angles[j];
        angles[j] = angles[j + 1];
        angles[j + 1] = temp;
      }
    }
  }

  // Winkel clustern
  bool* grouped = new bool[numAngles]{0};
  numMeans = 0;

  for (int i = 0; i < numAngles; i++) {
    if (grouped[i]) continue;

    float sum = angles[i];
    int count = 1;
    grouped[i] = true;

    // Vorwärts suchen
    for (int j = i + 1; j < numAngles; j++) {
      if (!grouped[j] && angleDiff(angles[j], angles[j - 1]) <= 5.0f) {
        sum += angles[j];
        count++;
        grouped[j] = true;
      } else {
        break;
      }
    }

    // Rückwärts suchen für 0/360-Überlauf
    if (i == 0 && numAngles > 1) {
      int last = numAngles - 1;
      while (!grouped[last] && angleDiff(angles[0], angles[last]) <= 5.0f) {
        sum += angles[last];
        count++;
        grouped[last] = true;
        last--;
        if (last < 0) break;
      }
    }

    resultMeans[numMeans++] = sum / count;
  }
  delete[] grouped;
}


// Prüft die Checksumme (Summe der ersten 8 Bytes & 0xFF == Byte 9)
bool checkChecksum(uint8_t* buf) {
  uint8_t sum = 0;
  for (int i = 0; i < 8; i++) sum += buf[i];
  return (sum & 0xFF) == buf[8];
}

// Parst ein vollständiges Paket und gibt Werte aus
void parsePacket(uint8_t* buf) {
  uint16_t distance = buf[3] | (buf[4] << 8);  // Distanz in mm
  uint16_t strength = buf[5] | (buf[6] << 8);  // Signalstärke
  uint8_t temperature = buf[7];                // Temperatur laut Wiki

  Serial.print("Distanz: ");
  Serial.print(distance);
  Serial.print(" mm, Stärke: ");
  Serial.print(strength);
  Serial.print(", Temp: ");
  Serial.println(temperature);
}