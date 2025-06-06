# FPV-Sense

![Screenshot 2025-06-06 164422](https://github.com/user-attachments/assets/b2163677-7c08-4599-b3ef-6f799d8c01f2)


FPV-Sense ist ein Forschungs- und Entwicklungsprojekt aus dem Bachelorstudium Digital Ideation (Fokus Informatik) an der Hochschule Luzern.  
Das Ziel ist die Entwicklung eines Sensorsystems zur Verbesserung des Einstiegs von neuen FPV-Piloten. Dabei fokusiere ich mich vorallem auf den Sprung vom Simulator zum ersten Flug, in der FPV-Community Maidenflug gennant.
Es gibt viele Hürden beim Einstieg in das FPV Hobby, mit einem System was neue Piloten bei Fliegen eingreiffen kann mächte ich diese minimieren.
Die Lösung kombiniert zuverlässige Hinderniserkennung per Lidar mit direkter Flugsteuerung über den Betaflight-Controller und ist für experimentelle, akademische Zwecke konzipiert.

---

## Inhalt

- [Über das Projekt](#über-das-projekt)
- [Materialliste](#materialliste)
- [Software & Bibliotheken](#software--bibliotheken)
- [Schritt-für-Schritt-Anleitung](#schritt-für-schritt-anleitung)
- [Demo-Video](#demo-video)
- [Download](#download)
- [Lizenz](#lizenz)
- [Kontakt](#kontakt)

---

## Über das Projekt

FPV-Sense erweitert klassische FPV-Drohnen um autonome Ausweichmanöver und Sensordatenverarbeitung. 
Ziel ist es, sowohl die Flugsicherheit als auch die Forschung im Bereich „Human-Drone Interaction“ zu fördern.
Das System erkennt Hindernisse mit Lidar, berechnet Ausweichmanöver und sendet Steuerbefehle in Echtzeit an den Betaflight-Flightcontroller. 
Die Sensorik besteht in diesem Projekt aus einem D500 Waveshare Lidar Developerkit. Für die Verarbeitung der Sensordaten und der Berechnung der Ausweichmanöver wird ein ESP32 Devkit benutzt.
Das Projekt ist so konzipiert, dass es mit den geläufigsten FPV-Drohnen kompatible ist und die Komponenten des systems nicht deterministisch verbaut werden. So können diese bei Bedarf auch zu einem späteren Zeitpunkt für andere Aufgaben genutzt werden.


---

## Materialliste

| Komponente                | Beschreibung / Modell                  |
|---------------------------|----------------------------------------|
| FPV-Drohne                | Hier funktioniern fasst alle Modelle wichtig ist genügend platz für ein Go-Pro Mount |
| Flightcontroller          | Als Fallbeispiel nutze ich einen F722 Pro4 von Skystar. Voraussetzung ist eine zusätzliche nicht genutzte UART Schnittstelle |
| Lidar-Sensor              | Ich nutze den D500 Waveshare Lidar. Dieser beruht auf dem STL-19P DTOF Kern. |
| ESP32 Modul               | ESP32 Development Board / ESP32-WROOM-32D|
| Stromversorgung           | Den Strom bezieht der Baukasten über den 5V anschluss des Flightcontrollers |
| Kabelung                  | Diverse Kabel, Empfehlenswert: Steckverbindung für einfaches montieren |
| Gehäuse                   | Das gehäuse ist aus TPU-HF von Bambulab gedruckt. Ähnliche TPU filamente funktionieren hier auch. Die Stl datei findest du unter [Schaltplan & Hardware-Dateien](./hardware/) |

---

## Software & Bibliotheken

- **Betaflight** (https://betaflight.com/)
- **Arduino IDE** (https://www.arduino.cc/en/software)
  - **ESP32 Board-Package für Arduino (by Espressif Systems**  
  - **Benötigte Bibliotheken** (über den Bibliotheksverwalter installierbar):
    - `HardwareSerial` (Standard)
    - `Esp32 BLE Arduino (by Neil Kolban)` (zusätzlich)


---

## Schritt-für-Schritt-Anleitung

https://www.youtube.com/watch?v=ff37cJdk86Q

1. **Hardware testen / drucken **
    - D500 Lidar testen. Dazu kann das Waveshare Programm verwendet werden eine genau Anleitung findest du Hier https://www.waveshare.com/wiki/D500_LiDAR_Kit
    - Das .stl File aus [Schaltplan & Hardware-Dateien](./hardware/) herunterladen und mit TPU drucken. Dies dauert ungefähr 2 Stunden. 
    
2. **Software flashen**
    - Arduino-IDE einrichten und ESP32-Projekt aus dem Ordner `/software/` öffnen
    - Bibliotheken installieren (siehe oben)
    - Sketch auf ESP32 flashen

3. **Betaflight konfigurieren**
    - Flightcontroller an PC anschließen
    - Im Betaflight Configurator unter „Ports“:
        - UART für Lidar (Sensor) und UART für ESP32 (MSP) aktivieren
    - MSP auf dem entsprechenden UART aktivieren
    - Speichern und FC neu starten

4. **Bausatz zusammenbauen**
    - 3D Druck von allen Stützen befreien.
    - Lidar, ESP32 und Flightcontroller gemäß [Schaltplan](#) verbinden
    - Falls das ESP und der Lidar keinen Strom bekommen kann dies daran liegen, dass der 5V Port am FLightcontroller nur mit angeschlossenem LIPO Strom erhält

5. **System testen**
    - Drohne ohne Propeller starten
    - Lidar-Ausgabe und Ausweichmanöver am Serial Monitor oder via Betaflight werten prüfen

6. **Flugtests**
    - Ich habe zum Zeitpunkt der Abgabe der Projektdokumentation noch keinen Flugtest druchführen können. Zum Momentanen Stand kann ich bezüglich Sicherheit und Zuverlässigkeit keine Garantien geben. Sie sind selbst für Schäden haftent. Es wird keine Haftung für die Software / Hardware übernommen. 
    - [Sicherheitsregeln beachten!](https://www.bazl.admin.ch/bazl/de/home/drohnen/open1.html)

---

## Download

- [Firmware und Beispielcode (ESP32)](./software/)
- [Schaltplan & Hardware-Dateien](./hardware/)
- [Bachelorarbeit als PDF](./docs/Bachelorarbeit_FPVSense.pdf)

---

## Kontakt

**Autor:**  
Hannes Salzmann
E-Mail: hannes.salzmann@stud.hslu.ch  
[GitHub-Profil](https://github.com/Zytnah)

---

*Hinweis: Dieses Projekt entstand im Rahmen der Bachelorarbeit Digital Ideation an der HSLU und dient Forschungs- sowie Lehrzwecken.*
