#include <PZEM004Tv30.h>
#include <TFT_eSPI.h> 
#include <SPI.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>

// pzem pin
#define RX1 16
#define TX1 17
#define RX2 25
#define TX2 26
#define RX0 3 
#define TX0 1

//output/input pin
#define LED_PF    13      
#define BUZZER    19      
#define LED_EXTRA 0   
#define ANALOG_DC_PIN 34 

#define BTN_RESET 5        
#define BTN_TRIP_MANUAL 14 
//encoder pin
#define ENC_CLK  32     
#define ENC_DT   33     
#define ENC_SW   27     

// ini sesuaikan aja lawan kapasitas mcu
#define EEPROM_SIZE 128
#define EEPROM_ADDR_MAGIC 120   
#define MAGIC_NUMBER 127       

//Variabel Grafik
#define MAX_SAMPLES 80      
#define GRAPH_H 85          
#define GRAPH_W 410         
float historyR[MAX_SAMPLES], historyS[MAX_SAMPLES], historyT[MAX_SAMPLES];
int graphIdx = 0;
bool showGraph = false;

PZEM004Tv30 pzemR(Serial1, RX1, TX1);//pembeda biar 3 sensor kd nabrak
PZEM004Tv30 pzemS(Serial2, RX2, TX2);
PZEM004Tv30 pzemT(Serial, RX0, TX0); 
TFT_eSPI tft = TFT_eSPI(); // khusus ini buka library sesuai layar yang dipake

//Variabel Konfigurasi
float limHighV, limLowV, limMaxA, limLowPF; 
float limDelayA = 5.0; 
float vDC = 0.0; 
bool enableProt = true;   
bool enablePFProt = true; 
bool enablePhaseLoss = false; // MENGGANTIKAN ledExtraState
bool manualProtActive = false; 
int activeProtPin = 12; 
int displayMode = 0; 

//Variabel Status
bool inMenu = false, editMode = false; 
bool isTripped = false; 
bool manualTripTriggered = false; 
int menuIndex = 0; 
unsigned long lastUpdate = 0;
bool lastClk;
unsigned long lastButtonPress = 0;
bool blinkState = false;
unsigned long lastBlinkTime = 0;
unsigned long tripStartTime = 0; 
unsigned long overcurrentStart = 0; 
bool isOvercurrent = false;         
bool needUpdateMenu = false; 

// --- JEDA WAKTU PEMULIHAN PHASE LOSS ---
unsigned long systemStartTime = 0;
const unsigned long phaseLossDelayBypass = 4000; // Delay awal nyala 4 detik
bool currentPhaseLossActive = false;             // Status realtime fasa hilang
unsigned long phaseRecoveryStart = 0;            // Tracker waktu saat fasa kembali normal
bool isWaitingForRecovery = false;               // Status penanda sedang dalam masa tenggang jeda pulih
const unsigned long phaseRecoveryDelay = 3000;   // Jeda 3 detik setelah fasa pulih

// --- STATUS PROTEKSI OTOMATIS TEGANGAN RENDAH ---
bool currentUnderVoltageActive = false;          // Status realtime jika ada fasa drop di bawah Volt Min
// -------------------------------------------------

//Variabel Monitoring
float cVR, cVS, cVT, cIR, cIS, cIT, cWR, cWS, cWT, cPFR, cPFS, cPFT, cFreq;
float cWhR, cWhS, cWhT; 
int pzemStep = 0;

#include "frame1.h"

// Daftar semua frame dalam array pointer
const uint16_t* frames[] = { frame1};
const int frameCount = sizeof(frames) / sizeof(frames[0]);

// Ukuran layar
const int imgW = 480;
const int imgH = 320;

// Menampilkan satu frame
void showFrame(const uint16_t* frame) {
  for (int i = 0; i < imgW * imgH; i++) {
    uint16_t c = pgm_read_word(&frame[i]);
    tft.drawPixel(i % imgW, i / imgW, c);
  }
}

void setup() { // konfigurasi in/out/data
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  tft.setCursor(200, 30);
  for (int i = 0; i < frameCount; i++) {
    showFrame(frames[i]);
    delay(3000);
    // kecepatan animasi (ms)
  }
  tft.setRotation(1);
  
  pinMode(12, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(LED_PF, OUTPUT);   
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_EXTRA, OUTPUT); 
  pinMode(ANALOG_DC_PIN, INPUT);
  pinMode(BTN_RESET, INPUT_PULLUP); 
  pinMode(BTN_TRIP_MANUAL, INPUT_PULLUP);
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP); 
  
  digitalWrite(12, HIGH);
  digitalWrite(21, LOW);
  digitalWrite(LED_PF, LOW);   
  digitalWrite(BUZZER, LOW);
  digitalWrite(LED_EXTRA, LOW);

  lastClk = digitalRead(ENC_CLK);
  EEPROM.begin(EEPROM_SIZE);
  loadSettings(); 
  drawUI(); 

  systemStartTime = millis(); 
}

void loop() {
  handleButton(); //intinya ngini buat tombol, kirim perintah data, dan lainnya kd ingat jua
  
  if (showGraph) {
    if (millis() - lastUpdate > 250) {
      readPZEMSequential();
      recordGraphData();
      drawOscilloscope();
      checkProteksi();
      lastUpdate = millis();
    }
  } else if (inMenu) {
    handleEncoder();
    if (needUpdateMenu) {
      updateMenuDisplay();
      needUpdateMenu = false;
    }
    if (millis() - lastUpdate > 200) {
      readPZEMSequential();
      checkProteksi();
      lastUpdate = millis();
    }
  } else {
    if (millis() - lastBlinkTime > 500) {
      blinkState = !blinkState;
      lastBlinkTime = millis();
      vDC = readDCVoltage();
      if (displayMode == 0) updateMonitoring();
      else updateMonitoringL2L();
    }

    if (digitalRead(BTN_TRIP_MANUAL) == LOW && !isTripped) {
      isTripped = true;
      manualTripTriggered = true; 
      tripStartTime = millis();    
      manualProtActive = true; 
    }

    if (digitalRead(BTN_RESET) == LOW && (isTripped || currentPhaseLossActive || currentUnderVoltageActive)) {
      resetProteksi();
    }

    handleBuzzer();

    if (millis() - lastUpdate > 200) {
      readPZEMSequential();
      recordGraphData();
      checkProteksi();
      lastUpdate = millis();
    }
  }
}

// PROTEKSI lawan PZEM

void recordGraphData() { // ini buat grafik arus
  historyR[graphIdx] = isnan(cIR) ? 0 : cIR;
  historyS[graphIdx] = isnan(cIS) ? 0 : cIS;
  historyT[graphIdx] = isnan(cIT) ? 0 : cIT;
  graphIdx = (graphIdx + 1) % MAX_SAMPLES;
}

void readPZEMSequential() {  // untuk meinta data dari sensor Pzem
  pzemStep++;
  if (pzemStep > 3) pzemStep = 1;
  if (pzemStep == 1) { 
    cVR = pzemR.voltage(); cIR = pzemR.current(); cWR = pzemR.power(); 
    cWhR = pzemR.energy(); cPFR = pzemR.pf(); cFreq = pzemR.frequency(); 
  }
  else if (pzemStep == 2) { 
    cVS = pzemS.voltage(); cIS = pzemS.current(); cWS = pzemS.power(); 
    cWhS = pzemS.energy(); cPFS = pzemS.pf(); 
  }
  else if (pzemStep == 3) { 
    cVT = pzemT.voltage(); cIT = pzemT.current(); cWT = pzemT.power(); 
    cWhT = pzemT.energy(); cPFT = pzemT.pf(); 
  }
}

void checkProteksi() {
  bool phaseLossDetected = false; 
  
  // Cek Phase Loss hanya jika melewati bypass delay awal nyala
  if (enablePhaseLoss && (millis() - systemStartTime > phaseLossDelayBypass)) {
      if (isnan(cVR) || cVR < 50.0) phaseLossDetected = true;
      if (isnan(cVS) || cVS < 50.0) phaseLossDetected = true;
      if (isnan(cVT) || cVT < 50.0) phaseLossDetected = true;
  }

  // --- LOGIKA JEDA RECOVERY PHASE LOSS ---
  if (phaseLossDetected) {
      isWaitingForRecovery = false; 
      if (!currentPhaseLossActive) {
          currentPhaseLossActive = true;
          tripStartTime = millis(); 
      }
  } else {
      if (currentPhaseLossActive) {
          if (!isWaitingForRecovery) {
              phaseRecoveryStart = millis(); 
              isWaitingForRecovery = true;
          }
          if (millis() - phaseRecoveryStart >= phaseRecoveryDelay) {
              currentPhaseLossActive = false; 
              isWaitingForRecovery = false;
          }
      }
  }

  if (!enableProt) { 
    digitalWrite(LED_PF, LOW); 
    isOvercurrent = false;
    currentUnderVoltageActive = false;
    // Kontrol relay utama saat proteksi mati total
    bool adaMasalahBiasa = (isTripped || manualProtActive || currentPhaseLossActive);
    digitalWrite(activeProtPin, adaMasalahBiasa ? LOW : HIGH);
    digitalWrite((activeProtPin == 12 ? 21 : 12), LOW);
    return; 
  }

  // --- CEK KONDISI TEGANGAN ---
  bool vHighViolation = false; 
  bool vLowViolation = false; 

  // Pengecekan Over-Voltage (Volt Max)
  if (!isnan(cVR) && cVR > 50.0 && cVR > limHighV) vHighViolation = true;
  if (!isnan(cVS) && cVS > 50.0 && cVS > limHighV) vHighViolation = true;
  if (!isnan(cVT) && cVT > 50.0 && cVT > limHighV) vHighViolation = true;

  // Pengecekan Under-Voltage (Volt Min)
  if (!isnan(cVR) && cVR > 50.0 && cVR < limLowV) vLowViolation = true;
  if (!isnan(cVS) && cVS > 50.0 && cVS < limLowV) vLowViolation = true;
  if (!isnan(cVT) && cVT > 50.0 && cVT < limLowV) vLowViolation = true;

  // --- LOGIKA AUTO-RECOVERY UNTUK TEGANGAN RENDAH (UNDER-VOLTAGE) ---
  if (millis() - systemStartTime > phaseLossDelayBypass) {
    if (vLowViolation) {
      if (!currentUnderVoltageActive) {
        currentUnderVoltageActive = true;
        tripStartTime = millis(); // Set timer awal buzzer berbunyi panjang
      }
    } else {
      // Jika semua tegangan fasa sudah berada di atas batas minimal, otomatis normal kembali
      currentUnderVoltageActive = false;
    }
  }

  // --- LOGIKA LOCK TRIP UNTUK OVER-VOLTAGE (TEGANGAN TINGGI) ---
  if (vHighViolation && !isTripped && (millis() - systemStartTime > phaseLossDelayBypass)) {
    isTripped = true;
    manualTripTriggered = false; 
    tripStartTime = millis();    
    manualProtActive = true; 
  }

  // Pengecekan Overcurrent (Arus Berlebih)
  bool currentNowOver = false; 
  if (!isnan(cIR) && cIR > 0.01 && cIR > limMaxA) currentNowOver = true;
  if (!isnan(cIS) && cIS > 0.01 && cIS > limMaxA) currentNowOver = true;
  if (!isnan(cIT) && cIT > 0.01 && cIT > limMaxA) currentNowOver = true;

  if (currentNowOver && !isTripped) {
    if (!isOvercurrent) {
      overcurrentStart = millis(); 
      isOvercurrent = true;
    } else if (millis() - overcurrentStart > (limDelayA * 1000)) { 
        isTripped = true;
        manualTripTriggered = false;
        tripStartTime = millis();
        manualProtActive = true;
    }
  } else {
    isOvercurrent = false; 
  }

  // --- KONTROL RELAY UTAMA ---
  // Relay akan OFF (LOW) jika: Trip Terkunci, Phase Loss Aktif, ATAU Tegangan Rendah Aktif
  bool adaMasalah = (isTripped || manualProtActive || currentPhaseLossActive || currentUnderVoltageActive);
  digitalWrite(activeProtPin, adaMasalah ? LOW : HIGH);
  digitalWrite((activeProtPin == 12 ? 21 : 12), LOW); 

  // Pengecekan Power Factor
  bool pfLow = false; 
  if (enablePFProt) {
     if ((!isnan(cPFR) && cPFR > 0.01 && cPFR < limLowPF) || 
         (!isnan(cPFS) && cPFS > 0.01 && cPFS < limLowPF) || 
         (!isnan(cPFT) && cPFT > 0.01 && cPFT < limLowPF)) pfLow = true;
  }
  digitalWrite(LED_PF, pfLow ? HIGH : LOW); 
}

void handleEncoder() { 
  bool currentClk = digitalRead(ENC_CLK);
  if (currentClk != lastClk) {
    if (currentClk == LOW) {
      int direction = (digitalRead(ENC_DT) != currentClk) ? 1 : -1;
      if (!editMode) {
        menuIndex += direction;
        if (menuIndex > 13) menuIndex = 0; 
        if (menuIndex < 0) menuIndex = 13;
      } else {
        switch(menuIndex) {
          case 0: limHighV += direction; break;
          case 1: limLowV  += direction; break;
          case 2: limMaxA  += (direction * 0.1); break;
          case 3: enableProt = !enableProt; break;
          case 4: limLowPF += (direction * 0.01); break;
          case 5: enablePFProt = !enablePFProt; break;
          case 6: manualProtActive = !manualProtActive; if (!manualProtActive && isTripped) isTripped = false; break;
          case 7: activeProtPin = (activeProtPin == 12) ? 21 : 12; break;
          case 8: enablePhaseLoss = !enablePhaseLoss; break; 
          case 10: limDelayA += direction; if(limDelayA < 0) limDelayA = 0; break;
          case 11: displayMode = (displayMode == 0) ? 1 : 0; break;
        }
        limMaxA = constrain(limMaxA, 0.0, 100.0);
        limLowPF = constrain(limLowPF, 0.0, 1.0);
      }
      needUpdateMenu = true; 
    }
  }
  lastClk = currentClk;
}

void handleButton() {
  if (digitalRead(ENC_SW) == LOW) {
    if (millis() - lastButtonPress > 400) { 
      if (showGraph) {
        showGraph = false;
        tft.fillScreen(TFT_BLACK);
        drawUI();
      } else if (!inMenu) {
        inMenu = true;
        tft.fillScreen(TFT_BLACK);
        drawMenuBase();
      } else {
        if (menuIndex == 9) { 
          inMenu = false;
          showGraph = true;
          tft.fillScreen(TFT_BLACK);
        } else if (menuIndex == 12) { 
           resetProteksi();
           inMenu = false;
           tft.fillScreen(TFT_BLACK);
           drawUI();
        } else if (menuIndex == 13) { 
          saveSettings(); 
          inMenu = false;
          tft.fillScreen(TFT_BLACK);
          drawUI();
        } else {
          editMode = !editMode;
          needUpdateMenu = true;
        }
      }
      lastButtonPress = millis();
    }
  }
}

void updateMenuDisplay() {
  tft.setTextSize(2);
  String labels[] = { 
    "Volt Max   ", "Volt Min   ", "Amp Max    ", "V/A Protec ", 
    "PF min set ", "PF Protec  ", "Cut Off    ", "relay in-ex ", "Phas prot lost", 
    "Tampilan Monitoring Grafik Arus", "I Protc over", "Tampilan1/2", "Reset Protec", "Simpan keluar"
  };
  for (int i = 0; i < 14; i++) {
    uint16_t textColor = (i == menuIndex) ? (editMode ? TFT_RED : TFT_GREEN) : TFT_WHITE;
    tft.setTextColor(textColor, TFT_BLACK);
    int yPos = 25 + (i * 21);
    tft.drawString((i == menuIndex ? ">" : " "), 5, yPos);
    tft.setCursor(30, yPos);
    tft.print(labels[i]);
    if (i != 9 && i != 12 && i != 13) tft.print(": ");

    switch(i) { 
      case 0: tft.print(limHighV, 1); break;
      case 1: tft.print(limLowV, 1); break;
      case 2: tft.print(limMaxA, 2); break;
      case 3: tft.print(enableProt ? "Protc V/A on" : "Protc V/A OFF"); break;
      case 4: tft.print(limLowPF, 2); break;
      case 5: tft.print(enablePFProt ? "Protc PF ON" : "Protc PF OFF"); break;
      case 6: tft.print(manualProtActive ? "ON" : "OFF"); break;
      case 7: tft.print(String(activeProtPin) + ": 12inter/21exter "); break;
      case 8: tft.print(enablePhaseLoss ? "Aktif" : "OFF"); break;
      case 10: tft.print(limDelayA, 0); break;
      case 11: tft.print(displayMode == 0 ? "Fasa-Netral" : "Fasa-Fasa"); break;
    }
    tft.print("      "); 
  }
}
void updateMonitoring() { 
  updateRow("R", cVR, cIR, cWR, cWhR, cPFR, 25, TFT_RED);
  updateRow("S", cVS, cIS, cWS, cWhS, cPFS, 110, TFT_YELLOW);
  updateRow("T", cVT, cIT, cWT, cWhT, cPFT, 195, TFT_GREEN);
  
  float Fequensii = pzemR.frequency();
  tft.setTextSize(4);
  tft.setTextColor(TFT_PINK, TFT_BLACK);
  tft.drawString("F: " + String(Fequensii, 1) + " Hz", 10, 287);
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("DC: " + String(vDC, 1) + " V  ", 340, 280);
  tft.setCursor(340, 300);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("PhLos: ");
  tft.setTextColor(enablePhaseLoss ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.print(enablePhaseLoss ? "ON " : "OFF");
}
void updateRow(String phase, float v, float i, float w, float wh, float pf, int y, uint16_t phaseColor) {
  tft.setTextSize(5);
  tft.setTextColor(phaseColor, TFT_BLACK);
  tft.drawString(phase, 5, y + 15); 
  
  bool isLoss = (isnan(v) || v < 50.0);
  if (enablePhaseLoss && isLoss) {
      tft.setTextColor(blinkState ? TFT_RED : TFT_BLACK, TFT_BLACK);
  } else {
      tft.setTextColor(TFT_CYAN, TFT_BLACK);
  }

  if (isnan(v)) tft.drawString("000.0V", 45, y); 
  else { char bufV[12]; dtostrf(v, 5, 1, bufV); tft.drawString(String(bufV) + "V", 45, y); }

  tft.setTextColor((isTripped || currentPhaseLossActive || currentUnderVoltageActive) ? TFT_RED : TFT_MAGENTA, TFT_BLACK);
  if (isnan(i)) tft.drawString("00.00A", 45, y + 40);
  else { char bufI[12]; dtostrf(i, 5, 2, bufI); tft.drawString(String(bufI) + "A", 45, y + 40); }
  
  tft.setTextSize(3);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("P : " + (isnan(w) ? "0.0" : String(w, 1)) + " W", 260, y);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.drawString("E : " + (isnan(wh) ? "0" : String(wh, 2)) + " Wh", 260, y + 25);
  
  uint16_t pfTextColor = (enablePFProt && !isnan(pf) && pf < limLowPF) ? (blinkState ? TFT_RED : TFT_WHITE) : TFT_WHITE;
  tft.setTextColor(pfTextColor, TFT_BLACK);
  tft.drawString("PF: " + (isnan(pf) ? "0.00" : String(pf, 2)), 260, y + 50);
}
void updateMonitoringL2L() { 
  float vRS = sqrt(sq(cVR) + sqrt(cVS) + (cVR * cVS));
  float vST = sqrt(sq(cVS) + sqrt(cVT) + (cVS * cVT));
  float vTR = sqrt(sq(cVT) + sqrt(cVR) + (cVT * cVR));
  updateRowL2L("RS", vRS, cIR, cWR, 25, TFT_RED);
  updateRowL2L("ST", vST, cIS, cWS, 110, TFT_YELLOW);
  updateRowL2L("TR", vTR, cIT, cWT, 195, TFT_GREEN);
  tft.setTextSize(2);
  tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
  tft.drawString("V antar Fasa", 10, 285);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("DC: " + String(vDC, 1) + "V", 340, 285);
}
void updateRowL2L(String label, float v, float i, float w, int y, uint16_t color) {
  tft.setTextSize(4); tft.setTextColor(color, TFT_BLACK);
  tft.drawString(label, 5, y + 20);
  tft.setTextSize(7); tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(65, y + 10);
  tft.print(isnan(v) ? "000" : String((int)v)); tft.print("V");
  tft.setTextSize(5); tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
  tft.setCursor(240, y);
  tft.print("I:"); tft.print(isnan(i) ? 0.0 : i, 2); tft.print("A");
  tft.setCursor(240, y + 45);
  tft.setTextSize(4); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("W:"); tft.print(isnan(w) ? 0.0 : w, 0); tft.print("W");
}
void drawOscilloscope() { 
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Garage Project - Grafik Mode", 10, 5);
  renderGraph("R", historyR, 18, TFT_RED);
  renderGraph("S", historyS, 118, TFT_YELLOW);
  renderGraph("T", historyT, 218, TFT_GREEN);

  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(400, 8);
    tft.print("A:"); tft.print(cIR);
  tft.setCursor(400, 108);
    tft.print("A:"); tft.print(cIS);
  tft.setCursor(400, 208);
    tft.print("A:"); tft.print(cIT);
}
void renderGraph(String label, float data[], int yPos, uint16_t color) {
  int xStart = 35;
  tft.setTextSize(3); tft.setTextColor(color, TFT_BLACK);
  tft.drawString(label, 5, yPos + (GRAPH_H / 2) - 10);
  tft.drawRect(xStart, yPos, GRAPH_W, GRAPH_H, TFT_DARKGREY);
  tft.fillRect(xStart + 1, yPos + 1, GRAPH_W - 2, GRAPH_H - 2, TFT_BLACK);
  for (int i = 0; i < MAX_SAMPLES - 1; i++) {
    int idx1 = (graphIdx + i) % MAX_SAMPLES;
    int idx2 = (graphIdx + i + 1) % MAX_SAMPLES;
    int y1 = map(data[idx1] * 100, 0, limMaxA * 100, GRAPH_H - 4, 0);
    int y2 = map(data[idx2] * 100, 0, limMaxA * 100, GRAPH_H - 4, 0);
    y1 = constrain(y1, 0, GRAPH_H - 4); y2 = constrain(y2, 0, GRAPH_H - 4);
    tft.drawLine(xStart + 1 + (i * 5), yPos + 2 + y1, xStart + 1 + ((i + 1) * 5), yPos + 2 + y2, color);
  }
}
void drawUI() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.drawString("Hidden Garage", 5, 0);
  tft.drawFastHLine(0, 21, 480, TFT_WHITE);
  tft.drawFastHLine(0, 106, 480, TFT_WHITE);
  tft.drawFastHLine(0, 190, 480, TFT_WHITE);
  tft.drawFastHLine(0, 277, 480, TFT_WHITE);
  tft.setCursor(240, 0); tft.setTextColor(TFT_PINK, TFT_BLACK);
  tft.print("C:"); tft.print(limMaxA, 2); tft.print("A | PF:"); tft.print(limLowPF, 2);
}
void drawMenuBase() {
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(2);
  tft.drawString("Menu setingan", 10, 3);
  tft.drawFastHLine(0, 20, 480, TFT_WHITE);
  tft.setCursor(200, 3);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print("r"); tft.print(cVR);
  tft.setCursor(300, 3);
    tft.print("s"); tft.print(cVS);
  tft.setCursor(400, 3);
    tft.print("t"); tft.print(cVT);
  updateMenuDisplay();
}
void loadSettings() { 
  if (EEPROM.read(EEPROM_ADDR_MAGIC) != MAGIC_NUMBER) {
    limHighV = 245.0; limLowV = 175.0; limMaxA = 10.0; limLowPF = 0.85; limDelayA = 5.0;
    enableProt = true; enablePFProt = true; manualProtActive = false; 
    enablePhaseLoss = false; activeProtPin = 12;
    saveSettings(); 
  } else {
    EEPROM.get(0, limHighV); EEPROM.get(10, limLowV); EEPROM.get(20, limMaxA);
    EEPROM.get(30, limLowPF); EEPROM.get(40, enableProt); EEPROM.get(42, enablePFProt);
    EEPROM.get(44, manualProtActive); EEPROM.get(46, enablePhaseLoss); EEPROM.get(60, activeProtPin); 
    EEPROM.get(70, limDelayA);
    if (activeProtPin != 12 && activeProtPin != 21) activeProtPin = 12;
    if (isnan(limDelayA)) limDelayA = 5.0;
  }
}
void saveSettings() { 
  EEPROM.put(0, limHighV); EEPROM.put(10, limLowV); EEPROM.put(20, limMaxA);
  EEPROM.put(30, limLowPF); EEPROM.put(40, enableProt); EEPROM.put(42, enablePFProt);
  EEPROM.put(44, manualProtActive); EEPROM.put(46, enablePhaseLoss); EEPROM.put(60, activeProtPin); 
  EEPROM.put(70, limDelayA);
  EEPROM.write(EEPROM_ADDR_MAGIC, MAGIC_NUMBER); EEPROM.commit(); 
}
float readDCVoltage() { 
  int raw = analogRead(ANALOG_DC_PIN);
  return (raw / 4095.0) * 3.3 * 11.0; 
}
void resetProteksi() { 
  isTripped = false; manualTripTriggered = false; manualProtActive = false; isOvercurrent = false;
  currentPhaseLossActive = false; isWaitingForRecovery = false;
  currentUnderVoltageActive = false;
  
  digitalWrite(activeProtPin, HIGH); 
  digitalWrite((activeProtPin == 12 ? 21 : 12), LOW); 
  digitalWrite(BUZZER, LOW);
  systemStartTime = millis(); 
}
void handleBuzzer() { 
  // Kondisi realtime Phase Loss ATAU Under-Voltage aktif & tidak ada proteksi lock lain
  if ((currentPhaseLossActive || currentUnderVoltageActive) && !isTripped) {
    unsigned long duration = millis() - tripStartTime;
    if (duration < 2000) digitalWrite(BUZZER, HIGH); 
    else digitalWrite(BUZZER, blinkState ? HIGH : LOW); 
    return; 
  }

  if (!isTripped) { digitalWrite(BUZZER, LOW); return; }
  if (manualTripTriggered) digitalWrite(BUZZER, blinkState ? HIGH : LOW);
  else {
    unsigned long duration = millis() - tripStartTime;
    if (duration < 2000) digitalWrite(BUZZER, HIGH);
    else digitalWrite(BUZZER, blinkState ? HIGH : LOW);
  }
}