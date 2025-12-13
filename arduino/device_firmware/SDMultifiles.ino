/********************************************************************
* XIAO nRF52840 (Sense) — Dual Ladder:
*  - Ladder A (A0): Button 1 (NEXT), 2, 3, 4
*  - Ladder B (A1): Button 5, 6, 7 (ERROR), 8 (POWER)
*
* BLE UART + SD logger (batching, file rotation + write verification)
*
* CSV Row (1x9):
*  [c1,c2,c3,c4,c5,timestamp_ms,bt_sent,sd_sent,token]
*     (Button2..Button6 map to c1..c5; Button1 = NEXT; 7 = ERROR; 8 = POWER)
*
* Flow (single-press NEXT):
*  - First NEXT → arms new row (zeros + timestamp).
*  - While armed, Buttons 2..6 → set c1..c5 = 1 once per row.
*  - ERROR (Button 7) while armed:
*       → sets c1..c5 to 0 for that row, saves it, and immediately
*         arms a new row (new timestamp) ready for collection.
*  - ERROR (Button 7) with no row armed:
*       → starts a new zeroed row (like NEXT starting a row).
*  - NEXT again → saves current row AND immediately arms a new row.
*  - POWER (Button 8) short-press:
*       → toggles systemEnabled:
*           - if systemEnabled == false, ignore Buttons 1..7 and pause
*             BLE/SD work until POWER is pressed again.
*           - BLE advertising only when systemEnabled == true.
*  - POWER (Button 8) long-press (>= 2 seconds):
*       → switches to a NEW CSV file on the SD card:
*           - first:  DATALOG.CSV
*           - then:   DLG0001.CSV, DLG0002.CSV, ...
*         no files are deleted; in-memory buffer is cleared.
*  - BLE sends CSV rows only (no button messages, unless enabled).
*  - SD appends every SD_BATCH_SIZE unsent rows (with header).
*  - After write, file is read back and lines verified.
*  - Row is removed once bt_sent==1 && sd_sent==1.
*
*  SECURITY NOTE:
*  - A static SECRET_TOKEN is appended as the last CSV column.
*  - The receiving app should parse the last field and only trust rows
*    where token == SECRET_TOKEN.
********************************************************************/

#include <ArduinoBLE.h>
#include <SPI.h>
#include <SD.h>

// -------------------- SD config -----------------
const int chipSelect = 7;                  // board-specific CS pin
const size_t SD_BATCH_SIZE = 1;            // write when >= this many pending

// Multi-file logging
uint16_t currentFileIndex = 0;                // 0 = DATALOG.CSV, 1+ = DLG0001.CSV, ...
char currentFilename[20] = "DATALOG.CSV";     // active log file

// -------------------- Ladder inputs -------------
const int DATA_PIN  = A0;  // Ladder A: Buttons 1..4
const int DATA_PIN1 = A1;  // Ladder B: Buttons 5..8
const float VREF = 3.3;
const uint16_t DEBOUNCE_MS = 60;

// Thresholds for Ladder A (A0)
const float THRESH_B1 = 2.5;  // Button 1 (NEXT)
const float THRESH_B2 = 1.9;  // Button 2 -> c1
const float THRESH_B3 = 1.5;  // Button 3 -> c2
const float THRESH_B4 = 1.0;  // Button 4 -> c3
const float RELEASE_V = 0.7;  // release

// Thresholds for Ladder B (A1)
const float THRESH_B5 = 2.5;  // Button 5 -> c4
const float THRESH_B6 = 1.9;  // Button 6 -> c5
const float THRESH_B7 = 1.4;  // Button 7 (ERROR)
const float THRESH_B8 = 1.0;  // Button 8 (POWER)

// -------------------- BLE UART-like Service ----
BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic txChar("6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
                         BLERead | BLENotify, 100);
BLECharacteristic rxChar("6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
                         BLEWrite, 100);

// Toggle: send human-readable button messages over BLE?
// false = CSV rows only over BLE; button logs only to Serial.
const bool BLE_SEND_BUTTON_MESSAGES = false;

// Track BLE state
bool bleOk = false;
bool bleAdvertising = false;

// -------------------- Ladder state --------------
int activeButtonA = 0; // last pressed on Ladder A (A0)
int activeButtonB = 0; // last pressed on Ladder B (A1)
uint32_t lastChangeMsA = 0;
uint32_t lastChangeMsB = 0;

// -------------------- Power state ---------------
bool systemEnabled = true;  // true = run as normal, false = paused waiting for POWER

// Long-press timing for POWER (Button 8)
const uint32_t POWER_HOLD_MS = 2000;  // 2s hold = new CSV file
uint32_t powerPressStartMs = 0;

// -------------------- Security token ------------
const uint32_t SECRET_TOKEN = 834729;  // <-- change this if you want a different token

// -------------------- Row buffer + logging -----
struct Row {
  uint8_t  c1, c2, c3, c4, c5;  // columns 1..5 (Button 2..6)
  uint32_t ts;                  // column 6
  uint8_t  btSent;              // column 7
  uint8_t  sdSent;              // column 8
};

const size_t MAX_ROWS = 256;
Row rows[MAX_ROWS];
size_t head = 0; // first valid row
size_t tail = 0; // one past last valid row

bool rowArmed = false;
Row current;

// -------------------- Helpers ------------------
uint16_t readCodeA() { return analogRead(DATA_PIN); }
uint16_t readCodeB() { return analogRead(DATA_PIN1); }
float codeToVoltage(uint16_t code) { return (float)code * VREF / 4095.0; }

void sendMessage(const String &msg) {
  if (!BLE_SEND_BUTTON_MESSAGES) {
    Serial.print("[LOG] "); Serial.println(msg);
    return;
  }
  txChar.writeValue(msg.c_str());
  Serial.print("[BLE] "); Serial.println(msg);
}

bool isBufferEmpty() { return head == tail; }
bool isBufferFull()  { return ((tail + 1) % MAX_ROWS) == head; }
size_t bufferCount() { return (tail + MAX_ROWS - head) % MAX_ROWS; }

void pushRow(const Row &r) {
  if (isBufferFull()) {
    Serial.println("[BUF] Full → dropping oldest row");
    head = (head + 1) % MAX_ROWS;
  }
  rows[tail] = r;
  tail = (tail + 1) % MAX_ROWS;
}

void popDoneRows() {
  bool any = false;
  while (!isBufferEmpty()) {
    Row &r = rows[head];
    if (r.btSent == 1 && r.sdSent == 1) {
      head = (head + 1) % MAX_ROWS;
      any = true;
    } else break;
  }
  if (any) Serial.println("[BUF] Deleted rows that are sent to BLE & SD");
}

// Now includes the SECRET_TOKEN as the last CSV column
String rowToCSV(const Row &r) {
  // c1,c2,c3,c4,c5,ts,btSent,sdSent,token
  String s;
  s.reserve(64);
  s += String(r.c1) + ",";
  s += String(r.c2) + ",";
  s += String(r.c3) + ",";
  s += String(r.c4) + ",";
  s += String(r.c5) + ",";
  s += String(r.ts) + ",";
  s += String(r.btSent) + ",";
  s += String(r.sdSent) + ",";
  s += String(SECRET_TOKEN);
  return s;
}

String rowPretty(const Row &r) {
  // Debug print (no token here, just internal state)
  return String("[") + r.c1 + "," + r.c2 + "," + r.c3 + "," + r.c4 + "," + r.c5 + "," +
         r.ts + "," + r.btSent + "," + r.sdSent + "]";
}

void initCurrentRow() {
  current.c1 = current.c2 = current.c3 = current.c4 = current.c5 = 0;
  current.btSent = current.sdSent = 0;
  current.ts = millis(); // timestamp set at NEXT or ERROR start
}

void armNewRow() {
  initCurrentRow();
  rowArmed = true;
  Serial.print("[ROW] Armed new row: ");
  Serial.println(rowPretty(current));
}

void finalizeCurrentRow() {
  pushRow(current);
  rowArmed = false;
  Serial.print("[ROW] Saved row: ");
  Serial.println(rowPretty(current));
}

// ONE-PRESS behavior: if a row is armed, save it and immediately arm a new one
void onNextPressed() {
  if (!rowArmed) {
    Serial.println("[NEXT] Start new row (zeros + timestamp)");
    armNewRow();
  } else {
    Serial.println("[NEXT] Save current row and start next");
    finalizeCurrentRow();   // pushes current to buffer
    armNewRow();            // fresh row with new timestamp
  }
}

// -------------------- SD helpers ----------------

// Generate filename based on index:
//   idx == 0  -> "DATALOG.CSV"
//   idx >= 1  -> "DLG0001.CSV", "DLG0002.CSV", ...
void makeFilename(uint16_t idx) {
  if (idx == 0) {
    strcpy(currentFilename, "DATALOG.CSV");
  } else {
    snprintf(currentFilename, sizeof(currentFilename), "DLG%04u.CSV", idx);
  }
}

// Select active log file, ensure header exists (create if missing)
void selectLogFile(uint16_t idx) {
  currentFileIndex = idx;
  makeFilename(currentFileIndex);

  Serial.print("[SD] Active log file: ");
  Serial.println(currentFilename);

  if (!SD.exists(currentFilename)) {
    File f = SD.open(currentFilename, FILE_WRITE);
    if (f) {
      // Header now includes token column
      f.println("c1,c2,c3,c4,c5,timestamp_ms,bt_sent,sd_sent,token");
      f.close();
      Serial.println("[SD] Wrote CSV header to new file");
    } else {
      Serial.println("[SD] Failed to create file for header");
    }
  } else {
    Serial.println("[SD] File exists; will append");
  }
}

size_t countPendingSD() {
  size_t cnt = 0;
  for (size_t i = head; i != tail; i = (i + 1) % MAX_ROWS) {
    if (rows[i].sdSent == 0) cnt++;
  }
  return cnt;
}

// Read entire file and verify the presence of specific lines.
// `expected` is the number of newly-written unsent rows we attempted.
void sdReadBackAndVerify(size_t expected) {
  File f = SD.open(currentFilename, FILE_READ);
  if (!f) {
    Serial.println("[SD] Verify open failed");
    return;
  }
  // Collect file content into a string (small logs: OK)
  String fileContent;
  while (f.available()) {
    fileContent += (char)f.read();
  }
  f.close();

  size_t verified = 0;
  size_t checked = 0;
  if (expected > bufferCount()) expected = bufferCount();
  int idx = (int)tail - 1;
  for (; expected > 0 && checked < 200; idx--, checked++) {
    if (idx < 0) idx += MAX_ROWS;
    Row &r = rows[(size_t)idx];
    String needle = rowToCSV(r);
    if (fileContent.indexOf(needle) >= 0) {
      verified++;
    } else {
      Serial.print("[SD] Verify MISSING: ");
      Serial.println(needle);
    }
    expected--;
  }
  Serial.print("[SD] Verify OK count: ");
  Serial.println(verified);
}

void sdWriteBatch(size_t maxBatch) {
  // Determine how many to write (sdSent==0)
  size_t toWrite = 0;
  for (size_t i = head; i != tail; i = (i + 1) % MAX_ROWS) {
    if (rows[i].sdSent == 0) {
      toWrite++;
      if (toWrite >= maxBatch) break;
    }
  }
  if (toWrite == 0) return;

  File dataFile = SD.open(currentFilename, FILE_WRITE);
  if (!dataFile) {
    Serial.println("[SD] Open (FILE_WRITE) failed");
    return;
  }

  size_t written = 0;
  for (size_t i = head; i != tail && written < toWrite; i = (i + 1) % MAX_ROWS) {
    if (rows[i].sdSent == 0) {
      String line = rowToCSV(rows[i]);
      dataFile.println(line);
      rows[i].sdSent = 1; // optimistic mark
      written++;
      Serial.print("[SD] Wrote: ");
      Serial.println(line);
    }
  }
  dataFile.flush();
  dataFile.close();
  Serial.print("[SD] Appended rows: ");
  Serial.println(written);

  // Read back the file to verify we actually see those lines
  sdReadBackAndVerify(written);
}

// -------------------- BLE send unsent CSV rows --
void bleSendUnsent() {
  size_t sent = 0;
  for (size_t i = head; i != tail; i = (i + 1) % MAX_ROWS) {
    if (rows[i].btSent == 0) {
      String payload = rowToCSV(rows[i]);
      if (txChar.writeValue(payload.c_str())) {
        rows[i].btSent = 1;
        sent++;
        Serial.print("[BLE CSV] Sent: ");
        Serial.println(payload);
      } else {
        Serial.println("[BLE] writeValue failed; will retry");
        break;
      }
      delay(5);
    }
  }
  if (sent) {
    Serial.print("[BLE] Rows sent this pass: ");
    Serial.println(sent);
  }
}

// -------------------- Ladder helpers ------------
int decodeLadderA(float v) {
  // Return button ID for Ladder A (1..4) or 0 if none
  if (v > THRESH_B1) return 1;
  else if (v > THRESH_B2) return 2;
  else if (v > THRESH_B3) return 3;
  else if (v > THRESH_B4) return 4;
  else if (v < RELEASE_V) return 0;
  else return -1; // in-between noise region
}

int decodeLadderB(float v) {
  // Return button ID for Ladder B (5..8) or 0 if none
  if (v > THRESH_B5) return 5;
  else if (v > THRESH_B6) return 6;
  else if (v > THRESH_B7) return 7;
  else if (v > THRESH_B8) return 8;
  else if (v < RELEASE_V) return 0;
  else return -1; // in-between noise region
}

void onPowerShortPress() {
  // POWER toggle (original behavior)
  systemEnabled = !systemEnabled;
  Serial.print("[POWER] System ");
  Serial.println(systemEnabled ? "ON" : "OFF");

  // Control BLE advertising according to power state
  if (bleOk) {
    if (systemEnabled) {
      if (!bleAdvertising) {
        BLE.advertise();
        bleAdvertising = true;
        Serial.println("[BLE] Advertising ENABLED (POWER ON)");
      }
    } else {
      if (bleAdvertising) {
        BLE.stopAdvertise();
        bleAdvertising = false;
        Serial.println("[BLE] Advertising DISABLED (POWER OFF)");
      }
    }
  }
}

void onPowerLongPress() {
  Serial.println("[POWER] LONG PRESS: switching to new log file");

  // Increment file index and select the new file
  selectLogFile(currentFileIndex + 1);

  // Clear in-memory buffer so the new file starts "clean"
  head = 0;
  tail = 0;
  rowArmed = false;

  Serial.println("[BUF] Cleared all rows; new logging session in new file");
}

void handleButtonPress(int btn) {
  // When power is OFF, ignore everything except Button 8 (POWER)
  if (!systemEnabled && btn != 8) {
    Serial.print("[POWER] Ignoring button ");
    Serial.print(btn);
    Serial.println(" press while system is OFF");
    return;
  }

  if (btn == 1) {
    Serial.println("[LADDER] Button 1 (NEXT) PRESSED");
    onNextPressed();
  } else if (btn >= 2 && btn <= 6) {
    if (rowArmed) {
      if (btn == 2 && current.c1 == 0) { current.c1 = 1; Serial.println("[INPUT] c1 -> 1 (Button 2)"); }
      if (btn == 3 && current.c2 == 0) { current.c2 = 1; Serial.println("[INPUT] c2 -> 1 (Button 3)"); }
      if (btn == 4 && current.c3 == 0) { current.c3 = 1; Serial.println("[INPUT] c3 -> 1 (Button 4)"); }
      if (btn == 5 && current.c4 == 0) { current.c4 = 1; Serial.println("[INPUT] c4 -> 1 (Button 5)"); }
      if (btn == 6 && current.c5 == 0) { current.c5 = 1; Serial.println("[INPUT] c5 -> 1 (Button 6)"); }
      Serial.print("[ROW] "); Serial.println(rowPretty(current));
    } else {
      Serial.println("[INPUT] Ignored (no active row). Press Button 1 to start.");
    }
  } else if (btn == 7) {
    // ERROR button: make row all zeros, save, and start new row ready for collection
    if (rowArmed) {
      current.c1 = current.c2 = current.c3 = current.c4 = current.c5 = 0;
      Serial.println("[ERROR] Button 7: cleared current row to zeros, saving and starting new row");
      finalizeCurrentRow();  // save zeroed row with existing timestamp
      armNewRow();           // new row with new timestamp
    } else {
      Serial.println("[ERROR] Button 7: no active row; starting new zeroed row");
      armNewRow();
    }
  } else if (btn == 8) {
    // Button 8 logic handled in handleLadderEdges() to detect short vs long press
    // Do nothing here.
  }

  sendMessage(String("Button ") + btn + " PRESSED");
}

void handleLadderEdges() {
  uint32_t now = millis();

  // Ladder A
  float vA = codeToVoltage(readCodeA());
  int btnA = decodeLadderA(vA);
  int currentA = activeButtonA;
  if (currentA == 0) {
    if (btnA > 0) currentA = btnA;
  } else if (btnA <= 0) {
    // treat 0 or -1 (noise) as release
    currentA = 0;
  }
  if (currentA != activeButtonA && (now - lastChangeMsA) > DEBOUNCE_MS) {
    lastChangeMsA = now;
    if (currentA > 0) {
      handleButtonPress(currentA);
    } else if (activeButtonA > 0) {
      Serial.print("Button "); Serial.print(activeButtonA); Serial.println(" RELEASED (A)");
      sendMessage(String("Button ") + activeButtonA + " RELEASED");
    }
    activeButtonA = currentA;
  }

  // Ladder B
  float vB = codeToVoltage(readCodeB());
  int btnB = decodeLadderB(vB);
  int currentB = activeButtonB;
  if (currentB == 0) {
    if (btnB > 0) currentB = btnB;
  } else if (btnB <= 0) {
    // treat 0 or -1 (noise) as release
    currentB = 0;
  }
  if (currentB != activeButtonB && (now - lastChangeMsB) > DEBOUNCE_MS) {
    lastChangeMsB = now;
    if (currentB > 0) {
      // Press edge
      if (currentB == 8) {
        powerPressStartMs = now;
        Serial.println("[POWER] Button 8 PRESSED");
        sendMessage("Button 8 PRESSED");
        // Short/long behavior decided on release
      } else {
        handleButtonPress(currentB);
      }
    } else if (activeButtonB > 0) {
      // Release edge
      if (activeButtonB == 8) {
        uint32_t held = now - powerPressStartMs;
        Serial.print("[POWER] Button 8 RELEASED after ");
        Serial.print(held);
        Serial.println(" ms");

        if (held >= POWER_HOLD_MS) {
          onPowerLongPress();   // new CSV / new session
        } else {
          onPowerShortPress();  // original power toggle
        }
      } else {
        Serial.print("Button "); Serial.print(activeButtonB); Serial.println(" RELEASED (B)");
        sendMessage(String("Button ") + activeButtonB + " RELEASED");
      }
    }
    activeButtonB = currentB;
  }
}

// -------------------- Setup / Loop --------------
void setup() {
  // Serial with millis() timeout (NO while(!Serial))
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 2000) { }
  Serial.println("\nXIAO nRF52840 — Dual Ladder (A0:1..4, A1:5..8) + BLE + SD");
  Serial.println("[INFO] CSV: [c1,c2,c3,c4,c5,timestamp_ms,bt_sent,sd_sent,token]");
  Serial.println("[POWER] Initial state: ON");
  
  if (!SD.begin(chipSelect)) {
    Serial.println("[SD] SD.begin FAILED!");
  } else {
    Serial.println("[SD] SD.begin OK");
    // Default: use primary file (DATALOG.CSV), no deletion of old files.
    selectLogFile(0);
  }

  analogReadResolution(12);
  pinMode(DATA_PIN, INPUT);
  pinMode(DATA_PIN1, INPUT);

  // BLE init
  if (!BLE.begin()) {
    Serial.println("[BLE] Start failed! Continuing without BLE.");
    bleOk = false;
  } else {
    bleOk = true;
    BLE.setDeviceName("XIAO-Buttons");
    BLE.setLocalName("XIAO-Buttons");
    BLE.setAdvertisedService(uartService);
    uartService.addCharacteristic(txChar);
    uartService.addCharacteristic(rxChar);
    BLE.addService(uartService);
    if (BLE_SEND_BUTTON_MESSAGES) {
      txChar.writeValue("READY"); // optional banner
    }
    BLE.advertise();
    bleAdvertising = true;
    Serial.println("[BLE] Advertising as XIAO-Buttons (initial ON)");
  }
}

void loop() {
  // Always scan ladder edges so POWER button can wake system
  handleLadderEdges();

  if (systemEnabled) {
    // BLE: if connected, push any unsent rows (CSV only)
    if (bleOk) {
      BLEDevice central = BLE.central();
      if (central && central.connected()) {
        bleSendUnsent();
      }
    }

    // SD: when enough unsent rows exist, append a batch
    if (countPendingSD() >= SD_BATCH_SIZE) {
      sdWriteBatch(SD_BATCH_SIZE);
    }

    // Cleanup rows that are fully sent (BT + SD)
    popDoneRows();
  }

  delay(10);
}
