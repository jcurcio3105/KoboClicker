# KoboClicker
Firmware • Android App • CSV Uploader

KoboClicker is a complete hardware + software toolkit for rapid individual counting, data logging, and CSV export to KoboToolbox. The project contains **three major components**:

1. **Arduino firmware** for the Seeed XIAO nRF52840 Sense Plus  
2. **Android app** (Jetpack Compose) for BLE table display, session management, and CSV export  
3. **Uploader application** (Windows `.exe` + macOS `.dmg`) for uploading CSV data into KoboToolbox

---

## 1. Arduino Firmware (Seeed XIAO nRF52840 Sense Plus)

The KoboClicker device runs on the **Seeed XIAO nRF52840 Sense Plus**. Follow these steps to compile and upload the firmware.

### A. Install Arduino IDE  
Download **Arduino IDE 2.x** from the official Arduino website.

### B. Install Board Support  
Navigate to:

```
Tools → Board → Boards Manager
```

Search for:

```
Seeed nRF52 mbed-enabled Boards
```

Install the package, then select:

```
Tools → Board → Seeed nRF52 mbed-enabled Boards → XIAO nRF52840 Sense Plus (No Updates)
```

### C. Install Required Libraries  
Open:

```
Tools → Manage Libraries…
```

Install:

- ArduinoBLE  
- SD or SdFat  
- SPI  

### D. Uploading the Firmware  
1. Connect via USB-C  
2. Select the board and port  
3. Open the `.ino`  
4. Click **Verify** then **Upload**

### E. Firmware Behavior  
- Reads 8 buttons via resistor ladders  
- Generates rows (c1–c5)  
- NEXT / ERROR / POWER logic  
- BLE UART data output  
- SD card logging with batching  
- File rotation on long POWER press  
- Advertises as `XIAO-Buttons`

---

## 2. Android App (Jetpack Compose)

The Android app receives BLE data, displays rows, manages sessions, and exports CSVs.

### Files Required  
Place these in:

```
app/src/main/java/com/example/individualcounter/
```

- MainActivity.kt  
- BLEManager.kt  
- BatchEntry.kt  
- ScanDialog.kt  

### App Features  
- Live row table  
- Editable column headers  
- Multi-file session management  
- BLE scanning + connection  
- CSV export to Downloads  
- CSV sharing  
- Undo last row & clear data  

---

## 3. Uploader Application (Windows & macOS)

The repository includes an **Uploader ZIP** containing:

- A **Windows `.exe`**  
- A **macOS `.dmg`**

### Purpose  
This tool uploads CSV files produced by KoboClicker into **KoboToolbox**.

### Requirements  
- The CSV file must already be on your computer  
  (transfer via microSD from the KoboClicker device)

### How to Use  
1. Download and unzip `uploader.zip`  
2. Run the `.exe` (Windows) or `.dmg` (macOS)  
3. Follow the on-screen prompts  
4. Select your CSV  
5. The uploader will handle the KoboToolbox submission

---

## Summary

### Arduino  
- Use **Seeed nRF52 mbed-enabled Boards → XIAO nRF52840 Sense Plus (No Updates)**  
- Install ArduinoBLE + SD libraries  
- Upload `.ino` → device broadcasts BLE UART  

### Android  
- Add 4 Kotlin files to `com.example.individualcounter`  
- Build in Android Studio  
- Connect to `XIAO-Buttons` over BLE  
- Manage sessions + export CSVs  

### Uploader  
- Run `.exe` or `.dmg`  
- Select CSV → auto-upload to KoboToolbox  

---

## License
ES96 License
