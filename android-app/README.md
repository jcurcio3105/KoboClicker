Simulated Individual Counter App for KoboClicker

A lightweight Android app built with Jetpack Compose that dynamically logs data entries in real time — simulating sensor or microcontroller data (e.g. from an Arduino). 
Each entry batch is timestamped, can be customized by column headers, and exported or shared as a CSV file.

Features:
- Automatically generates simulated data batches every 5 seconds
- Editable column headers (default: Women, Men, Elderly)
- Add or remove columns dynamically (up to 5 total)
- Undo last batch
- Export data as .csv to device storage
- Share CSV files via email, Drive, etc.
- Automatically updates totals for each column

How It Works:
The app periodically generates random “batch” entries (1s or 0s) to simulate data arriving from a microcontroller.
Each batch behaves like a row in a CSV file:

Timestamp	Women	Men	Elderly

12:05:01, 1, 0, 1

Later, you can replace this simulation loop with real data over Bluetooth or serial input, using the same BatchEntry model.

Setup & Run:

Requirements
- Android Studio (Koala+ recommended)
- Android SDK 33+
- Kotlin + Jetpack Compose

Steps:
1) Clone the repository:
2) Open the project in Android Studio.
3) Connect an Android device or use the Emulator.
4) Press Run.

File Overview:
1) IndividualCounterActivity.kt: Main app logic and UI
2) AndroidManifest.xml:	App configuration and file sharing permissions
3) res/layout/:	Compose-managed UI (no XML layouts)

CSV Output Format


Each saved CSV file includes:

Timestamp,Women,Men,Elderly

12:05:01,1,0,1

12:05:06,0,1,1

12:05:11,1,0,0
