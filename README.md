# Multiband Antenna Array for Secure Critical Wireless Communications
> **Code Repository – Master's Dissertation in Industrial Electronics and Computers Engineering (Universidade do Minho)**

This repository contains the source code developed for my Master's dissertation, focusing on the control and calibration of a multiband antenna array for UAVs. The system allows for phase configuration, beam sweeping, and null steering for interference mitigation (anti-jamming).

---

## 🧰 Technologies Used

* **C++ (Arduino)**: Firmware for hardware control and RF channel calibration.
* **Python 3**: Test automation, data extraction, and system control (bridging Arduino and GNU Radio).
* **GNU Radio**: Software Defined Radio (SDR) signal processing and power measurement.
* **Python Libraries**: `scikit-rf` (S-parameter analysis), `matplotlib` (visualization), and `pandas`.

---

## 📁 Repository Structure

```text
Dissertacao_Codigos/
├── /Firmware             # Arduino Microcontroller firmware
│   └── main_control.ino  # Calibration and phase control code (LUT)
│
├── /GNU_Radio            # GNU Radio flowgraphs and generated files
│   ├── rssi_monitor.grc  # Flowgraph for signal power (RSSI) measurement
│   └── rssi_monitor.py   # Generated Python script that communicates with the main automation script
│
├── /Scripts              # Scripts developed in VS Code
│   └── beam_sweeping.py  # Beam sweeping, null positioning, and system communication
│
├── /Logs                 # Data logs and practical test results
│   ├── jammer_log_20260924_160501.csv
│   └── ...
│
└── README.md
```

---

## ⚙️ Execution and Usage

### 1️⃣ Firmware (Arduino)
1. Open the files in the `/Firmware` directory using the Arduino IDE or VS Code (with the Arduino extension).
2. Compile and upload the firmware to the connected board.
3. The system will wait for phase configuration commands via serial communication.

### 2️⃣ GNU Radio (SDR)
1. Open `rssi_monitor.grc` in the `/GNU_Radio` folder using **GNU Radio Companion** (or run `rssi_monitor.py` directly).
2. Once executed, this block continuously measures the signal power (RSSI) and establishes communication with the main automation script to provide real-time power data.

### 3️⃣ Automation Scripts (Python)
The scripts located in the `/Scripts` folder act as the central controller. They communicate with the Arduino (via serial port) to set the phases, and retrieve the RSSI data from the active GNU Radio block to evaluate the beam sweeping and anti-jamming performance.

**Prerequisites:**
It is highly recommended to create a virtual environment (venv) and install the required dependencies:
```bash
pip install pyserial matplotlib scikit-rf pandas
```

**Run the Beam Sweeping test:**
```bash
cd Scripts
python beam_sweeping.py
```
*(Note: Ensure the correct COM port for the Arduino is configured at the beginning of the `beam_sweeping.py` file, and that the GNU Radio `rssi_monitor` is actively running before execution).*

---

## 👤 Author

**Tiago Matos**