# Wave Energy Converter (WEC) Prototype Control System

## 1. Project Overview
Control system for testing **wave energy converter (WEC) prototypes** using a **Raspberry Pi touchscreen UI** and **Arduino motor control**.  
The system allows real-time interaction with prototypes, including selecting WEC type, adjusting frequency/amplitude, and running tests.


## 2. Components / Hardware
- **Raspberry Pi** – Runs the touchscreen UI  
- **Arduino Mega** – Controls the motor  
- **VESC & VESC Express** – Motor controller and Data Logger  
- **WEC prototypes** – Point Absorber and Oscillating Flap types  
- **Touchscreen** – User interface for control  

## 3. Software
- **Python** – Raspberry Pi GUI and control code (`wec_rpi_gui.py`)  
- **MATLAB** – Hydrodynamic analyses (`PA_buoyancy.m`) 
- **Arduino** – Motor control code (`wec_arduino_motorcontrol.ino`)  
- **VESC logging script** (`log_can.lisp`) – Captures motor data points  

## 4. Folder Structure 
```
WEC-Prototype/
├── Arduino_Mega/
│ └── wec_arduino_motorcontrol.ino # Arduino firmware controlling motor based on RPi commands
├── CAD/
│ └── Multiple CAD pdf files
├── Raspberry_Pi/
│ └── wec_rpi_gui.py # Python UI for controlling WEC tests
└── VESC/
└── log_can.lisp # Logs motor data points during testing
├── hydrodynamics/
  └── PA_buoyancy.m # Evaluates the hydrodynamic repsonse of the PA
├── gear_ratio_appendix/
  └── Files required to simulate both the PA and OSWEC to estimate required GR
```
## 5. Code Setup Instructions

1. **Raspberry Pi**  
   - Copy `wec_rpi_gui.py` to the Pi.  
   - Configure the script to run automatically using **cron**.  
   - Connect Raspberry Pi to Arduino via USB.  

2. **Arduino Mega**  
   - Flash `wec_arduino_motorcontrol.ino` onto the Arduino.  
   - Ensure the **VESC UART library** is downloaded from GitHub and stored in Arduino’s libraries folder.  

3. **VESC / VESC Express**  
   - Flash firmware and edit log_can.lisp to VESC and VESC Express.  

4. **Dependencies & Hardware Connections**  
   - Proper USB connection between Pi and Arduino.  
   - Ensure VESC and motor connections are secure before powering the system.  

## 6. Usage Instructions

1. Power on Raspberry Pi and Arduino.  
2. Start the Python UI (`wec_rpi_gui.py`) via touchscreen or cron.  
3. Select the WEC type (Point Absorber or Oscillating Flap).  
4. Verify that all **electrical connections** are secure.  
5. Run tests by selecting frequency/amplitude options in the UI.  
6. Data from the VESC can be logged using `log_can.lisp`.
