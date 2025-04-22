
# 📦 eKanister – Master Thesis Repository

**Title:** Portable Energy Storage System for Emergency Charging of Electric Vehicles  
**Author:** Michał Makowski  
**University:** Warsaw University of Technology  
**Department:** Institute of Control and Industrial Electronics  
**Field of Study:** Electromobility  
**Year:** 2025

---

## 📘 Abstract

This repository contains the complete implementation, simulation, and codebase for the master thesis project titled **"eKanister"**, a portable emergency charging solution for electric vehicles. The system integrates modern power electronics such as Dual Active Bridge (DAB), Phase-Locked Loop (PLL)-based active rectifiers, and full-bridge inverters. The control algorithms are implemented for the Texas Instruments DSP `TMS320F28335`, tested under Hardware-in-the-Loop (HIL) conditions using the PLECS RT Box.

---

## 📂 Repository Structure

```
.
├── DAB_Inverter.plecs                        # PLECS model of DAB inverter
├── DAB_Active_Rectifier_Grid_to_Bank_TRANSFER.plecs  # PLECS model for active rectifier
├── DAB_PLOT.m                                # MATLAB script to visualize input/output voltages
├── INVERTER_FINAL.c                          # Full-bridge inverter control code for DSP
├── PLL_UNIPOLAR_NEW_FINAL.c                  # PLL-based active rectifier control code for DSP
├── F28335_controlCARD_ZJZ_Schematic_[R2.2].pdf # TI controlCARD hardware schematic
├── Magisterka_druk.pdf                       # Full thesis PDF (in Polish and English abstract)
└── README.md                                 # Project overview and guidance
```

---

## ⚙️ Key Components

### 🔌 Power Converter Topologies
- **Dual Active Bridge (DAB):** Enables bidirectional energy conversion with galvanic isolation.
- **Active Rectifier with PLL:** Allows synchronization with grid phase, reducing harmonics and improving power factor.
- **Full-Bridge Inverter:** Converts DC from storage to AC output compatible with standard EV chargers.

### 🎯 Control Implementation
- **Controller Platform:** TI TMS320F28335 DSP
- **Modulation Techniques:** Unipolar PWM for inverters, Phase Shift Modulation for DAB
- **PLL Design:** Synchronizes input AC phase for rectifier stability

### 🧪 Simulation & Testing
- **Simulation Tool:** PLECS (modeling of DAB and active rectifier)
- **Hardware-in-the-Loop (HIL):** RT Box 1 used to validate real-time control performance
- **Visualization:** MATLAB used for signal plotting and verification

---

## 📈 Example Plot

The MATLAB script `DAB_PLOT.m` visualizes the comparison between the setpoint input voltage and the measured DAB output voltage. Make sure you have the CSV files `data_DAB_solo.csv` and `data_DAB_solo_input.csv` in the working directory.

---

## 📑 Thesis Highlights

- **Cell Selection Analysis:** Compared 18650 Molicel P30B and 21700 LG M50 for energy density, weight, and power delivery.
- **Target Output Power:** ~7.68 kW AC via onboard charger equivalent.
- **Emergency Use Case:** Enables EVs to drive up to 20 km to reach the next charging station.
- **Compact & Portable:** Final battery pack configuration: 14S21P of LG M50 cells (~25 kg total mass).

> Detailed design choices, safety considerations, and HIL testing results are found in `Magisterka_druk.pdf`.

---

## 🛠 Requirements

- **Software:**  
  - PLECS Standalone (for simulation)  
  - MATLAB (for plotting)  
  - Code Composer Studio (for DSP programming)

- **Hardware:**  
  - TI ControlCARD TMS320F28335  
  - RT Box 1 for HIL testing

---

## 📜 License

This repository is part of a university thesis project. If you wish to use or extend the work, please contact the author for permission.
