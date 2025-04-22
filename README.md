
# eKanister – Master Thesis Repository

**Title:** Portable Energy Storage System for Emergency Charging of Electric Vehicles  
**Author:** Michał Makowski  
**University:** Warsaw University of Technology  
**Department:** Institute of Control and Industrial Electronics  
**Field of Study:** Electromobility  
**Year:** 2025

## Abstract

This repository supports a Master's thesis focused on the design and implementation of a portable energy storage system—eKanister—for emergency charging of electric vehicles. The work combines theoretical analysis with practical application in the field of power electronics. Key components include a Dual Active Bridge (DAB), a phase-locked loop (PLL)-based active rectifier, and a full-bridge inverter. The control software was implemented on a Texas Instruments TMS320F28335 DSP and tested using a Hardware-in-the-Loop (HIL) setup.

## Key Components

### Power Electronics

- **Dual Active Bridge (DAB):** Used for high-efficiency bidirectional DC-DC power conversion with galvanic isolation.
- **Active Rectifier with PLL:** Synchronizes with grid phase and reduces harmonic distortion.
- **Full-Bridge Inverter:** Converts DC to AC for compatibility with standard electric vehicle charging.

### Control and Testing

- Control algorithms were implemented in C and tested on a TMS320F28335 DSP.
- MATLAB was used for plotting and visualizing system outputs.
- Real-time simulations were conducted using a PLECS model and verified with an RT Box HIL platform.

## Simulation and Visualization

The MATLAB script `DAB_PLOT.m` provides a comparison between setpoint and measured output voltage for the DAB converter. It requires the CSV files `data_DAB_solo.csv` and `data_DAB_solo_input.csv` to be in the same directory.

## Thesis Focus Areas

- Analysis and selection of electrochemical cells (18650 Molicel P30B vs. 21700 LG M50).
- Battery pack design to support emergency driving range (around 20 km).
- Efficiency-focused design of converters and control algorithms.
- System validation using real-time simulation and experimental results.

## Requirements

**Software:**
- MATLAB
- PLECS Standalone
- Code Composer Studio

**Hardware:**
- TI ControlCARD TMS320F28335
- RT Box 1 (for HIL testing)

## License

This repository contains original work submitted as part of a university thesis. For academic or commercial reuse, please contact the author.
