# Advanced Capacitor-Start Motor Analysis Lab

## Problem Statement

**Motor Specifications:**
- Power: 250 W
- Voltage: 230 V
- Frequency: 50 Hz
- Main winding impedance: Zm = (4.5 + j3.7) Ω
- Auxiliary winding impedance: Za = (9.5 + j3.5) Ω

**Objective:** Determine the value of the starting capacitor that will place the main and auxiliary winding currents in quadrature at starting.

## Theoretical Solution

### Quadrature Condition

For currents to be in quadrature, the phase angle between Im and Ia should be 90°.

### Calculation Steps

1. **Main Winding Current:**
   - Zm = 4.5 + j3.7 Ω
   - Im = V / Zm = 230 / (4.5 + j3.7)
   - θm = arctan(3.7 / 4.5) = 39.4°

2. **Auxiliary Winding with Capacitor:**
   - For quadrature: θa = θm - 90°
   - θa = 39.4° - 90° = -50.6°
   - Required: Za_total = Ra + j(Xa - Xc)
   - tan(θa) = (Xa - Xc) / Ra
   - Xa - Xc = Ra × tan(θa)
   - Xc = Xa - Ra × tan(θa)
   - Xc = 3.5 - 9.5 × tan(-50.6°)
   - Xc ≈ 15.14 Ω

3. **Capacitor Value:**
   - Xc = 1 / (2πfC)
   - C = 1 / (2π × 50 × 15.14)
   - **C ≈ 210.3 µF**

### Verification

With C ≈ 210.3 µF:
- Main current: Im ≈ 39.6 A ∠ -39.4°
- Auxiliary current: Ia ≈ 15.5 A ∠ 50.6°
- Phase difference: ≈ 90° ✓

## Application Features

### 1. User Interface (Tkinter GUI)

#### Main Menu
- **Control Panel:** Left sidebar with all input parameters and controls
- **Tabbed Interface:** Multiple tabs for different analysis views
- **Responsive Design:** Auto-scaling with window resize

#### Input Parameters
- Motor power, voltage, frequency
- Main winding resistance and reactance
- Auxiliary winding resistance and reactance
- Thermal parameters
- Mechanical parameters

#### Control Sliders
- **Load Torque:** Adjust mechanical load (0-5 Nm)
- **Voltage Adjustment:** Vary supply voltage (50-150%)

#### Visualization
- Real-time dynamic graphs
- Phasor diagrams
- Temperature distributions
- Economic analysis charts

### 2. Calculation Modules

#### Circuit Model
- Complex impedance calculations
- RMS current calculations
- Phasor analysis
- Power factor correction

#### Differential Equations
The motor dynamics are modeled using:

```
J × dω/dt = T_motor - T_load - B × ω
```

Where:
- J = Moment of inertia (kg·m²)
- ω = Angular velocity (rad/s)
- T_motor = Motor torque (Nm)
- T_load = Load torque (Nm)
- B = Friction coefficient

#### Thermal Model

```
C_th × dT/dt = P_loss - (T - T_ambient) / R_th
```

Where:
- C_th = Thermal capacitance (J/°C)
- T = Motor temperature (°C)
- P_loss = Total losses (W)
- R_th = Thermal resistance (°C/W)
- T_ambient = Ambient temperature (°C)

#### Dynamic Simulation

**ODE Solvers:**
1. **Euler Method:** Simple first-order method
   - Fast computation
   - Lower accuracy
   - Formula: y(n+1) = y(n) + h × f(t, y)

2. **RK45 (Runge-Kutta 4th Order):** Higher accuracy
   - Better stability
   - More computational cost
   - Adaptive step size

### 3. Results Visualization

#### Simulation Tab
- **Speed vs Time:** Motor speed in RPM
- **Torque vs Time:** Developed torque
- **Current vs Time:** Main and auxiliary winding currents (RMS)
- **Power vs Time:** Input power consumption
- **Temperature vs Time:** Motor temperature with rated limit
- **Efficiency vs Time:** Instantaneous efficiency

#### Analysis Tab
- **Phasor Diagram:** Current phasors showing quadrature condition
- **Impedance Comparison:** Bar chart of winding impedances
- **Loss Distribution:** Pie chart of loss breakdown
- **Vector Diagram:** Impedance vectors in complex plane

#### Thermal Analysis Tab
- **Temperature Profile:** Temperature rise over time
- **Derating Curve:** Power capacity vs temperature
- **Temperature Distribution:** 2D heat map
- **Thermal Stress:** Rate of temperature change

#### Economic Analysis Tab
- **Operating Cost:** Cumulative cost over time
- **Energy Loss Breakdown:** Cost impact of different losses

#### Advanced Controls Tab
- **Speed Control:** Comparison with synchronous speed
- **Torque Control:** Motor vs load torque
- **Magnetic Flux:** Flux linkage estimation
- **Control Signals:** Different control methods

### 4. Multi-Physics Simulation

#### Electromagnetic-Thermal Coupling
- Copper losses: I²R heating in windings
- Iron losses: Core losses proportional to V² and frequency
- Temperature affects winding resistance
- Heat transfer to environment

#### Mechanical Stress Analysis
- Shaft torque transients
- Rotational dynamics
- Friction losses
- Speed-dependent effects

#### Detailed Loss Breakdown
1. **Copper Losses:** Resistive heating in windings
   - Main winding: Im² × Rm
   - Auxiliary winding: Ia² × Ra

2. **Iron Losses:** Hysteresis and eddy currents
   - Proportional to V² and frequency

3. **Mechanical Losses:** Friction and windage
   - Proportional to speed²

4. **Stray Losses:** Other losses
   - ~1% of input power

### 5. Advanced Control Methods

Four control strategies implemented:

1. **Open Loop:** Direct voltage control
2. **V/f Control:** Voltage/frequency ratio control
3. **Field Oriented Control (FOC):** Vector control
4. **Direct Torque Control (DTC):** Direct torque regulation

### 6. Thermal Derating

- Automatic power derating above rated temperature
- Linear derating: 100% at rated temp → 0% at limit
- Real-time capacity indicator
- Protection against overheating

## Installation and Usage

### Requirements

```bash
pip install numpy matplotlib scipy tkinter
```

### Running the Application

```bash
python3 capacitor_motor_lab.py
```

### User Guide

1. **Initial Setup:**
   - Application opens with default motor parameters
   - Capacitor value is automatically calculated for quadrature
   - Review calculated capacitor value in control panel

2. **Adjusting Parameters:**
   - Modify motor parameters in input fields
   - Click "Recalculate Capacitor" to update
   - Adjust load torque and voltage sliders as needed

3. **Running Simulation:**
   - Select ODE solver (RK45 or Euler)
   - Click "Start" to begin simulation
   - Watch real-time graphs update
   - Click "Stop" to pause
   - Click "Reset" to clear and restart

4. **Exploring Tabs:**
   - **Simulation:** Monitor real-time performance
   - **Analysis:** View phasor diagrams and loss distribution
   - **Thermal Analysis:** Check temperature and derating
   - **Economic Analysis:** Track operating costs
   - **Advanced Controls:** Experiment with control methods

5. **Control Methods:**
   - Select control method in Advanced Controls tab
   - Observe different control behaviors
   - Compare performance metrics

## Key Features for Electrical Engineering Practice

### Educational Value
- **Phasor Analysis:** Visual understanding of AC circuits
- **Capacitor Sizing:** Practical power factor correction
- **Motor Dynamics:** Transient and steady-state behavior
- **Thermal Management:** Real-world temperature limitations

### Practical Applications
- **Motor Selection:** Compare different motor configurations
- **Energy Efficiency:** Identify and minimize losses
- **Cost Analysis:** Calculate operating expenses
- **Thermal Design:** Ensure adequate cooling

### Advanced Topics
- **Multi-physics Coupling:** Electromagnetic-thermal-mechanical interaction
- **Control Theory:** Implementation of modern control methods
- **Numerical Methods:** ODE solver comparison
- **System Optimization:** Balance performance, efficiency, and cost

## Technical Details

### RMS Values in Simulation
All currents and voltages in the simulation use RMS (Root Mean Square) values, which represent the effective AC quantities for power calculations.

### Time Step
- Default: dt = 0.001 s (1 ms)
- Provides smooth visualization
- Balances accuracy and performance

### Update Rates
- Simulation step: Every 10 ms
- Plot update: Every 50 steps
- Status update: Every 100 steps

### Auto-scaling
- Window resize automatically adjusts all components
- Grid weights ensure proportional scaling
- Graphs maintain aspect ratios

## Troubleshooting

### Application Won't Start
- Check Python version (3.6+)
- Verify all dependencies installed
- Check matplotlib backend configuration

### Graphs Not Updating
- Ensure simulation is started
- Check if time step is too large
- Verify solver selection

### High CPU Usage
- Reduce plot update frequency
- Use Euler instead of RK45
- Decrease simulation speed

## Future Enhancements

Potential additions:
- Export data to CSV
- Load/save motor configurations
- Compare multiple motors
- 3D visualization
- Real motor database
- Harmonic analysis
- Finite element integration

## References

1. Chapman, S. J. (2005). Electric Machinery Fundamentals. McGraw-Hill.
2. Fitzgerald, A. E., et al. (2003). Electric Machinery. McGraw-Hill.
3. Krause, P. C., et al. (2002). Analysis of Electric Machinery. IEEE Press.

## License

MIT License - Free for educational and commercial use

## Author

Created for advanced electrical engineering education and motor analysis.

---

**Version:** 1.0
**Date:** 2025
**Platform:** Python 3.6+ with Tkinter, NumPy, SciPy, Matplotlib
