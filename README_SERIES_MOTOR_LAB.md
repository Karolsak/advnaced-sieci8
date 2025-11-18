# Advanced Series Motor-Generator Analysis Lab

## Overview

This comprehensive Python + Tkinter laboratory provides advanced analysis and simulation capabilities for series DC motors operating in both motor and generator modes. The application features multi-physics simulation, dynamic modeling, and economic analysis tools.

## Features

### 1. **Main Analysis Tab**
- Interactive parameter adjustment using sliders (voltage, speed, load resistance)
- Real-time calculation of generator operating current
- Comprehensive results display including:
  - Electrical parameters (current, back EMF, power, efficiency)
  - Power loss breakdown (copper, iron, mechanical, stray losses)
  - Mechanical performance metrics
  - Thermal considerations
- Dynamic plotting of motor characteristics

### 2. **Loss Analysis Tab**
- Detailed breakdown of all loss components:
  - Copper losses (I²R)
  - Iron losses (hysteresis + eddy current)
  - Mechanical losses (friction and windage)
  - Stray load losses
- Visual representations:
  - Stacked area plots
  - Pie charts
  - Efficiency curves
  - Individual loss component trends

### 3. **Dynamic Simulation Tab**
- Real-time ODE solver implementation:
  - **RK45** (Runge-Kutta 4th/5th order adaptive)
  - **Euler** method
  - **RK23** (Runge-Kutta 2nd/3rd order)
- Coupled electromagnetic-thermal modeling
- Dynamic visualization of:
  - Current transients
  - Speed dynamics
  - Thermal behavior (winding and core temperatures)
  - Torque development
  - Power output
  - Efficiency variation
- Progress bar for simulation tracking
- Start/Stop/Reset controls

### 4. **Economic Analysis Tab**
- Operating cost calculation:
  - Energy costs
  - Maintenance costs
  - Cost per operating hour
- Efficiency impact analysis
- Investment payback analysis
- 10-year lifetime cost projections
- ROI calculations
- Cost breakdown per kWh

### 5. **Thermal & Derating Tab**
- Temperature derating curves
- Thermal transient response
- Thermal resistance network modeling
- Class F insulation temperature limits
- Heat transfer equations solved simultaneously with electrical equations
- Copper and iron loss temperature rise calculations

### 6. **Mechanical Stress Tab**
- Shaft torque analysis
- Shear stress calculations
- Bearing load evaluation (radial and axial)
- Mechanical safety factor analysis
- Centrifugal force calculations
- Yield strength comparison

## Mathematical Models

### Electromagnetic Model
For series motors/generators:
- `E = k·φ·N` where φ ∝ I (flux proportional to current)
- Voltage equation: `V = E ± I·R` (+ for motor, - for generator)
- Torque: `T = k·φ·I`

### Thermal Model
Coupled thermal equations:
```
dT_winding/dt = (P_copper - (T_winding - T_ambient)/R_thermal) / C_thermal
dT_core/dt = (P_iron - (T_core - T_ambient)/R_thermal) / C_thermal
```

### Mechanical Model
Dynamic equation:
```
J·dω/dt = T_electromagnetic - T_load - T_friction
```

### Loss Models
1. **Copper losses**: P_copper = I²·R·(1 + α·ΔT)
2. **Iron losses**: P_iron = k_h·f·B² + k_e·f²·B²
3. **Mechanical losses**: P_mech = k·(n/1000)²
4. **Stray losses**: P_stray ≈ 1% of output power

## Problem Solution

### Given Data:
- Voltage: 525 V
- Motor resistance: 3.5 Ω
- Load resistance: 3.25 Ω
- Target speed: 1000 rpm

| Current (A) | Speed (rpm) |
|------------|-------------|
| 75         | 1200        |
| 125        | 950         |
| 175        | 840         |
| 225        | 745         |

### Solution:
The application calculates the generator current using an iterative method:
1. Interpolate flux-current relationship from motor data
2. Calculate back EMF at target speed for estimated current
3. Apply generator equation: E = V + I·R_total
4. Iterate until convergence

**Result**: Current ≈ 133.8 A at 1000 rpm as generator

## Installation & Requirements

### Required Libraries:
```bash
pip install numpy scipy matplotlib
```

### Standard Library Requirements:
- tkinter (usually included with Python)
- threading
- dataclasses

## Usage

### Running the Application:
```bash
python3 series_motor_generator_lab.py
```

### Basic Workflow:
1. **Adjust Parameters**: Use sliders to modify voltage, speed, and load resistance
2. **Calculate**: Click "Calculate" button to update results
3. **Run Simulation**: Navigate to "Dynamic Simulation" tab and click "Start"
4. **Analyze Results**: Explore different tabs for detailed analysis
5. **Export Data**: Use "Export Data" button to save results to CSV

### Advanced Features:
- **Real-time Updates**: Parameters update dynamically as sliders move
- **Auto-scaling**: Window and plots automatically adjust to window size
- **Multi-threaded Simulation**: Simulations run in background without freezing GUI
- **Professional Visualization**: High-quality matplotlib plots with proper labeling

## Technical Specifications

### Numerical Methods:
- **RK45**: 4th/5th order Runge-Kutta with adaptive step sizing
- **Euler**: Simple first-order explicit method
- **Interpolation**: Cubic spline interpolation for characteristic curves

### Physical Constants:
- Copper density: 8960 kg/m³
- Iron density: 7874 kg/m³
- Copper specific heat: 385 J/(kg·K)
- Iron specific heat: 449 J/(kg·K)
- Temperature coefficient of copper: 0.00393 per °C

### Default Parameters:
- Moment of inertia: 0.5 kg·m²
- Friction coefficient: 0.01 N·m·s
- Thermal resistance: 2.5 °C/W
- Thermal capacitance: 800 J/°C
- Ambient temperature: 25 °C

## Applications

This lab is designed for:
- Electrical engineering education
- Motor design and analysis
- Generator performance evaluation
- Power system studies
- Control system design
- Economic feasibility studies
- Thermal management analysis
- Predictive maintenance planning

## Key Features for Practical Engineering Use

1. **Multi-Physics Coupling**: Electromagnetic, thermal, and mechanical domains solved simultaneously
2. **Real-time ODE Solver**: Accurate dynamic behavior prediction
3. **Economic Analysis**: Cost-benefit analysis for motor selection and upgrades
4. **Thermal Derating**: Proper temperature-dependent current rating
5. **Mechanical Safety**: Stress analysis ensures mechanical integrity
6. **Professional UI**: Clean, intuitive interface suitable for industrial use

## File Structure

```
series_motor_generator_lab.py
├── Classes:
│   ├── MotorParameters (dataclass)
│   ├── SeriesMotorModel
│   ├── MultiPhysicsSimulator
│   ├── EconomicAnalyzer
│   └── AdvancedMotorLab (main GUI)
└── Functions:
    ├── Electromagnetic calculations
    ├── Thermal modeling
    ├── Mechanical stress analysis
    ├── Economic analysis
    └── Visualization methods
```

## Author Notes

This laboratory combines theoretical rigor with practical engineering considerations. All models are based on established electrical machine theory and include real-world effects such as:
- Temperature-dependent resistance
- Saturation effects (via measured characteristics)
- Mechanical losses (friction and windage)
- Thermal time constants
- Economic considerations

The application is suitable for both educational purposes and preliminary engineering design work.

## License

Educational and research use.

## Version

1.0 - Initial comprehensive release with all features implemented
