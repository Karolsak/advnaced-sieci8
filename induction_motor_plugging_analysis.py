"""
Advanced Induction Motor Plugging Torque Analysis Lab
Multi-Physics Simulation with Dynamic Controls

Features:
- Plugging torque calculation
- Multi-physics simulation (Electromagnetic, Thermal, Mechanical)
- Dynamic ODE solvers (RK45, Euler)
- Advanced controls and thermal derating
- Economic analysis
- Real-time visualization
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import solve_ivp, odeint
import math
from datetime import datetime

class InductionMotorPluggingLab:
    """Advanced Induction Motor Analysis with Multi-Physics Simulation"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Induction Motor Plugging Torque Analysis Lab")
        self.root.geometry("1400x900")

        # Motor parameters
        self.params = {
            'P_rated': 30000,      # Rated power (W)
            'V_line': 400,         # Line voltage (V)
            'phases': 3,           # Number of phases
            'poles': 4,            # Number of poles
            'frequency': 50,       # Frequency (Hz)
            's_fl': 0.05,          # Full-load slip
            'XR_ratio': 4,         # X/R ratio at standstill
            'efficiency': 0.90,    # Motor efficiency
            'pf': 0.85,            # Power factor
        }

        # Calculated parameters
        self.calculate_motor_parameters()

        # Simulation state
        self.simulation_running = False
        self.simulation_data = {
            'time': [],
            'speed': [],
            'torque': [],
            'current': [],
            'temperature': [],
            'power': [],
            'losses': []
        }

        # Thermal parameters
        self.thermal = {
            'temp_ambient': 25,      # Ambient temperature (°C)
            'temp_current': 25,      # Current temperature (°C)
            'temp_rated': 75,        # Rated temperature (°C)
            'temp_max': 155,         # Maximum temperature (°C)
            'thermal_time_constant': 1800,  # Thermal time constant (s)
            'cooling_coefficient': 0.02,
        }

        # Control parameters
        self.control = {
            'control_mode': 'V/F',   # V/F, Vector, DTC
            'speed_reference': 1425,  # Reference speed (rpm)
            'torque_limit': 250,      # Torque limit (Nm)
            'current_limit': 60,      # Current limit (A)
        }

        # Economic parameters
        self.economics = {
            'electricity_cost': 0.12,  # $/kWh
            'maintenance_cost': 500,   # $/year
            'operating_hours': 8760,   # hours/year
            'motor_cost': 5000,        # $
            'lifespan': 15,            # years
        }

        # Create GUI
        self.create_gui()

        # Bind resize event
        self.root.bind('<Configure>', self.on_resize)

    def calculate_motor_parameters(self):
        """Calculate motor electrical parameters"""
        # Synchronous speed
        self.Ns = 120 * self.params['frequency'] / self.params['poles']  # rpm
        self.omega_s = 2 * np.pi * self.Ns / 60  # rad/s

        # Full-load speed
        self.N_fl = self.Ns * (1 - self.params['s_fl'])  # rpm
        self.omega_fl = 2 * np.pi * self.N_fl / 60  # rad/s

        # Full-load torque
        self.T_fl = self.params['P_rated'] / self.omega_fl  # Nm

        # Phase voltage (RMS)
        self.V_phase = self.params['V_line'] / np.sqrt(3)  # V

        # Full-load current (approximate)
        self.I_fl = self.params['P_rated'] / (np.sqrt(3) * self.params['V_line'] *
                                               self.params['pf'] * self.params['efficiency'])

        # Estimate motor circuit parameters
        # Using simplified equivalent circuit
        X_R_ratio = self.params['XR_ratio']

        # Rotor resistance (referred to stator)
        self.R2 = (3 * self.V_phase**2 * self.params['s_fl']) / \
                  (self.omega_s * self.T_fl * (1 + X_R_ratio**2))

        # Rotor reactance (referred to stator)
        self.X2 = X_R_ratio * self.R2

        # Stator resistance (approximate)
        self.R1 = 0.5 * self.R2

        # Stator reactance (approximate)
        self.X1 = 0.5 * self.X2

        # Magnetizing reactance
        self.Xm = 20 * self.X2

        # Store parameters
        self.params['Ns'] = self.Ns
        self.params['omega_s'] = self.omega_s
        self.params['N_fl'] = self.N_fl
        self.params['T_fl'] = self.T_fl
        self.params['V_phase'] = self.V_phase
        self.params['I_fl'] = self.I_fl
        self.params['R1'] = self.R1
        self.params['R2'] = self.R2
        self.params['X1'] = self.X1
        self.params['X2'] = self.X2
        self.params['Xm'] = self.Xm

    def calculate_torque(self, slip, voltage_factor=1.0):
        """Calculate motor torque at given slip"""
        V = self.V_phase * voltage_factor
        R1 = self.R1
        R2 = self.R2
        X1 = self.X1
        X2 = self.X2

        if slip == 0:
            return 0

        # Thevenin equivalent
        Zth = 1j * self.Xm * (R1 + 1j * X1) / (R1 + 1j * (X1 + self.Xm))
        Vth = V * self.Xm / np.sqrt(self.Xm**2 + X1**2)
        Rth = Zth.real
        Xth = Zth.imag

        # Torque calculation
        numerator = 3 * Vth**2 * (R2 / slip)
        denominator = self.omega_s * ((Rth + R2/slip)**2 + (Xth + X2)**2)

        torque = numerator / denominator
        return torque

    def calculate_current(self, slip, voltage_factor=1.0):
        """Calculate motor current at given slip"""
        V = self.V_phase * voltage_factor
        R1 = self.R1
        R2 = self.R2
        X1 = self.X1
        X2 = self.X2

        if slip == 0:
            slip = 0.001  # Avoid division by zero

        # Input impedance
        Z2 = (R2/slip) + 1j*X2
        Zm = 1j * self.Xm
        Z_parallel = (Z2 * Zm) / (Z2 + Zm)
        Z_total = (R1 + 1j*X1) + Z_parallel

        # Current
        I = V / abs(Z_total)
        return I

    def calculate_plugging_torque(self):
        """Calculate plugging torque at full speed"""
        # At full speed with slip s_fl, when reversed, effective slip = 2 - s_fl
        s_plugging = 2 - self.params['s_fl']

        # Calculate torque at plugging condition
        T_plugging = self.calculate_torque(s_plugging)

        # Calculate current at plugging
        I_plugging = self.calculate_current(s_plugging)

        # Ratio to full-load torque
        plugging_ratio = T_plugging / self.T_fl

        return {
            's_plugging': s_plugging,
            'T_plugging': T_plugging,
            'I_plugging': I_plugging,
            'plugging_ratio': plugging_ratio,
            'T_fl': self.T_fl
        }

    def calculate_losses(self, slip, current, torque, speed):
        """Calculate detailed loss breakdown"""
        # Copper losses (stator)
        P_cu_stator = 3 * current**2 * self.R1

        # Copper losses (rotor)
        P_cu_rotor = 3 * current**2 * self.R2 * slip / (1 - slip + 0.001)

        # Iron losses (approximate, frequency dependent)
        P_iron = 0.02 * self.params['P_rated'] * (slip + 0.5)

        # Mechanical losses (friction and windage)
        P_mech = 0.01 * self.params['P_rated'] * (abs(speed) / self.N_fl)

        # Stray load losses
        load_factor = abs(torque) / self.T_fl
        P_stray = 0.01 * self.params['P_rated'] * load_factor**2

        # Total losses
        P_total_loss = P_cu_stator + P_cu_rotor + P_iron + P_mech + P_stray

        return {
            'copper_stator': P_cu_stator,
            'copper_rotor': P_cu_rotor,
            'iron': P_iron,
            'mechanical': P_mech,
            'stray': P_stray,
            'total': P_total_loss
        }

    def thermal_model(self, temp, losses, speed):
        """Thermal model for temperature rise"""
        # Heat generation
        Q_gen = losses

        # Cooling effect (depends on speed)
        cooling_factor = max(0.3, abs(speed) / self.N_fl)

        # Heat dissipation
        Q_diss = self.thermal['cooling_coefficient'] * (temp - self.thermal['temp_ambient']) * cooling_factor

        # Temperature rate of change
        dT_dt = (Q_gen - Q_diss) / self.thermal['thermal_time_constant']

        return dT_dt

    def mechanical_stress_analysis(self, torque, speed, acceleration):
        """Analyze mechanical stresses"""
        # Shaft torque
        shaft_torque = torque

        # Shaft stress (simplified, assuming solid shaft)
        # τ = T*r/J, where J is polar moment
        shaft_diameter = 0.05  # 50mm (estimate)
        J = np.pi * shaft_diameter**4 / 32
        r = shaft_diameter / 2
        shaft_stress = abs(shaft_torque) * r / J if J > 0 else 0

        # Bearing load (radial load estimate)
        bearing_load = abs(torque) / (shaft_diameter / 2) if shaft_diameter > 0 else 0

        # Inertia effect
        J_motor = 0.5  # kg⋅m² (estimate)
        inertial_torque = J_motor * acceleration

        return {
            'shaft_torque': shaft_torque,
            'shaft_stress': shaft_stress,
            'bearing_load': bearing_load,
            'inertial_torque': inertial_torque
        }

    def motor_dynamics_ode(self, t, y, control_mode='V/F'):
        """
        ODE system for motor dynamics
        y = [speed (rpm), temperature (°C)]
        """
        speed = y[0]
        temperature = y[1]

        # Calculate slip
        slip = (self.Ns - speed) / self.Ns if self.Ns != 0 else 0
        slip = max(-2, min(2, slip))  # Limit slip range

        # Voltage control based on mode
        if control_mode == 'V/F':
            voltage_factor = abs(speed) / self.Ns if self.Ns != 0 else 1
            voltage_factor = min(1.0, max(0.1, voltage_factor))
        else:
            voltage_factor = 1.0

        # Calculate torque and current
        torque_motor = self.calculate_torque(slip, voltage_factor)
        current = self.calculate_current(slip, voltage_factor)

        # Apply torque limit
        torque_motor = np.sign(torque_motor) * min(abs(torque_motor), self.control['torque_limit'])

        # Load torque (assume constant load)
        torque_load = self.T_fl * 0.5

        # Calculate losses
        losses = self.calculate_losses(slip, current, torque_motor, speed)

        # Thermal derating
        if temperature > self.thermal['temp_rated']:
            derating_factor = max(0.5, 1 - 0.01 * (temperature - self.thermal['temp_rated']))
            torque_motor *= derating_factor

        # Net torque
        torque_net = torque_motor - torque_load

        # Mechanical equation: J * dω/dt = T_net
        J_motor = 0.5  # kg⋅m² (estimate)
        omega = 2 * np.pi * speed / 60  # rad/s
        d_omega_dt = torque_net / J_motor
        d_speed_dt = d_omega_dt * 60 / (2 * np.pi)  # rpm/s

        # Temperature dynamics
        d_temp_dt = self.thermal_model(temperature, losses['total'], speed)

        return [d_speed_dt, d_temp_dt]

    def run_dynamic_simulation(self, solver='RK45', duration=10):
        """Run dynamic simulation with specified ODE solver"""
        # Initial conditions
        y0 = [self.N_fl, self.thermal['temp_ambient']]

        # Time span
        t_span = (0, duration)
        t_eval = np.linspace(0, duration, 500)

        # Solve ODE
        if solver == 'RK45':
            sol = solve_ivp(
                lambda t, y: self.motor_dynamics_ode(t, y, self.control['control_mode']),
                t_span, y0, method='RK45', t_eval=t_eval, max_step=0.1
            )
            time = sol.t
            speed = sol.y[0]
            temperature = sol.y[1]
        elif solver == 'Euler':
            # Euler method implementation
            dt = 0.01
            time = np.arange(0, duration, dt)
            speed = np.zeros_like(time)
            temperature = np.zeros_like(time)
            speed[0] = y0[0]
            temperature[0] = y0[1]

            for i in range(1, len(time)):
                y = [speed[i-1], temperature[i-1]]
                dy_dt = self.motor_dynamics_ode(time[i-1], y, self.control['control_mode'])
                speed[i] = speed[i-1] + dy_dt[0] * dt
                temperature[i] = temperature[i-1] + dy_dt[1] * dt

        # Calculate additional quantities
        torque = np.zeros_like(time)
        current = np.zeros_like(time)
        power = np.zeros_like(time)
        losses_total = np.zeros_like(time)

        for i in range(len(time)):
            slip = (self.Ns - speed[i]) / self.Ns if self.Ns != 0 else 0
            torque[i] = self.calculate_torque(slip)
            current[i] = self.calculate_current(slip)
            omega = 2 * np.pi * speed[i] / 60
            power[i] = torque[i] * omega
            losses = self.calculate_losses(slip, current[i], torque[i], speed[i])
            losses_total[i] = losses['total']

        return {
            'time': time,
            'speed': speed,
            'torque': torque,
            'current': current,
            'temperature': temperature,
            'power': power,
            'losses': losses_total
        }

    def calculate_economics(self):
        """Calculate economic analysis"""
        # Annual energy consumption
        avg_power = self.params['P_rated'] * 0.75  # Assume 75% average load
        annual_energy = avg_power * self.economics['operating_hours'] / 1000  # kWh

        # Annual operating cost
        annual_energy_cost = annual_energy * self.economics['electricity_cost']
        annual_total_cost = annual_energy_cost + self.economics['maintenance_cost']

        # Lifetime cost
        lifetime_cost = annual_total_cost * self.economics['lifespan'] + self.economics['motor_cost']

        # Efficiency improvement savings
        improved_efficiency = 0.92
        if improved_efficiency > self.params['efficiency']:
            energy_saved = annual_energy * (1/self.params['efficiency'] - 1/improved_efficiency)
            annual_savings = energy_saved * self.economics['electricity_cost']
            payback_period = (self.economics['motor_cost'] * 0.2) / annual_savings if annual_savings > 0 else float('inf')
        else:
            annual_savings = 0
            payback_period = float('inf')

        return {
            'annual_energy': annual_energy,
            'annual_energy_cost': annual_energy_cost,
            'annual_total_cost': annual_total_cost,
            'lifetime_cost': lifetime_cost,
            'annual_savings': annual_savings,
            'payback_period': payback_period
        }

    def create_gui(self):
        """Create the main GUI"""
        # Create notebook for tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Create tabs
        self.create_main_tab()
        self.create_plugging_analysis_tab()
        self.create_dynamic_simulation_tab()
        self.create_multiphysics_tab()
        self.create_control_tab()
        self.create_economic_tab()

    def create_main_tab(self):
        """Create main control tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Main Control")

        # Left panel - Parameters
        left_frame = ttk.LabelFrame(tab, text="Motor Parameters", padding=10)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=5, pady=5)

        # Create parameter inputs
        self.param_widgets = {}
        param_labels = {
            'P_rated': 'Rated Power (kW)',
            'V_line': 'Line Voltage (V)',
            'poles': 'Number of Poles',
            'frequency': 'Frequency (Hz)',
            's_fl': 'Full-Load Slip',
            'XR_ratio': 'X/R Ratio',
            'efficiency': 'Efficiency',
            'pf': 'Power Factor'
        }

        row = 0
        for key, label in param_labels.items():
            ttk.Label(left_frame, text=label).grid(row=row, column=0, sticky=tk.W, pady=2)

            if key == 'P_rated':
                value = self.params[key] / 1000
            else:
                value = self.params[key]

            entry = ttk.Entry(left_frame, width=15)
            entry.insert(0, str(value))
            entry.grid(row=row, column=1, pady=2, padx=5)
            self.param_widgets[key] = entry
            row += 1

        # Update button
        ttk.Button(left_frame, text="Update Parameters",
                  command=self.update_parameters).grid(row=row, column=0, columnspan=2, pady=10)

        # Right panel - Results display
        right_frame = ttk.LabelFrame(tab, text="Calculated Parameters", padding=10)
        right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.results_text = tk.Text(right_frame, height=25, width=60, font=('Courier', 10))
        self.results_text.pack(fill=tk.BOTH, expand=True)

        # Scrollbar
        scrollbar = ttk.Scrollbar(right_frame, command=self.results_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.results_text.config(yscrollcommand=scrollbar.set)

        # Display initial results
        self.display_main_results()

    def create_plugging_analysis_tab(self):
        """Create plugging torque analysis tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Plugging Analysis")

        # Control panel
        control_frame = ttk.LabelFrame(tab, text="Analysis Control", padding=10)
        control_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        ttk.Button(control_frame, text="Calculate Plugging Torque",
                  command=self.analyze_plugging).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Plot Torque-Speed Curve",
                  command=self.plot_torque_speed).pack(side=tk.LEFT, padx=5)

        # Results panel
        results_frame = ttk.LabelFrame(tab, text="Plugging Analysis Results", padding=10)
        results_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.plugging_text = tk.Text(results_frame, height=10, font=('Courier', 10))
        self.plugging_text.pack(fill=tk.BOTH, expand=True)

        # Graph canvas
        self.plugging_fig = Figure(figsize=(10, 6))
        self.plugging_canvas = FigureCanvasTkAgg(self.plugging_fig, tab)
        self.plugging_canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True, padx=5, pady=5)

    def create_dynamic_simulation_tab(self):
        """Create dynamic simulation tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Dynamic Simulation")

        # Control panel
        control_frame = ttk.LabelFrame(tab, text="Simulation Control", padding=10)
        control_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        # Solver selection
        ttk.Label(control_frame, text="ODE Solver:").pack(side=tk.LEFT, padx=5)
        self.solver_var = tk.StringVar(value='RK45')
        ttk.Radiobutton(control_frame, text="RK45", variable=self.solver_var,
                       value='RK45').pack(side=tk.LEFT)
        ttk.Radiobutton(control_frame, text="Euler", variable=self.solver_var,
                       value='Euler').pack(side=tk.LEFT)

        ttk.Label(control_frame, text="Duration (s):").pack(side=tk.LEFT, padx=5)
        self.duration_var = tk.StringVar(value='10')
        ttk.Entry(control_frame, textvariable=self.duration_var, width=10).pack(side=tk.LEFT)

        ttk.Button(control_frame, text="Start Simulation",
                  command=self.start_simulation).pack(side=tk.LEFT, padx=10)
        ttk.Button(control_frame, text="Stop",
                  command=self.stop_simulation).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Reset",
                  command=self.reset_simulation).pack(side=tk.LEFT, padx=5)

        # Graph canvas
        self.sim_fig = Figure(figsize=(12, 8))
        self.sim_canvas = FigureCanvasTkAgg(self.sim_fig, tab)
        self.sim_canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True, padx=5, pady=5)

    def create_multiphysics_tab(self):
        """Create multi-physics analysis tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Multi-Physics Analysis")

        # Control panel
        control_frame = ttk.LabelFrame(tab, text="Multi-Physics Control", padding=10)
        control_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        ttk.Button(control_frame, text="Run Multi-Physics Simulation",
                  command=self.run_multiphysics).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Show Loss Breakdown",
                  command=self.show_loss_breakdown).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Mechanical Stress Analysis",
                  command=self.show_mechanical_stress).pack(side=tk.LEFT, padx=5)

        # Results display
        results_frame = ttk.LabelFrame(tab, text="Multi-Physics Results", padding=10)
        results_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=False, padx=5, pady=5)

        self.multiphysics_text = tk.Text(results_frame, height=8, font=('Courier', 9))
        self.multiphysics_text.pack(fill=tk.BOTH, expand=True)

        # Graph canvas
        self.multi_fig = Figure(figsize=(12, 7))
        self.multi_canvas = FigureCanvasTkAgg(self.multi_fig, tab)
        self.multi_canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True, padx=5, pady=5)

    def create_control_tab(self):
        """Create advanced control tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Advanced Controls")

        # Control parameters
        param_frame = ttk.LabelFrame(tab, text="Control Parameters", padding=10)
        param_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=5, pady=5)

        # Control mode
        ttk.Label(param_frame, text="Control Mode:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.control_mode_var = tk.StringVar(value='V/F')
        control_combo = ttk.Combobox(param_frame, textvariable=self.control_mode_var,
                                     values=['V/F', 'Vector', 'DTC'], width=15)
        control_combo.grid(row=0, column=1, pady=5, padx=5)
        control_combo.bind('<<ComboboxSelected>>', self.update_control_mode)

        # Speed reference slider
        ttk.Label(param_frame, text="Speed Reference (rpm):").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.speed_ref_var = tk.DoubleVar(value=self.control['speed_reference'])
        self.speed_ref_label = ttk.Label(param_frame, text=f"{self.control['speed_reference']:.0f}")
        self.speed_ref_label.grid(row=1, column=2, pady=5)
        speed_scale = ttk.Scale(param_frame, from_=0, to=self.Ns*1.2,
                               variable=self.speed_ref_var, orient=tk.HORIZONTAL,
                               command=self.update_speed_reference, length=200)
        speed_scale.grid(row=1, column=1, pady=5, padx=5)

        # Torque limit slider
        ttk.Label(param_frame, text="Torque Limit (Nm):").grid(row=2, column=0, sticky=tk.W, pady=5)
        self.torque_limit_var = tk.DoubleVar(value=self.control['torque_limit'])
        self.torque_limit_label = ttk.Label(param_frame, text=f"{self.control['torque_limit']:.0f}")
        self.torque_limit_label.grid(row=2, column=2, pady=5)
        torque_scale = ttk.Scale(param_frame, from_=0, to=500,
                                variable=self.torque_limit_var, orient=tk.HORIZONTAL,
                                command=self.update_torque_limit, length=200)
        torque_scale.grid(row=2, column=1, pady=5, padx=5)

        # Current limit slider
        ttk.Label(param_frame, text="Current Limit (A):").grid(row=3, column=0, sticky=tk.W, pady=5)
        self.current_limit_var = tk.DoubleVar(value=self.control['current_limit'])
        self.current_limit_label = ttk.Label(param_frame, text=f"{self.control['current_limit']:.0f}")
        self.current_limit_label.grid(row=3, column=2, pady=5)
        current_scale = ttk.Scale(param_frame, from_=0, to=150,
                                 variable=self.current_limit_var, orient=tk.HORIZONTAL,
                                 command=self.update_current_limit, length=200)
        current_scale.grid(row=3, column=1, pady=5, padx=5)

        # Thermal derating
        thermal_frame = ttk.LabelFrame(tab, text="Thermal Management", padding=10)
        thermal_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        ttk.Label(thermal_frame, text="Ambient Temperature (°C):").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.ambient_temp_var = tk.DoubleVar(value=self.thermal['temp_ambient'])
        ttk.Entry(thermal_frame, textvariable=self.ambient_temp_var, width=10).grid(row=0, column=1, pady=5)

        ttk.Label(thermal_frame, text="Rated Temperature (°C):").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.rated_temp_var = tk.DoubleVar(value=self.thermal['temp_rated'])
        ttk.Entry(thermal_frame, textvariable=self.rated_temp_var, width=10).grid(row=1, column=1, pady=5)

        ttk.Label(thermal_frame, text="Max Temperature (°C):").grid(row=2, column=0, sticky=tk.W, pady=5)
        self.max_temp_var = tk.DoubleVar(value=self.thermal['temp_max'])
        ttk.Entry(thermal_frame, textvariable=self.max_temp_var, width=10).grid(row=2, column=1, pady=5)

        ttk.Button(thermal_frame, text="Update Thermal Settings",
                  command=self.update_thermal_settings).grid(row=3, column=0, columnspan=2, pady=10)

        # Current temperature display
        ttk.Label(thermal_frame, text="Current Temperature:", font=('Arial', 12, 'bold')).grid(
            row=4, column=0, columnspan=2, pady=10)
        self.current_temp_label = ttk.Label(thermal_frame, text=f"{self.thermal['temp_current']:.1f} °C",
                                           font=('Arial', 14))
        self.current_temp_label.grid(row=5, column=0, columnspan=2, pady=5)

    def create_economic_tab(self):
        """Create economic analysis tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Economic Analysis")

        # Input parameters
        param_frame = ttk.LabelFrame(tab, text="Economic Parameters", padding=10)
        param_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=5, pady=5)

        economic_params = {
            'electricity_cost': ('Electricity Cost ($/kWh):', 0.12),
            'maintenance_cost': ('Annual Maintenance Cost ($):', 500),
            'operating_hours': ('Operating Hours (h/year):', 8760),
            'motor_cost': ('Motor Initial Cost ($):', 5000),
            'lifespan': ('Expected Lifespan (years):', 15)
        }

        self.economic_widgets = {}
        row = 0
        for key, (label, default) in economic_params.items():
            ttk.Label(param_frame, text=label).grid(row=row, column=0, sticky=tk.W, pady=5)
            entry = ttk.Entry(param_frame, width=15)
            entry.insert(0, str(default))
            entry.grid(row=row, column=1, pady=5, padx=5)
            self.economic_widgets[key] = entry
            row += 1

        ttk.Button(param_frame, text="Calculate Economics",
                  command=self.calculate_and_display_economics).grid(row=row, column=0, columnspan=2, pady=10)

        # Results display
        results_frame = ttk.LabelFrame(tab, text="Economic Analysis Results", padding=10)
        results_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.economic_text = tk.Text(results_frame, height=20, font=('Courier', 10))
        self.economic_text.pack(fill=tk.BOTH, expand=True)

        # Initial calculation
        self.calculate_and_display_economics()

    def update_parameters(self):
        """Update motor parameters from GUI inputs"""
        try:
            # Update parameters
            self.params['P_rated'] = float(self.param_widgets['P_rated'].get()) * 1000
            self.params['V_line'] = float(self.param_widgets['V_line'].get())
            self.params['poles'] = int(self.param_widgets['poles'].get())
            self.params['frequency'] = float(self.param_widgets['frequency'].get())
            self.params['s_fl'] = float(self.param_widgets['s_fl'].get())
            self.params['XR_ratio'] = float(self.param_widgets['XR_ratio'].get())
            self.params['efficiency'] = float(self.param_widgets['efficiency'].get())
            self.params['pf'] = float(self.param_widgets['pf'].get())

            # Recalculate parameters
            self.calculate_motor_parameters()

            # Display updated results
            self.display_main_results()

            messagebox.showinfo("Success", "Parameters updated successfully!")

        except ValueError as e:
            messagebox.showerror("Error", f"Invalid input: {e}")

    def display_main_results(self):
        """Display calculated motor parameters"""
        self.results_text.delete(1.0, tk.END)

        results = f"""
{'='*60}
INDUCTION MOTOR CALCULATED PARAMETERS
{'='*60}

Input Parameters:
  Rated Power:           {self.params['P_rated']/1000:.1f} kW
  Line Voltage:          {self.params['V_line']:.0f} V
  Number of Poles:       {self.params['poles']}
  Frequency:             {self.params['frequency']:.0f} Hz
  Full-Load Slip:        {self.params['s_fl']:.4f} ({self.params['s_fl']*100:.2f}%)
  X/R Ratio:             {self.params['XR_ratio']:.2f}
  Efficiency:            {self.params['efficiency']:.3f} ({self.params['efficiency']*100:.1f}%)
  Power Factor:          {self.params['pf']:.3f}

Calculated Parameters:
  Synchronous Speed:     {self.Ns:.2f} rpm
  Angular Sync Speed:    {self.omega_s:.2f} rad/s
  Full-Load Speed:       {self.N_fl:.2f} rpm
  Full-Load Torque:      {self.T_fl:.2f} Nm
  Phase Voltage (RMS):   {self.V_phase:.2f} V
  Full-Load Current:     {self.I_fl:.2f} A

Equivalent Circuit Parameters:
  Stator Resistance R1:  {self.R1:.4f} Ω
  Rotor Resistance R2:   {self.R2:.4f} Ω
  Stator Reactance X1:   {self.X1:.4f} Ω
  Rotor Reactance X2:    {self.X2:.4f} Ω
  Magnetizing React. Xm: {self.Xm:.4f} Ω

{'='*60}
"""
        self.results_text.insert(1.0, results)

    def analyze_plugging(self):
        """Analyze plugging torque"""
        results = self.calculate_plugging_torque()

        output = f"""
{'='*60}
PLUGGING TORQUE ANALYSIS
{'='*60}

Operating Conditions:
  Full-Load Speed:       {self.N_fl:.2f} rpm
  Full-Load Slip:        {self.params['s_fl']:.4f} ({self.params['s_fl']*100:.2f}%)
  Full-Load Torque:      {results['T_fl']:.2f} Nm

Plugging Conditions:
  Effective Slip:        {results['s_plugging']:.4f} ({results['s_plugging']*100:.2f}%)
  Plugging Torque:       {results['T_plugging']:.2f} Nm
  Plugging Current:      {results['I_plugging']:.2f} A

Performance Ratios:
  T_plugging / T_fl:     {results['plugging_ratio']:.3f}
  I_plugging / I_fl:     {results['I_plugging']/self.I_fl:.3f}

Analysis:
  The plugging torque is {results['plugging_ratio']:.2f} times the full-load
  torque. This high braking torque allows for rapid deceleration but
  generates significant heat and mechanical stress.

  The plugging current is {results['I_plugging']/self.I_fl:.2f} times the full-load
  current, which requires careful consideration of the motor's thermal
  capacity and protection settings.

Recommendations:
  • Limit plugging duration to prevent overheating
  • Ensure motor insulation can withstand high currents
  • Consider thermal relays for protection
  • Check mechanical components for stress tolerance

{'='*60}
"""

        self.plugging_text.delete(1.0, tk.END)
        self.plugging_text.insert(1.0, output)

    def plot_torque_speed(self):
        """Plot torque-speed characteristic"""
        # Generate speed range
        slip_range = np.linspace(-0.2, 2.0, 500)
        speed_range = self.Ns * (1 - slip_range)
        torque_range = [self.calculate_torque(s) for s in slip_range]
        current_range = [self.calculate_current(s) for s in slip_range]

        # Clear figure
        self.plugging_fig.clear()

        # Create subplots
        ax1 = self.plugging_fig.add_subplot(2, 1, 1)
        ax2 = self.plugging_fig.add_subplot(2, 1, 2)

        # Plot torque-speed curve
        ax1.plot(speed_range, torque_range, 'b-', linewidth=2, label='Torque')
        ax1.axhline(y=self.T_fl, color='g', linestyle='--', label=f'Full-Load Torque ({self.T_fl:.1f} Nm)')
        ax1.axvline(x=self.N_fl, color='r', linestyle='--', label=f'Full-Load Speed ({self.N_fl:.0f} rpm)')

        # Mark plugging point
        s_plug = 2 - self.params['s_fl']
        T_plug = self.calculate_torque(s_plug)
        N_plug = self.Ns * (1 - s_plug)
        ax1.plot(N_plug, T_plug, 'ro', markersize=10, label=f'Plugging Point ({T_plug:.1f} Nm)')

        ax1.set_xlabel('Speed (rpm)', fontsize=10)
        ax1.set_ylabel('Torque (Nm)', fontsize=10)
        ax1.set_title('Torque-Speed Characteristic Curve', fontsize=12, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.legend(fontsize=8)

        # Plot current-speed curve
        ax2.plot(speed_range, current_range, 'r-', linewidth=2, label='Current')
        ax2.axhline(y=self.I_fl, color='g', linestyle='--', label=f'Full-Load Current ({self.I_fl:.1f} A)')
        ax2.axvline(x=self.N_fl, color='b', linestyle='--', label=f'Full-Load Speed ({self.N_fl:.0f} rpm)')

        # Mark plugging point
        I_plug = self.calculate_current(s_plug)
        ax2.plot(N_plug, I_plug, 'ro', markersize=10, label=f'Plugging Point ({I_plug:.1f} A)')

        ax2.set_xlabel('Speed (rpm)', fontsize=10)
        ax2.set_ylabel('Current (A)', fontsize=10)
        ax2.set_title('Current-Speed Characteristic Curve', fontsize=12, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.legend(fontsize=8)

        self.plugging_fig.tight_layout()
        self.plugging_canvas.draw()

    def start_simulation(self):
        """Start dynamic simulation"""
        try:
            solver = self.solver_var.get()
            duration = float(self.duration_var.get())

            # Run simulation
            results = self.run_dynamic_simulation(solver=solver, duration=duration)

            # Store results
            self.simulation_data = results

            # Plot results
            self.plot_simulation_results()

        except Exception as e:
            messagebox.showerror("Error", f"Simulation error: {e}")

    def stop_simulation(self):
        """Stop simulation"""
        self.simulation_running = False

    def reset_simulation(self):
        """Reset simulation"""
        self.simulation_data = {
            'time': [],
            'speed': [],
            'torque': [],
            'current': [],
            'temperature': [],
            'power': [],
            'losses': []
        }
        self.sim_fig.clear()
        self.sim_canvas.draw()

    def plot_simulation_results(self):
        """Plot dynamic simulation results"""
        if not self.simulation_data['time']:
            return

        # Clear figure
        self.sim_fig.clear()

        # Create subplots
        ax1 = self.sim_fig.add_subplot(3, 2, 1)
        ax2 = self.sim_fig.add_subplot(3, 2, 2)
        ax3 = self.sim_fig.add_subplot(3, 2, 3)
        ax4 = self.sim_fig.add_subplot(3, 2, 4)
        ax5 = self.sim_fig.add_subplot(3, 2, 5)
        ax6 = self.sim_fig.add_subplot(3, 2, 6)

        time = self.simulation_data['time']

        # Speed
        ax1.plot(time, self.simulation_data['speed'], 'b-', linewidth=2)
        ax1.set_ylabel('Speed (rpm)', fontsize=9)
        ax1.set_title('Motor Speed', fontsize=10, fontweight='bold')
        ax1.grid(True, alpha=0.3)

        # Torque
        ax2.plot(time, self.simulation_data['torque'], 'r-', linewidth=2)
        ax2.set_ylabel('Torque (Nm)', fontsize=9)
        ax2.set_title('Motor Torque', fontsize=10, fontweight='bold')
        ax2.grid(True, alpha=0.3)

        # Current
        ax3.plot(time, self.simulation_data['current'], 'g-', linewidth=2)
        ax3.set_ylabel('Current (A)', fontsize=9)
        ax3.set_title('Motor Current', fontsize=10, fontweight='bold')
        ax3.grid(True, alpha=0.3)

        # Temperature
        ax4.plot(time, self.simulation_data['temperature'], 'm-', linewidth=2)
        ax4.axhline(y=self.thermal['temp_rated'], color='orange', linestyle='--',
                   label=f'Rated ({self.thermal["temp_rated"]}°C)')
        ax4.axhline(y=self.thermal['temp_max'], color='r', linestyle='--',
                   label=f'Max ({self.thermal["temp_max"]}°C)')
        ax4.set_ylabel('Temperature (°C)', fontsize=9)
        ax4.set_title('Winding Temperature', fontsize=10, fontweight='bold')
        ax4.grid(True, alpha=0.3)
        ax4.legend(fontsize=7)

        # Power
        ax5.plot(time, np.array(self.simulation_data['power'])/1000, 'c-', linewidth=2)
        ax5.set_xlabel('Time (s)', fontsize=9)
        ax5.set_ylabel('Power (kW)', fontsize=9)
        ax5.set_title('Output Power', fontsize=10, fontweight='bold')
        ax5.grid(True, alpha=0.3)

        # Losses
        ax6.plot(time, np.array(self.simulation_data['losses'])/1000, 'orange', linewidth=2)
        ax6.set_xlabel('Time (s)', fontsize=9)
        ax6.set_ylabel('Losses (kW)', fontsize=9)
        ax6.set_title('Total Losses', fontsize=10, fontweight='bold')
        ax6.grid(True, alpha=0.3)

        self.sim_fig.tight_layout()
        self.sim_canvas.draw()

        # Update current temperature
        final_temp = self.simulation_data['temperature'][-1]
        self.thermal['temp_current'] = final_temp
        self.current_temp_label.config(text=f"{final_temp:.1f} °C")

    def run_multiphysics(self):
        """Run multi-physics simulation"""
        try:
            # Run dynamic simulation
            results = self.run_dynamic_simulation(solver='RK45', duration=5)

            # Calculate detailed analysis at final state
            final_speed = results['speed'][-1]
            final_torque = results['torque'][-1]
            final_current = results['current'][-1]
            final_temp = results['temperature'][-1]

            slip = (self.Ns - final_speed) / self.Ns if self.Ns != 0 else 0

            # Loss breakdown
            losses = self.calculate_losses(slip, final_current, final_torque, final_speed)

            # Mechanical stress
            acceleration = 0  # Steady state
            stress = self.mechanical_stress_analysis(final_torque, final_speed, acceleration)

            # Display results
            output = f"""
{'='*60}
MULTI-PHYSICS SIMULATION RESULTS
{'='*60}

Electromagnetic Analysis:
  Speed:                 {final_speed:.2f} rpm
  Slip:                  {slip:.4f} ({slip*100:.2f}%)
  Torque:                {final_torque:.2f} Nm
  Current (RMS):         {final_current:.2f} A
  Power Output:          {final_torque * 2*np.pi*final_speed/60 / 1000:.2f} kW

Thermal Analysis:
  Winding Temperature:   {final_temp:.2f} °C
  Ambient Temperature:   {self.thermal['temp_ambient']:.2f} °C
  Temperature Rise:      {final_temp - self.thermal['temp_ambient']:.2f} °C
  Thermal Margin:        {self.thermal['temp_max'] - final_temp:.2f} °C

Loss Breakdown:
  Stator Copper Loss:    {losses['copper_stator']/1000:.3f} kW ({losses['copper_stator']/losses['total']*100:.1f}%)
  Rotor Copper Loss:     {losses['copper_rotor']/1000:.3f} kW ({losses['copper_rotor']/losses['total']*100:.1f}%)
  Iron Loss:             {losses['iron']/1000:.3f} kW ({losses['iron']/losses['total']*100:.1f}%)
  Mechanical Loss:       {losses['mechanical']/1000:.3f} kW ({losses['mechanical']/losses['total']*100:.1f}%)
  Stray Load Loss:       {losses['stray']/1000:.3f} kW ({losses['stray']/losses['total']*100:.1f}%)
  Total Losses:          {losses['total']/1000:.3f} kW

Mechanical Stress Analysis:
  Shaft Torque:          {stress['shaft_torque']:.2f} Nm
  Shaft Stress:          {stress['shaft_stress']/1e6:.2f} MPa
  Bearing Load:          {stress['bearing_load']/1000:.2f} kN

Efficiency:
  Output Power:          {final_torque * 2*np.pi*final_speed/60 / 1000:.2f} kW
  Input Power:           {(final_torque * 2*np.pi*final_speed/60 + losses['total']) / 1000:.2f} kW
  Efficiency:            {final_torque * 2*np.pi*final_speed/60 / (final_torque * 2*np.pi*final_speed/60 + losses['total']) * 100:.2f}%

{'='*60}
"""

            self.multiphysics_text.delete(1.0, tk.END)
            self.multiphysics_text.insert(1.0, output)

        except Exception as e:
            messagebox.showerror("Error", f"Multi-physics simulation error: {e}")

    def show_loss_breakdown(self):
        """Show detailed loss breakdown chart"""
        # Calculate losses at full load
        slip = self.params['s_fl']
        current = self.I_fl
        torque = self.T_fl
        speed = self.N_fl

        losses = self.calculate_losses(slip, current, torque, speed)

        # Clear figure
        self.multi_fig.clear()

        # Create pie chart
        ax1 = self.multi_fig.add_subplot(1, 2, 1)

        loss_labels = ['Stator Copper', 'Rotor Copper', 'Iron', 'Mechanical', 'Stray Load']
        loss_values = [
            losses['copper_stator'],
            losses['copper_rotor'],
            losses['iron'],
            losses['mechanical'],
            losses['stray']
        ]

        colors = ['#ff9999', '#ff6666', '#66b3ff', '#99ff99', '#ffcc99']
        explode = (0.1, 0.1, 0, 0, 0)

        ax1.pie(loss_values, explode=explode, labels=loss_labels, colors=colors,
                autopct='%1.1f%%', shadow=True, startangle=90)
        ax1.set_title('Loss Distribution at Full Load', fontsize=12, fontweight='bold')

        # Create bar chart
        ax2 = self.multi_fig.add_subplot(1, 2, 2)

        loss_values_kW = [v/1000 for v in loss_values]
        bars = ax2.bar(loss_labels, loss_values_kW, color=colors)
        ax2.set_ylabel('Loss (kW)', fontsize=10)
        ax2.set_title('Loss Breakdown', fontsize=12, fontweight='bold')
        ax2.grid(True, alpha=0.3, axis='y')

        # Add value labels on bars
        for bar in bars:
            height = bar.get_height()
            ax2.text(bar.get_x() + bar.get_width()/2., height,
                    f'{height:.3f} kW',
                    ha='center', va='bottom', fontsize=8)

        plt.setp(ax2.xaxis.get_majorticklabels(), rotation=45, ha='right')

        self.multi_fig.tight_layout()
        self.multi_canvas.draw()

    def show_mechanical_stress(self):
        """Show mechanical stress analysis"""
        # Simulate transient condition
        time = np.linspace(0, 2, 100)
        torque_transient = self.T_fl * (1 + 0.5 * np.sin(2*np.pi*time))
        speed_transient = self.N_fl * np.ones_like(time)
        acceleration = np.gradient(speed_transient * 2*np.pi/60) / np.gradient(time)

        # Calculate stresses
        shaft_stress = []
        bearing_load = []
        inertial_torque = []

        for i in range(len(time)):
            stress = self.mechanical_stress_analysis(torque_transient[i], speed_transient[i], acceleration[i])
            shaft_stress.append(stress['shaft_stress'] / 1e6)  # MPa
            bearing_load.append(stress['bearing_load'] / 1000)  # kN
            inertial_torque.append(stress['inertial_torque'])

        # Clear figure
        self.multi_fig.clear()

        # Create subplots
        ax1 = self.multi_fig.add_subplot(2, 2, 1)
        ax2 = self.multi_fig.add_subplot(2, 2, 2)
        ax3 = self.multi_fig.add_subplot(2, 2, 3)
        ax4 = self.multi_fig.add_subplot(2, 2, 4)

        # Torque
        ax1.plot(time, torque_transient, 'b-', linewidth=2)
        ax1.set_ylabel('Torque (Nm)', fontsize=9)
        ax1.set_title('Shaft Torque', fontsize=10, fontweight='bold')
        ax1.grid(True, alpha=0.3)

        # Shaft stress
        ax2.plot(time, shaft_stress, 'r-', linewidth=2)
        ax2.set_ylabel('Stress (MPa)', fontsize=9)
        ax2.set_title('Shaft Shear Stress', fontsize=10, fontweight='bold')
        ax2.grid(True, alpha=0.3)

        # Bearing load
        ax3.plot(time, bearing_load, 'g-', linewidth=2)
        ax3.set_xlabel('Time (s)', fontsize=9)
        ax3.set_ylabel('Load (kN)', fontsize=9)
        ax3.set_title('Bearing Radial Load', fontsize=10, fontweight='bold')
        ax3.grid(True, alpha=0.3)

        # Inertial torque
        ax4.plot(time, inertial_torque, 'm-', linewidth=2)
        ax4.set_xlabel('Time (s)', fontsize=9)
        ax4.set_ylabel('Torque (Nm)', fontsize=9)
        ax4.set_title('Inertial Torque', fontsize=10, fontweight='bold')
        ax4.grid(True, alpha=0.3)

        self.multi_fig.tight_layout()
        self.multi_canvas.draw()

    def update_control_mode(self, event=None):
        """Update control mode"""
        self.control['control_mode'] = self.control_mode_var.get()

    def update_speed_reference(self, value):
        """Update speed reference"""
        self.control['speed_reference'] = float(value)
        self.speed_ref_label.config(text=f"{float(value):.0f}")

    def update_torque_limit(self, value):
        """Update torque limit"""
        self.control['torque_limit'] = float(value)
        self.torque_limit_label.config(text=f"{float(value):.0f}")

    def update_current_limit(self, value):
        """Update current limit"""
        self.control['current_limit'] = float(value)
        self.current_limit_label.config(text=f"{float(value):.0f}")

    def update_thermal_settings(self):
        """Update thermal settings"""
        try:
            self.thermal['temp_ambient'] = float(self.ambient_temp_var.get())
            self.thermal['temp_rated'] = float(self.rated_temp_var.get())
            self.thermal['temp_max'] = float(self.max_temp_var.get())
            messagebox.showinfo("Success", "Thermal settings updated!")
        except ValueError:
            messagebox.showerror("Error", "Invalid thermal parameters")

    def calculate_and_display_economics(self):
        """Calculate and display economic analysis"""
        try:
            # Update economic parameters
            self.economics['electricity_cost'] = float(self.economic_widgets['electricity_cost'].get())
            self.economics['maintenance_cost'] = float(self.economic_widgets['maintenance_cost'].get())
            self.economics['operating_hours'] = float(self.economic_widgets['operating_hours'].get())
            self.economics['motor_cost'] = float(self.economic_widgets['motor_cost'].get())
            self.economics['lifespan'] = float(self.economic_widgets['lifespan'].get())

            # Calculate economics
            econ = self.calculate_economics()

            output = f"""
{'='*60}
ECONOMIC ANALYSIS
{'='*60}

Input Parameters:
  Electricity Cost:      ${self.economics['electricity_cost']:.3f} /kWh
  Maintenance Cost:      ${self.economics['maintenance_cost']:.2f} /year
  Operating Hours:       {self.economics['operating_hours']:.0f} hours/year
  Motor Initial Cost:    ${self.economics['motor_cost']:.2f}
  Expected Lifespan:     {self.economics['lifespan']:.0f} years

Annual Operating Costs:
  Energy Consumption:    {econ['annual_energy']:.2f} kWh/year
  Energy Cost:           ${econ['annual_energy_cost']:.2f} /year
  Maintenance Cost:      ${self.economics['maintenance_cost']:.2f} /year
  Total Annual Cost:     ${econ['annual_total_cost']:.2f} /year

Lifetime Costs:
  Total Energy Cost:     ${econ['annual_energy_cost'] * self.economics['lifespan']:.2f}
  Total Maintenance:     ${self.economics['maintenance_cost'] * self.economics['lifespan']:.2f}
  Initial Investment:    ${self.economics['motor_cost']:.2f}
  Total Lifetime Cost:   ${econ['lifetime_cost']:.2f}

Cost per Operating Hour: ${econ['lifetime_cost'] / (self.economics['operating_hours'] * self.economics['lifespan']):.4f} /hour

Efficiency Improvement Analysis:
  Current Efficiency:    {self.params['efficiency']*100:.1f}%
  Target Efficiency:     92.0%
  Annual Savings:        ${econ['annual_savings']:.2f} /year
  Payback Period:        {econ['payback_period']:.2f} years

Environmental Impact:
  Annual CO2 Emissions:  {econ['annual_energy'] * 0.5:.2f} kg CO2/year
  (assuming 0.5 kg CO2/kWh)

Recommendations:
  • Consider high-efficiency motor upgrade if payback < 3 years
  • Regular maintenance extends motor life and maintains efficiency
  • Variable frequency drives can reduce energy consumption by 20-30%
  • Monitor power factor and correct if necessary

{'='*60}
"""

            self.economic_text.delete(1.0, tk.END)
            self.economic_text.insert(1.0, output)

        except Exception as e:
            messagebox.showerror("Error", f"Economic calculation error: {e}")

    def on_resize(self, event):
        """Handle window resize for auto-scaling"""
        # Only handle main window resize
        if event.widget == self.root:
            pass  # Auto-scaling is handled by pack geometry manager


def main():
    """Main application entry point"""
    root = tk.Tk()
    app = InductionMotorPluggingLab(root)
    root.mainloop()


if __name__ == "__main__":
    main()
