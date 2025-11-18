#!/usr/bin/env python3
"""
Advanced Induction Motor Analysis Lab
Multi-Physics Simulation with Electromagnetic-Thermal Coupling

Features:
- Complete solution to induction motor problem
- Dynamic ODE simulation (RK45, Euler methods)
- Real-time visualization
- Multi-physics coupling (electromagnetic, thermal, mechanical)
- Economic analysis
- Detailed loss breakdown
- Advanced controls
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import solve_ivp, odeint
import threading
import time
from dataclasses import dataclass
from typing import Tuple, List, Dict
import json


@dataclass
class MotorParameters:
    """Motor parameters data class"""
    V_line: float = 460.0  # Line-to-line voltage (V)
    I_line: float = 25.0   # Line current (A)
    f: float = 50.0        # Frequency (Hz)
    poles: int = 4         # Number of poles
    pf: float = 0.85       # Power factor
    P_stator_loss: float = 1000.0  # Stator winding loss (W)
    P_rotor_loss: float = 500.0    # Rotor winding loss (W)
    P_rotational: float = 250.0    # Rotational losses (W)
    P_core: float = 800.0          # Core loss (W)
    P_stray: float = 200.0         # Stray load loss (W)

    # Additional parameters for advanced simulation
    R1: float = 0.5        # Stator resistance per phase (Ω)
    R2: float = 0.3        # Rotor resistance per phase (Ω)
    X1: float = 1.2        # Stator reactance per phase (Ω)
    X2: float = 1.0        # Rotor reactance per phase (Ω)
    Xm: float = 50.0       # Magnetizing reactance (Ω)
    J: float = 0.5         # Moment of inertia (kg·m²)

    # Thermal parameters
    thermal_resistance: float = 0.5  # K/W
    thermal_capacitance: float = 1000.0  # J/K
    ambient_temp: float = 25.0  # °C
    max_temp: float = 155.0     # °C (Class F insulation)

    # Economic parameters
    electricity_cost: float = 0.12  # $/kWh
    maintenance_cost_per_hour: float = 2.0  # $/hour


class InductionMotorCalculator:
    """Core calculation engine for induction motor analysis"""

    def __init__(self, params: MotorParameters):
        self.params = params
        self.results = {}

    def calculate_basic_parameters(self) -> Dict:
        """Calculate basic motor parameters (Problem 5 solution)"""
        p = self.params

        # Input power (3-phase)
        P_in = np.sqrt(3) * p.V_line * p.I_line * p.pf

        # (a) Electromagnetic (air gap) power
        P_elm = P_in - p.P_stator_loss - p.P_core - p.P_stray

        # (b) Mechanical power
        P_m = P_elm - p.P_rotor_loss

        # (c) Output power
        P_out = P_m - p.P_rotational

        # (d) Efficiency
        eta = P_out / P_in if P_in > 0 else 0

        # (e) Slip and operating speed
        # Rotor copper loss = slip * air gap power
        s = p.P_rotor_loss / P_elm if P_elm > 0 else 0
        n_s = 120 * p.f / p.poles  # Synchronous speed (rpm)
        n = n_s * (1 - s)  # Operating speed (rpm)

        # (f) Electromagnetic torque
        omega_s = 2 * np.pi * p.f / (p.poles / 2)  # Synchronous speed (rad/s)
        T_elm = P_elm / omega_s if omega_s > 0 else 0

        # (g) Shaft (output) torque
        omega = 2 * np.pi * n / 60  # Operating speed (rad/s)
        T_out = P_out / omega if omega > 0 else 0

        results = {
            'P_in': P_in,
            'P_elm': P_elm,
            'P_m': P_m,
            'P_out': P_out,
            'eta': eta,
            's': s,
            'n_s': n_s,
            'n': n,
            'omega_s': omega_s,
            'omega': omega,
            'T_elm': T_elm,
            'T_out': T_out
        }

        self.results = results
        return results

    def calculate_losses_breakdown(self) -> Dict:
        """Calculate detailed loss breakdown"""
        p = self.params

        total_losses = (p.P_stator_loss + p.P_rotor_loss + p.P_rotational +
                       p.P_core + p.P_stray)

        return {
            'Stator Copper Loss': p.P_stator_loss,
            'Rotor Copper Loss': p.P_rotor_loss,
            'Core Loss': p.P_core,
            'Rotational Loss': p.P_rotational,
            'Stray Load Loss': p.P_stray,
            'Total Losses': total_losses
        }

    def calculate_thermal_steady_state(self) -> float:
        """Calculate steady-state temperature"""
        losses = self.calculate_losses_breakdown()
        total_losses = losses['Total Losses']

        # Steady-state temperature rise
        delta_T = total_losses * self.params.thermal_resistance
        T_steady = self.params.ambient_temp + delta_T

        return T_steady

    def calculate_economic_analysis(self, operating_hours: float = 1.0) -> Dict:
        """Calculate economic parameters"""
        if not self.results:
            self.calculate_basic_parameters()

        P_in_kW = self.results['P_in'] / 1000.0
        P_out_kW = self.results['P_out'] / 1000.0

        energy_cost = P_in_kW * operating_hours * self.params.electricity_cost
        maintenance_cost = operating_hours * self.params.maintenance_cost_per_hour
        total_cost = energy_cost + maintenance_cost

        # Cost per kWh output
        cost_per_kwh_out = total_cost / (P_out_kW * operating_hours) if P_out_kW > 0 else 0

        return {
            'energy_consumption_kwh': P_in_kW * operating_hours,
            'energy_cost': energy_cost,
            'maintenance_cost': maintenance_cost,
            'total_cost': total_cost,
            'cost_per_kwh_output': cost_per_kwh_out
        }


class DynamicSimulator:
    """Dynamic simulation engine with ODE solvers"""

    def __init__(self, params: MotorParameters):
        self.params = params
        self.time_history = []
        self.state_history = []
        self.solver_method = 'RK45'

    def motor_dynamics_ode(self, t: float, y: np.ndarray, T_load: float) -> np.ndarray:
        """
        Induction motor dynamic equations
        State vector y = [i_d, i_q, omega, theta]
        where i_d, i_q are d-q axis currents, omega is rotor speed, theta is rotor angle
        """
        p = self.params

        i_d, i_q, omega, theta = y

        # Synchronous speed
        omega_s = 2 * np.pi * p.f / (p.poles / 2)

        # Slip frequency
        omega_slip = omega_s - omega

        # Voltage in d-q frame (RMS values converted to peak)
        V_peak = p.V_line * np.sqrt(2) / np.sqrt(3)
        v_d = V_peak * np.cos(2 * np.pi * p.f * t)
        v_q = V_peak * np.sin(2 * np.pi * p.f * t)

        # Electrical equations (simplified d-q model)
        L_s = (p.X1 + p.Xm) / (2 * np.pi * p.f)  # Stator inductance
        L_r = (p.X2 + p.Xm) / (2 * np.pi * p.f)  # Rotor inductance
        L_m = p.Xm / (2 * np.pi * p.f)           # Mutual inductance

        # Current derivatives (simplified)
        di_d_dt = (v_d - p.R1 * i_d + omega_slip * L_s * i_q) / L_s
        di_q_dt = (v_q - p.R1 * i_q - omega_slip * L_s * i_d) / L_s

        # Electromagnetic torque (simplified)
        T_elm = (3 / 2) * (p.poles / 2) * L_m * (i_d * i_q)

        # Mechanical equation
        domega_dt = (T_elm - T_load) / p.J if p.J > 0 else 0

        # Rotor angle
        dtheta_dt = omega

        return np.array([di_d_dt, di_q_dt, domega_dt, dtheta_dt])

    def thermal_dynamics_ode(self, t: float, T: float, P_loss: float) -> float:
        """
        Thermal dynamics equation
        dT/dt = (P_loss - (T - T_ambient) / R_th) / C_th
        """
        p = self.params
        dT_dt = (P_loss - (T - p.ambient_temp) / p.thermal_resistance) / p.thermal_capacitance
        return dT_dt

    def coupled_electromagnetic_thermal_ode(self, t: float, y: np.ndarray, T_load: float) -> np.ndarray:
        """
        Coupled electromagnetic-thermal dynamics
        State vector y = [i_d, i_q, omega, theta, T]
        """
        i_d, i_q, omega, theta, T = y

        # Electromagnetic dynamics
        electrical_states = self.motor_dynamics_ode(t, np.array([i_d, i_q, omega, theta]), T_load)

        # Calculate losses for thermal model
        P_copper = self.params.R1 * (i_d**2 + i_q**2) * 3 / 2
        P_loss_total = P_copper + self.params.P_core + self.params.P_rotational

        # Thermal dynamics
        dT_dt = self.thermal_dynamics_ode(t, T, P_loss_total)

        return np.array([electrical_states[0], electrical_states[1],
                        electrical_states[2], electrical_states[3], dT_dt])

    def simulate_rk45(self, t_span: Tuple[float, float], y0: np.ndarray,
                     T_load: float, coupled: bool = True) -> Dict:
        """Simulate using RK45 method"""
        if coupled:
            ode_func = lambda t, y: self.coupled_electromagnetic_thermal_ode(t, y, T_load)
        else:
            ode_func = lambda t, y: self.motor_dynamics_ode(t, y, T_load)

        sol = solve_ivp(ode_func, t_span, y0, method='RK45',
                       dense_output=True, max_step=0.001)

        return {
            't': sol.t,
            'y': sol.y,
            'success': sol.success
        }

    def simulate_euler(self, t_span: Tuple[float, float], y0: np.ndarray,
                      T_load: float, dt: float = 0.001, coupled: bool = True) -> Dict:
        """Simulate using Euler method"""
        t_start, t_end = t_span
        t = np.arange(t_start, t_end, dt)
        n_steps = len(t)
        n_states = len(y0)

        y = np.zeros((n_states, n_steps))
        y[:, 0] = y0

        if coupled:
            ode_func = lambda t, y: self.coupled_electromagnetic_thermal_ode(t, y, T_load)
        else:
            ode_func = lambda t, y: self.motor_dynamics_ode(t, y, T_load)

        for i in range(1, n_steps):
            dy = ode_func(t[i-1], y[:, i-1])
            y[:, i] = y[:, i-1] + dy * dt

        return {
            't': t,
            'y': y,
            'success': True
        }


class AdvancedInductionMotorLab(tk.Tk):
    """Main application window with comprehensive GUI"""

    def __init__(self):
        super().__init__()

        self.title("Advanced Induction Motor Analysis Lab - Multi-Physics Simulation")
        self.geometry("1400x900")

        # Initialize parameters
        self.params = MotorParameters()
        self.calculator = InductionMotorCalculator(self.params)
        self.simulator = DynamicSimulator(self.params)

        # Simulation state
        self.simulation_running = False
        self.simulation_thread = None
        self.simulation_data = {
            'time': [],
            'speed': [],
            'torque': [],
            'current': [],
            'temperature': [],
            'power': []
        }

        # Configure window resizing
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        # Bind resize event
        self.bind('<Configure>', self.on_window_resize)

        # Create UI
        self.create_widgets()
        self.update_calculations()

    def create_widgets(self):
        """Create all GUI widgets"""
        # Create notebook (tabs)
        self.notebook = ttk.Notebook(self)
        self.notebook.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        # Create tabs
        self.create_main_tab()
        self.create_dynamic_simulation_tab()
        self.create_multi_physics_tab()
        self.create_economic_analysis_tab()
        self.create_loss_breakdown_tab()
        self.create_control_tab()

    def create_main_tab(self):
        """Main calculation and input tab"""
        main_frame = ttk.Frame(self.notebook)
        self.notebook.add(main_frame, text="Main Analysis")

        # Configure grid
        main_frame.grid_rowconfigure(1, weight=1)
        main_frame.grid_columnconfigure(0, weight=1)
        main_frame.grid_columnconfigure(1, weight=2)

        # Left panel - Input parameters
        input_frame = ttk.LabelFrame(main_frame, text="Input Parameters", padding=10)
        input_frame.grid(row=0, column=0, rowspan=2, sticky='nsew', padx=5, pady=5)

        # Parameter inputs with sliders
        self.param_vars = {}
        self.param_sliders = {}

        param_configs = [
            ('V_line', 'Line Voltage (V)', 100, 1000, 460),
            ('I_line', 'Line Current (A)', 1, 100, 25),
            ('f', 'Frequency (Hz)', 40, 70, 50),
            ('pf', 'Power Factor', 0.5, 1.0, 0.85),
            ('P_stator_loss', 'Stator Loss (W)', 0, 5000, 1000),
            ('P_rotor_loss', 'Rotor Loss (W)', 0, 5000, 500),
            ('P_rotational', 'Rotational Loss (W)', 0, 2000, 250),
            ('P_core', 'Core Loss (W)', 0, 5000, 800),
            ('P_stray', 'Stray Loss (W)', 0, 2000, 200),
        ]

        for i, (param, label, min_val, max_val, default) in enumerate(param_configs):
            frame = ttk.Frame(input_frame)
            frame.grid(row=i, column=0, sticky='ew', pady=3)

            ttk.Label(frame, text=label, width=20).pack(side=tk.LEFT)

            var = tk.DoubleVar(value=default)
            self.param_vars[param] = var

            slider = ttk.Scale(frame, from_=min_val, to=max_val,
                             variable=var, orient=tk.HORIZONTAL,
                             command=lambda v, p=param: self.on_param_change(p))
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            self.param_sliders[param] = slider

            entry = ttk.Entry(frame, textvariable=var, width=10)
            entry.pack(side=tk.LEFT)
            entry.bind('<Return>', lambda e, p=param: self.on_param_change(p))

        # Poles selector
        poles_frame = ttk.Frame(input_frame)
        poles_frame.grid(row=len(param_configs), column=0, sticky='ew', pady=3)
        ttk.Label(poles_frame, text="Number of Poles", width=20).pack(side=tk.LEFT)
        self.poles_var = tk.IntVar(value=4)
        poles_combo = ttk.Combobox(poles_frame, textvariable=self.poles_var,
                                   values=[2, 4, 6, 8, 10, 12], state='readonly', width=10)
        poles_combo.pack(side=tk.LEFT, padx=5)
        poles_combo.bind('<<ComboboxSelected>>', lambda e: self.on_param_change('poles'))

        # Calculate button
        ttk.Button(input_frame, text="Calculate",
                  command=self.update_calculations).grid(row=len(param_configs)+1,
                                                         column=0, pady=10)

        # Right panel - Results
        results_frame = ttk.LabelFrame(main_frame, text="Calculation Results", padding=10)
        results_frame.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)

        # Results text widget
        self.results_text = tk.Text(results_frame, height=20, width=60,
                                   font=('Courier', 10), wrap=tk.WORD)
        self.results_text.pack(fill=tk.BOTH, expand=True)

        scrollbar = ttk.Scrollbar(results_frame, command=self.results_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.results_text.config(yscrollcommand=scrollbar.set)

        # Visualization panel
        viz_frame = ttk.LabelFrame(main_frame, text="Power Flow Diagram", padding=10)
        viz_frame.grid(row=1, column=1, sticky='nsew', padx=5, pady=5)

        # Create matplotlib figure
        self.main_fig = Figure(figsize=(8, 4), dpi=100)
        self.main_canvas = FigureCanvasTkAgg(self.main_fig, master=viz_frame)
        self.main_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def create_dynamic_simulation_tab(self):
        """Dynamic simulation tab with ODE solvers"""
        sim_frame = ttk.Frame(self.notebook)
        self.notebook.add(sim_frame, text="Dynamic Simulation")

        sim_frame.grid_rowconfigure(1, weight=1)
        sim_frame.grid_columnconfigure(0, weight=1)

        # Control panel
        control_frame = ttk.LabelFrame(sim_frame, text="Simulation Controls", padding=10)
        control_frame.grid(row=0, column=0, sticky='ew', padx=5, pady=5)

        # Solver selection
        ttk.Label(control_frame, text="ODE Solver:").grid(row=0, column=0, padx=5)
        self.solver_var = tk.StringVar(value='RK45')
        solver_combo = ttk.Combobox(control_frame, textvariable=self.solver_var,
                                    values=['RK45', 'Euler'], state='readonly', width=10)
        solver_combo.grid(row=0, column=1, padx=5)

        # Load torque
        ttk.Label(control_frame, text="Load Torque (Nm):").grid(row=0, column=2, padx=5)
        self.load_torque_var = tk.DoubleVar(value=50.0)
        ttk.Entry(control_frame, textvariable=self.load_torque_var, width=10).grid(row=0, column=3, padx=5)

        # Simulation time
        ttk.Label(control_frame, text="Simulation Time (s):").grid(row=0, column=4, padx=5)
        self.sim_time_var = tk.DoubleVar(value=2.0)
        ttk.Entry(control_frame, textvariable=self.sim_time_var, width=10).grid(row=0, column=5, padx=5)

        # Control buttons
        self.start_btn = ttk.Button(control_frame, text="Start",
                                    command=self.start_simulation, style='Success.TButton')
        self.start_btn.grid(row=1, column=0, columnspan=2, padx=5, pady=5)

        self.stop_btn = ttk.Button(control_frame, text="Stop",
                                   command=self.stop_simulation, state='disabled')
        self.stop_btn.grid(row=1, column=2, columnspan=2, padx=5, pady=5)

        self.reset_btn = ttk.Button(control_frame, text="Reset",
                                    command=self.reset_simulation)
        self.reset_btn.grid(row=1, column=4, columnspan=2, padx=5, pady=5)

        # Graphs frame
        graphs_frame = ttk.LabelFrame(sim_frame, text="Real-Time Visualization", padding=10)
        graphs_frame.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)

        # Create matplotlib figure for dynamic plots
        self.sim_fig = Figure(figsize=(12, 8), dpi=100)
        self.sim_canvas = FigureCanvasTkAgg(self.sim_fig, master=graphs_frame)
        self.sim_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Create subplots
        self.ax_speed = self.sim_fig.add_subplot(2, 2, 1)
        self.ax_torque = self.sim_fig.add_subplot(2, 2, 2)
        self.ax_current = self.sim_fig.add_subplot(2, 2, 3)
        self.ax_power = self.sim_fig.add_subplot(2, 2, 4)

        self.sim_fig.tight_layout()

    def create_multi_physics_tab(self):
        """Multi-physics simulation tab (electromagnetic-thermal-mechanical)"""
        mp_frame = ttk.Frame(self.notebook)
        self.notebook.add(mp_frame, text="Multi-Physics Analysis")

        mp_frame.grid_rowconfigure(1, weight=1)
        mp_frame.grid_columnconfigure(0, weight=1)

        # Parameters frame
        params_frame = ttk.LabelFrame(mp_frame, text="Thermal & Mechanical Parameters", padding=10)
        params_frame.grid(row=0, column=0, sticky='ew', padx=5, pady=5)

        # Thermal parameters
        ttk.Label(params_frame, text="Thermal Resistance (K/W):").grid(row=0, column=0, padx=5)
        self.thermal_r_var = tk.DoubleVar(value=0.5)
        ttk.Entry(params_frame, textvariable=self.thermal_r_var, width=10).grid(row=0, column=1, padx=5)

        ttk.Label(params_frame, text="Thermal Capacitance (J/K):").grid(row=0, column=2, padx=5)
        self.thermal_c_var = tk.DoubleVar(value=1000.0)
        ttk.Entry(params_frame, textvariable=self.thermal_c_var, width=10).grid(row=0, column=3, padx=5)

        ttk.Label(params_frame, text="Ambient Temp (°C):").grid(row=0, column=4, padx=5)
        self.ambient_temp_var = tk.DoubleVar(value=25.0)
        ttk.Entry(params_frame, textvariable=self.ambient_temp_var, width=10).grid(row=0, column=5, padx=5)

        # Mechanical parameters
        ttk.Label(params_frame, text="Moment of Inertia (kg·m²):").grid(row=1, column=0, padx=5)
        self.inertia_var = tk.DoubleVar(value=0.5)
        ttk.Entry(params_frame, textvariable=self.inertia_var, width=10).grid(row=1, column=1, padx=5)

        # Run coupled simulation button
        ttk.Button(params_frame, text="Run Coupled Simulation",
                  command=self.run_coupled_simulation).grid(row=2, column=0, columnspan=6, pady=10)

        # Results frame
        mp_results_frame = ttk.LabelFrame(mp_frame, text="Coupled Analysis Results", padding=10)
        mp_results_frame.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)

        # Create matplotlib figure
        self.mp_fig = Figure(figsize=(12, 8), dpi=100)
        self.mp_canvas = FigureCanvasTkAgg(self.mp_fig, master=mp_results_frame)
        self.mp_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def create_economic_analysis_tab(self):
        """Economic analysis tab"""
        econ_frame = ttk.Frame(self.notebook)
        self.notebook.add(econ_frame, text="Economic Analysis")

        econ_frame.grid_rowconfigure(1, weight=1)
        econ_frame.grid_columnconfigure(0, weight=1)

        # Input frame
        input_frame = ttk.LabelFrame(econ_frame, text="Economic Parameters", padding=10)
        input_frame.grid(row=0, column=0, sticky='ew', padx=5, pady=5)

        ttk.Label(input_frame, text="Operating Hours:").grid(row=0, column=0, padx=5)
        self.op_hours_var = tk.DoubleVar(value=1000.0)
        ttk.Entry(input_frame, textvariable=self.op_hours_var, width=15).grid(row=0, column=1, padx=5)

        ttk.Label(input_frame, text="Electricity Cost ($/kWh):").grid(row=0, column=2, padx=5)
        self.elec_cost_var = tk.DoubleVar(value=0.12)
        ttk.Entry(input_frame, textvariable=self.elec_cost_var, width=15).grid(row=0, column=3, padx=5)

        ttk.Label(input_frame, text="Maintenance Cost ($/hr):").grid(row=0, column=4, padx=5)
        self.maint_cost_var = tk.DoubleVar(value=2.0)
        ttk.Entry(input_frame, textvariable=self.maint_cost_var, width=15).grid(row=0, column=5, padx=5)

        ttk.Button(input_frame, text="Calculate Economics",
                  command=self.update_economic_analysis).grid(row=1, column=0, columnspan=6, pady=10)

        # Results frame
        econ_results_frame = ttk.LabelFrame(econ_frame, text="Economic Analysis Results", padding=10)
        econ_results_frame.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)

        # Split into text and visualization
        text_frame = ttk.Frame(econ_results_frame)
        text_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.econ_text = tk.Text(text_frame, height=15, width=50, font=('Courier', 10))
        self.econ_text.pack(fill=tk.BOTH, expand=True)

        # Economic visualization
        viz_frame = ttk.Frame(econ_results_frame)
        viz_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.econ_fig = Figure(figsize=(6, 6), dpi=100)
        self.econ_canvas = FigureCanvasTkAgg(self.econ_fig, master=viz_frame)
        self.econ_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def create_loss_breakdown_tab(self):
        """Detailed loss breakdown tab"""
        loss_frame = ttk.Frame(self.notebook)
        self.notebook.add(loss_frame, text="Loss Breakdown")

        loss_frame.grid_rowconfigure(0, weight=1)
        loss_frame.grid_columnconfigure(0, weight=1)

        # Create matplotlib figure for loss visualization
        self.loss_fig = Figure(figsize=(12, 8), dpi=100)
        self.loss_canvas = FigureCanvasTkAgg(self.loss_fig, master=loss_frame)
        self.loss_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Update button
        ttk.Button(loss_frame, text="Update Loss Analysis",
                  command=self.update_loss_breakdown).pack(pady=5)

    def create_control_tab(self):
        """Advanced control tab"""
        control_frame = ttk.Frame(self.notebook)
        self.notebook.add(control_frame, text="Advanced Controls")

        control_frame.grid_rowconfigure(2, weight=1)
        control_frame.grid_columnconfigure(0, weight=1)

        # Control method selection
        method_frame = ttk.LabelFrame(control_frame, text="Control Method", padding=10)
        method_frame.grid(row=0, column=0, sticky='ew', padx=5, pady=5)

        self.control_method_var = tk.StringVar(value='V/f Control')
        methods = ['V/f Control', 'Vector Control', 'Direct Torque Control', 'Soft Start']

        for i, method in enumerate(methods):
            ttk.Radiobutton(method_frame, text=method, variable=self.control_method_var,
                           value=method).grid(row=0, column=i, padx=10)

        # Derating analysis
        derating_frame = ttk.LabelFrame(control_frame, text="Thermal Derating", padding=10)
        derating_frame.grid(row=1, column=0, sticky='ew', padx=5, pady=5)

        ttk.Label(derating_frame, text="Operating Temperature (°C):").grid(row=0, column=0, padx=5)
        self.operating_temp_var = tk.DoubleVar(value=75.0)
        ttk.Scale(derating_frame, from_=25, to=155, variable=self.operating_temp_var,
                 orient=tk.HORIZONTAL, length=300).grid(row=0, column=1, padx=5)
        ttk.Label(derating_frame, textvariable=self.operating_temp_var).grid(row=0, column=2, padx=5)

        self.derating_label = ttk.Label(derating_frame, text="Derating Factor: 100%",
                                       font=('Arial', 12, 'bold'))
        self.derating_label.grid(row=1, column=0, columnspan=3, pady=10)

        ttk.Button(derating_frame, text="Calculate Derating",
                  command=self.calculate_derating).grid(row=2, column=0, columnspan=3, pady=5)

        # Control visualization
        control_viz_frame = ttk.LabelFrame(control_frame, text="Control Response", padding=10)
        control_viz_frame.grid(row=2, column=0, sticky='nsew', padx=5, pady=5)

        self.control_fig = Figure(figsize=(12, 6), dpi=100)
        self.control_canvas = FigureCanvasTkAgg(self.control_fig, master=control_viz_frame)
        self.control_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def on_param_change(self, param_name):
        """Handle parameter change"""
        # Update parameter object
        if param_name in self.param_vars:
            setattr(self.params, param_name, self.param_vars[param_name].get())
        elif param_name == 'poles':
            self.params.poles = self.poles_var.get()

        # Recreate calculator and simulator
        self.calculator = InductionMotorCalculator(self.params)
        self.simulator = DynamicSimulator(self.params)

    def update_calculations(self):
        """Update all calculations and displays"""
        # Sync parameters
        for param, var in self.param_vars.items():
            setattr(self.params, param, var.get())
        self.params.poles = self.poles_var.get()

        # Recreate calculator
        self.calculator = InductionMotorCalculator(self.params)
        results = self.calculator.calculate_basic_parameters()

        # Display results
        self.results_text.delete('1.0', tk.END)

        output = "=" * 60 + "\n"
        output += "INDUCTION MOTOR ANALYSIS - PROBLEM 5 SOLUTION\n"
        output += "=" * 60 + "\n\n"

        output += "INPUT PARAMETERS:\n"
        output += "-" * 60 + "\n"
        output += f"Line Voltage (V_line):           {self.params.V_line:.2f} V\n"
        output += f"Line Current (I_line):           {self.params.I_line:.2f} A\n"
        output += f"Frequency (f):                   {self.params.f:.2f} Hz\n"
        output += f"Number of Poles:                 {self.params.poles}\n"
        output += f"Power Factor (pf):               {self.params.pf:.3f} lagging\n"
        output += f"Stator Winding Loss (ΔP1w):     {self.params.P_stator_loss:.2f} W\n"
        output += f"Rotor Winding Loss (ΔP2w):      {self.params.P_rotor_loss:.2f} W\n"
        output += f"Rotational Losses (ΔProt):      {self.params.P_rotational:.2f} W\n"
        output += f"Core Loss (ΔPFe):               {self.params.P_core:.2f} W\n"
        output += f"Stray Load Loss (ΔPstr):        {self.params.P_stray:.2f} W\n"

        output += "\n" + "=" * 60 + "\n"
        output += "CALCULATED RESULTS:\n"
        output += "=" * 60 + "\n\n"

        output += f"(a) Electromagnetic Power (P_elm):     {results['P_elm']:.2f} W\n"
        output += f"                                       {results['P_elm']/1000:.2f} kW\n\n"

        output += f"(b) Mechanical Power (P_m):            {results['P_m']:.2f} W\n"
        output += f"                                       {results['P_m']/1000:.2f} kW\n\n"

        output += f"(c) Output Power (P_out):              {results['P_out']:.2f} W\n"
        output += f"                                       {results['P_out']/1000:.2f} kW\n\n"

        output += f"(d) Efficiency (η):                    {results['eta']*100:.2f} %\n\n"

        output += f"(e) Slip (s):                          {results['s']:.4f}\n"
        output += f"                                       {results['s']*100:.2f} %\n"
        output += f"    Synchronous Speed (n_s):           {results['n_s']:.2f} rpm\n"
        output += f"    Operating Speed (n):               {results['n']:.2f} rpm\n\n"

        output += f"(f) Electromagnetic Torque (T_elm):    {results['T_elm']:.2f} N·m\n\n"

        output += f"(g) Shaft (Output) Torque (T_out):     {results['T_out']:.2f} N·m\n\n"

        output += "=" * 60 + "\n"
        output += "ADDITIONAL INFORMATION:\n"
        output += "=" * 60 + "\n\n"

        output += f"Input Power (P_in):                    {results['P_in']:.2f} W\n"
        output += f"                                       {results['P_in']/1000:.2f} kW\n"
        output += f"Total Losses:                          {results['P_in'] - results['P_out']:.2f} W\n"
        output += f"Angular Velocity (ω):                  {results['omega']:.2f} rad/s\n"
        output += f"Synchronous Angular Velocity (ω_s):    {results['omega_s']:.2f} rad/s\n"

        # Calculate thermal steady-state
        T_steady = self.calculator.calculate_thermal_steady_state()
        output += f"\nSteady-State Temperature:              {T_steady:.2f} °C\n"

        self.results_text.insert('1.0', output)

        # Update power flow diagram
        self.plot_power_flow(results)

    def plot_power_flow(self, results):
        """Plot power flow diagram"""
        self.main_fig.clear()
        ax = self.main_fig.add_subplot(111)

        # Power values
        P_in = results['P_in'] / 1000  # Convert to kW
        P_elm = results['P_elm'] / 1000
        P_m = results['P_m'] / 1000
        P_out = results['P_out'] / 1000

        # Losses
        P_stator = self.params.P_stator_loss / 1000
        P_core = self.params.P_core / 1000
        P_stray = self.params.P_stray / 1000
        P_rotor = self.params.P_rotor_loss / 1000
        P_rot = self.params.P_rotational / 1000

        # Create power flow
        stages = ['Input\nPower', 'Air Gap\nPower', 'Mechanical\nPower', 'Output\nPower']
        powers = [P_in, P_elm, P_m, P_out]

        x_pos = np.arange(len(stages))
        bars = ax.bar(x_pos, powers, color=['#3498db', '#2ecc71', '#f39c12', '#e74c3c'],
                     alpha=0.7, edgecolor='black', linewidth=2)

        # Add value labels on bars
        for i, (bar, power) in enumerate(zip(bars, powers)):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{power:.2f} kW',
                   ha='center', va='bottom', fontweight='bold', fontsize=10)

        # Add loss annotations
        ax.annotate(f'Stator Loss\n{P_stator:.2f} kW',
                   xy=(0.5, (P_in + P_elm)/2), fontsize=8,
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        ax.annotate(f'Rotor Loss\n{P_rotor:.2f} kW',
                   xy=(1.5, (P_elm + P_m)/2), fontsize=8,
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        ax.annotate(f'Rot. Loss\n{P_rot:.2f} kW',
                   xy=(2.5, (P_m + P_out)/2), fontsize=8,
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        ax.set_xticks(x_pos)
        ax.set_xticklabels(stages, fontsize=10, fontweight='bold')
        ax.set_ylabel('Power (kW)', fontweight='bold', fontsize=11)
        ax.set_title('Induction Motor Power Flow Diagram', fontweight='bold', fontsize=12)
        ax.grid(True, alpha=0.3, linestyle='--')

        self.main_fig.tight_layout()
        self.main_canvas.draw()

    def start_simulation(self):
        """Start dynamic simulation"""
        if not self.simulation_running:
            self.simulation_running = True
            self.start_btn.config(state='disabled')
            self.stop_btn.config(state='normal')

            # Reset data
            self.simulation_data = {
                'time': [],
                'speed': [],
                'torque': [],
                'current': [],
                'temperature': [],
                'power': []
            }

            # Start simulation thread
            self.simulation_thread = threading.Thread(target=self.run_simulation, daemon=True)
            self.simulation_thread.start()

            # Start update loop
            self.update_simulation_plots()

    def stop_simulation(self):
        """Stop dynamic simulation"""
        self.simulation_running = False
        self.start_btn.config(state='normal')
        self.stop_btn.config(state='disabled')

    def reset_simulation(self):
        """Reset simulation"""
        self.stop_simulation()

        # Clear data
        self.simulation_data = {
            'time': [],
            'speed': [],
            'torque': [],
            'current': [],
            'temperature': [],
            'power': []
        }

        # Clear plots
        self.ax_speed.clear()
        self.ax_torque.clear()
        self.ax_current.clear()
        self.ax_power.clear()
        self.sim_canvas.draw()

    def run_simulation(self):
        """Run simulation in background thread"""
        # Update simulator parameters
        self.params.thermal_resistance = self.thermal_r_var.get()
        self.params.thermal_capacitance = self.thermal_c_var.get()
        self.params.ambient_temp = self.ambient_temp_var.get()
        self.params.J = self.inertia_var.get()

        self.simulator = DynamicSimulator(self.params)

        # Initial conditions [i_d, i_q, omega, theta, T]
        omega_s = 2 * np.pi * self.params.f / (self.params.poles / 2)
        y0 = np.array([0.0, 0.0, omega_s * 0.95, 0.0, self.params.ambient_temp])

        T_load = self.load_torque_var.get()
        sim_time = self.sim_time_var.get()
        solver = self.solver_var.get()

        # Run simulation
        if solver == 'RK45':
            result = self.simulator.simulate_rk45((0, sim_time), y0, T_load, coupled=True)
        else:
            result = self.simulator.simulate_euler((0, sim_time), y0, T_load, coupled=True)

        # Process results
        t = result['t']
        y = result['y']

        # Simulate in real-time
        start_time = time.time()
        dt = 0.05  # Update interval

        for i in range(len(t)):
            if not self.simulation_running:
                break

            # Wait to simulate real-time
            elapsed = time.time() - start_time
            target_time = t[i]
            if elapsed < target_time:
                time.sleep(min(dt, target_time - elapsed))

            # Extract data
            if y.ndim == 1:
                i_d, i_q, omega, theta, T = y
            else:
                i_d, i_q, omega, theta, T = y[:, i]

            # Calculate derived quantities
            speed_rpm = omega * 60 / (2 * np.pi)
            current_rms = np.sqrt(i_d**2 + i_q**2) / np.sqrt(2)
            torque = (3/2) * (self.params.poles/2) * (self.params.Xm / (2*np.pi*self.params.f)) * i_d * i_q
            power = torque * omega / 1000  # kW

            # Store data
            self.simulation_data['time'].append(t[i])
            self.simulation_data['speed'].append(speed_rpm)
            self.simulation_data['torque'].append(torque)
            self.simulation_data['current'].append(current_rms)
            self.simulation_data['temperature'].append(T)
            self.simulation_data['power'].append(power)

    def update_simulation_plots(self):
        """Update simulation plots"""
        if not self.simulation_running and len(self.simulation_data['time']) == 0:
            return

        if len(self.simulation_data['time']) > 0:
            # Speed plot
            self.ax_speed.clear()
            self.ax_speed.plot(self.simulation_data['time'],
                              self.simulation_data['speed'], 'b-', linewidth=2)
            self.ax_speed.set_xlabel('Time (s)', fontweight='bold')
            self.ax_speed.set_ylabel('Speed (rpm)', fontweight='bold')
            self.ax_speed.set_title('Rotor Speed', fontweight='bold')
            self.ax_speed.grid(True, alpha=0.3)

            # Torque plot
            self.ax_torque.clear()
            self.ax_torque.plot(self.simulation_data['time'],
                               self.simulation_data['torque'], 'r-', linewidth=2)
            self.ax_torque.set_xlabel('Time (s)', fontweight='bold')
            self.ax_torque.set_ylabel('Torque (N·m)', fontweight='bold')
            self.ax_torque.set_title('Electromagnetic Torque', fontweight='bold')
            self.ax_torque.grid(True, alpha=0.3)

            # Current plot
            self.ax_current.clear()
            self.ax_current.plot(self.simulation_data['time'],
                                self.simulation_data['current'], 'g-', linewidth=2)
            self.ax_current.set_xlabel('Time (s)', fontweight='bold')
            self.ax_current.set_ylabel('Current (A)', fontweight='bold')
            self.ax_current.set_title('Stator Current (RMS)', fontweight='bold')
            self.ax_current.grid(True, alpha=0.3)

            # Power plot
            self.ax_power.clear()
            self.ax_power.plot(self.simulation_data['time'],
                              self.simulation_data['power'], 'm-', linewidth=2)
            self.ax_power.set_xlabel('Time (s)', fontweight='bold')
            self.ax_power.set_ylabel('Power (kW)', fontweight='bold')
            self.ax_power.set_title('Mechanical Power', fontweight='bold')
            self.ax_power.grid(True, alpha=0.3)

            self.sim_fig.tight_layout()
            self.sim_canvas.draw()

        if self.simulation_running:
            self.after(100, self.update_simulation_plots)

    def run_coupled_simulation(self):
        """Run coupled electromagnetic-thermal-mechanical simulation"""
        # Update parameters
        self.params.thermal_resistance = self.thermal_r_var.get()
        self.params.thermal_capacitance = self.thermal_c_var.get()
        self.params.ambient_temp = self.ambient_temp_var.get()
        self.params.J = self.inertia_var.get()

        self.simulator = DynamicSimulator(self.params)

        # Initial conditions [i_d, i_q, omega, theta, T]
        omega_s = 2 * np.pi * self.params.f / (self.params.poles / 2)
        y0 = np.array([0.0, 0.0, omega_s * 0.95, 0.0, self.params.ambient_temp])

        T_load = 50.0  # Example load torque
        sim_time = 5.0

        # Run simulation
        result = self.simulator.simulate_rk45((0, sim_time), y0, T_load, coupled=True)

        # Plot results
        self.mp_fig.clear()

        t = result['t']
        y = result['y']

        # Extract variables
        i_d, i_q, omega, theta, T = y

        speed_rpm = omega * 60 / (2 * np.pi)
        current = np.sqrt(i_d**2 + i_q**2) / np.sqrt(2)
        torque = (3/2) * (self.params.poles/2) * (self.params.Xm / (2*np.pi*self.params.f)) * i_d * i_q

        # Create subplots
        ax1 = self.mp_fig.add_subplot(2, 2, 1)
        ax1.plot(t, speed_rpm, 'b-', linewidth=2)
        ax1.set_xlabel('Time (s)', fontweight='bold')
        ax1.set_ylabel('Speed (rpm)', fontweight='bold')
        ax1.set_title('Rotor Speed (Mechanical)', fontweight='bold')
        ax1.grid(True, alpha=0.3)

        ax2 = self.mp_fig.add_subplot(2, 2, 2)
        ax2.plot(t, current, 'g-', linewidth=2)
        ax2.set_xlabel('Time (s)', fontweight='bold')
        ax2.set_ylabel('Current (A)', fontweight='bold')
        ax2.set_title('Stator Current (Electromagnetic)', fontweight='bold')
        ax2.grid(True, alpha=0.3)

        ax3 = self.mp_fig.add_subplot(2, 2, 3)
        ax3.plot(t, T, 'r-', linewidth=2)
        ax3.axhline(y=self.params.max_temp, color='r', linestyle='--',
                   label=f'Max Temp ({self.params.max_temp}°C)')
        ax3.set_xlabel('Time (s)', fontweight='bold')
        ax3.set_ylabel('Temperature (°C)', fontweight='bold')
        ax3.set_title('Winding Temperature (Thermal)', fontweight='bold')
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        ax4 = self.mp_fig.add_subplot(2, 2, 4)
        ax4.plot(t, torque, 'm-', linewidth=2)
        ax4.set_xlabel('Time (s)', fontweight='bold')
        ax4.set_ylabel('Torque (N·m)', fontweight='bold')
        ax4.set_title('Electromagnetic Torque', fontweight='bold')
        ax4.grid(True, alpha=0.3)

        self.mp_fig.tight_layout()
        self.mp_canvas.draw()

        messagebox.showinfo("Coupled Simulation Complete",
                          f"Multi-physics simulation completed successfully!\n\n"
                          f"Final Temperature: {T[-1]:.2f}°C\n"
                          f"Final Speed: {speed_rpm[-1]:.2f} rpm\n"
                          f"Final Torque: {torque[-1]:.2f} N·m")

    def update_economic_analysis(self):
        """Update economic analysis"""
        # Update parameters
        self.params.electricity_cost = self.elec_cost_var.get()
        self.params.maintenance_cost_per_hour = self.maint_cost_var.get()

        # Recalculate
        self.calculator = InductionMotorCalculator(self.params)
        self.calculator.calculate_basic_parameters()

        operating_hours = self.op_hours_var.get()
        econ = self.calculator.calculate_economic_analysis(operating_hours)

        # Display results
        self.econ_text.delete('1.0', tk.END)

        output = "=" * 50 + "\n"
        output += "ECONOMIC ANALYSIS\n"
        output += "=" * 50 + "\n\n"

        output += f"Operating Hours:              {operating_hours:.1f} hours\n"
        output += f"Electricity Cost:             ${self.params.electricity_cost:.3f}/kWh\n"
        output += f"Maintenance Cost:             ${self.params.maintenance_cost_per_hour:.2f}/hour\n\n"

        output += f"Energy Consumption:           {econ['energy_consumption_kwh']:.2f} kWh\n"
        output += f"Energy Cost:                  ${econ['energy_cost']:.2f}\n"
        output += f"Maintenance Cost:             ${econ['maintenance_cost']:.2f}\n"
        output += f"Total Operating Cost:         ${econ['total_cost']:.2f}\n\n"

        output += f"Cost per kWh Output:          ${econ['cost_per_kwh_output']:.4f}/kWh\n\n"

        # Add efficiency analysis
        results = self.calculator.results
        if results:
            output += "=" * 50 + "\n"
            output += "EFFICIENCY ANALYSIS\n"
            output += "=" * 50 + "\n\n"

            output += f"Motor Efficiency:             {results['eta']*100:.2f}%\n"
            output += f"Input Power:                  {results['P_in']/1000:.2f} kW\n"
            output += f"Output Power:                 {results['P_out']/1000:.2f} kW\n"
            output += f"Losses:                       {(results['P_in']-results['P_out'])/1000:.2f} kW\n\n"

            # Annual projection
            annual_hours = 8760  # Hours in a year
            annual_econ = self.calculator.calculate_economic_analysis(annual_hours)

            output += "=" * 50 + "\n"
            output += "ANNUAL PROJECTION (8760 hours)\n"
            output += "=" * 50 + "\n\n"

            output += f"Annual Energy Consumption:    {annual_econ['energy_consumption_kwh']:.2f} kWh\n"
            output += f"Annual Energy Cost:           ${annual_econ['energy_cost']:.2f}\n"
            output += f"Annual Maintenance Cost:      ${annual_econ['maintenance_cost']:.2f}\n"
            output += f"Total Annual Cost:            ${annual_econ['total_cost']:.2f}\n"

        self.econ_text.insert('1.0', output)

        # Plot cost breakdown
        self.econ_fig.clear()

        # Pie chart
        ax1 = self.econ_fig.add_subplot(1, 2, 1)
        costs = [econ['energy_cost'], econ['maintenance_cost']]
        labels = ['Energy Cost', 'Maintenance Cost']
        colors = ['#3498db', '#e74c3c']

        ax1.pie(costs, labels=labels, colors=colors, autopct='%1.1f%%',
               startangle=90, textprops={'fontweight': 'bold'})
        ax1.set_title('Cost Breakdown', fontweight='bold')

        # Bar chart for annual projection
        ax2 = self.econ_fig.add_subplot(1, 2, 2)

        periods = ['Daily\n(24h)', 'Weekly\n(168h)', 'Monthly\n(720h)', 'Annual\n(8760h)']
        hours = [24, 168, 720, 8760]
        total_costs = [self.calculator.calculate_economic_analysis(h)['total_cost']
                      for h in hours]

        bars = ax2.bar(periods, total_costs, color=['#2ecc71', '#3498db', '#f39c12', '#e74c3c'],
                      alpha=0.7, edgecolor='black', linewidth=2)

        for bar, cost in zip(bars, total_costs):
            height = bar.get_height()
            ax2.text(bar.get_x() + bar.get_width()/2., height,
                    f'${cost:.0f}',
                    ha='center', va='bottom', fontweight='bold')

        ax2.set_ylabel('Total Cost ($)', fontweight='bold')
        ax2.set_title('Operating Cost Projection', fontweight='bold')
        ax2.grid(True, alpha=0.3, axis='y')

        self.econ_fig.tight_layout()
        self.econ_canvas.draw()

    def update_loss_breakdown(self):
        """Update loss breakdown visualization"""
        losses = self.calculator.calculate_losses_breakdown()

        self.loss_fig.clear()

        # Create subplots
        ax1 = self.loss_fig.add_subplot(2, 2, 1)
        ax2 = self.loss_fig.add_subplot(2, 2, 2)
        ax3 = self.loss_fig.add_subplot(2, 2, (3, 4))

        # Pie chart
        loss_values = [losses['Stator Copper Loss'], losses['Rotor Copper Loss'],
                      losses['Core Loss'], losses['Rotational Loss'], losses['Stray Load Loss']]
        loss_labels = ['Stator Copper', 'Rotor Copper', 'Core', 'Rotational', 'Stray Load']
        colors = ['#e74c3c', '#3498db', '#2ecc71', '#f39c12', '#9b59b6']

        wedges, texts, autotexts = ax1.pie(loss_values, labels=loss_labels, colors=colors,
                                           autopct='%1.1f%%', startangle=90)
        for text in texts:
            text.set_fontweight('bold')
        for autotext in autotexts:
            autotext.set_color('white')
            autotext.set_fontweight('bold')

        ax1.set_title('Loss Distribution', fontweight='bold', fontsize=12)

        # Bar chart
        bars = ax2.bar(loss_labels, loss_values, color=colors, alpha=0.7,
                      edgecolor='black', linewidth=2)

        for bar, val in zip(bars, loss_values):
            height = bar.get_height()
            ax2.text(bar.get_x() + bar.get_width()/2., height,
                    f'{val:.0f}W',
                    ha='center', va='bottom', fontweight='bold', fontsize=8)

        ax2.set_ylabel('Power Loss (W)', fontweight='bold')
        ax2.set_title('Loss Breakdown (W)', fontweight='bold', fontsize=12)
        ax2.tick_params(axis='x', rotation=45)
        ax2.grid(True, alpha=0.3, axis='y')

        # Waterfall chart showing power flow
        if self.calculator.results:
            results = self.calculator.results

            stages = ['Input', 'Stator\nLoss', 'Core\nLoss', 'Stray\nLoss',
                     'Air Gap', 'Rotor\nLoss', 'Mech.', 'Rot.\nLoss', 'Output']

            P_in = results['P_in'] / 1000
            values = [
                P_in,
                -self.params.P_stator_loss / 1000,
                -self.params.P_core / 1000,
                -self.params.P_stray / 1000,
                0,  # Air gap (calculated point)
                -self.params.P_rotor_loss / 1000,
                0,  # Mechanical (calculated point)
                -self.params.P_rotational / 1000,
                0   # Output (calculated point)
            ]

            # Calculate cumulative
            cumulative = [P_in]
            for i in range(1, len(values)-1):
                if values[i] != 0:
                    cumulative.append(cumulative[-1] + values[i])
                else:
                    cumulative.append(cumulative[-1])
            cumulative.append(results['P_out'] / 1000)

            # Plot waterfall
            x_pos = np.arange(len(stages))

            for i in range(len(stages)):
                if i == 0:
                    ax3.bar(i, cumulative[i], color='#2ecc71', alpha=0.7,
                           edgecolor='black', linewidth=2)
                elif i == len(stages) - 1:
                    ax3.bar(i, cumulative[i], color='#3498db', alpha=0.7,
                           edgecolor='black', linewidth=2)
                elif values[i] < 0:
                    ax3.bar(i, abs(values[i]), bottom=cumulative[i],
                           color='#e74c3c', alpha=0.7, edgecolor='black', linewidth=2)
                else:
                    ax3.bar(i, cumulative[i], color='#95a5a6', alpha=0.5,
                           edgecolor='black', linewidth=1, linestyle='--')

            # Add connecting lines
            for i in range(len(stages)-1):
                ax3.plot([i+0.4, i+0.6], [cumulative[i], cumulative[i+1]],
                        'k--', linewidth=1, alpha=0.5)

            # Add value labels
            for i, (stage, val) in enumerate(zip(stages, cumulative)):
                ax3.text(i, val, f'{val:.2f}', ha='center', va='bottom',
                        fontweight='bold', fontsize=9)

            ax3.set_xticks(x_pos)
            ax3.set_xticklabels(stages, fontweight='bold')
            ax3.set_ylabel('Power (kW)', fontweight='bold')
            ax3.set_title('Power Flow Waterfall Chart', fontweight='bold', fontsize=12)
            ax3.grid(True, alpha=0.3, axis='y')

        self.loss_fig.tight_layout()
        self.loss_canvas.draw()

    def calculate_derating(self):
        """Calculate thermal derating factor"""
        T_operating = self.operating_temp_var.get()
        T_max = self.params.max_temp
        T_rated = 75.0  # Rated temperature

        # Simple linear derating
        if T_operating <= T_rated:
            derating_factor = 1.0
        elif T_operating >= T_max:
            derating_factor = 0.0
        else:
            derating_factor = 1.0 - (T_operating - T_rated) / (T_max - T_rated)

        self.derating_label.config(text=f"Derating Factor: {derating_factor*100:.1f}%")

        # Plot derating curve
        self.control_fig.clear()
        ax = self.control_fig.add_subplot(111)

        temps = np.linspace(25, T_max, 100)
        factors = np.zeros_like(temps)

        for i, T in enumerate(temps):
            if T <= T_rated:
                factors[i] = 1.0
            elif T >= T_max:
                factors[i] = 0.0
            else:
                factors[i] = 1.0 - (T - T_rated) / (T_max - T_rated)

        ax.plot(temps, factors * 100, 'b-', linewidth=3, label='Derating Curve')
        ax.axvline(x=T_operating, color='r', linestyle='--', linewidth=2,
                  label=f'Operating: {T_operating:.1f}°C')
        ax.axhline(y=derating_factor*100, color='r', linestyle='--', linewidth=2,
                  label=f'Factor: {derating_factor*100:.1f}%')

        ax.fill_between(temps, 0, factors * 100, alpha=0.3)

        ax.set_xlabel('Temperature (°C)', fontweight='bold', fontsize=11)
        ax.set_ylabel('Derating Factor (%)', fontweight='bold', fontsize=11)
        ax.set_title('Thermal Derating Curve', fontweight='bold', fontsize=12)
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.set_xlim([25, T_max])
        ax.set_ylim([0, 105])

        self.control_fig.tight_layout()
        self.control_canvas.draw()

    def on_window_resize(self, event):
        """Handle window resize event for auto-scaling"""
        # This ensures matplotlib canvases redraw properly
        if hasattr(self, 'main_canvas'):
            self.main_canvas.draw_idle()
        if hasattr(self, 'sim_canvas'):
            self.sim_canvas.draw_idle()
        if hasattr(self, 'mp_canvas'):
            self.mp_canvas.draw_idle()
        if hasattr(self, 'econ_canvas'):
            self.econ_canvas.draw_idle()
        if hasattr(self, 'loss_canvas'):
            self.loss_canvas.draw_idle()
        if hasattr(self, 'control_canvas'):
            self.control_canvas.draw_idle()


def main():
    """Main entry point"""
    app = AdvancedInductionMotorLab()
    app.mainloop()


if __name__ == "__main__":
    main()
