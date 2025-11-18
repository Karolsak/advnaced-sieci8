"""
Advanced Series Motor-Generator Analysis Lab
Comprehensive multi-physics simulation with thermal, mechanical, and electromagnetic modeling
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.interpolate import interp1d
from scipy.integrate import solve_ivp
import threading
import time
from dataclasses import dataclass
from typing import Tuple, Dict, List


@dataclass
class MotorParameters:
    """Motor electrical and mechanical parameters"""
    voltage: float = 525.0  # V
    resistance_motor: float = 3.5  # Ω
    resistance_load: float = 3.25  # Ω
    total_resistance: float = 6.75  # Ω (motor + load)
    target_speed: float = 1000.0  # rpm

    # Physical parameters
    ambient_temp: float = 25.0  # °C
    thermal_resistance: float = 2.5  # °C/W
    thermal_capacitance: float = 800.0  # J/°C
    moment_inertia: float = 0.5  # kg·m²
    friction_coefficient: float = 0.01  # N·m·s

    # Material properties
    copper_density: float = 8960.0  # kg/m³
    iron_density: float = 7874.0  # kg/m³
    specific_heat_copper: float = 385.0  # J/(kg·K)
    specific_heat_iron: float = 449.0  # J/(kg·K)

    # Economic parameters
    electricity_cost: float = 0.12  # $/kWh
    maintenance_cost: float = 50.0  # $/year
    efficiency_target: float = 0.85


class SeriesMotorModel:
    """Mathematical model for series motor/generator"""

    def __init__(self):
        # Experimental data from problem statement
        self.current_data = np.array([75, 125, 175, 225])  # A
        self.speed_data = np.array([1200, 950, 840, 745])  # rpm

        # Calculate flux-current relationship
        self.flux_current_relation = self._calculate_flux_relation()

        # Create interpolation functions
        self.speed_vs_current = interp1d(
            self.current_data, self.speed_data,
            kind='cubic', fill_value='extrapolate'
        )
        self.current_vs_speed = interp1d(
            self.speed_data, self.current_data,
            kind='cubic', fill_value='extrapolate'
        )

    def _calculate_flux_relation(self):
        """Calculate flux per ampere from motor characteristics"""
        # For series motor: E = k * φ * N, where φ ∝ I
        # V = E + I*R, so E = V - I*R
        voltage = 525.0
        resistance = 3.5

        back_emf = voltage - self.current_data * resistance
        # k*φ = E/N, and φ ∝ I for series motor
        k_phi = back_emf / self.speed_data

        return interp1d(self.current_data, k_phi, kind='cubic', fill_value='extrapolate')

    def calculate_generator_current(self, speed_rpm, R_total, voltage=525.0):
        """
        Calculate current when operating as generator
        For standalone generator: E = I*R_total
        E = k*φ(I)*N, where φ depends on I for series machine
        """
        from scipy.optimize import fsolve

        def generator_equation(I):
            """
            For series generator: E = k·φ(I)·N = I·R_total
            """
            if I <= 0 or I < 1:
                return 1e10  # Large error for invalid current

            try:
                k_phi = self.flux_current_relation(I)
                E_generated = k_phi * speed_rpm
                E_required = I * R_total
                return E_generated - E_required
            except:
                return 1e10

        # Try multiple initial guesses to find solution
        best_solution = None
        min_error = float('inf')

        for initial_guess in [30, 40, 50, 60, 70, 80]:
            try:
                sol = fsolve(generator_equation, initial_guess, full_output=True)
                if sol[2] == 1 and sol[0][0] > 0:  # Solution found and positive
                    error = abs(generator_equation(sol[0][0]))
                    if error < min_error:
                        min_error = error
                        best_solution = sol[0][0]
            except:
                pass

        # If fsolve didn't work, use manual iteration
        if best_solution is None or min_error > 1.0:
            current_guess = 50.0  # Initial guess

            for iteration in range(200):
                try:
                    k_phi = self.flux_current_relation(current_guess)
                    E_generated = k_phi * speed_rpm
                    current_new = E_generated / R_total

                    # Check convergence
                    if abs(current_new - current_guess) < 0.01:
                        return current_new

                    # Relaxation with adaptive factor
                    alpha = 0.5 if iteration < 50 else 0.3
                    current_guess = alpha * current_guess + (1 - alpha) * current_new
                except:
                    current_guess *= 0.9  # Reduce if error

            return current_guess

        return best_solution

    def calculate_power_loss(self, current, speed_rpm, temperature=25.0):
        """Calculate detailed power losses"""
        # Temperature coefficient for copper
        alpha = 0.00393  # per °C
        R_temp = 3.5 * (1 + alpha * (temperature - 25))

        # Copper losses (I²R)
        copper_loss = current**2 * R_temp

        # Iron losses (hysteresis + eddy current)
        # P_iron = k_h*f*B² + k_e*f²*B²
        frequency = speed_rpm / 60.0  # Hz (for 2-pole machine)
        flux_density = current * 0.01  # Simplified, B ∝ I
        hysteresis_loss = 10.0 * frequency * flux_density**2
        eddy_loss = 5.0 * frequency**2 * flux_density**2
        iron_loss = hysteresis_loss + eddy_loss

        # Mechanical losses (friction and windage)
        mechanical_loss = 0.01 * (speed_rpm / 1000)**2 * 1000  # W

        # Stray load losses (approximately 1% of output)
        output_power = 525 * current * 0.85  # Approximate
        stray_loss = 0.01 * output_power

        return {
            'copper': copper_loss,
            'iron': iron_loss,
            'hysteresis': hysteresis_loss,
            'eddy': eddy_loss,
            'mechanical': mechanical_loss,
            'stray': stray_loss,
            'total': copper_loss + iron_loss + mechanical_loss + stray_loss
        }


class MultiPhysicsSimulator:
    """Advanced multi-physics simulation engine"""

    def __init__(self, motor_model: SeriesMotorModel, params: MotorParameters):
        self.motor_model = motor_model
        self.params = params
        self.simulation_running = False
        self.simulation_data = {
            'time': [],
            'current': [],
            'speed': [],
            'temperature': [],
            'torque': [],
            'power': [],
            'efficiency': []
        }

    def electromagnetic_thermal_ode(self, t, y):
        """
        Coupled electromagnetic-thermal ODE system
        State vector y = [current, speed, temperature_winding, temperature_core]
        """
        I, omega, T_wind, T_core = y

        # Convert angular velocity to rpm
        N = omega * 60 / (2 * np.pi)

        # Electromagnetic equations
        k_phi = self.motor_model.flux_current_relation(I)
        back_emf = k_phi * N

        # Voltage equation: V = E + I*R + L*dI/dt
        # Simplified: dI/dt = (V - E - I*R) / L
        L_equivalent = 0.1  # H, estimated inductance
        dI_dt = (self.params.voltage - back_emf - I * self.params.total_resistance) / L_equivalent

        # Mechanical equation: J*dω/dt = T_em - T_load - T_friction
        torque_em = k_phi * I  # Electromagnetic torque
        torque_friction = self.params.friction_coefficient * omega
        torque_load = 0.5 * k_phi * I  # Variable load

        dOmega_dt = (torque_em - torque_load - torque_friction) / self.params.moment_inertia

        # Thermal equations (separate for windings and core)
        # dT/dt = (P_loss - (T-T_amb)/R_th) / C_th

        losses = self.motor_model.calculate_power_loss(I, N, T_wind)

        # Winding temperature
        P_copper = losses['copper']
        dT_wind_dt = (P_copper - (T_wind - self.params.ambient_temp) / self.params.thermal_resistance) / self.params.thermal_capacitance

        # Core temperature
        P_iron = losses['iron']
        dT_core_dt = (P_iron - (T_core - self.params.ambient_temp) / (self.params.thermal_resistance * 1.5)) / (self.params.thermal_capacitance * 0.8)

        return [dI_dt, dOmega_dt, dT_wind_dt, dT_core_dt]

    def run_dynamic_simulation(self, duration=10.0, method='RK45'):
        """Run dynamic simulation with specified ODE solver"""
        # Initial conditions: [current, angular_velocity, T_winding, T_core]
        y0 = [50.0, 1000 * 2 * np.pi / 60, 25.0, 25.0]

        t_span = (0, duration)
        t_eval = np.linspace(0, duration, 1000)

        if method == 'RK45':
            sol = solve_ivp(
                self.electromagnetic_thermal_ode,
                t_span, y0,
                method='RK45',
                t_eval=t_eval,
                max_step=0.01
            )
        elif method == 'Euler':
            sol = self._euler_method(t_eval, y0)
        else:
            sol = solve_ivp(
                self.electromagnetic_thermal_ode,
                t_span, y0,
                t_eval=t_eval
            )

        return sol

    def _euler_method(self, t_eval, y0):
        """Simple Euler method for ODE integration"""
        y = np.zeros((len(y0), len(t_eval)))
        y[:, 0] = y0

        for i in range(1, len(t_eval)):
            dt = t_eval[i] - t_eval[i-1]
            dydt = self.electromagnetic_thermal_ode(t_eval[i-1], y[:, i-1])
            y[:, i] = y[:, i-1] + np.array(dydt) * dt

        # Create solution object similar to solve_ivp
        class Solution:
            pass

        sol = Solution()
        sol.t = t_eval
        sol.y = y
        sol.success = True

        return sol

    def calculate_mechanical_stress(self, current, speed_rpm):
        """Evaluate mechanical stress, shaft torque, and bearing loads"""
        k_phi = self.motor_model.flux_current_relation(current)

        # Torque calculation
        torque = k_phi * current  # N·m

        # Shaft stress (assuming solid shaft, diameter 50mm)
        shaft_diameter = 0.05  # m
        polar_moment = np.pi * shaft_diameter**4 / 32
        shear_stress = torque * (shaft_diameter/2) / polar_moment  # Pa

        # Bearing loads (radial and axial)
        # Simplified model: radial load from belt tension
        radial_load = torque / 0.1  # N (assuming 0.1m pulley radius)
        axial_load = 0.1 * radial_load  # Simplified

        # Centrifugal forces on rotor
        omega = speed_rpm * 2 * np.pi / 60
        rotor_mass = 50.0  # kg
        rotor_radius = 0.15  # m
        centrifugal_force = rotor_mass * rotor_radius * omega**2

        return {
            'torque': torque,
            'shaft_stress': shear_stress / 1e6,  # MPa
            'radial_bearing_load': radial_load,
            'axial_bearing_load': axial_load,
            'centrifugal_force': centrifugal_force,
            'safety_factor': 250 / (shear_stress / 1e6)  # Assuming 250 MPa yield strength
        }


class EconomicAnalyzer:
    """Economic analysis for motor operation"""

    def __init__(self, params: MotorParameters):
        self.params = params

    def calculate_operating_cost(self, power_kw, hours_per_year=4000):
        """Calculate annual operating cost"""
        energy_cost = power_kw * hours_per_year * self.params.electricity_cost
        total_cost = energy_cost + self.params.maintenance_cost

        return {
            'energy_cost': energy_cost,
            'maintenance_cost': self.params.maintenance_cost,
            'total_annual_cost': total_cost,
            'cost_per_hour': total_cost / hours_per_year
        }

    def calculate_efficiency_impact(self, efficiency_actual):
        """Calculate cost impact of efficiency deviation"""
        power_rated = 100.0  # kW, example

        power_actual = power_rated / efficiency_actual
        power_target = power_rated / self.params.efficiency_target

        extra_cost = (power_actual - power_target) * 4000 * self.params.electricity_cost

        return {
            'efficiency_actual': efficiency_actual,
            'efficiency_target': self.params.efficiency_target,
            'extra_annual_cost': extra_cost,
            'savings_potential': extra_cost
        }

    def payback_analysis(self, investment_cost, annual_savings):
        """Calculate payback period for efficiency improvements"""
        if annual_savings <= 0:
            return float('inf')

        payback_period = investment_cost / annual_savings
        roi = (annual_savings * 10 - investment_cost) / investment_cost * 100

        return {
            'payback_years': payback_period,
            'roi_10year': roi,
            'net_savings_10year': annual_savings * 10 - investment_cost
        }


class AdvancedMotorLab(tk.Tk):
    """Main application class with comprehensive GUI"""

    def __init__(self):
        super().__init__()

        self.title("Advanced Series Motor-Generator Analysis Lab")
        self.geometry("1400x900")

        # Initialize models
        self.motor_model = SeriesMotorModel()
        self.params = MotorParameters()
        self.simulator = MultiPhysicsSimulator(self.motor_model, self.params)
        self.economic_analyzer = EconomicAnalyzer(self.params)

        # Simulation control
        self.simulation_thread = None
        self.is_simulating = False

        # Configure grid weight for auto-scaling
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        # Create main container
        self.main_container = ttk.Frame(self)
        self.main_container.grid(row=0, column=0, sticky='nsew')
        self.main_container.grid_rowconfigure(0, weight=1)
        self.main_container.grid_columnconfigure(0, weight=1)

        # Create notebook for tabs
        self.notebook = ttk.Notebook(self.main_container)
        self.notebook.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        # Create tabs
        self.create_main_tab()
        self.create_analysis_tab()
        self.create_simulation_tab()
        self.create_economic_tab()
        self.create_thermal_tab()
        self.create_mechanical_tab()

        # Bind resize event
        self.bind('<Configure>', self.on_resize)

        # Perform initial calculation
        self.calculate_generator_operation()

    def on_resize(self, event):
        """Handle window resize for auto-scaling"""
        # Update figure sizes when window is resized
        if hasattr(self, 'canvas_main'):
            self.canvas_main.draw()
        if hasattr(self, 'canvas_simulation'):
            self.canvas_simulation.draw()

    def create_main_tab(self):
        """Main analysis tab with input parameters and results"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Main Analysis")

        # Configure grid
        tab.grid_rowconfigure(1, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Control panel
        control_frame = ttk.LabelFrame(tab, text="Motor Parameters", padding=10)
        control_frame.grid(row=0, column=0, sticky='ew', padx=5, pady=5)

        # Voltage control
        ttk.Label(control_frame, text="Voltage (V):").grid(row=0, column=0, sticky='w')
        self.voltage_var = tk.DoubleVar(value=525.0)
        self.voltage_slider = ttk.Scale(
            control_frame, from_=0, to=1000,
            variable=self.voltage_var, orient='horizontal',
            command=lambda v: self.update_parameter('voltage', float(v))
        )
        self.voltage_slider.grid(row=0, column=1, sticky='ew', padx=5)
        self.voltage_label = ttk.Label(control_frame, text="525.0 V")
        self.voltage_label.grid(row=0, column=2)

        # Speed control
        ttk.Label(control_frame, text="Target Speed (rpm):").grid(row=1, column=0, sticky='w')
        self.speed_var = tk.DoubleVar(value=1000.0)
        self.speed_slider = ttk.Scale(
            control_frame, from_=500, to=1500,
            variable=self.speed_var, orient='horizontal',
            command=lambda v: self.update_parameter('speed', float(v))
        )
        self.speed_slider.grid(row=1, column=1, sticky='ew', padx=5)
        self.speed_label = ttk.Label(control_frame, text="1000.0 rpm")
        self.speed_label.grid(row=1, column=2)

        # Load resistance control
        ttk.Label(control_frame, text="Load Resistance (Ω):").grid(row=2, column=0, sticky='w')
        self.resistance_var = tk.DoubleVar(value=3.25)
        self.resistance_slider = ttk.Scale(
            control_frame, from_=0, to=10,
            variable=self.resistance_var, orient='horizontal',
            command=lambda v: self.update_parameter('resistance', float(v))
        )
        self.resistance_slider.grid(row=2, column=1, sticky='ew', padx=5)
        self.resistance_label = ttk.Label(control_frame, text="3.25 Ω")
        self.resistance_label.grid(row=2, column=2)

        control_frame.grid_columnconfigure(1, weight=1)

        # Buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=3, column=0, columnspan=3, pady=10)

        ttk.Button(button_frame, text="Calculate", command=self.calculate_generator_operation).pack(side='left', padx=5)
        ttk.Button(button_frame, text="Reset", command=self.reset_parameters).pack(side='left', padx=5)
        ttk.Button(button_frame, text="Export Data", command=self.export_data).pack(side='left', padx=5)

        # Results display
        results_frame = ttk.LabelFrame(tab, text="Results", padding=10)
        results_frame.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)
        results_frame.grid_rowconfigure(0, weight=1)
        results_frame.grid_columnconfigure(0, weight=1)

        # Create text widget with scrollbar
        text_scroll = ttk.Scrollbar(results_frame)
        text_scroll.grid(row=0, column=1, sticky='ns')

        self.results_text = tk.Text(results_frame, height=15, wrap='word', yscrollcommand=text_scroll.set)
        self.results_text.grid(row=0, column=0, sticky='nsew')
        text_scroll.config(command=self.results_text.yview)

        # Graph frame
        graph_frame = ttk.LabelFrame(tab, text="Motor Characteristics", padding=5)
        graph_frame.grid(row=2, column=0, sticky='nsew', padx=5, pady=5)
        graph_frame.grid_rowconfigure(0, weight=1)
        graph_frame.grid_columnconfigure(0, weight=1)

        self.fig_main = Figure(figsize=(12, 4), dpi=100)
        self.canvas_main = FigureCanvasTkAgg(self.fig_main, master=graph_frame)
        self.canvas_main.get_tk_widget().grid(row=0, column=0, sticky='nsew')

        self.plot_motor_characteristics()

    def create_analysis_tab(self):
        """Detailed analysis tab with loss breakdown"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Loss Analysis")

        tab.grid_rowconfigure(0, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Create figure for loss analysis
        self.fig_losses = Figure(figsize=(12, 8), dpi=100)
        canvas = FigureCanvasTkAgg(self.fig_losses, master=tab)
        canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        self.plot_loss_analysis()

    def create_simulation_tab(self):
        """Dynamic simulation tab with ODE solvers"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Dynamic Simulation")

        tab.grid_rowconfigure(1, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Control panel
        control_frame = ttk.LabelFrame(tab, text="Simulation Controls", padding=10)
        control_frame.grid(row=0, column=0, sticky='ew', padx=5, pady=5)

        ttk.Label(control_frame, text="Duration (s):").grid(row=0, column=0)
        self.sim_duration_var = tk.DoubleVar(value=10.0)
        ttk.Entry(control_frame, textvariable=self.sim_duration_var, width=10).grid(row=0, column=1, padx=5)

        ttk.Label(control_frame, text="Solver:").grid(row=0, column=2, padx=(20, 5))
        self.solver_var = tk.StringVar(value='RK45')
        solver_combo = ttk.Combobox(control_frame, textvariable=self.solver_var,
                                    values=['RK45', 'Euler', 'RK23'], width=10, state='readonly')
        solver_combo.grid(row=0, column=3)

        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=0, column=4, padx=20)

        self.start_button = ttk.Button(button_frame, text="Start", command=self.start_simulation)
        self.start_button.pack(side='left', padx=2)

        self.stop_button = ttk.Button(button_frame, text="Stop", command=self.stop_simulation, state='disabled')
        self.stop_button.pack(side='left', padx=2)

        ttk.Button(button_frame, text="Reset", command=self.reset_simulation).pack(side='left', padx=2)

        # Progress bar
        self.progress_var = tk.DoubleVar()
        self.progress_bar = ttk.Progressbar(control_frame, variable=self.progress_var, maximum=100)
        self.progress_bar.grid(row=1, column=0, columnspan=5, sticky='ew', pady=10)

        # Results frame
        results_frame = ttk.LabelFrame(tab, text="Simulation Results", padding=5)
        results_frame.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)
        results_frame.grid_rowconfigure(0, weight=1)
        results_frame.grid_columnconfigure(0, weight=1)

        self.fig_simulation = Figure(figsize=(12, 8), dpi=100)
        self.canvas_simulation = FigureCanvasTkAgg(self.fig_simulation, master=results_frame)
        self.canvas_simulation.get_tk_widget().grid(row=0, column=0, sticky='nsew')

    def create_economic_tab(self):
        """Economic analysis tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Economic Analysis")

        tab.grid_rowconfigure(1, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Input parameters
        input_frame = ttk.LabelFrame(tab, text="Economic Parameters", padding=10)
        input_frame.grid(row=0, column=0, sticky='ew', padx=5, pady=5)

        ttk.Label(input_frame, text="Electricity Cost ($/kWh):").grid(row=0, column=0, sticky='w')
        self.elec_cost_var = tk.DoubleVar(value=0.12)
        ttk.Entry(input_frame, textvariable=self.elec_cost_var, width=15).grid(row=0, column=1, padx=5)

        ttk.Label(input_frame, text="Operating Hours/Year:").grid(row=1, column=0, sticky='w')
        self.op_hours_var = tk.DoubleVar(value=4000)
        ttk.Entry(input_frame, textvariable=self.op_hours_var, width=15).grid(row=1, column=1, padx=5)

        ttk.Label(input_frame, text="Maintenance Cost ($/year):").grid(row=2, column=0, sticky='w')
        self.maint_cost_var = tk.DoubleVar(value=50)
        ttk.Entry(input_frame, textvariable=self.maint_cost_var, width=15).grid(row=2, column=1, padx=5)

        ttk.Button(input_frame, text="Calculate Economics",
                  command=self.calculate_economics).grid(row=3, column=0, columnspan=2, pady=10)

        # Results
        results_frame = ttk.LabelFrame(tab, text="Economic Results", padding=10)
        results_frame.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)
        results_frame.grid_rowconfigure(0, weight=1)
        results_frame.grid_columnconfigure(0, weight=1)

        self.economic_text = tk.Text(results_frame, height=20, wrap='word')
        self.economic_text.grid(row=0, column=0, sticky='nsew')

        scroll = ttk.Scrollbar(results_frame, command=self.economic_text.yview)
        scroll.grid(row=0, column=1, sticky='ns')
        self.economic_text.config(yscrollcommand=scroll.set)

    def create_thermal_tab(self):
        """Thermal analysis and derating tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Thermal & Derating")

        tab.grid_rowconfigure(0, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Create thermal visualization
        self.fig_thermal = Figure(figsize=(12, 8), dpi=100)
        canvas = FigureCanvasTkAgg(self.fig_thermal, master=tab)
        canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        self.plot_thermal_analysis()

    def create_mechanical_tab(self):
        """Mechanical stress analysis tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Mechanical Stress")

        tab.grid_rowconfigure(0, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Create mechanical analysis visualization
        self.fig_mechanical = Figure(figsize=(12, 8), dpi=100)
        canvas = FigureCanvasTkAgg(self.fig_mechanical, master=tab)
        canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        self.plot_mechanical_analysis()

    def update_parameter(self, param_name, value):
        """Update parameter and refresh display"""
        if param_name == 'voltage':
            self.params.voltage = value
            self.voltage_label.config(text=f"{value:.1f} V")
        elif param_name == 'speed':
            self.params.target_speed = value
            self.speed_label.config(text=f"{value:.1f} rpm")
        elif param_name == 'resistance':
            self.params.resistance_load = value
            self.params.total_resistance = self.params.resistance_motor + value
            self.resistance_label.config(text=f"{value:.2f} Ω")

    def calculate_generator_operation(self):
        """Calculate generator operating point"""
        current = self.motor_model.calculate_generator_current(
            self.params.target_speed,
            self.params.total_resistance,
            self.params.voltage
        )

        # Calculate performance metrics
        k_phi = self.motor_model.flux_current_relation(current)
        back_emf = k_phi * self.params.target_speed

        # Power calculations (RMS values)
        power_generated = back_emf * current  # W
        power_dissipated = current**2 * self.params.total_resistance
        power_output = power_generated - power_dissipated
        efficiency = power_output / power_generated * 100 if power_generated > 0 else 0

        # Get losses
        losses = self.motor_model.calculate_power_loss(current, self.params.target_speed)

        # Mechanical stress
        stress = self.simulator.calculate_mechanical_stress(current, self.params.target_speed)

        # Display results
        self.results_text.delete('1.0', tk.END)
        results = f"""
═══════════════════════════════════════════════════════════
        SERIES MOTOR-GENERATOR ANALYSIS RESULTS
═══════════════════════════════════════════════════════════

OPERATING CONDITIONS:
───────────────────────────────────────────────────────────
  Operating Mode:           Generator
  Target Speed:             {self.params.target_speed:.1f} rpm
  Terminal Voltage (RMS):   {self.params.voltage:.1f} V
  Motor Resistance:         {self.params.resistance_motor:.2f} Ω
  Load Resistance:          {self.params.resistance_load:.2f} Ω
  Total Resistance:         {self.params.total_resistance:.2f} Ω

ELECTRICAL PARAMETERS:
───────────────────────────────────────────────────────────
  Generator Current (RMS):  {current:.2f} A
  Back EMF:                 {back_emf:.2f} V
  Generated Power:          {power_generated/1000:.2f} kW
  Power Output:             {power_output/1000:.2f} kW
  Efficiency:               {efficiency:.2f} %

POWER LOSS BREAKDOWN:
───────────────────────────────────────────────────────────
  Copper Losses (I²R):      {losses['copper']:.2f} W
  Iron Losses:              {losses['iron']:.2f} W
    - Hysteresis:           {losses['hysteresis']:.2f} W
    - Eddy Current:         {losses['eddy']:.2f} W
  Mechanical Losses:        {losses['mechanical']:.2f} W
  Stray Load Losses:        {losses['stray']:.2f} W
  ─────────────────────────
  Total Losses:             {losses['total']:.2f} W

MECHANICAL PERFORMANCE:
───────────────────────────────────────────────────────────
  Electromagnetic Torque:   {stress['torque']:.2f} N·m
  Shaft Shear Stress:       {stress['shaft_stress']:.2f} MPa
  Safety Factor:            {stress['safety_factor']:.2f}
  Radial Bearing Load:      {stress['radial_bearing_load']:.2f} N
  Axial Bearing Load:       {stress['axial_bearing_load']:.2f} N
  Centrifugal Force:        {stress['centrifugal_force']:.2f} N

THERMAL CONSIDERATIONS:
───────────────────────────────────────────────────────────
  Ambient Temperature:      {self.params.ambient_temp:.1f} °C
  Est. Winding Temp Rise:   {losses['copper'] * self.params.thermal_resistance:.1f} °C
  Est. Core Temp Rise:      {losses['iron'] * self.params.thermal_resistance:.1f} °C

═══════════════════════════════════════════════════════════
"""
        self.results_text.insert('1.0', results)

        # Update plots
        self.plot_motor_characteristics()
        self.plot_loss_analysis()
        self.plot_thermal_analysis()
        self.plot_mechanical_analysis()

    def plot_motor_characteristics(self):
        """Plot motor speed-current and torque characteristics"""
        self.fig_main.clear()

        current_range = np.linspace(50, 250, 100)
        speed_range = self.motor_model.speed_vs_current(current_range)

        # Calculate torque for each point
        torque_range = []
        power_range = []
        for i, curr in enumerate(current_range):
            k_phi = self.motor_model.flux_current_relation(curr)
            torque = k_phi * curr
            power = k_phi * speed_range[i] * curr * 2 * np.pi / 60 / 1000  # kW
            torque_range.append(torque)
            power_range.append(power)

        # Create subplots
        ax1 = self.fig_main.add_subplot(131)
        ax2 = self.fig_main.add_subplot(132)
        ax3 = self.fig_main.add_subplot(133)

        # Speed vs Current
        ax1.plot(current_range, speed_range, 'b-', linewidth=2, label='Calculated')
        ax1.plot(self.motor_model.current_data, self.motor_model.speed_data,
                'ro', markersize=8, label='Measured Data')
        ax1.axvline(x=self.motor_model.calculate_generator_current(
            self.params.target_speed, self.params.total_resistance, self.params.voltage
        ), color='g', linestyle='--', label='Operating Point')
        ax1.axhline(y=self.params.target_speed, color='orange', linestyle='--', alpha=0.5)
        ax1.set_xlabel('Current (A)', fontsize=10, fontweight='bold')
        ax1.set_ylabel('Speed (rpm)', fontsize=10, fontweight='bold')
        ax1.set_title('Speed vs Current Characteristics', fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.legend()

        # Torque vs Current
        ax2.plot(current_range, torque_range, 'r-', linewidth=2)
        ax2.set_xlabel('Current (A)', fontsize=10, fontweight='bold')
        ax2.set_ylabel('Torque (N·m)', fontsize=10, fontweight='bold')
        ax2.set_title('Torque vs Current', fontweight='bold')
        ax2.grid(True, alpha=0.3)

        # Power vs Current
        ax3.plot(current_range, power_range, 'g-', linewidth=2)
        ax3.set_xlabel('Current (A)', fontsize=10, fontweight='bold')
        ax3.set_ylabel('Power (kW)', fontsize=10, fontweight='bold')
        ax3.set_title('Power vs Current', fontweight='bold')
        ax3.grid(True, alpha=0.3)

        self.fig_main.tight_layout()
        self.canvas_main.draw()

    def plot_loss_analysis(self):
        """Plot detailed loss breakdown"""
        self.fig_losses.clear()

        current_range = np.linspace(50, 250, 50)

        copper_losses = []
        iron_losses = []
        mechanical_losses = []
        stray_losses = []
        efficiency_values = []

        for curr in current_range:
            speed = self.motor_model.speed_vs_current(curr)
            losses = self.motor_model.calculate_power_loss(curr, speed)

            copper_losses.append(losses['copper'])
            iron_losses.append(losses['iron'])
            mechanical_losses.append(losses['mechanical'])
            stray_losses.append(losses['stray'])

            # Calculate efficiency
            k_phi = self.motor_model.flux_current_relation(curr)
            power_out = k_phi * speed * curr * 2 * np.pi / 60
            power_in = power_out + losses['total']
            eff = power_out / power_in * 100 if power_in > 0 else 0
            efficiency_values.append(eff)

        # Create subplots
        ax1 = self.fig_losses.add_subplot(221)
        ax2 = self.fig_losses.add_subplot(222)
        ax3 = self.fig_losses.add_subplot(223)
        ax4 = self.fig_losses.add_subplot(224)

        # Stacked area plot for losses
        ax1.fill_between(current_range, 0, copper_losses, alpha=0.7, label='Copper Losses')
        ax1.fill_between(current_range, copper_losses,
                        np.array(copper_losses) + np.array(iron_losses),
                        alpha=0.7, label='Iron Losses')
        ax1.fill_between(current_range,
                        np.array(copper_losses) + np.array(iron_losses),
                        np.array(copper_losses) + np.array(iron_losses) + np.array(mechanical_losses),
                        alpha=0.7, label='Mechanical Losses')
        ax1.set_xlabel('Current (A)', fontweight='bold')
        ax1.set_ylabel('Power Loss (W)', fontweight='bold')
        ax1.set_title('Loss Breakdown vs Current', fontweight='bold')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Pie chart for losses at operating point
        current_op = self.motor_model.calculate_generator_current(
            self.params.target_speed, self.params.total_resistance, self.params.voltage
        )
        speed_op = self.params.target_speed
        losses_op = self.motor_model.calculate_power_loss(current_op, speed_op)

        loss_labels = ['Copper', 'Iron', 'Mechanical', 'Stray']
        loss_values = [losses_op['copper'], losses_op['iron'],
                      losses_op['mechanical'], losses_op['stray']]
        colors = ['#ff9999', '#66b3ff', '#99ff99', '#ffcc99']

        ax2.pie(loss_values, labels=loss_labels, autopct='%1.1f%%', colors=colors, startangle=90)
        ax2.set_title('Loss Distribution at Operating Point', fontweight='bold')

        # Efficiency curve
        ax3.plot(current_range, efficiency_values, 'b-', linewidth=2)
        ax3.axvline(x=current_op, color='r', linestyle='--', label='Operating Point')
        ax3.set_xlabel('Current (A)', fontweight='bold')
        ax3.set_ylabel('Efficiency (%)', fontweight='bold')
        ax3.set_title('Efficiency vs Current', fontweight='bold')
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # Individual loss components
        ax4.plot(current_range, copper_losses, label='Copper', linewidth=2)
        ax4.plot(current_range, iron_losses, label='Iron', linewidth=2)
        ax4.plot(current_range, mechanical_losses, label='Mechanical', linewidth=2)
        ax4.plot(current_range, stray_losses, label='Stray', linewidth=2)
        ax4.set_xlabel('Current (A)', fontweight='bold')
        ax4.set_ylabel('Power Loss (W)', fontweight='bold')
        ax4.set_title('Individual Loss Components', fontweight='bold')
        ax4.legend()
        ax4.grid(True, alpha=0.3)

        self.fig_losses.tight_layout()
        self.canvas_main.draw()

    def start_simulation(self):
        """Start dynamic simulation in separate thread"""
        if not self.is_simulating:
            self.is_simulating = True
            self.start_button.config(state='disabled')
            self.stop_button.config(state='normal')
            self.progress_var.set(0)

            self.simulation_thread = threading.Thread(target=self.run_simulation)
            self.simulation_thread.daemon = True
            self.simulation_thread.start()

    def stop_simulation(self):
        """Stop ongoing simulation"""
        self.is_simulating = False
        self.start_button.config(state='normal')
        self.stop_button.config(state='disabled')

    def reset_simulation(self):
        """Reset simulation parameters"""
        self.sim_duration_var.set(10.0)
        self.progress_var.set(0)
        self.fig_simulation.clear()
        self.canvas_simulation.draw()

    def run_simulation(self):
        """Execute dynamic simulation"""
        try:
            duration = self.sim_duration_var.get()
            solver = self.solver_var.get()

            # Run simulation
            sol = self.simulator.run_dynamic_simulation(duration, solver)

            if sol.success:
                # Extract results
                time_vec = sol.t
                current_vec = sol.y[0]
                omega_vec = sol.y[1]
                temp_wind_vec = sol.y[2]
                temp_core_vec = sol.y[3]
                speed_vec = omega_vec * 60 / (2 * np.pi)

                # Calculate additional quantities
                torque_vec = []
                power_vec = []
                efficiency_vec = []

                for i, curr in enumerate(current_vec):
                    k_phi = self.motor_model.flux_current_relation(curr)
                    torque = k_phi * curr
                    power = torque * omega_vec[i] / 1000  # kW

                    # Efficiency
                    losses = self.motor_model.calculate_power_loss(curr, speed_vec[i], temp_wind_vec[i])
                    power_in = power * 1000 + losses['total']
                    eff = power * 1000 / power_in * 100 if power_in > 0 else 0

                    torque_vec.append(torque)
                    power_vec.append(power)
                    efficiency_vec.append(eff)

                    # Update progress
                    progress = (i / len(current_vec)) * 100
                    self.progress_var.set(progress)

                # Plot results
                self.plot_simulation_results(
                    time_vec, current_vec, speed_vec, temp_wind_vec, temp_core_vec,
                    torque_vec, power_vec, efficiency_vec
                )

                self.progress_var.set(100)
            else:
                messagebox.showerror("Simulation Error", "Simulation failed to converge")

        except Exception as e:
            messagebox.showerror("Error", f"Simulation error: {str(e)}")
        finally:
            self.is_simulating = False
            self.start_button.config(state='normal')
            self.stop_button.config(state='disabled')

    def plot_simulation_results(self, time_vec, current_vec, speed_vec,
                                temp_wind_vec, temp_core_vec, torque_vec,
                                power_vec, efficiency_vec):
        """Plot dynamic simulation results"""
        self.fig_simulation.clear()

        # Create 3x2 subplot grid
        ax1 = self.fig_simulation.add_subplot(321)
        ax2 = self.fig_simulation.add_subplot(322)
        ax3 = self.fig_simulation.add_subplot(323)
        ax4 = self.fig_simulation.add_subplot(324)
        ax5 = self.fig_simulation.add_subplot(325)
        ax6 = self.fig_simulation.add_subplot(326)

        # Current vs Time
        ax1.plot(time_vec, current_vec, 'b-', linewidth=2)
        ax1.set_xlabel('Time (s)', fontweight='bold')
        ax1.set_ylabel('Current (A)', fontweight='bold')
        ax1.set_title('Current Dynamics', fontweight='bold')
        ax1.grid(True, alpha=0.3)

        # Speed vs Time
        ax2.plot(time_vec, speed_vec, 'r-', linewidth=2)
        ax2.set_xlabel('Time (s)', fontweight='bold')
        ax2.set_ylabel('Speed (rpm)', fontweight='bold')
        ax2.set_title('Speed Dynamics', fontweight='bold')
        ax2.grid(True, alpha=0.3)

        # Temperature vs Time
        ax3.plot(time_vec, temp_wind_vec, 'orange', linewidth=2, label='Winding')
        ax3.plot(time_vec, temp_core_vec, 'brown', linewidth=2, label='Core')
        ax3.set_xlabel('Time (s)', fontweight='bold')
        ax3.set_ylabel('Temperature (°C)', fontweight='bold')
        ax3.set_title('Thermal Dynamics', fontweight='bold')
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # Torque vs Time
        ax4.plot(time_vec, torque_vec, 'g-', linewidth=2)
        ax4.set_xlabel('Time (s)', fontweight='bold')
        ax4.set_ylabel('Torque (N·m)', fontweight='bold')
        ax4.set_title('Torque Dynamics', fontweight='bold')
        ax4.grid(True, alpha=0.3)

        # Power vs Time
        ax5.plot(time_vec, power_vec, 'm-', linewidth=2)
        ax5.set_xlabel('Time (s)', fontweight='bold')
        ax5.set_ylabel('Power (kW)', fontweight='bold')
        ax5.set_title('Power Output', fontweight='bold')
        ax5.grid(True, alpha=0.3)

        # Efficiency vs Time
        ax6.plot(time_vec, efficiency_vec, 'c-', linewidth=2)
        ax6.set_xlabel('Time (s)', fontweight='bold')
        ax6.set_ylabel('Efficiency (%)', fontweight='bold')
        ax6.set_title('Efficiency Dynamics', fontweight='bold')
        ax6.grid(True, alpha=0.3)

        self.fig_simulation.tight_layout()
        self.canvas_simulation.draw()

    def calculate_economics(self):
        """Calculate and display economic analysis"""
        self.params.electricity_cost = self.elec_cost_var.get()
        self.params.maintenance_cost = self.maint_cost_var.get()

        # Calculate operating current
        current = self.motor_model.calculate_generator_current(
            self.params.target_speed, self.params.total_resistance, self.params.voltage
        )

        # Power calculation
        k_phi = self.motor_model.flux_current_relation(current)
        back_emf = k_phi * self.params.target_speed
        power_kw = back_emf * current / 1000

        # Economic calculations
        op_cost = self.economic_analyzer.calculate_operating_cost(
            power_kw, self.op_hours_var.get()
        )

        # Efficiency analysis
        losses = self.motor_model.calculate_power_loss(current, self.params.target_speed)
        efficiency = power_kw * 1000 / (power_kw * 1000 + losses['total'])

        eff_impact = self.economic_analyzer.calculate_efficiency_impact(efficiency)

        # Payback analysis (example improvement investment)
        investment = 5000  # $
        annual_savings = eff_impact['extra_annual_cost'] * 0.5  # 50% improvement
        payback = self.economic_analyzer.payback_analysis(investment, annual_savings)

        # Display results
        self.economic_text.delete('1.0', tk.END)

        results = f"""
═══════════════════════════════════════════════════════════
               ECONOMIC ANALYSIS REPORT
═══════════════════════════════════════════════════════════

OPERATING PARAMETERS:
───────────────────────────────────────────────────────────
  Power Output:              {power_kw:.2f} kW
  Operating Hours/Year:      {self.op_hours_var.get():.0f} hours
  Electricity Rate:          ${self.params.electricity_cost:.3f}/kWh
  Maintenance Cost:          ${self.params.maintenance_cost:.2f}/year

ANNUAL OPERATING COSTS:
───────────────────────────────────────────────────────────
  Energy Cost:               ${op_cost['energy_cost']:.2f}/year
  Maintenance Cost:          ${op_cost['maintenance_cost']:.2f}/year
  ─────────────────────────
  Total Annual Cost:         ${op_cost['total_annual_cost']:.2f}/year
  Cost per Operating Hour:   ${op_cost['cost_per_hour']:.3f}/hour

EFFICIENCY ANALYSIS:
───────────────────────────────────────────────────────────
  Actual Efficiency:         {eff_impact['efficiency_actual']*100:.2f}%
  Target Efficiency:         {eff_impact['efficiency_target']*100:.2f}%
  Efficiency Gap:            {(eff_impact['efficiency_target']-eff_impact['efficiency_actual'])*100:.2f}%

  Additional Cost (Gap):     ${eff_impact['extra_annual_cost']:.2f}/year
  Potential Savings:         ${eff_impact['savings_potential']:.2f}/year

INVESTMENT ANALYSIS (Efficiency Improvement):
───────────────────────────────────────────────────────────
  Investment Required:       ${investment:.2f}
  Annual Savings:            ${annual_savings:.2f}
  Payback Period:            {payback['payback_years']:.2f} years
  10-Year ROI:               {payback['roi_10year']:.1f}%
  Net Savings (10 years):    ${payback['net_savings_10year']:.2f}

COST BREAKDOWN (per kWh):
───────────────────────────────────────────────────────────
  Energy:                    ${self.params.electricity_cost:.3f}
  Maintenance:               ${self.params.maintenance_cost/(power_kw*self.op_hours_var.get()):.4f}
  Total:                     ${op_cost['cost_per_hour']/power_kw:.4f}

LIFETIME ANALYSIS (10 years):
───────────────────────────────────────────────────────────
  Total Energy Cost:         ${op_cost['energy_cost']*10:.2f}
  Total Maintenance:         ${op_cost['maintenance_cost']*10:.2f}
  Total Lifetime Cost:       ${op_cost['total_annual_cost']*10:.2f}

  With Efficiency Upgrade:   ${(op_cost['total_annual_cost']-annual_savings)*10:.2f}
  Lifetime Savings:          ${annual_savings*10:.2f}

RECOMMENDATIONS:
───────────────────────────────────────────────────────────
"""

        if payback['payback_years'] < 3:
            results += "  ✓ Efficiency upgrade highly recommended (short payback)\n"
        elif payback['payback_years'] < 5:
            results += "  → Consider efficiency upgrade (moderate payback)\n"
        else:
            results += "  × Efficiency upgrade may not be cost-effective\n"

        if efficiency < 0.80:
            results += "  ⚠ Low efficiency - investigate motor condition\n"

        results += "\n═══════════════════════════════════════════════════════════\n"

        self.economic_text.insert('1.0', results)

    def plot_thermal_analysis(self):
        """Plot thermal analysis and derating curves"""
        self.fig_thermal.clear()

        # Temperature range
        temp_ambient = np.linspace(0, 60, 50)
        current_range = np.linspace(50, 250, 50)

        # Calculate derating factors
        derating_factors = []
        max_temp = 155  # °C, Class F insulation

        for temp in temp_ambient:
            # Available temperature rise
            available_rise = max_temp - temp
            reference_rise = max_temp - 25
            derating = available_rise / reference_rise
            derating_factors.append(max(0, min(1, derating)))

        # Calculate temperature rise for different currents
        temp_rise_data = []
        for curr in current_range:
            speed = self.motor_model.speed_vs_current(curr)
            losses = self.motor_model.calculate_power_loss(curr, speed)
            temp_rise = losses['copper'] * self.params.thermal_resistance
            temp_rise_data.append(temp_rise)

        # Create subplots
        ax1 = self.fig_thermal.add_subplot(221)
        ax2 = self.fig_thermal.add_subplot(222)
        ax3 = self.fig_thermal.add_subplot(223)
        ax4 = self.fig_thermal.add_subplot(224)

        # Derating curve
        ax1.plot(temp_ambient, derating_factors, 'r-', linewidth=2)
        ax1.axvline(x=self.params.ambient_temp, color='g', linestyle='--', label='Current Ambient')
        ax1.fill_between(temp_ambient, 0, derating_factors, alpha=0.3)
        ax1.set_xlabel('Ambient Temperature (°C)', fontweight='bold')
        ax1.set_ylabel('Derating Factor', fontweight='bold')
        ax1.set_title('Temperature Derating Curve', fontweight='bold')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim([0, 1.2])

        # Temperature rise vs current
        ax2.plot(current_range, temp_rise_data, 'orange', linewidth=2)
        ax2.axhline(y=max_temp-25, color='r', linestyle='--', label='Max Rise (Class F)')
        ax2.set_xlabel('Current (A)', fontweight='bold')
        ax2.set_ylabel('Temperature Rise (°C)', fontweight='bold')
        ax2.set_title('Winding Temperature Rise', fontweight='bold')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Thermal time constant
        time_thermal = np.linspace(0, 3600, 100)  # 1 hour
        thermal_time_constant = self.params.thermal_capacitance * self.params.thermal_resistance

        # Step response for different power levels
        for power_loss in [500, 1000, 2000]:
            steady_state_temp = self.params.ambient_temp + power_loss * self.params.thermal_resistance
            temp_transient = self.params.ambient_temp + (steady_state_temp - self.params.ambient_temp) * \
                           (1 - np.exp(-time_thermal / thermal_time_constant))
            ax3.plot(time_thermal/60, temp_transient, linewidth=2, label=f'{power_loss}W loss')

        ax3.axhline(y=max_temp, color='r', linestyle='--', label='Max Temp (Class F)')
        ax3.set_xlabel('Time (minutes)', fontweight='bold')
        ax3.set_ylabel('Temperature (°C)', fontweight='bold')
        ax3.set_title('Thermal Transient Response', fontweight='bold')
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # Thermal resistance network
        # Create a simple bar chart showing thermal resistances
        components = ['Winding-Core', 'Core-Frame', 'Frame-Ambient']
        resistances = [0.5, 1.0, 1.0]  # °C/W

        ax4.bar(components, resistances, color=['#ff9999', '#66b3ff', '#99ff99'])
        ax4.set_ylabel('Thermal Resistance (°C/W)', fontweight='bold')
        ax4.set_title('Thermal Resistance Network', fontweight='bold')
        ax4.grid(True, alpha=0.3, axis='y')

        self.fig_thermal.tight_layout()

    def plot_mechanical_analysis(self):
        """Plot mechanical stress and bearing analysis"""
        self.fig_mechanical.clear()

        current_range = np.linspace(50, 250, 50)

        torque_data = []
        stress_data = []
        bearing_radial = []
        bearing_axial = []
        safety_factors = []

        for curr in current_range:
            speed = self.motor_model.speed_vs_current(curr)
            stress = self.simulator.calculate_mechanical_stress(curr, speed)

            torque_data.append(stress['torque'])
            stress_data.append(stress['shaft_stress'])
            bearing_radial.append(stress['radial_bearing_load'])
            bearing_axial.append(stress['axial_bearing_load'])
            safety_factors.append(stress['safety_factor'])

        # Create subplots
        ax1 = self.fig_mechanical.add_subplot(221)
        ax2 = self.fig_mechanical.add_subplot(222)
        ax3 = self.fig_mechanical.add_subplot(223)
        ax4 = self.fig_mechanical.add_subplot(224)

        # Torque vs Current
        ax1.plot(current_range, torque_data, 'b-', linewidth=2)
        ax1.set_xlabel('Current (A)', fontweight='bold')
        ax1.set_ylabel('Torque (N·m)', fontweight='bold')
        ax1.set_title('Electromagnetic Torque', fontweight='bold')
        ax1.grid(True, alpha=0.3)

        # Shaft stress
        ax2.plot(current_range, stress_data, 'r-', linewidth=2)
        ax2.axhline(y=250, color='orange', linestyle='--', label='Yield Strength (250 MPa)')
        ax2.set_xlabel('Current (A)', fontweight='bold')
        ax2.set_ylabel('Shear Stress (MPa)', fontweight='bold')
        ax2.set_title('Shaft Shear Stress', fontweight='bold')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Bearing loads
        ax3.plot(current_range, bearing_radial, 'g-', linewidth=2, label='Radial')
        ax3.plot(current_range, bearing_axial, 'm-', linewidth=2, label='Axial')
        ax3.set_xlabel('Current (A)', fontweight='bold')
        ax3.set_ylabel('Load (N)', fontweight='bold')
        ax3.set_title('Bearing Loads', fontweight='bold')
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # Safety factor
        ax4.plot(current_range, safety_factors, 'c-', linewidth=2)
        ax4.axhline(y=2.0, color='r', linestyle='--', label='Min Safety Factor')
        ax4.set_xlabel('Current (A)', fontweight='bold')
        ax4.set_ylabel('Safety Factor', fontweight='bold')
        ax4.set_title('Mechanical Safety Factor', fontweight='bold')
        ax4.legend()
        ax4.grid(True, alpha=0.3)

        self.fig_mechanical.tight_layout()

    def reset_parameters(self):
        """Reset all parameters to default values"""
        self.voltage_var.set(525.0)
        self.speed_var.set(1000.0)
        self.resistance_var.set(3.25)

        self.params.voltage = 525.0
        self.params.target_speed = 1000.0
        self.params.resistance_load = 3.25
        self.params.total_resistance = 6.75

        self.voltage_label.config(text="525.0 V")
        self.speed_label.config(text="1000.0 rpm")
        self.resistance_label.config(text="3.25 Ω")

        self.calculate_generator_operation()

    def export_data(self):
        """Export results to CSV file"""
        try:
            import csv
            from datetime import datetime

            filename = f"motor_analysis_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

            current = self.motor_model.calculate_generator_current(
                self.params.target_speed, self.params.total_resistance, self.params.voltage
            )

            k_phi = self.motor_model.flux_current_relation(current)
            back_emf = k_phi * self.params.target_speed
            losses = self.motor_model.calculate_power_loss(current, self.params.target_speed)
            stress = self.simulator.calculate_mechanical_stress(current, self.params.target_speed)

            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Parameter', 'Value', 'Unit'])
                writer.writerow(['Operating Mode', 'Generator', ''])
                writer.writerow(['Speed', self.params.target_speed, 'rpm'])
                writer.writerow(['Voltage', self.params.voltage, 'V'])
                writer.writerow(['Current', current, 'A'])
                writer.writerow(['Back EMF', back_emf, 'V'])
                writer.writerow(['Torque', stress['torque'], 'N.m'])
                writer.writerow(['Copper Loss', losses['copper'], 'W'])
                writer.writerow(['Iron Loss', losses['iron'], 'W'])
                writer.writerow(['Mechanical Loss', losses['mechanical'], 'W'])
                writer.writerow(['Total Loss', losses['total'], 'W'])

            messagebox.showinfo("Export Success", f"Data exported to {filename}")

        except Exception as e:
            messagebox.showerror("Export Error", f"Failed to export data: {str(e)}")


def main():
    """Main entry point"""
    app = AdvancedMotorLab()
    app.mainloop()


if __name__ == "__main__":
    main()
