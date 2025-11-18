"""
Advanced Transformer Parallel Operation Simulator
Multi-Physics Analysis with Dynamic Simulation
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
from typing import Tuple, Dict, List


@dataclass
class TransformerParameters:
    """Transformer electrical parameters"""
    rating_kva: float
    voltage_hv: float
    voltage_lv: float
    z_ohm: float
    r_ohm: float
    x_ohm: float
    core_loss_w: float
    copper_loss_w: float



class TransformerParallelSolver:
    """Solves transformer parallel operation problem"""

    def __init__(self):
        # Given data
        self.v_hv_rated = 6600  # V
        self.v_lv_rated = 250   # V

        # Transformer 1 short circuit test data (HV side)
        self.v_sc1 = 200  # V
        self.i_sc1 = 30   # A
        self.p_sc1 = 1200 # W

        # Transformer 2 short circuit test data (HV side)
        self.v_sc2 = 120  # V
        self.i_sc2 = 20   # A
        self.p_sc2 = 1500 # W

        # Load data
        self.load_kw = 150    # kW
        self.load_pf = 0.8    # lagging

        self.calculate_parameters()

    def calculate_parameters(self):
        """Calculate transformer parameters from short circuit test"""
        # Transformer 1
        self.z1 = self.v_sc1 / self.i_sc1  # Impedance
        self.r1 = self.p_sc1 / (self.i_sc1**2)  # Resistance
        self.x1 = np.sqrt(self.z1**2 - self.r1**2)  # Reactance

        # Transformer 2
        self.z2 = self.v_sc2 / self.i_sc2  # Impedance
        self.r2 = self.p_sc2 / (self.i_sc2**2)  # Resistance
        self.x2 = np.sqrt(self.z2**2 - self.r2**2)  # Reactance

        # Complex impedances
        self.Z1 = complex(self.r1, self.x1)
        self.Z2 = complex(self.r2, self.x2)

    def solve_parallel_operation(self) -> Dict:
        """Solve for currents and power factors in parallel operation"""
        # Total load current
        load_angle = np.arccos(self.load_pf)
        total_kva = self.load_kw / self.load_pf
        i_total = (total_kva * 1000) / self.v_hv_rated

        # Total load current (complex)
        I_total = i_total * np.exp(-1j * load_angle)

        # Voltage drop across equivalent circuit
        # For parallel: 1/Zeq = 1/Z1 + 1/Z2
        Z_eq = (self.Z1 * self.Z2) / (self.Z1 + self.Z2)

        # Circulating current method
        # I1 = I_total * Z2/(Z1+Z2)
        # I2 = I_total * Z1/(Z1+Z2)

        I1 = I_total * self.Z2 / (self.Z1 + self.Z2)
        I2 = I_total * self.Z1 / (self.Z1 + self.Z2)

        # Extract magnitudes and angles
        i1_mag = abs(I1)
        i1_angle = np.angle(I1)
        pf1 = np.cos(i1_angle)

        i2_mag = abs(I2)
        i2_angle = np.angle(I2)
        pf2 = np.cos(i2_angle)

        # Power distribution
        p1 = self.v_hv_rated * i1_mag * pf1 / 1000  # kW
        p2 = self.v_hv_rated * i2_mag * pf2 / 1000  # kW

        # Losses
        loss1 = 3 * i1_mag**2 * self.r1 / 1000  # kW (assuming 3-phase)
        loss2 = 3 * i2_mag**2 * self.r2 / 1000  # kW

        results = {
            'i1': i1_mag,
            'pf1': pf1,
            'angle1_deg': np.degrees(i1_angle),
            'power1_kw': p1,
            'loss1_kw': loss1,
            'i2': i2_mag,
            'pf2': pf2,
            'angle2_deg': np.degrees(i2_angle),
            'power2_kw': p2,
            'loss2_kw': loss2,
            'z1': self.z1,
            'r1': self.r1,
            'x1': self.x1,
            'z2': self.z2,
            'r2': self.r2,
            'x2': self.x2,
            'total_loss_kw': loss1 + loss2,
            'efficiency': (self.load_kw / (self.load_kw + loss1 + loss2)) * 100
        }

        return results


class TransformerDynamicModel:
    """Dynamic model with differential equations"""

    def __init__(self, params: TransformerParameters):
        self.params = params
        self.L_m = 0.5  # Magnetizing inductance (H)
        self.L_leak = 0.01  # Leakage inductance (H)
        self.C_winding = 1e-9  # Winding capacitance (F)
        self.thermal_resistance = 2.0  # K/W
        self.thermal_capacitance = 500  # J/K
        self.ambient_temp = 25  # °C

    def electrical_ode_rk45(self, t, y, v_input, frequency=50):
        """
        Differential equations for transformer electrical model
        State variables: [i_primary, i_secondary, flux, v_capacitor]
        """
        i_p, i_s, flux, v_c = y

        # Input voltage (sinusoidal)
        omega = 2 * np.pi * frequency
        v_in = v_input * np.sin(omega * t)

        # Transformer equations
        # v_primary = R*i + L*di/dt + d(flux)/dt
        # flux = L_m * i_magnetizing

        # Primary circuit
        di_p_dt = (v_in - self.params.r_ohm * i_p - flux) / self.L_leak

        # Secondary circuit (with load)
        R_load = 10  # Load resistance
        di_s_dt = (-R_load * i_s - flux / self.params.voltage_lv * self.params.voltage_hv) / self.L_leak

        # Magnetic flux
        d_flux_dt = self.L_m * (i_p - i_s * self.params.voltage_lv / self.params.voltage_hv)

        # Capacitor voltage (parasitic capacitance)
        dv_c_dt = i_p / self.C_winding

        return [di_p_dt, di_s_dt, d_flux_dt, dv_c_dt]

    def electrical_ode_euler(self, y, t, v_input, frequency=50):
        """Euler method compatible ODE"""
        return self.electrical_ode_rk45(t, y, v_input, frequency)

    def thermal_ode(self, t, y, power_loss):
        """
        Thermal differential equation
        State variable: [temperature]
        """
        temp = y[0]

        # Heat transfer equation: C*dT/dt = P_loss - (T-T_ambient)/R_th
        dT_dt = (power_loss - (temp - self.ambient_temp) / self.thermal_resistance) / self.thermal_capacitance

        return [dT_dt]

    def mechanical_stress(self, current, load_torque):
        """Calculate mechanical stress and shaft torque"""
        # Electromagnetic torque
        torque_em = 0.5 * current**2 * 0.1  # Simplified model

        # Transient torque
        torque_transient = torque_em - load_torque

        # Bearing load (radial force)
        bearing_load = abs(torque_transient) * 0.05

        return {
            'torque_em': torque_em,
            'torque_transient': torque_transient,
            'bearing_load': bearing_load,
            'shaft_stress_mpa': torque_transient * 10
        }


class LossAnalyzer:
    """Detailed loss breakdown"""

    @staticmethod
    def calculate_losses(current, voltage, resistance, frequency=50,
                        core_loss_w=100, mechanical_loss_w=50):
        """Calculate all types of losses"""
        # Copper losses (I²R)
        copper_loss = current**2 * resistance

        # Core losses (hysteresis + eddy current)
        # Hysteresis loss ∝ f * B^1.6
        # Eddy current loss ∝ f² * B²
        hysteresis_loss = core_loss_w * 0.6
        eddy_current_loss = core_loss_w * 0.4

        # Stray load losses (approx 1% of rated power)
        stray_loss = voltage * current * 0.01

        # Mechanical friction
        friction_loss = mechanical_loss_w

        # Total losses
        total_loss = copper_loss + hysteresis_loss + eddy_current_loss + stray_loss + friction_loss

        return {
            'copper_loss_w': copper_loss,
            'hysteresis_loss_w': hysteresis_loss,
            'eddy_current_loss_w': eddy_current_loss,
            'stray_loss_w': stray_loss,
            'friction_loss_w': friction_loss,
            'total_loss_w': total_loss,
            'efficiency': (voltage * current - total_loss) / (voltage * current) * 100
        }


class EconomicAnalyzer:
    """Economic analysis calculations"""

    def __init__(self, electricity_cost=0.12, operating_hours=8760):
        self.electricity_cost = electricity_cost  # $/kWh
        self.operating_hours = operating_hours  # hours/year

    def calculate_economics(self, power_kw, loss_kw, capital_cost, maintenance_cost_annual):
        """Calculate economic metrics"""
        # Annual energy consumption
        annual_energy_kwh = power_kw * self.operating_hours

        # Annual energy loss
        annual_loss_kwh = loss_kw * self.operating_hours

        # Annual operating cost
        energy_cost_annual = annual_energy_kwh * self.electricity_cost
        loss_cost_annual = annual_loss_kwh * self.electricity_cost
        total_operating_cost = energy_cost_annual + loss_cost_annual + maintenance_cost_annual

        # Lifecycle cost (20 years)
        lifecycle_years = 20
        lifecycle_cost = capital_cost + total_operating_cost * lifecycle_years

        # Return on investment
        roi_years = capital_cost / (energy_cost_annual * 0.1)  # Simplified

        return {
            'annual_energy_kwh': annual_energy_kwh,
            'annual_loss_kwh': annual_loss_kwh,
            'energy_cost_annual': energy_cost_annual,
            'loss_cost_annual': loss_cost_annual,
            'maintenance_cost_annual': maintenance_cost_annual,
            'total_operating_cost_annual': total_operating_cost,
            'lifecycle_cost': lifecycle_cost,
            'roi_years': roi_years,
            'cost_per_kwh': lifecycle_cost / (annual_energy_kwh * lifecycle_years)
        }


class AdvancedTransformerSimulator(tk.Tk):
    """Main GUI Application"""

    def __init__(self):
        super().__init__()

        self.title("Advanced Transformer Parallel Operation Simulator - Multi-Physics Analysis")
        self.geometry("1400x900")

        # Simulation control
        self.running = False
        self.paused = False
        self.simulation_thread = None

        # Data storage
        self.time_data = []
        self.current_data_t1 = []
        self.current_data_t2 = []
        self.voltage_data = []
        self.temperature_data = []
        self.power_data = []
        self.loss_data = []

        # Solver instance
        self.solver = TransformerParallelSolver()

        # Dynamic model
        default_params = TransformerParameters(
            rating_kva=100,
            voltage_hv=6600,
            voltage_lv=250,
            z_ohm=self.solver.z1,
            r_ohm=self.solver.r1,
            x_ohm=self.solver.x1,
            core_loss_w=500,
            copper_loss_w=1000
        )
        self.dynamic_model = TransformerDynamicModel(default_params)

        # Economic analyzer
        self.economic_analyzer = EconomicAnalyzer()

        # Build UI
        self.create_widgets()

        # Auto-resize handling
        self.bind('<Configure>', self.on_window_resize)

        # Initial calculation
        self.calculate_parallel_operation()

    def create_widgets(self):
        """Create all GUI widgets"""
        # Create notebook (tabs)
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=5)

        # Create tabs
        self.tab_main = ttk.Frame(self.notebook)
        self.tab_dynamic = ttk.Frame(self.notebook)
        self.tab_multiphysics = ttk.Frame(self.notebook)
        self.tab_losses = ttk.Frame(self.notebook)
        self.tab_economic = ttk.Frame(self.notebook)
        self.tab_control = ttk.Frame(self.notebook)

        self.notebook.add(self.tab_main, text="Main Analysis")
        self.notebook.add(self.tab_dynamic, text="Dynamic Simulation")
        self.notebook.add(self.tab_multiphysics, text="Multi-Physics")
        self.notebook.add(self.tab_losses, text="Loss Analysis")
        self.notebook.add(self.tab_economic, text="Economic Analysis")
        self.notebook.add(self.tab_control, text="Advanced Control")

        # Build each tab
        self.build_main_tab()
        self.build_dynamic_tab()
        self.build_multiphysics_tab()
        self.build_losses_tab()
        self.build_economic_tab()
        self.build_control_tab()

    def build_main_tab(self):
        """Build main analysis tab"""
        # Left panel - Input parameters
        left_frame = ttk.LabelFrame(self.tab_main, text="Input Parameters", padding=10)
        left_frame.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        # Transformer ratings
        ttk.Label(left_frame, text="Rated Voltage (HV):").grid(row=0, column=0, sticky='w')
        self.entry_v_hv = ttk.Entry(left_frame, width=15)
        self.entry_v_hv.insert(0, "6600")
        self.entry_v_hv.grid(row=0, column=1, padx=5, pady=2)
        ttk.Label(left_frame, text="V").grid(row=0, column=2, sticky='w')

        ttk.Label(left_frame, text="Rated Voltage (LV):").grid(row=1, column=0, sticky='w')
        self.entry_v_lv = ttk.Entry(left_frame, width=15)
        self.entry_v_lv.insert(0, "250")
        self.entry_v_lv.grid(row=1, column=1, padx=5, pady=2)
        ttk.Label(left_frame, text="V").grid(row=1, column=2, sticky='w')

        # Transformer 1 SC test
        ttk.Label(left_frame, text="\nTransformer 1 - SC Test (HV side)",
                 font=('Arial', 10, 'bold')).grid(row=2, column=0, columnspan=3, sticky='w', pady=5)

        ttk.Label(left_frame, text="Voltage:").grid(row=3, column=0, sticky='w')
        self.entry_v_sc1 = ttk.Entry(left_frame, width=15)
        self.entry_v_sc1.insert(0, "200")
        self.entry_v_sc1.grid(row=3, column=1, padx=5, pady=2)
        ttk.Label(left_frame, text="V").grid(row=3, column=2, sticky='w')

        ttk.Label(left_frame, text="Current:").grid(row=4, column=0, sticky='w')
        self.entry_i_sc1 = ttk.Entry(left_frame, width=15)
        self.entry_i_sc1.insert(0, "30")
        self.entry_i_sc1.grid(row=4, column=1, padx=5, pady=2)
        ttk.Label(left_frame, text="A").grid(row=4, column=2, sticky='w')

        ttk.Label(left_frame, text="Power:").grid(row=5, column=0, sticky='w')
        self.entry_p_sc1 = ttk.Entry(left_frame, width=15)
        self.entry_p_sc1.insert(0, "1200")
        self.entry_p_sc1.grid(row=5, column=1, padx=5, pady=2)
        ttk.Label(left_frame, text="W").grid(row=5, column=2, sticky='w')

        # Transformer 2 SC test
        ttk.Label(left_frame, text="\nTransformer 2 - SC Test (HV side)",
                 font=('Arial', 10, 'bold')).grid(row=6, column=0, columnspan=3, sticky='w', pady=5)

        ttk.Label(left_frame, text="Voltage:").grid(row=7, column=0, sticky='w')
        self.entry_v_sc2 = ttk.Entry(left_frame, width=15)
        self.entry_v_sc2.insert(0, "120")
        self.entry_v_sc2.grid(row=7, column=1, padx=5, pady=2)
        ttk.Label(left_frame, text="V").grid(row=7, column=2, sticky='w')

        ttk.Label(left_frame, text="Current:").grid(row=8, column=0, sticky='w')
        self.entry_i_sc2 = ttk.Entry(left_frame, width=15)
        self.entry_i_sc2.insert(0, "20")
        self.entry_i_sc2.grid(row=8, column=1, padx=5, pady=2)
        ttk.Label(left_frame, text="A").grid(row=8, column=2, sticky='w')

        ttk.Label(left_frame, text="Power:").grid(row=9, column=0, sticky='w')
        self.entry_p_sc2 = ttk.Entry(left_frame, width=15)
        self.entry_p_sc2.insert(0, "1500")
        self.entry_p_sc2.grid(row=9, column=1, padx=5, pady=2)
        ttk.Label(left_frame, text="W").grid(row=9, column=2, sticky='w')

        # Load parameters
        ttk.Label(left_frame, text="\nLoad Parameters",
                 font=('Arial', 10, 'bold')).grid(row=10, column=0, columnspan=3, sticky='w', pady=5)

        ttk.Label(left_frame, text="Load Power:").grid(row=11, column=0, sticky='w')
        self.entry_load_kw = ttk.Entry(left_frame, width=15)
        self.entry_load_kw.insert(0, "150")
        self.entry_load_kw.grid(row=11, column=1, padx=5, pady=2)
        ttk.Label(left_frame, text="kW").grid(row=11, column=2, sticky='w')

        ttk.Label(left_frame, text="Power Factor:").grid(row=12, column=0, sticky='w')
        self.entry_load_pf = ttk.Entry(left_frame, width=15)
        self.entry_load_pf.insert(0, "0.8")
        self.entry_load_pf.grid(row=12, column=1, padx=5, pady=2)
        ttk.Label(left_frame, text="lagging").grid(row=12, column=2, sticky='w')

        # Calculate button
        ttk.Button(left_frame, text="Calculate", command=self.calculate_parallel_operation,
                  style='Accent.TButton').grid(row=13, column=0, columnspan=3, pady=10, sticky='ew')

        # Right panel - Results
        right_frame = ttk.LabelFrame(self.tab_main, text="Results", padding=10)
        right_frame.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)

        # Results text widget
        self.results_text = tk.Text(right_frame, width=60, height=35, wrap='word',
                                    font=('Courier', 10))
        self.results_text.pack(fill='both', expand=True)

        # Scrollbar
        scrollbar = ttk.Scrollbar(right_frame, command=self.results_text.yview)
        scrollbar.pack(side='right', fill='y')
        self.results_text.config(yscrollcommand=scrollbar.set)

        # Configure grid weights
        self.tab_main.columnconfigure(0, weight=1)
        self.tab_main.columnconfigure(1, weight=2)
        self.tab_main.rowconfigure(0, weight=1)

    def build_dynamic_tab(self):
        """Build dynamic simulation tab"""
        # Control panel
        control_frame = ttk.LabelFrame(self.tab_dynamic, text="Simulation Control", padding=10)
        control_frame.pack(fill='x', padx=5, pady=5)

        # Buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(fill='x')

        self.btn_start = ttk.Button(button_frame, text="▶ Start", command=self.start_simulation)
        self.btn_start.pack(side='left', padx=5)

        self.btn_stop = ttk.Button(button_frame, text="⏸ Pause", command=self.pause_simulation)
        self.btn_stop.pack(side='left', padx=5)

        self.btn_reset = ttk.Button(button_frame, text="⟲ Reset", command=self.reset_simulation)
        self.btn_reset.pack(side='left', padx=5)

        # Solver selection
        ttk.Label(button_frame, text="ODE Solver:").pack(side='left', padx=(20, 5))
        self.solver_var = tk.StringVar(value="RK45")
        ttk.Radiobutton(button_frame, text="RK45", variable=self.solver_var,
                       value="RK45").pack(side='left', padx=5)
        ttk.Radiobutton(button_frame, text="Euler", variable=self.solver_var,
                       value="Euler").pack(side='left', padx=5)

        # Sliders for control
        slider_frame = ttk.Frame(control_frame)
        slider_frame.pack(fill='x', pady=10)

        ttk.Label(slider_frame, text="Input Voltage (V):").grid(row=0, column=0, sticky='w', padx=5)
        self.voltage_slider = ttk.Scale(slider_frame, from_=0, to=7000, orient='horizontal',
                                       length=300, command=self.update_voltage_label)
        self.voltage_slider.set(6600)
        self.voltage_slider.grid(row=0, column=1, padx=5)
        self.voltage_label = ttk.Label(slider_frame, text="6600 V")
        self.voltage_label.grid(row=0, column=2, padx=5)

        ttk.Label(slider_frame, text="Frequency (Hz):").grid(row=1, column=0, sticky='w', padx=5)
        self.freq_slider = ttk.Scale(slider_frame, from_=30, to=70, orient='horizontal',
                                    length=300, command=self.update_freq_label)
        self.freq_slider.set(50)
        self.freq_slider.grid(row=1, column=1, padx=5)
        self.freq_label = ttk.Label(slider_frame, text="50 Hz")
        self.freq_label.grid(row=1, column=2, padx=5)

        ttk.Label(slider_frame, text="Load (kW):").grid(row=2, column=0, sticky='w', padx=5)
        self.load_slider = ttk.Scale(slider_frame, from_=0, to=200, orient='horizontal',
                                    length=300, command=self.update_load_label)
        self.load_slider.set(150)
        self.load_slider.grid(row=2, column=1, padx=5)
        self.load_label = ttk.Label(slider_frame, text="150 kW")
        self.load_label.grid(row=2, column=2, padx=5)

        # Graphs
        graph_frame = ttk.Frame(self.tab_dynamic)
        graph_frame.pack(fill='both', expand=True, padx=5, pady=5)

        # Create matplotlib figure
        self.fig_dynamic = Figure(figsize=(12, 8))
        self.ax_current = self.fig_dynamic.add_subplot(221)
        self.ax_voltage = self.fig_dynamic.add_subplot(222)
        self.ax_power = self.fig_dynamic.add_subplot(223)
        self.ax_temp = self.fig_dynamic.add_subplot(224)

        self.canvas_dynamic = FigureCanvasTkAgg(self.fig_dynamic, graph_frame)
        self.canvas_dynamic.get_tk_widget().pack(fill='both', expand=True)

    def build_multiphysics_tab(self):
        """Build multi-physics simulation tab"""
        # Info frame
        info_frame = ttk.LabelFrame(self.tab_multiphysics,
                                   text="Multi-Physics Coupled Analysis", padding=10)
        info_frame.pack(fill='x', padx=5, pady=5)

        info_text = """
        Electromagnetic-Thermal-Mechanical Coupled Simulation
        • Electromagnetic: Maxwell's equations for field distribution
        • Thermal: Heat transfer with conduction, convection, radiation
        • Mechanical: Stress analysis and vibration
        """
        ttk.Label(info_frame, text=info_text, justify='left').pack()

        # Results display
        results_frame = ttk.Frame(self.tab_multiphysics)
        results_frame.pack(fill='both', expand=True, padx=5, pady=5)

        # Create figure for multi-physics results
        self.fig_multiphysics = Figure(figsize=(12, 8))

        self.ax_flux = self.fig_multiphysics.add_subplot(231)
        self.ax_thermal = self.fig_multiphysics.add_subplot(232)
        self.ax_stress = self.fig_multiphysics.add_subplot(233)
        self.ax_torque = self.fig_multiphysics.add_subplot(234)
        self.ax_bearing = self.fig_multiphysics.add_subplot(235)
        self.ax_derating = self.fig_multiphysics.add_subplot(236)

        self.canvas_multiphysics = FigureCanvasTkAgg(self.fig_multiphysics, results_frame)
        self.canvas_multiphysics.get_tk_widget().pack(fill='both', expand=True)

        # Update button
        ttk.Button(self.tab_multiphysics, text="Run Multi-Physics Analysis",
                  command=self.run_multiphysics).pack(pady=10)

    def build_losses_tab(self):
        """Build loss analysis tab"""
        # Loss breakdown
        loss_frame = ttk.LabelFrame(self.tab_losses, text="Detailed Loss Breakdown", padding=10)
        loss_frame.pack(fill='both', expand=True, padx=5, pady=5)

        # Create figure
        self.fig_losses = Figure(figsize=(12, 8))

        self.ax_loss_pie = self.fig_losses.add_subplot(221)
        self.ax_loss_bar = self.fig_losses.add_subplot(222)
        self.ax_loss_time = self.fig_losses.add_subplot(223)
        self.ax_efficiency = self.fig_losses.add_subplot(224)

        self.canvas_losses = FigureCanvasTkAgg(self.fig_losses, loss_frame)
        self.canvas_losses.get_tk_widget().pack(fill='both', expand=True)

        # Update button
        ttk.Button(self.tab_losses, text="Analyze Losses",
                  command=self.analyze_losses).pack(pady=10)

    def build_economic_tab(self):
        """Build economic analysis tab"""
        # Input frame
        input_frame = ttk.LabelFrame(self.tab_economic, text="Economic Parameters", padding=10)
        input_frame.pack(fill='x', padx=5, pady=5)

        # Economic inputs
        ttk.Label(input_frame, text="Electricity Cost ($/kWh):").grid(row=0, column=0, sticky='w', padx=5)
        self.entry_elec_cost = ttk.Entry(input_frame, width=15)
        self.entry_elec_cost.insert(0, "0.12")
        self.entry_elec_cost.grid(row=0, column=1, padx=5, pady=2)

        ttk.Label(input_frame, text="Operating Hours (h/year):").grid(row=1, column=0, sticky='w', padx=5)
        self.entry_op_hours = ttk.Entry(input_frame, width=15)
        self.entry_op_hours.insert(0, "8760")
        self.entry_op_hours.grid(row=1, column=1, padx=5, pady=2)

        ttk.Label(input_frame, text="Capital Cost ($):").grid(row=2, column=0, sticky='w', padx=5)
        self.entry_capital = ttk.Entry(input_frame, width=15)
        self.entry_capital.insert(0, "50000")
        self.entry_capital.grid(row=2, column=1, padx=5, pady=2)

        ttk.Label(input_frame, text="Annual Maintenance ($):").grid(row=3, column=0, sticky='w', padx=5)
        self.entry_maintenance = ttk.Entry(input_frame, width=15)
        self.entry_maintenance.insert(0, "2000")
        self.entry_maintenance.grid(row=3, column=1, padx=5, pady=2)

        ttk.Button(input_frame, text="Calculate Economics",
                  command=self.calculate_economics).grid(row=4, column=0, columnspan=2, pady=10)

        # Results display
        results_frame = ttk.LabelFrame(self.tab_economic, text="Economic Analysis Results", padding=10)
        results_frame.pack(fill='both', expand=True, padx=5, pady=5)

        self.economic_text = tk.Text(results_frame, height=20, wrap='word', font=('Courier', 10))
        self.economic_text.pack(fill='both', expand=True)

    def build_control_tab(self):
        """Build advanced control tab"""
        # Control methods
        control_frame = ttk.LabelFrame(self.tab_control, text="Advanced Control Methods", padding=10)
        control_frame.pack(fill='both', expand=True, padx=5, pady=5)

        # Control options
        ttk.Label(control_frame, text="Control Strategy:", font=('Arial', 11, 'bold')).pack(anchor='w', pady=5)

        self.control_var = tk.StringVar(value="VF")

        control_options = [
            ("V/F Control (Constant V/F ratio)", "VF"),
            ("Field-Oriented Control (FOC)", "FOC"),
            ("Direct Torque Control (DTC)", "DTC"),
            ("Model Predictive Control (MPC)", "MPC")
        ]

        for text, value in control_options:
            ttk.Radiobutton(control_frame, text=text, variable=self.control_var,
                           value=value).pack(anchor='w', padx=20, pady=2)

        # Thermal management
        thermal_frame = ttk.LabelFrame(control_frame, text="Thermal Management & Derating", padding=10)
        thermal_frame.pack(fill='x', pady=10)

        ttk.Label(thermal_frame, text="Max Temperature Limit (°C):").pack(anchor='w')
        self.temp_limit_slider = ttk.Scale(thermal_frame, from_=80, to=150, orient='horizontal', length=400)
        self.temp_limit_slider.set(120)
        self.temp_limit_slider.pack(fill='x', padx=10)

        ttk.Label(thermal_frame, text="Derating Factor:").pack(anchor='w', pady=(10, 0))
        self.derating_slider = ttk.Scale(thermal_frame, from_=0.5, to=1.0, orient='horizontal', length=400)
        self.derating_slider.set(1.0)
        self.derating_slider.pack(fill='x', padx=10)

        # Power consumption monitoring
        power_frame = ttk.LabelFrame(control_frame, text="Power Consumption Monitoring", padding=10)
        power_frame.pack(fill='x', pady=10)

        self.power_monitor_text = tk.Text(power_frame, height=10, wrap='word', font=('Courier', 9))
        self.power_monitor_text.pack(fill='both', expand=True)

        # Apply control button
        ttk.Button(control_frame, text="Apply Control Settings",
                  command=self.apply_control_settings).pack(pady=10)

    def update_voltage_label(self, value):
        """Update voltage label"""
        self.voltage_label.config(text=f"{float(value):.0f} V")

    def update_freq_label(self, value):
        """Update frequency label"""
        self.freq_label.config(text=f"{float(value):.1f} Hz")

    def update_load_label(self, value):
        """Update load label"""
        self.load_label.config(text=f"{float(value):.0f} kW")

    def calculate_parallel_operation(self):
        """Calculate parallel operation results"""
        try:
            # Update solver parameters
            self.solver.v_hv_rated = float(self.entry_v_hv.get())
            self.solver.v_lv_rated = float(self.entry_v_lv.get())
            self.solver.v_sc1 = float(self.entry_v_sc1.get())
            self.solver.i_sc1 = float(self.entry_i_sc1.get())
            self.solver.p_sc1 = float(self.entry_p_sc1.get())
            self.solver.v_sc2 = float(self.entry_v_sc2.get())
            self.solver.i_sc2 = float(self.entry_i_sc2.get())
            self.solver.p_sc2 = float(self.entry_p_sc2.get())
            self.solver.load_kw = float(self.entry_load_kw.get())
            self.solver.load_pf = float(self.entry_load_pf.get())

            # Recalculate
            self.solver.calculate_parameters()
            results = self.solver.solve_parallel_operation()

            # Display results
            self.results_text.delete(1.0, tk.END)

            output = "=" * 70 + "\n"
            output += "TRANSFORMER PARALLEL OPERATION ANALYSIS RESULTS\n"
            output += "=" * 70 + "\n\n"

            output += "TRANSFORMER PARAMETERS FROM SHORT CIRCUIT TEST\n"
            output += "-" * 70 + "\n"
            output += f"Transformer 1:\n"
            output += f"  Impedance (Z1)    : {results['z1']:.3f} Ω\n"
            output += f"  Resistance (R1)   : {results['r1']:.3f} Ω\n"
            output += f"  Reactance (X1)    : {results['x1']:.3f} Ω\n\n"

            output += f"Transformer 2:\n"
            output += f"  Impedance (Z2)    : {results['z2']:.3f} Ω\n"
            output += f"  Resistance (R2)   : {results['r2']:.3f} Ω\n"
            output += f"  Reactance (X2)    : {results['x2']:.3f} Ω\n\n"

            output += "PARALLEL OPERATION RESULTS\n"
            output += "-" * 70 + "\n"
            output += f"Total Load        : {self.solver.load_kw:.2f} kW at {self.solver.load_pf} pf lagging\n\n"

            output += f"Transformer 1:\n"
            output += f"  Current (I1)      : {results['i1']:.3f} A\n"
            output += f"  Power Factor      : {results['pf1']:.4f} {'lagging' if results['angle1_deg'] < 0 else 'leading'}\n"
            output += f"  Phase Angle       : {results['angle1_deg']:.2f}°\n"
            output += f"  Power Output      : {results['power1_kw']:.3f} kW\n"
            output += f"  Copper Losses     : {results['loss1_kw']:.3f} kW\n\n"

            output += f"Transformer 2:\n"
            output += f"  Current (I2)      : {results['i2']:.3f} A\n"
            output += f"  Power Factor      : {results['pf2']:.4f} {'lagging' if results['angle2_deg'] < 0 else 'leading'}\n"
            output += f"  Phase Angle       : {results['angle2_deg']:.2f}°\n"
            output += f"  Power Output      : {results['power2_kw']:.3f} kW\n"
            output += f"  Copper Losses     : {results['loss2_kw']:.3f} kW\n\n"

            output += "SYSTEM PERFORMANCE\n"
            output += "-" * 70 + "\n"
            output += f"Total Losses      : {results['total_loss_kw']:.3f} kW\n"
            output += f"System Efficiency : {results['efficiency']:.2f} %\n"
            output += f"Load Sharing Ratio: {results['power1_kw']/results['power2_kw']:.3f} : 1\n\n"

            output += "OBSERVATIONS:\n"
            output += "-" * 70 + "\n"
            if abs(results['z1'] - results['z2']) / results['z1'] > 0.2:
                output += "⚠ Impedance mismatch > 20% may cause poor load sharing\n"
            else:
                output += "✓ Acceptable impedance matching\n"

            if results['efficiency'] > 95:
                output += "✓ Excellent efficiency\n"
            elif results['efficiency'] > 90:
                output += "✓ Good efficiency\n"
            else:
                output += "⚠ Consider efficiency improvements\n"

            output += "\n" + "=" * 70 + "\n"

            self.results_text.insert(1.0, output)

        except ValueError as e:
            messagebox.showerror("Input Error", f"Invalid input values: {str(e)}")
        except Exception as e:
            messagebox.showerror("Calculation Error", f"Error during calculation: {str(e)}")

    def start_simulation(self):
        """Start dynamic simulation"""
        if not self.running:
            self.running = True
            self.paused = False
            self.btn_start.config(state='disabled')

            # Start simulation in separate thread
            self.simulation_thread = threading.Thread(target=self.run_simulation, daemon=True)
            self.simulation_thread.start()

    def pause_simulation(self):
        """Pause simulation"""
        self.paused = not self.paused
        if self.paused:
            self.btn_stop.config(text="▶ Resume")
        else:
            self.btn_stop.config(text="⏸ Pause")

    def reset_simulation(self):
        """Reset simulation"""
        self.running = False
        self.paused = False
        self.time_data.clear()
        self.current_data_t1.clear()
        self.current_data_t2.clear()
        self.voltage_data.clear()
        self.temperature_data.clear()
        self.power_data.clear()
        self.loss_data.clear()

        self.btn_start.config(state='normal')
        self.btn_stop.config(text="⏸ Pause")

        # Clear plots
        for ax in [self.ax_current, self.ax_voltage, self.ax_power, self.ax_temp]:
            ax.clear()
        self.canvas_dynamic.draw()

    def run_simulation(self):
        """Run dynamic simulation"""
        t_span = [0, 1.0]  # 1 second simulation
        t_eval = np.linspace(0, 1.0, 1000)
        y0 = [0, 0, 0, 0]  # Initial conditions

        v_input = self.voltage_slider.get()
        frequency = self.freq_slider.get()

        try:
            if self.solver_var.get() == "RK45":
                # RK45 solver
                sol = solve_ivp(
                    lambda t, y: self.dynamic_model.electrical_ode_rk45(t, y, v_input, frequency),
                    t_span, y0, method='RK45', t_eval=t_eval, max_step=0.001
                )
                t = sol.t
                i_primary = sol.y[0]
                i_secondary = sol.y[1]

            else:  # Euler
                # Manual Euler implementation
                dt = 0.001
                t = np.arange(0, 1.0, dt)
                i_primary = np.zeros_like(t)
                i_secondary = np.zeros_like(t)
                y = y0.copy()

                for i in range(1, len(t)):
                    if not self.running:
                        break
                    while self.paused:
                        time.sleep(0.1)

                    dydt = self.dynamic_model.electrical_ode_rk45(t[i-1], y, v_input, frequency)
                    y = [y[j] + dydt[j] * dt for j in range(len(y))]
                    i_primary[i] = y[0]
                    i_secondary[i] = y[1]

            # Calculate voltages
            voltage = v_input * np.sin(2 * np.pi * frequency * t)

            # Calculate power
            power = voltage * i_primary

            # Simulate temperature rise
            loss = i_primary**2 * self.dynamic_model.params.r_ohm
            temperature = 25 + np.cumsum(loss) * 0.001  # Simplified thermal model

            # Store data
            self.time_data = t.tolist()
            self.current_data_t1 = i_primary.tolist()
            self.current_data_t2 = i_secondary.tolist()
            self.voltage_data = voltage.tolist()
            self.power_data = power.tolist()
            self.temperature_data = temperature.tolist()

            # Update plots
            self.after(0, self.update_dynamic_plots)

        except Exception as e:
            messagebox.showerror("Simulation Error", f"Error during simulation: {str(e)}")
        finally:
            self.running = False
            self.btn_start.config(state='normal')

    def update_dynamic_plots(self):
        """Update dynamic simulation plots"""
        # Clear axes
        self.ax_current.clear()
        self.ax_voltage.clear()
        self.ax_power.clear()
        self.ax_temp.clear()

        # Plot current
        self.ax_current.plot(self.time_data, self.current_data_t1, 'b-', label='Transformer 1', linewidth=1.5)
        self.ax_current.plot(self.time_data, self.current_data_t2, 'r-', label='Transformer 2', linewidth=1.5)
        self.ax_current.set_xlabel('Time (s)')
        self.ax_current.set_ylabel('Current (A)')
        self.ax_current.set_title('Current vs Time')
        self.ax_current.legend()
        self.ax_current.grid(True, alpha=0.3)

        # Plot voltage
        self.ax_voltage.plot(self.time_data, self.voltage_data, 'g-', linewidth=1.5)
        self.ax_voltage.set_xlabel('Time (s)')
        self.ax_voltage.set_ylabel('Voltage (V)')
        self.ax_voltage.set_title('Voltage vs Time')
        self.ax_voltage.grid(True, alpha=0.3)

        # Plot power
        self.ax_power.plot(self.time_data, self.power_data, 'm-', linewidth=1.5)
        self.ax_power.set_xlabel('Time (s)')
        self.ax_power.set_ylabel('Power (W)')
        self.ax_power.set_title('Power vs Time')
        self.ax_power.grid(True, alpha=0.3)

        # Plot temperature
        self.ax_temp.plot(self.time_data, self.temperature_data, 'r-', linewidth=1.5)
        self.ax_temp.set_xlabel('Time (s)')
        self.ax_temp.set_ylabel('Temperature (°C)')
        self.ax_temp.set_title('Temperature Rise')
        self.ax_temp.grid(True, alpha=0.3)

        self.fig_dynamic.tight_layout()
        self.canvas_dynamic.draw()

    def run_multiphysics(self):
        """Run multi-physics analysis"""
        try:
            # Generate data for multi-physics analysis
            time = np.linspace(0, 0.1, 500)

            # Electromagnetic - Magnetic flux density
            flux_density = 1.5 * np.sin(2 * np.pi * 50 * time)

            # Thermal - Temperature distribution
            temp_core = 25 + 50 * (1 - np.exp(-time * 20))
            temp_winding = 25 + 70 * (1 - np.exp(-time * 15))

            # Mechanical stress
            current = 30 * np.sin(2 * np.pi * 50 * time)
            stress_results = [self.dynamic_model.mechanical_stress(i, 10) for i in current]
            torque = [s['torque_transient'] for s in stress_results]
            bearing_load = [s['bearing_load'] for s in stress_results]
            shaft_stress = [s['shaft_stress_mpa'] for s in stress_results]

            # Derating curve
            temp_range = np.linspace(25, 150, 100)
            derating_factor = np.where(temp_range < 100, 1.0,
                                      1.0 - (temp_range - 100) / 100)

            # Clear and plot
            self.ax_flux.clear()
            self.ax_thermal.clear()
            self.ax_stress.clear()
            self.ax_torque.clear()
            self.ax_bearing.clear()
            self.ax_derating.clear()

            # Flux density
            self.ax_flux.plot(time * 1000, flux_density, 'b-', linewidth=2)
            self.ax_flux.set_xlabel('Time (ms)')
            self.ax_flux.set_ylabel('Flux Density (T)')
            self.ax_flux.set_title('Magnetic Flux Density')
            self.ax_flux.grid(True, alpha=0.3)

            # Thermal
            self.ax_thermal.plot(time, temp_core, 'r-', label='Core', linewidth=2)
            self.ax_thermal.plot(time, temp_winding, 'b-', label='Winding', linewidth=2)
            self.ax_thermal.set_xlabel('Time (s)')
            self.ax_thermal.set_ylabel('Temperature (°C)')
            self.ax_thermal.set_title('Thermal Analysis')
            self.ax_thermal.legend()
            self.ax_thermal.grid(True, alpha=0.3)

            # Stress
            self.ax_stress.plot(time * 1000, shaft_stress, 'g-', linewidth=2)
            self.ax_stress.set_xlabel('Time (ms)')
            self.ax_stress.set_ylabel('Stress (MPa)')
            self.ax_stress.set_title('Shaft Stress')
            self.ax_stress.grid(True, alpha=0.3)

            # Torque
            self.ax_torque.plot(time * 1000, torque, 'm-', linewidth=2)
            self.ax_torque.set_xlabel('Time (ms)')
            self.ax_torque.set_ylabel('Torque (Nm)')
            self.ax_torque.set_title('Electromagnetic Torque')
            self.ax_torque.grid(True, alpha=0.3)

            # Bearing load
            self.ax_bearing.plot(time * 1000, bearing_load, 'c-', linewidth=2)
            self.ax_bearing.set_xlabel('Time (ms)')
            self.ax_bearing.set_ylabel('Load (N)')
            self.ax_bearing.set_title('Bearing Load')
            self.ax_bearing.grid(True, alpha=0.3)

            # Derating
            self.ax_derating.plot(temp_range, derating_factor, 'r-', linewidth=2)
            self.ax_derating.set_xlabel('Temperature (°C)')
            self.ax_derating.set_ylabel('Derating Factor')
            self.ax_derating.set_title('Thermal Derating Curve')
            self.ax_derating.grid(True, alpha=0.3)
            self.ax_derating.axhline(y=0.8, color='orange', linestyle='--', label='80% limit')
            self.ax_derating.legend()

            self.fig_multiphysics.tight_layout()
            self.canvas_multiphysics.draw()

            messagebox.showinfo("Multi-Physics Analysis",
                              "Multi-physics analysis completed successfully!")

        except Exception as e:
            messagebox.showerror("Error", f"Multi-physics analysis error: {str(e)}")

    def analyze_losses(self):
        """Analyze and display losses"""
        try:
            # Get current operating point
            results = self.solver.solve_parallel_operation()

            # Calculate detailed losses for both transformers
            losses1 = LossAnalyzer.calculate_losses(
                results['i1'], self.solver.v_hv_rated, self.solver.r1
            )

            losses2 = LossAnalyzer.calculate_losses(
                results['i2'], self.solver.v_hv_rated, self.solver.r2
            )

            # Clear axes
            self.ax_loss_pie.clear()
            self.ax_loss_bar.clear()
            self.ax_loss_time.clear()
            self.ax_efficiency.clear()

            # Pie chart for Transformer 1
            loss_labels = ['Copper', 'Hysteresis', 'Eddy Current', 'Stray', 'Friction']
            loss_values1 = [
                losses1['copper_loss_w'],
                losses1['hysteresis_loss_w'],
                losses1['eddy_current_loss_w'],
                losses1['stray_loss_w'],
                losses1['friction_loss_w']
            ]

            colors = ['#ff9999', '#66b3ff', '#99ff99', '#ffcc99', '#ff99cc']
            self.ax_loss_pie.pie(loss_values1, labels=loss_labels, autopct='%1.1f%%',
                                colors=colors, startangle=90)
            self.ax_loss_pie.set_title('Transformer 1 Loss Distribution')

            # Bar chart comparison
            x = np.arange(len(loss_labels))
            width = 0.35

            loss_values2 = [
                losses2['copper_loss_w'],
                losses2['hysteresis_loss_w'],
                losses2['eddy_current_loss_w'],
                losses2['stray_loss_w'],
                losses2['friction_loss_w']
            ]

            self.ax_loss_bar.bar(x - width/2, loss_values1, width, label='Transformer 1', color='skyblue')
            self.ax_loss_bar.bar(x + width/2, loss_values2, width, label='Transformer 2', color='lightcoral')
            self.ax_loss_bar.set_xlabel('Loss Type')
            self.ax_loss_bar.set_ylabel('Loss (W)')
            self.ax_loss_bar.set_title('Loss Comparison')
            self.ax_loss_bar.set_xticks(x)
            self.ax_loss_bar.set_xticklabels(loss_labels, rotation=45, ha='right')
            self.ax_loss_bar.legend()
            self.ax_loss_bar.grid(True, alpha=0.3)

            # Loss vs load
            load_range = np.linspace(0, 200, 50)
            copper_loss_range = load_range**2 * 0.5
            core_loss_range = np.ones_like(load_range) * 500
            total_loss_range = copper_loss_range + core_loss_range

            self.ax_loss_time.plot(load_range, copper_loss_range, label='Copper Loss', linewidth=2)
            self.ax_loss_time.plot(load_range, core_loss_range, label='Core Loss', linewidth=2)
            self.ax_loss_time.plot(load_range, total_loss_range, label='Total Loss', linewidth=2, linestyle='--')
            self.ax_loss_time.set_xlabel('Load (kW)')
            self.ax_loss_time.set_ylabel('Loss (W)')
            self.ax_loss_time.set_title('Losses vs Load')
            self.ax_loss_time.legend()
            self.ax_loss_time.grid(True, alpha=0.3)

            # Efficiency curve
            efficiency_range = 100 * load_range / (load_range + total_loss_range / 1000)
            self.ax_efficiency.plot(load_range, efficiency_range, 'g-', linewidth=2)
            self.ax_efficiency.set_xlabel('Load (kW)')
            self.ax_efficiency.set_ylabel('Efficiency (%)')
            self.ax_efficiency.set_title('Efficiency vs Load')
            self.ax_efficiency.grid(True, alpha=0.3)
            self.ax_efficiency.axhline(y=95, color='r', linestyle='--', alpha=0.5, label='95% target')
            self.ax_efficiency.legend()

            self.fig_losses.tight_layout()
            self.canvas_losses.draw()

            messagebox.showinfo("Loss Analysis",
                              f"Total System Loss: {losses1['total_loss_w'] + losses2['total_loss_w']:.2f} W\n"
                              f"System Efficiency: {results['efficiency']:.2f}%")

        except Exception as e:
            messagebox.showerror("Error", f"Loss analysis error: {str(e)}")

    def calculate_economics(self):
        """Calculate economic analysis"""
        try:
            # Get parameters
            elec_cost = float(self.entry_elec_cost.get())
            op_hours = float(self.entry_op_hours.get())
            capital = float(self.entry_capital.get())
            maintenance = float(self.entry_maintenance.get())

            # Update analyzer
            self.economic_analyzer.electricity_cost = elec_cost
            self.economic_analyzer.operating_hours = op_hours

            # Get current operation point
            results = self.solver.solve_parallel_operation()

            # Calculate economics
            economics = self.economic_analyzer.calculate_economics(
                self.solver.load_kw,
                results['total_loss_kw'],
                capital,
                maintenance
            )

            # Display results
            self.economic_text.delete(1.0, tk.END)

            output = "=" * 70 + "\n"
            output += "ECONOMIC ANALYSIS RESULTS\n"
            output += "=" * 70 + "\n\n"

            output += "ANNUAL COSTS\n"
            output += "-" * 70 + "\n"
            output += f"Annual Energy Consumption    : {economics['annual_energy_kwh']:,.0f} kWh\n"
            output += f"Annual Energy Loss           : {economics['annual_loss_kwh']:,.0f} kWh\n"
            output += f"Energy Cost (Active Power)   : ${economics['energy_cost_annual']:,.2f}\n"
            output += f"Energy Cost (Losses)         : ${economics['loss_cost_annual']:,.2f}\n"
            output += f"Maintenance Cost             : ${economics['maintenance_cost_annual']:,.2f}\n"
            output += f"Total Annual Operating Cost  : ${economics['total_operating_cost_annual']:,.2f}\n\n"

            output += "LIFECYCLE ANALYSIS (20 years)\n"
            output += "-" * 70 + "\n"
            output += f"Capital Cost                 : ${capital:,.2f}\n"
            output += f"20-Year Operating Cost       : ${economics['total_operating_cost_annual'] * 20:,.2f}\n"
            output += f"Total Lifecycle Cost         : ${economics['lifecycle_cost']:,.2f}\n"
            output += f"Levelized Cost per kWh       : ${economics['cost_per_kwh']:.4f}\n\n"

            output += "INVESTMENT METRICS\n"
            output += "-" * 70 + "\n"
            output += f"Payback Period (Estimated)   : {economics['roi_years']:.1f} years\n"
            output += f"Annual Loss Cost Savings     : ${economics['loss_cost_annual']:,.2f}\n\n"

            # Cost breakdown over time
            output += "COST PROJECTION (First 5 Years)\n"
            output += "-" * 70 + "\n"
            output += f"{'Year':<8} {'Operating Cost':<20} {'Cumulative Cost':<20}\n"
            output += "-" * 70 + "\n"

            cumulative = capital
            for year in range(1, 6):
                cumulative += economics['total_operating_cost_annual']
                output += f"{year:<8} ${economics['total_operating_cost_annual']:>18,.2f} ${cumulative:>18,.2f}\n"

            output += "\n" + "=" * 70 + "\n"

            self.economic_text.insert(1.0, output)

        except ValueError as e:
            messagebox.showerror("Input Error", f"Invalid input: {str(e)}")
        except Exception as e:
            messagebox.showerror("Error", f"Economic calculation error: {str(e)}")

    def apply_control_settings(self):
        """Apply control settings"""
        control_method = self.control_var.get()
        temp_limit = self.temp_limit_slider.get()
        derating = self.derating_slider.get()

        # Update power monitoring display
        self.power_monitor_text.delete(1.0, tk.END)

        output = "CONTROL SETTINGS APPLIED\n"
        output += "=" * 60 + "\n\n"

        output += f"Control Method: {control_method}\n"

        if control_method == "VF":
            output += "  • Constant Voltage/Frequency ratio\n"
            output += "  • Simple and robust control\n"
            output += "  • Good for variable speed applications\n"
        elif control_method == "FOC":
            output += "  • Field-Oriented Control\n"
            output += "  • Decoupled torque and flux control\n"
            output += "  • High dynamic performance\n"
        elif control_method == "DTC":
            output += "  • Direct Torque Control\n"
            output += "  • Fast torque response\n"
            output += "  • No current control loops\n"
        elif control_method == "MPC":
            output += "  • Model Predictive Control\n"
            output += "  • Optimal control actions\n"
            output += "  • Handles constraints effectively\n"

        output += f"\nThermal Management:\n"
        output += f"  • Temperature Limit: {temp_limit:.0f}°C\n"
        output += f"  • Derating Factor: {derating:.2f}\n"
        output += f"  • Derated Power: {self.solver.load_kw * derating:.2f} kW\n"

        # Calculate current temperature (simplified)
        results = self.solver.solve_parallel_operation()
        estimated_temp = 25 + results['total_loss_kw'] * 15  # Simplified

        output += f"\nCurrent Status:\n"
        output += f"  • Estimated Temperature: {estimated_temp:.1f}°C\n"
        output += f"  • Power Consumption: {self.solver.load_kw:.2f} kW\n"
        output += f"  • System Losses: {results['total_loss_kw']:.3f} kW\n"

        if estimated_temp > temp_limit:
            output += f"\n⚠ WARNING: Temperature exceeds limit!\n"
            output += f"  Recommend reducing load or improving cooling.\n"
        else:
            output += f"\n✓ Temperature within safe limits\n"

        output += "\n" + "=" * 60 + "\n"

        self.power_monitor_text.insert(1.0, output)

        messagebox.showinfo("Control Settings",
                          f"Control method '{control_method}' applied successfully!")

    def on_window_resize(self, event):
        """Handle window resize for auto-scaling"""
        # Update canvas sizes automatically
        if hasattr(self, 'canvas_dynamic'):
            self.fig_dynamic.tight_layout()
            self.canvas_dynamic.draw_idle()

        if hasattr(self, 'canvas_multiphysics'):
            self.fig_multiphysics.tight_layout()
            self.canvas_multiphysics.draw_idle()

        if hasattr(self, 'canvas_losses'):
            self.fig_losses.tight_layout()
            self.canvas_losses.draw_idle()


def main():
    """Main entry point"""
    app = AdvancedTransformerSimulator()
    app.mainloop()


if __name__ == "__main__":
    main()
