"""
Advanced Capacitor-Start Motor Analysis and Multi-Physics Simulation Lab
Features:
- Quadrature capacitor calculation
- Multi-physics simulation (electromagnetic-thermal-mechanical)
- Real-time ODE solvers (RK45, Euler)
- Dynamic visualization
- Economic analysis
- Advanced control methods
- Thermal derating and power consumption analysis
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import solve_ivp
import cmath
import math
from datetime import datetime

class CapacitorMotorLab:
    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Capacitor-Start Motor Analysis Lab")
        self.root.geometry("1400x900")

        # Motor parameters (default values from problem)
        self.params = {
            'power': 250.0,           # W
            'voltage': 230.0,         # V
            'frequency': 50.0,        # Hz
            'Rm': 4.5,                # Main winding resistance (ohm)
            'Xm': 3.7,                # Main winding reactance (ohm)
            'Ra': 9.5,                # Auxiliary winding resistance (ohm)
            'Xa': 3.5,                # Auxiliary winding reactance (ohm)
            'J': 0.01,                # Moment of inertia (kg.m^2)
            'B': 0.001,               # Friction coefficient
            'poles': 4,               # Number of poles
            'efficiency': 0.75,       # Motor efficiency
            'ambient_temp': 25.0,     # Ambient temperature (°C)
            'thermal_resistance': 5.0, # Thermal resistance (°C/W)
            'thermal_capacitance': 100.0, # Thermal capacitance (J/°C)
            'rated_temp': 100.0,      # Rated temperature (°C)
        }

        # Simulation parameters
        self.sim_params = {
            'time': 0.0,
            'dt': 0.001,
            'running': False,
            'capacitor': 0.0,
            'method': 'RK45',
            'load_torque': 0.5,
        }

        # State variables
        self.state = {
            'speed': 0.0,
            'torque': 0.0,
            'Im': 0.0,
            'Ia': 0.0,
            'temp': 25.0,
            'power_consumed': 0.0,
            'losses': {'copper': 0.0, 'iron': 0.0, 'mechanical': 0.0, 'stray': 0.0},
        }

        # History for plotting
        self.history = {
            'time': [],
            'speed': [],
            'torque': [],
            'Im': [],
            'Ia': [],
            'temp': [],
            'power': [],
            'efficiency': [],
        }

        # Economic parameters
        self.economics = {
            'electricity_cost': 0.12,  # $/kWh
            'operating_hours': 0.0,
            'total_cost': 0.0,
        }

        # Setup GUI
        self.setup_gui()

        # Calculate initial capacitor value
        self.calculate_quadrature_capacitor()

        # Bind resize event
        self.root.bind('<Configure>', self.on_resize)

    def setup_gui(self):
        """Setup the main GUI with tabs and controls"""
        # Create main container with auto-scaling
        self.main_container = ttk.Frame(self.root)
        self.main_container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Configure grid weights for responsive design
        self.main_container.grid_rowconfigure(0, weight=1)
        self.main_container.grid_columnconfigure(1, weight=1)

        # Left panel - Controls
        self.control_panel = ttk.LabelFrame(self.main_container, text="Control Panel", padding=10)
        self.control_panel.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        # Right panel - Tabs
        self.notebook = ttk.Notebook(self.main_container)
        self.notebook.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)

        # Setup control panel
        self.setup_control_panel()

        # Setup tabs
        self.setup_simulation_tab()
        self.setup_analysis_tab()
        self.setup_thermal_tab()
        self.setup_economics_tab()
        self.setup_advanced_controls_tab()

    def setup_control_panel(self):
        """Setup the left control panel with inputs and buttons"""
        row = 0

        # Motor Parameters Section
        ttk.Label(self.control_panel, text="Motor Parameters", font=('Arial', 10, 'bold')).grid(
            row=row, column=0, columnspan=2, pady=5)
        row += 1

        # Create input fields
        self.input_vars = {}
        motor_params = [
            ('Power (W)', 'power'),
            ('Voltage (V)', 'voltage'),
            ('Frequency (Hz)', 'frequency'),
            ('Rm (Ω)', 'Rm'),
            ('Xm (Ω)', 'Xm'),
            ('Ra (Ω)', 'Ra'),
            ('Xa (Ω)', 'Xa'),
        ]

        for label, key in motor_params:
            ttk.Label(self.control_panel, text=label).grid(row=row, column=0, sticky='w', pady=2)
            var = tk.DoubleVar(value=self.params[key])
            entry = ttk.Entry(self.control_panel, textvariable=var, width=15)
            entry.grid(row=row, column=1, pady=2)
            self.input_vars[key] = var
            row += 1

        ttk.Separator(self.control_panel, orient='horizontal').grid(
            row=row, column=0, columnspan=2, sticky='ew', pady=10)
        row += 1

        # Calculated Capacitor Display
        ttk.Label(self.control_panel, text="Calculated Capacitor", font=('Arial', 10, 'bold')).grid(
            row=row, column=0, columnspan=2, pady=5)
        row += 1

        self.cap_label = ttk.Label(self.control_panel, text="C = 0.0 µF",
                                    font=('Arial', 12), foreground='blue')
        self.cap_label.grid(row=row, column=0, columnspan=2, pady=5)
        row += 1

        ttk.Button(self.control_panel, text="Recalculate Capacitor",
                   command=self.calculate_quadrature_capacitor).grid(
            row=row, column=0, columnspan=2, pady=5)
        row += 1

        ttk.Separator(self.control_panel, orient='horizontal').grid(
            row=row, column=0, columnspan=2, sticky='ew', pady=10)
        row += 1

        # Simulation Controls
        ttk.Label(self.control_panel, text="Simulation Controls", font=('Arial', 10, 'bold')).grid(
            row=row, column=0, columnspan=2, pady=5)
        row += 1

        # Load Torque Slider
        ttk.Label(self.control_panel, text="Load Torque (Nm)").grid(
            row=row, column=0, sticky='w', pady=2)
        row += 1
        self.load_slider = ttk.Scale(self.control_panel, from_=0, to=5,
                                      orient='horizontal', command=self.update_load)
        self.load_slider.set(self.sim_params['load_torque'])
        self.load_slider.grid(row=row, column=0, columnspan=2, sticky='ew', pady=2)
        self.load_label = ttk.Label(self.control_panel, text=f"{self.sim_params['load_torque']:.2f} Nm")
        row += 1
        self.load_label.grid(row=row, column=0, columnspan=2)
        row += 1

        # Voltage Adjustment Slider
        ttk.Label(self.control_panel, text="Voltage Adjust (%)").grid(
            row=row, column=0, sticky='w', pady=2)
        row += 1
        self.voltage_slider = ttk.Scale(self.control_panel, from_=50, to=150,
                                         orient='horizontal', command=self.update_voltage_adjust)
        self.voltage_slider.set(100)
        self.voltage_slider.grid(row=row, column=0, columnspan=2, sticky='ew', pady=2)
        self.voltage_adj_label = ttk.Label(self.control_panel, text="100%")
        row += 1
        self.voltage_adj_label.grid(row=row, column=0, columnspan=2)
        row += 1

        # Solver Method
        ttk.Label(self.control_panel, text="ODE Solver").grid(
            row=row, column=0, sticky='w', pady=2)
        self.solver_var = tk.StringVar(value='RK45')
        solver_combo = ttk.Combobox(self.control_panel, textvariable=self.solver_var,
                                     values=['RK45', 'Euler'], state='readonly', width=12)
        solver_combo.grid(row=row, column=1, pady=2)
        row += 1

        ttk.Separator(self.control_panel, orient='horizontal').grid(
            row=row, column=0, columnspan=2, sticky='ew', pady=10)
        row += 1

        # Control Buttons
        button_frame = ttk.Frame(self.control_panel)
        button_frame.grid(row=row, column=0, columnspan=2, pady=5)

        self.start_btn = ttk.Button(button_frame, text="Start", command=self.start_simulation,
                                     style='Success.TButton')
        self.start_btn.pack(side=tk.LEFT, padx=2)

        self.stop_btn = ttk.Button(button_frame, text="Stop", command=self.stop_simulation,
                                    state='disabled')
        self.stop_btn.pack(side=tk.LEFT, padx=2)

        self.reset_btn = ttk.Button(button_frame, text="Reset", command=self.reset_simulation)
        self.reset_btn.pack(side=tk.LEFT, padx=2)
        row += 1

        # Status Display
        ttk.Separator(self.control_panel, orient='horizontal').grid(
            row=row, column=0, columnspan=2, sticky='ew', pady=10)
        row += 1

        ttk.Label(self.control_panel, text="Status", font=('Arial', 10, 'bold')).grid(
            row=row, column=0, columnspan=2, pady=5)
        row += 1

        self.status_text = tk.Text(self.control_panel, height=8, width=30, wrap=tk.WORD)
        self.status_text.grid(row=row, column=0, columnspan=2, pady=5)
        scrollbar = ttk.Scrollbar(self.control_panel, command=self.status_text.yview)
        self.status_text.config(yscrollcommand=scrollbar.set)

        self.update_status("Ready")

    def setup_simulation_tab(self):
        """Setup simulation tab with dynamic graphs"""
        sim_tab = ttk.Frame(self.notebook)
        self.notebook.add(sim_tab, text="Simulation")

        # Configure grid
        sim_tab.grid_rowconfigure(0, weight=1)
        sim_tab.grid_columnconfigure(0, weight=1)

        # Create matplotlib figure
        self.sim_fig = Figure(figsize=(10, 8), dpi=100)
        self.sim_canvas = FigureCanvasTkAgg(self.sim_fig, sim_tab)
        self.sim_canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')

        # Create subplots
        self.ax_speed = self.sim_fig.add_subplot(3, 2, 1)
        self.ax_torque = self.sim_fig.add_subplot(3, 2, 2)
        self.ax_current = self.sim_fig.add_subplot(3, 2, 3)
        self.ax_power = self.sim_fig.add_subplot(3, 2, 4)
        self.ax_temp = self.sim_fig.add_subplot(3, 2, 5)
        self.ax_efficiency = self.sim_fig.add_subplot(3, 2, 6)

        self.sim_fig.tight_layout()

    def setup_analysis_tab(self):
        """Setup analysis tab for phasor diagrams and vector analysis"""
        analysis_tab = ttk.Frame(self.notebook)
        self.notebook.add(analysis_tab, text="Analysis")

        analysis_tab.grid_rowconfigure(0, weight=1)
        analysis_tab.grid_columnconfigure(0, weight=1)

        # Create figure for phasor diagram
        self.analysis_fig = Figure(figsize=(10, 8), dpi=100)
        self.analysis_canvas = FigureCanvasTkAgg(self.analysis_fig, analysis_tab)
        self.analysis_canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')

        # Create subplots
        self.ax_phasor = self.analysis_fig.add_subplot(2, 2, 1, projection='polar')
        self.ax_impedance = self.analysis_fig.add_subplot(2, 2, 2)
        self.ax_losses = self.analysis_fig.add_subplot(2, 2, 3)
        self.ax_vector = self.analysis_fig.add_subplot(2, 2, 4)

        self.analysis_fig.tight_layout()

    def setup_thermal_tab(self):
        """Setup thermal analysis tab"""
        thermal_tab = ttk.Frame(self.notebook)
        self.notebook.add(thermal_tab, text="Thermal Analysis")

        thermal_tab.grid_rowconfigure(0, weight=1)
        thermal_tab.grid_columnconfigure(0, weight=1)

        # Create figure for thermal analysis
        self.thermal_fig = Figure(figsize=(10, 8), dpi=100)
        self.thermal_canvas = FigureCanvasTkAgg(self.thermal_fig, thermal_tab)
        self.thermal_canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')

        # Create subplots
        self.ax_temp_time = self.thermal_fig.add_subplot(2, 2, 1)
        self.ax_derating = self.thermal_fig.add_subplot(2, 2, 2)
        self.ax_heat_map = self.thermal_fig.add_subplot(2, 2, 3)
        self.ax_thermal_stress = self.thermal_fig.add_subplot(2, 2, 4)

        self.thermal_fig.tight_layout()

    def setup_economics_tab(self):
        """Setup economic analysis tab"""
        economics_tab = ttk.Frame(self.notebook)
        self.notebook.add(economics_tab, text="Economic Analysis")

        economics_tab.grid_rowconfigure(1, weight=1)
        economics_tab.grid_columnconfigure(0, weight=1)

        # Input frame
        input_frame = ttk.LabelFrame(economics_tab, text="Economic Parameters", padding=10)
        input_frame.grid(row=0, column=0, sticky='ew', padx=5, pady=5)

        ttk.Label(input_frame, text="Electricity Cost ($/kWh):").grid(row=0, column=0, sticky='w', padx=5)
        self.elec_cost_var = tk.DoubleVar(value=self.economics['electricity_cost'])
        ttk.Entry(input_frame, textvariable=self.elec_cost_var, width=15).grid(row=0, column=1, padx=5)

        ttk.Button(input_frame, text="Update", command=self.update_economics).grid(row=0, column=2, padx=5)

        # Create figure for economic analysis
        self.econ_fig = Figure(figsize=(10, 6), dpi=100)
        self.econ_canvas = FigureCanvasTkAgg(self.econ_fig, economics_tab)
        self.econ_canvas.get_tk_widget().grid(row=1, column=0, sticky='nsew')

        self.ax_cost_time = self.econ_fig.add_subplot(1, 2, 1)
        self.ax_cost_breakdown = self.econ_fig.add_subplot(1, 2, 2)

        self.econ_fig.tight_layout()

    def setup_advanced_controls_tab(self):
        """Setup advanced control methods tab"""
        controls_tab = ttk.Frame(self.notebook)
        self.notebook.add(controls_tab, text="Advanced Controls")

        controls_tab.grid_rowconfigure(1, weight=1)
        controls_tab.grid_columnconfigure(0, weight=1)

        # Control options frame
        ctrl_frame = ttk.LabelFrame(controls_tab, text="Control Methods", padding=10)
        ctrl_frame.grid(row=0, column=0, sticky='ew', padx=5, pady=5)

        self.control_method = tk.StringVar(value='Open Loop')
        methods = ['Open Loop', 'V/f Control', 'Field Oriented Control', 'Direct Torque Control']

        for i, method in enumerate(methods):
            ttk.Radiobutton(ctrl_frame, text=method, variable=self.control_method,
                           value=method, command=self.update_control_method).grid(
                row=0, column=i, padx=5)

        # Create figure for control analysis
        self.ctrl_fig = Figure(figsize=(10, 6), dpi=100)
        self.ctrl_canvas = FigureCanvasTkAgg(self.ctrl_fig, controls_tab)
        self.ctrl_canvas.get_tk_widget().grid(row=1, column=0, sticky='nsew')

        self.ax_speed_ctrl = self.ctrl_fig.add_subplot(2, 2, 1)
        self.ax_torque_ctrl = self.ctrl_fig.add_subplot(2, 2, 2)
        self.ax_flux = self.ctrl_fig.add_subplot(2, 2, 3)
        self.ax_control_signal = self.ctrl_fig.add_subplot(2, 2, 4)

        self.ctrl_fig.tight_layout()

    def calculate_quadrature_capacitor(self):
        """Calculate capacitor value for quadrature condition"""
        try:
            # Update parameters from inputs
            for key, var in self.input_vars.items():
                self.params[key] = var.get()

            # Get parameters
            V = self.params['voltage']
            f = self.params['frequency']
            omega = 2 * np.pi * f

            # Main winding impedance
            Zm = complex(self.params['Rm'], self.params['Xm'])

            # Auxiliary winding impedance (without capacitor)
            Za_base = complex(self.params['Ra'], self.params['Xa'])

            # Current in main winding
            Im = V / Zm
            angle_Im = cmath.phase(Im)

            # For quadrature, auxiliary current should lead or lag by 90 degrees
            # We want Ia to lead Im by 90 degrees
            desired_angle_Ia = angle_Im + np.pi/2

            # Calculate required auxiliary impedance magnitude and angle
            # |Ia| can be different, but angle must be 90 degrees ahead

            # For quadrature: angle(Za_total) - angle(Zm) = 0 (impedances in phase)
            # or the current angles differ by 90 degrees

            # Better approach: For quadrature current condition
            # tan(angle_Im) * tan(angle_Ia) = -1
            # angle_Ia = -90° - angle_Im or 90° - angle_Im

            # Main winding angle
            theta_m = np.arctan2(self.params['Xm'], self.params['Rm'])

            # For auxiliary winding with capacitor:
            # We want theta_a such that theta_a - theta_m = 90° or -90°
            # Let's use: theta_a = theta_m - 90° (auxiliary leads main by 90°)

            theta_a_desired = theta_m - np.pi/2

            # Required auxiliary impedance angle
            # Za_total = Ra + j(Xa - Xc)
            # tan(theta_a) = (Xa - Xc) / Ra

            Xc_required = self.params['Xa'] - self.params['Ra'] * np.tan(theta_a_desired)

            # Capacitive reactance Xc = 1/(omega * C)
            # C = 1 / (omega * Xc)

            if Xc_required > 0:
                C = 1.0 / (omega * Xc_required)
                C_uF = C * 1e6  # Convert to microfarads

                self.sim_params['capacitor'] = C
                self.cap_label.config(text=f"C = {C_uF:.2f} µF")

                # Verify quadrature condition
                Za_total = complex(self.params['Ra'], self.params['Xa'] - Xc_required)
                Ia = V / Za_total

                angle_diff = abs(cmath.phase(Im) - cmath.phase(Ia))
                angle_diff_deg = np.degrees(angle_diff)

                self.update_status(f"Capacitor Calculated!\n"
                                 f"C = {C_uF:.2f} µF\n"
                                 f"Xc = {Xc_required:.2f} Ω\n"
                                 f"Phase difference: {angle_diff_deg:.2f}°\n"
                                 f"|Im| = {abs(Im):.2f} A\n"
                                 f"|Ia| = {abs(Ia):.2f} A")

                # Update analysis plots
                self.update_analysis_plots()

            else:
                messagebox.showwarning("Warning",
                    "Negative capacitive reactance required. Check winding parameters.")

        except Exception as e:
            messagebox.showerror("Error", f"Calculation error: {str(e)}")

    def update_load(self, value):
        """Update load torque from slider"""
        self.sim_params['load_torque'] = float(value)
        self.load_label.config(text=f"{self.sim_params['load_torque']:.2f} Nm")

    def update_voltage_adjust(self, value):
        """Update voltage adjustment from slider"""
        percent = float(value)
        self.voltage_adj_label.config(text=f"{percent:.0f}%")

    def update_control_method(self):
        """Update control method"""
        method = self.control_method.get()
        self.update_status(f"Control method: {method}")

    def update_economics(self):
        """Update economic parameters"""
        self.economics['electricity_cost'] = self.elec_cost_var.get()
        self.update_status(f"Electricity cost updated: ${self.economics['electricity_cost']:.3f}/kWh")

    def start_simulation(self):
        """Start the simulation"""
        self.sim_params['running'] = True
        self.sim_params['method'] = self.solver_var.get()
        self.start_btn.config(state='disabled')
        self.stop_btn.config(state='normal')
        self.update_status("Simulation started")
        self.simulate_step()

    def stop_simulation(self):
        """Stop the simulation"""
        self.sim_params['running'] = False
        self.start_btn.config(state='normal')
        self.stop_btn.config(state='disabled')
        self.update_status("Simulation stopped")

    def reset_simulation(self):
        """Reset the simulation"""
        self.sim_params['running'] = False
        self.sim_params['time'] = 0.0
        self.state = {
            'speed': 0.0,
            'torque': 0.0,
            'Im': 0.0,
            'Ia': 0.0,
            'temp': self.params['ambient_temp'],
            'power_consumed': 0.0,
            'losses': {'copper': 0.0, 'iron': 0.0, 'mechanical': 0.0, 'stray': 0.0},
        }
        self.history = {
            'time': [],
            'speed': [],
            'torque': [],
            'Im': [],
            'Ia': [],
            'temp': [],
            'power': [],
            'efficiency': [],
        }
        self.economics['operating_hours'] = 0.0
        self.economics['total_cost'] = 0.0

        self.start_btn.config(state='normal')
        self.stop_btn.config(state='disabled')
        self.update_status("Simulation reset")
        self.update_plots()

    def simulate_step(self):
        """Perform one simulation step"""
        if not self.sim_params['running']:
            return

        dt = self.sim_params['dt']

        # Update parameters from sliders
        voltage_percent = self.voltage_slider.get()
        V = self.params['voltage'] * voltage_percent / 100.0

        # Calculate currents
        omega = 2 * np.pi * self.params['frequency']

        # Main winding current (RMS)
        Zm = complex(self.params['Rm'], self.params['Xm'])
        Im = V / Zm
        Im_rms = abs(Im)

        # Auxiliary winding current (RMS) with capacitor
        if self.sim_params['capacitor'] > 0:
            Xc = 1.0 / (omega * self.sim_params['capacitor'])
        else:
            Xc = 0.0

        Za = complex(self.params['Ra'], self.params['Xa'] - Xc)
        Ia = V / Za
        Ia_rms = abs(Ia)

        # Total current
        Itotal = Im + Ia

        # Calculate torque (simplified model)
        # Torque is proportional to the product of currents and sine of phase angle
        phase_diff = cmath.phase(Im) - cmath.phase(Ia)

        # Starting torque proportional to Im * Ia * sin(phase_diff)
        k_torque = 0.5  # Torque constant
        torque_developed = k_torque * Im_rms * Ia_rms * abs(np.sin(phase_diff))

        # Mechanical equation: J * d(omega)/dt = T_motor - T_load - B * omega
        J = self.params['J']
        B = self.params['B']
        T_load = self.sim_params['load_torque']

        # Current speed (rad/s)
        omega_mech = self.state['speed']

        # Speed-dependent torque reduction (as motor speeds up, torque reduces)
        sync_speed = 4 * np.pi * self.params['frequency'] / self.params['poles']
        slip = (sync_speed - omega_mech) / sync_speed if sync_speed > 0 else 1.0
        slip = max(0.01, min(1.0, slip))  # Clamp slip between 0.01 and 1

        torque_motor = torque_developed * slip

        # Update speed using selected ODE solver
        if self.sim_params['method'] == 'Euler':
            # Euler method
            d_omega = (torque_motor - T_load - B * omega_mech) / J
            omega_new = omega_mech + d_omega * dt
        else:
            # RK45 (simplified single step)
            def derivatives(t, y):
                omega = y[0]
                sync_speed = 4 * np.pi * self.params['frequency'] / self.params['poles']
                slip = (sync_speed - omega) / sync_speed if sync_speed > 0 else 1.0
                slip = max(0.01, min(1.0, slip))
                T_motor = torque_developed * slip
                d_omega = (T_motor - T_load - B * omega) / J
                return [d_omega]

            # Single RK4 step
            k1 = np.array(derivatives(self.sim_params['time'], [omega_mech]))
            k2 = np.array(derivatives(self.sim_params['time'] + dt/2, [omega_mech + dt*k1[0]/2]))
            k3 = np.array(derivatives(self.sim_params['time'] + dt/2, [omega_mech + dt*k2[0]/2]))
            k4 = np.array(derivatives(self.sim_params['time'] + dt, [omega_mech + dt*k3[0]]))

            omega_new = omega_mech + dt * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]) / 6

        omega_new = max(0, omega_new)  # Speed cannot be negative

        # Calculate losses
        # Copper losses
        P_copper_main = Im_rms**2 * self.params['Rm']
        P_copper_aux = Ia_rms**2 * self.params['Ra']
        P_copper = P_copper_main + P_copper_aux

        # Iron losses (proportional to voltage squared and frequency)
        k_iron = 0.01
        P_iron = k_iron * V**2 * self.params['frequency'] / 50.0

        # Mechanical losses (friction, proportional to speed)
        P_mechanical = B * omega_new**2

        # Stray losses (estimated as 1% of input power)
        P_input = V * abs(Itotal)
        P_stray = 0.01 * P_input

        # Total losses
        P_loss = P_copper + P_iron + P_mechanical + P_stray

        # Output power
        P_out = torque_motor * omega_new

        # Efficiency
        efficiency = P_out / P_input if P_input > 0 else 0
        efficiency = min(1.0, max(0, efficiency))

        # Thermal model: C_th * dT/dt = P_loss - (T - T_ambient) / R_th
        C_th = self.params['thermal_capacitance']
        R_th = self.params['thermal_resistance']
        T_ambient = self.params['ambient_temp']
        T = self.state['temp']

        dT_dt = (P_loss - (T - T_ambient) / R_th) / C_th
        T_new = T + dT_dt * dt

        # Update state
        self.state['speed'] = omega_new
        self.state['torque'] = torque_motor
        self.state['Im'] = Im_rms
        self.state['Ia'] = Ia_rms
        self.state['temp'] = T_new
        self.state['power_consumed'] = P_input
        self.state['losses'] = {
            'copper': P_copper,
            'iron': P_iron,
            'mechanical': P_mechanical,
            'stray': P_stray
        }

        # Update history
        self.history['time'].append(self.sim_params['time'])
        self.history['speed'].append(omega_new * 60 / (2 * np.pi))  # Convert to RPM
        self.history['torque'].append(torque_motor)
        self.history['Im'].append(Im_rms)
        self.history['Ia'].append(Ia_rms)
        self.history['temp'].append(T_new)
        self.history['power'].append(P_input)
        self.history['efficiency'].append(efficiency * 100)

        # Update economics
        energy_consumed = P_input * dt / 3600.0  # Wh
        cost = energy_consumed * self.economics['electricity_cost'] / 1000.0  # $
        self.economics['operating_hours'] += dt / 3600.0
        self.economics['total_cost'] += cost

        # Limit history length
        max_points = 1000
        if len(self.history['time']) > max_points:
            for key in self.history:
                self.history[key] = self.history[key][-max_points:]

        # Update time
        self.sim_params['time'] += dt

        # Update plots every 50 steps
        if len(self.history['time']) % 50 == 0:
            self.update_plots()

        # Update status every 100 steps
        if len(self.history['time']) % 100 == 0:
            self.update_status(
                f"Time: {self.sim_params['time']:.2f} s\n"
                f"Speed: {omega_new * 60 / (2 * np.pi):.1f} RPM\n"
                f"Torque: {torque_motor:.3f} Nm\n"
                f"Temp: {T_new:.1f} °C\n"
                f"Efficiency: {efficiency*100:.1f} %\n"
                f"Power: {P_input:.1f} W"
            )

        # Schedule next step
        self.root.after(10, self.simulate_step)

    def update_plots(self):
        """Update all plots"""
        self.update_simulation_plots()
        self.update_analysis_plots()
        self.update_thermal_plots()
        self.update_economic_plots()
        self.update_control_plots()

    def update_simulation_plots(self):
        """Update simulation tab plots"""
        if len(self.history['time']) == 0:
            return

        t = self.history['time']

        # Speed plot
        self.ax_speed.clear()
        self.ax_speed.plot(t, self.history['speed'], 'b-', linewidth=2)
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (RPM)')
        self.ax_speed.set_title('Motor Speed')
        self.ax_speed.grid(True, alpha=0.3)

        # Torque plot
        self.ax_torque.clear()
        self.ax_torque.plot(t, self.history['torque'], 'r-', linewidth=2)
        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (Nm)')
        self.ax_torque.set_title('Motor Torque')
        self.ax_torque.grid(True, alpha=0.3)

        # Current plot
        self.ax_current.clear()
        self.ax_current.plot(t, self.history['Im'], 'g-', linewidth=2, label='Main')
        self.ax_current.plot(t, self.history['Ia'], 'm-', linewidth=2, label='Auxiliary')
        self.ax_current.set_xlabel('Time (s)')
        self.ax_current.set_ylabel('Current (A)')
        self.ax_current.set_title('Winding Currents (RMS)')
        self.ax_current.legend()
        self.ax_current.grid(True, alpha=0.3)

        # Power plot
        self.ax_power.clear()
        self.ax_power.plot(t, self.history['power'], 'c-', linewidth=2)
        self.ax_power.set_xlabel('Time (s)')
        self.ax_power.set_ylabel('Power (W)')
        self.ax_power.set_title('Input Power')
        self.ax_power.grid(True, alpha=0.3)

        # Temperature plot
        self.ax_temp.clear()
        self.ax_temp.plot(t, self.history['temp'], 'orange', linewidth=2)
        self.ax_temp.axhline(y=self.params['rated_temp'], color='r',
                            linestyle='--', label='Rated Temp')
        self.ax_temp.set_xlabel('Time (s)')
        self.ax_temp.set_ylabel('Temperature (°C)')
        self.ax_temp.set_title('Motor Temperature')
        self.ax_temp.legend()
        self.ax_temp.grid(True, alpha=0.3)

        # Efficiency plot
        self.ax_efficiency.clear()
        self.ax_efficiency.plot(t, self.history['efficiency'], 'purple', linewidth=2)
        self.ax_efficiency.set_xlabel('Time (s)')
        self.ax_efficiency.set_ylabel('Efficiency (%)')
        self.ax_efficiency.set_title('Motor Efficiency')
        self.ax_efficiency.set_ylim([0, 100])
        self.ax_efficiency.grid(True, alpha=0.3)

        self.sim_fig.tight_layout()
        self.sim_canvas.draw()

    def update_analysis_plots(self):
        """Update analysis tab plots"""
        # Phasor diagram
        self.ax_phasor.clear()

        V = self.params['voltage']
        omega = 2 * np.pi * self.params['frequency']

        # Main winding
        Zm = complex(self.params['Rm'], self.params['Xm'])
        Im = V / Zm

        # Auxiliary winding with capacitor
        if self.sim_params['capacitor'] > 0:
            Xc = 1.0 / (omega * self.sim_params['capacitor'])
        else:
            Xc = 0.0

        Za = complex(self.params['Ra'], self.params['Xa'] - Xc)
        Ia = V / Za

        # Plot phasors
        self.ax_phasor.plot([0, cmath.phase(Im)], [0, abs(Im)], 'b-',
                           linewidth=2, marker='o', label='Im')
        self.ax_phasor.plot([0, cmath.phase(Ia)], [0, abs(Ia)], 'r-',
                           linewidth=2, marker='o', label='Ia')

        self.ax_phasor.set_title('Phasor Diagram - Currents')
        self.ax_phasor.legend()
        self.ax_phasor.grid(True, alpha=0.3)

        # Impedance plot
        self.ax_impedance.clear()

        categories = ['Main\nWinding', 'Auxiliary\nWinding']
        resistances = [self.params['Rm'], self.params['Ra']]
        reactances = [self.params['Xm'], self.params['Xa'] - Xc]

        x = np.arange(len(categories))
        width = 0.35

        self.ax_impedance.bar(x - width/2, resistances, width, label='Resistance', color='orange')
        self.ax_impedance.bar(x + width/2, reactances, width, label='Reactance', color='blue')

        self.ax_impedance.set_ylabel('Impedance (Ω)')
        self.ax_impedance.set_title('Winding Impedances')
        self.ax_impedance.set_xticks(x)
        self.ax_impedance.set_xticklabels(categories)
        self.ax_impedance.legend()
        self.ax_impedance.grid(True, alpha=0.3, axis='y')

        # Losses breakdown
        self.ax_losses.clear()

        if len(self.history['time']) > 0:
            losses = self.state['losses']
            labels = ['Copper', 'Iron', 'Mechanical', 'Stray']
            values = [losses['copper'], losses['iron'], losses['mechanical'], losses['stray']]
            colors = ['#ff9999', '#66b3ff', '#99ff99', '#ffcc99']

            self.ax_losses.pie(values, labels=labels, colors=colors, autopct='%1.1f%%',
                              startangle=90)
            self.ax_losses.set_title('Loss Distribution')

        # Vector diagram
        self.ax_vector.clear()

        # Plot impedance vectors
        self.ax_vector.quiver(0, 0, self.params['Rm'], self.params['Xm'],
                             angles='xy', scale_units='xy', scale=1, color='b',
                             width=0.006, label='Zm')
        self.ax_vector.quiver(0, 0, self.params['Ra'], self.params['Xa'] - Xc,
                             angles='xy', scale_units='xy', scale=1, color='r',
                             width=0.006, label='Za')

        max_val = max(self.params['Rm'] + self.params['Xm'],
                     self.params['Ra'] + self.params['Xa'])
        self.ax_vector.set_xlim([-2, max_val + 2])
        self.ax_vector.set_ylim([-2, max_val + 2])
        self.ax_vector.set_xlabel('Resistance (Ω)')
        self.ax_vector.set_ylabel('Reactance (Ω)')
        self.ax_vector.set_title('Impedance Vectors')
        self.ax_vector.legend()
        self.ax_vector.grid(True, alpha=0.3)
        self.ax_vector.set_aspect('equal')

        self.analysis_fig.tight_layout()
        self.analysis_canvas.draw()

    def update_thermal_plots(self):
        """Update thermal analysis plots"""
        # Temperature vs time
        self.ax_temp_time.clear()

        if len(self.history['time']) > 0:
            t = self.history['time']
            temp = self.history['temp']

            self.ax_temp_time.plot(t, temp, 'r-', linewidth=2, label='Motor Temp')
            self.ax_temp_time.axhline(y=self.params['rated_temp'], color='orange',
                                     linestyle='--', label='Rated Temp')
            self.ax_temp_time.axhline(y=self.params['ambient_temp'], color='b',
                                     linestyle='--', label='Ambient Temp')
            self.ax_temp_time.set_xlabel('Time (s)')
            self.ax_temp_time.set_ylabel('Temperature (°C)')
            self.ax_temp_time.set_title('Temperature Profile')
            self.ax_temp_time.legend()
            self.ax_temp_time.grid(True, alpha=0.3)

        # Derating curve
        self.ax_derating.clear()

        temps = np.linspace(self.params['ambient_temp'],
                           self.params['rated_temp'] * 1.5, 100)
        derating = np.ones_like(temps)

        # Linear derating above rated temperature
        for i, temp in enumerate(temps):
            if temp > self.params['rated_temp']:
                derating[i] = max(0, 1 - (temp - self.params['rated_temp']) / 50)

        self.ax_derating.plot(temps, derating * 100, 'b-', linewidth=2)
        self.ax_derating.axvline(x=self.params['rated_temp'], color='r',
                                linestyle='--', label='Rated Temp')

        if len(self.history['temp']) > 0:
            current_temp = self.history['temp'][-1]
            current_derating = 100
            if current_temp > self.params['rated_temp']:
                current_derating = max(0, (1 - (current_temp - self.params['rated_temp']) / 50) * 100)
            self.ax_derating.plot(current_temp, current_derating, 'ro',
                                 markersize=10, label='Current')

        self.ax_derating.set_xlabel('Temperature (°C)')
        self.ax_derating.set_ylabel('Power Capacity (%)')
        self.ax_derating.set_title('Thermal Derating Curve')
        self.ax_derating.legend()
        self.ax_derating.grid(True, alpha=0.3)

        # Heat map (simplified)
        self.ax_heat_map.clear()

        # Create a simple 2D heat distribution
        x = np.linspace(0, 1, 20)
        y = np.linspace(0, 1, 20)
        X, Y = np.meshgrid(x, y)

        if len(self.history['temp']) > 0:
            center_temp = self.history['temp'][-1]
        else:
            center_temp = self.params['ambient_temp']

        # Gaussian distribution from center
        Z = center_temp - (self.params['rated_temp'] - self.params['ambient_temp']) * \
            ((X - 0.5)**2 + (Y - 0.5)**2)

        im = self.ax_heat_map.contourf(X, Y, Z, levels=20, cmap='hot')
        self.ax_heat_map.set_title('Temperature Distribution')
        self.ax_heat_map.set_xlabel('X Position')
        self.ax_heat_map.set_ylabel('Y Position')
        self.thermal_fig.colorbar(im, ax=self.ax_heat_map, label='Temperature (°C)')

        # Thermal stress
        self.ax_thermal_stress.clear()

        if len(self.history['time']) > 0:
            t = self.history['time']
            temp = np.array(self.history['temp'])

            # Thermal stress proportional to temperature gradient
            stress = np.gradient(temp)

            self.ax_thermal_stress.plot(t, stress, 'purple', linewidth=2)
            self.ax_thermal_stress.set_xlabel('Time (s)')
            self.ax_thermal_stress.set_ylabel('Thermal Stress (°C/s)')
            self.ax_thermal_stress.set_title('Thermal Stress Rate')
            self.ax_thermal_stress.grid(True, alpha=0.3)

        self.thermal_fig.tight_layout()
        self.thermal_canvas.draw()

    def update_economic_plots(self):
        """Update economic analysis plots"""
        # Cost vs time
        self.ax_cost_time.clear()

        if len(self.history['time']) > 0:
            t = np.array(self.history['time']) / 3600.0  # Convert to hours
            power = np.array(self.history['power']) / 1000.0  # Convert to kW

            # Cumulative energy
            energy = np.cumsum(power) * (t[1] - t[0]) if len(t) > 1 else [0]
            cost = energy * self.economics['electricity_cost']

            self.ax_cost_time.plot(t, cost, 'g-', linewidth=2)
            self.ax_cost_time.set_xlabel('Time (hours)')
            self.ax_cost_time.set_ylabel('Cumulative Cost ($)')
            self.ax_cost_time.set_title('Operating Cost Over Time')
            self.ax_cost_time.grid(True, alpha=0.3)

        # Cost breakdown
        self.ax_cost_breakdown.clear()

        if len(self.history['time']) > 0:
            losses = self.state['losses']
            total_loss = sum(losses.values())

            if total_loss > 0:
                # Energy lost in each category
                loss_percentages = {k: v/total_loss * 100 for k, v in losses.items()}

                categories = list(loss_percentages.keys())
                values = list(loss_percentages.values())

                bars = self.ax_cost_breakdown.bar(categories, values,
                                                  color=['#ff9999', '#66b3ff', '#99ff99', '#ffcc99'])

                self.ax_cost_breakdown.set_ylabel('Loss Percentage (%)')
                self.ax_cost_breakdown.set_title('Energy Loss Breakdown')
                self.ax_cost_breakdown.grid(True, alpha=0.3, axis='y')

                # Add value labels on bars
                for bar in bars:
                    height = bar.get_height()
                    self.ax_cost_breakdown.text(bar.get_x() + bar.get_width()/2., height,
                                               f'{height:.1f}%',
                                               ha='center', va='bottom')

        self.econ_fig.tight_layout()
        self.econ_canvas.draw()

    def update_control_plots(self):
        """Update advanced control plots"""
        # Speed control
        self.ax_speed_ctrl.clear()

        if len(self.history['time']) > 0:
            t = self.history['time']
            speed = self.history['speed']

            # Synchronous speed
            sync_speed = 60 * self.params['frequency'] * 2 / self.params['poles']

            self.ax_speed_ctrl.plot(t, speed, 'b-', linewidth=2, label='Actual')
            self.ax_speed_ctrl.axhline(y=sync_speed, color='r',
                                      linestyle='--', label='Synchronous')
            self.ax_speed_ctrl.set_xlabel('Time (s)')
            self.ax_speed_ctrl.set_ylabel('Speed (RPM)')
            self.ax_speed_ctrl.set_title('Speed Control')
            self.ax_speed_ctrl.legend()
            self.ax_speed_ctrl.grid(True, alpha=0.3)

        # Torque control
        self.ax_torque_ctrl.clear()

        if len(self.history['time']) > 0:
            t = self.history['time']
            torque = self.history['torque']

            self.ax_torque_ctrl.plot(t, torque, 'r-', linewidth=2, label='Motor Torque')
            self.ax_torque_ctrl.axhline(y=self.sim_params['load_torque'],
                                       color='g', linestyle='--', label='Load Torque')
            self.ax_torque_ctrl.set_xlabel('Time (s)')
            self.ax_torque_ctrl.set_ylabel('Torque (Nm)')
            self.ax_torque_ctrl.set_title('Torque Control')
            self.ax_torque_ctrl.legend()
            self.ax_torque_ctrl.grid(True, alpha=0.3)

        # Flux
        self.ax_flux.clear()

        if len(self.history['time']) > 0:
            t = self.history['time']
            Im = np.array(self.history['Im'])

            # Flux proportional to magnetizing current
            flux = Im * 0.1  # Simplified

            self.ax_flux.plot(t, flux, 'purple', linewidth=2)
            self.ax_flux.set_xlabel('Time (s)')
            self.ax_flux.set_ylabel('Flux (Wb)')
            self.ax_flux.set_title('Magnetic Flux')
            self.ax_flux.grid(True, alpha=0.3)

        # Control signal
        self.ax_control_signal.clear()

        if len(self.history['time']) > 0:
            t = self.history['time']

            # Generate control signal based on method
            method = self.control_method.get()

            if method == 'V/f Control':
                # Voltage/frequency ratio control
                signal = np.array(self.history['speed']) / 1500.0  # Normalized
            elif method == 'Field Oriented Control':
                # FOC signal (simplified)
                signal = np.sin(2 * np.pi * 0.5 * np.array(t))
            elif method == 'Direct Torque Control':
                # DTC signal (simplified)
                signal = np.array(self.history['torque']) / max(self.history['torque']) \
                        if max(self.history['torque']) > 0 else np.zeros_like(t)
            else:
                # Open loop
                signal = np.ones_like(t)

            self.ax_control_signal.plot(t, signal, 'orange', linewidth=2)
            self.ax_control_signal.set_xlabel('Time (s)')
            self.ax_control_signal.set_ylabel('Control Signal')
            self.ax_control_signal.set_title(f'{method} - Control Signal')
            self.ax_control_signal.grid(True, alpha=0.3)

        self.ctrl_fig.tight_layout()
        self.ctrl_canvas.draw()

    def update_status(self, message):
        """Update status text"""
        self.status_text.delete(1.0, tk.END)
        self.status_text.insert(1.0, message)

    def on_resize(self, event):
        """Handle window resize for auto-scaling"""
        # This method is called on window resize
        # The grid weights will handle the auto-scaling
        pass

def main():
    """Main function to run the application"""
    root = tk.Tk()
    app = CapacitorMotorLab(root)
    root.mainloop()

if __name__ == "__main__":
    main()
