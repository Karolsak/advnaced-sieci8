"""
Corrected Series Motor-Generator Calculation
Using proper magnetization curve approach
"""

import numpy as np
from scipy.interpolate import interp1d
from scipy.optimize import fsolve


def solve_series_generator_correct():
    """
    Corrected solution for series motor as generator
    """
    print("=" * 80)
    print("CORRECTED SERIES MOTOR-GENERATOR ANALYSIS")
    print("=" * 80)
    print()

    # Given data
    voltage_motor = 525.0  # V (motor operation voltage)
    R_motor = 3.5  # Ω
    R_load = 3.25  # Ω
    R_total = R_motor + R_load
    target_speed = 1000.0  # rpm

    # Motor characteristics at 525V
    current_data = np.array([75, 125, 175, 225], dtype=float)
    speed_data = np.array([1200, 950, 840, 745], dtype=float)

    print("GIVEN DATA:")
    print("-" * 80)
    print(f"Motor Operating Voltage:  {voltage_motor} V")
    print(f"Motor Resistance (Ra):    {R_motor} Ω")
    print(f"Load Resistance (RL):     {R_load} Ω")
    print(f"Total Resistance:         {R_total} Ω")
    print(f"Generator Speed:          {target_speed} rpm")
    print()

    print("MOTOR CHARACTERISTIC CURVE (at 525V):")
    print("-" * 80)
    print(" Current (A) | Speed (rpm) ")
    print("-" * 80)
    for i in range(len(current_data)):
        print(f"    {current_data[i]:5.0f}    |    {speed_data[i]:5.0f}")
    print()

    # Calculate back EMF from motor data
    print("ANALYSIS OF MOTOR DATA:")
    print("-" * 80)
    back_emf_motor = voltage_motor - current_data * R_motor

    print(" Current | Speed | Back EMF | k·φ=E/N  | Comments")
    print("-" * 80)
    k_phi_list = []
    valid_indices = []

    for i in range(len(current_data)):
        emf = back_emf_motor[i]
        k_phi = emf / speed_data[i] if speed_data[i] != 0 else 0
        k_phi_list.append(k_phi)

        comment = "OK" if emf > 0 else "Saturated"
        if emf > 0:
            valid_indices.append(i)

        print(f"  {current_data[i]:5.0f}  | {speed_data[i]:5.0f} | {emf:8.2f} | {k_phi:8.5f} | {comment}")

    print()
    print(f"Valid data points for magnetization curve: {len(valid_indices)}")
    print()

    # Use only valid points (positive back EMF)
    if len(valid_indices) >= 2:
        valid_currents = current_data[valid_indices]
        valid_k_phi = np.array([k_phi_list[i] for i in valid_indices])

        print(f"Using {len(valid_indices)} valid points for interpolation:")
        print("-" * 80)
        for i in range(len(valid_indices)):
            print(f"  I = {valid_currents[i]:5.0f} A,  k·φ = {valid_k_phi[i]:.6f}")
        print()

        # Create interpolation function
        k_phi_func = interp1d(valid_currents, valid_k_phi, kind='linear',
                             fill_value='extrapolate')
    else:
        # If not enough valid points, use alternative approach
        print("Not enough valid points. Using empirical series motor model.")
        print()

        # For series motor: E ∝ I·N in unsaturated region
        # From first valid point, calculate constant
        k_const = back_emf_motor[0] / (current_data[0] * speed_data[0])
        print(f"Empirical constant k = {k_const:.8f} V/(A·rpm)")
        print()

        def k_phi_func(I):
            return k_const * I

    # GENERATOR MODE ANALYSIS
    print("=" * 80)
    print("GENERATOR MODE CALCULATION")
    print("=" * 80)
    print()
    print("For a series generator at constant speed with resistive load:")
    print("  - Generated EMF: E = k·φ(I)·N")
    print("  - Armature circuit: E = I·(Ra + RL)")
    print("  - Solving: k·φ(I)·N = I·R_total")
    print()
    print(f"At N = {target_speed} rpm, R_total = {R_total} Ω")
    print()

    # Method 1: Graphical/iterative solution
    print("METHOD 1: Iterative Solution")
    print("-" * 80)

    def generator_equation(I):
        """
        For series generator: E = k·φ(I)·N = I·R_total
        Rearranged: k·φ(I)·N - I·R_total = 0
        """
        if I <= 0:
            return 1e10  # Large error for invalid current

        k_phi = k_phi_func(I)
        E_generated = k_phi * target_speed
        E_required = I * R_total

        return E_generated - E_required

    # Find solution using fsolve
    # Try multiple initial guesses
    solutions = []
    for initial_guess in [50, 75, 100, 125, 150]:
        try:
            sol = fsolve(generator_equation, initial_guess, full_output=True)
            if sol[2] == 1 and sol[0][0] > 0:  # Solution found and positive
                solutions.append(sol[0][0])
        except:
            pass

    if solutions:
        # Take the solution closest to the valid data range
        I_gen = np.mean(solutions)

        print(f"Solution converged: I = {I_gen:.2f} A")
        print()

        # Calculate performance parameters
        k_phi_gen = k_phi_func(I_gen)
        E_gen = k_phi_gen * target_speed
        V_terminal = E_gen - I_gen * R_motor  # Terminal voltage
        P_generated = E_gen * I_gen
        P_armature_loss = I_gen**2 * R_motor
        P_load = I_gen**2 * R_load
        P_output = P_generated - P_armature_loss
        efficiency = (P_output / P_generated * 100) if P_generated > 0 else 0

        print("GENERATOR PERFORMANCE:")
        print("-" * 80)
        print(f"Generator Current:           {I_gen:.2f} A")
        print(f"Flux constant k·φ:           {k_phi_gen:.6f} V·min/rev")
        print(f"Generated EMF:               {E_gen:.2f} V")
        print(f"Terminal Voltage:            {V_terminal:.2f} V")
        print()
        print(f"Generated Power (E·I):       {P_generated:.2f} W  ({P_generated/1000:.3f} kW)")
        print(f"Armature I²R Loss:           {P_armature_loss:.2f} W")
        print(f"Load Power:                  {P_load:.2f} W  ({P_load/1000:.3f} kW)")
        print(f"Output Power:                {P_output:.2f} W  ({P_output/1000:.3f} kW)")
        print(f"Efficiency (electrical):     {efficiency:.2f} %")
        print()

        # Torque
        omega = target_speed * 2 * np.pi / 60
        torque = E_gen * I_gen / omega
        print(f"Electromagnetic Torque:      {torque:.2f} N·m")
        print(f"Mechanical Input Power:      {torque * omega:.2f} W  ({torque*omega/1000:.3f} kW)")
        print()

        # Verification
        print("VERIFICATION:")
        print("-" * 80)
        print(f"LHS: E = k·φ·N = {k_phi_gen:.6f} × {target_speed} = {E_gen:.2f} V")
        print(f"RHS: I·R = {I_gen:.2f} × {R_total} = {I_gen*R_total:.2f} V")
        error = abs(E_gen - I_gen * R_total)
        print(f"Error: {error:.6f} V")

        if error < 0.1:
            print("✓ Equation satisfied!")
        print()

        # Compare with motor operation at same speed
        print("COMPARISON WITH MOTOR MODE AT SAME SPEED:")
        print("-" * 80)

        # Interpolate motor current at 1000 rpm
        if 1000 >= speed_data.min() and 1000 <= speed_data.max():
            current_at_1000_motor = np.interp(1000, speed_data[::-1], current_data[::-1])
            print(f"Motor current at 1000 rpm:   {current_at_1000_motor:.2f} A")
            print(f"Generator current at 1000 rpm: {I_gen:.2f} A")
            print()
            if I_gen < current_at_1000_motor:
                print("Generator current is lower than motor current at same speed.")
            else:
                print("Generator current is higher than motor current at same speed.")
        print()

        print("=" * 80)
        print("FINAL ANSWER")
        print("=" * 80)
        print()
        print(f"When operating as a SERIES GENERATOR at {target_speed} rpm,")
        print(f"with load resistance {R_load} Ω and armature resistance {R_motor} Ω:")
        print()
        print(f"    GENERATOR CURRENT = {I_gen:.2f} A")
        print(f"    TERMINAL VOLTAGE  = {V_terminal:.2f} V")
        print(f"    OUTPUT POWER      = {P_output/1000:.2f} kW")
        print()
        print("=" * 80)

        return I_gen

    else:
        print("✗ No valid solution found")
        print()
        print("This may indicate:")
        print("  - Speed is outside the valid operating range")
        print("  - Load resistance is too high/low")
        print("  - Machine cannot self-excite at this condition")
        return None


if __name__ == "__main__":
    result = solve_series_generator_correct()

    print()
    print("Note: This solution assumes the series generator is separately excited")
    print("or has residual magnetism to establish initial field.")
    print()
    print("To run the full GUI application:")
    print("  python3 series_motor_generator_lab.py")
    print()
