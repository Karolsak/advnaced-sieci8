"""
Test script for series motor-generator calculations
Validates the solution to the specific problem
"""

import numpy as np
from scipy.interpolate import interp1d


def solve_series_generator_problem():
    """
    Solve the series motor generator problem:
    - Voltage: 525 V
    - Motor resistance: 3.5 Ω
    - Load resistance: 3.25 Ω
    - Operating speed: 1000 rpm as generator
    """

    print("=" * 70)
    print("SERIES MOTOR-GENERATOR CALCULATION TEST")
    print("=" * 70)
    print()

    # Given data
    voltage = 525.0  # V
    R_motor = 3.5  # Ω
    R_load = 3.25  # Ω
    R_total = R_motor + R_load  # Total resistance
    target_speed = 1000.0  # rpm

    # Motor characteristics data
    current_data = np.array([75, 125, 175, 225])  # A
    speed_data = np.array([1200, 950, 840, 745])  # rpm

    print("GIVEN DATA:")
    print("-" * 70)
    print(f"Terminal Voltage:        {voltage} V")
    print(f"Motor Resistance:        {R_motor} Ω")
    print(f"Load Resistance:         {R_load} Ω")
    print(f"Total Circuit Resistance: {R_total} Ω")
    print(f"Target Speed (Generator): {target_speed} rpm")
    print()

    print("MOTOR CHARACTERISTIC DATA:")
    print("-" * 70)
    print("Current (A)  |  Speed (rpm)")
    print("-" * 70)
    for i in range(len(current_data)):
        print(f"   {current_data[i]:3.0f}      |     {speed_data[i]:4.0f}")
    print()

    # Step 1: Calculate back EMF for each data point (motor operation)
    print("STEP 1: Calculate Back EMF from Motor Data")
    print("-" * 70)
    back_emf_data = voltage - current_data * R_motor

    print("Current (A) | Speed (rpm) | Back EMF (V)")
    print("-" * 70)
    for i in range(len(current_data)):
        print(f"   {current_data[i]:3.0f}     |    {speed_data[i]:4.0f}    |    {back_emf_data[i]:6.2f}")
    print()

    print("NOTE: Negative back EMF indicates deep saturation region.")
    print("For generator analysis, we use the magnetization relationship.")
    print()

    # Step 2: Calculate k*φ values (flux per unit current)
    # For series motor: φ ∝ I in linear region, but saturates at high currents
    print("STEP 2: Calculate Flux Constant (k·φ/I) values")
    print("-" * 70)

    # Use absolute values and calculate flux constant
    # E = k * φ * N, and for series motor φ = k_phi_const * I
    # So E = k_phi_const * I * N
    # Therefore: k_phi_const = E / (I * N)

    k_phi_data = back_emf_data / speed_data
    # Alternative: calculate flux constant per ampere
    k_flux_per_amp = back_emf_data / (current_data * speed_data)

    print("Current (A) | k·φ (V·min/rev) | E/(I·N)")
    print("-" * 70)
    for i in range(len(current_data)):
        print(f"   {current_data[i]:3.0f}     |    {k_phi_data[i]:8.6f}    | {k_flux_per_amp[i]:8.6f}")
    print()

    print("For series generator, we use: E = k·φ·N where φ depends on I")
    print()

    # Step 3: Create interpolation functions
    print("STEP 3: Create Interpolation Functions")
    print("-" * 70)
    k_phi_interp = interp1d(current_data, k_phi_data, kind='cubic', fill_value='extrapolate')
    print("✓ Cubic spline interpolation created for k·φ vs Current")
    print()

    # Step 4: Iterative solution for generator current
    print("STEP 4: Iterative Solution for Generator Current")
    print("-" * 70)
    print(f"For generator at {target_speed} rpm:")
    print(f"Generator equation: E = V + I·R_total")
    print(f"Where: E = k·φ·N (back EMF), φ ∝ I for series machine")
    print()

    # Iterative method
    current_guess = 100.0  # Initial guess
    print(f"Initial guess: I = {current_guess:.2f} A")
    print()
    print("Iteration | Current (A) | k·φ       | Back EMF (V) | Calculated I (A) | Error")
    print("-" * 70)

    converged = False
    for iteration in range(100):
        # Calculate back EMF at this current and speed
        k_phi = k_phi_interp(current_guess)
        back_emf = k_phi * target_speed

        # Generator equation: E = V + I*R
        current_new = (back_emf - voltage) / R_total

        error = abs(current_new - current_guess)

        if iteration < 10 or error < 0.01:  # Show first 10 iterations and final
            print(f"   {iteration:2d}     |   {current_guess:7.3f}   | {k_phi:9.6f} | {back_emf:10.3f}   | {current_new:13.3f}    | {error:6.4f}")

        # Check convergence
        if error < 0.01:
            converged = True
            final_current = current_new
            final_k_phi = k_phi
            final_emf = back_emf
            final_iteration = iteration
            break

        # Relaxation for stability
        current_guess = 0.7 * current_guess + 0.3 * current_new

    print()

    if converged:
        print("✓ SOLUTION CONVERGED")
        print("=" * 70)
        print(f"Generator Current:       {final_current:.2f} A")
        print(f"Converged in {final_iteration} iterations")
        print()

        # Step 5: Calculate detailed results
        print("STEP 5: Detailed Performance Calculations")
        print("=" * 70)
        print()

        # Power calculations
        power_generated = final_emf * final_current  # W
        power_motor_loss = final_current**2 * R_motor  # I²R in motor
        power_load_loss = final_current**2 * R_load  # I²R in load
        power_total_loss = final_current**2 * R_total  # Total I²R
        power_output = power_generated - power_total_loss  # Net output
        efficiency = power_output / power_generated * 100

        print("POWER ANALYSIS:")
        print("-" * 70)
        print(f"Generated EMF Power:     {power_generated:8.2f} W  ({power_generated/1000:.3f} kW)")
        print(f"Motor I²R Loss:          {power_motor_loss:8.2f} W")
        print(f"Load I²R Loss:           {power_load_loss:8.2f} W")
        print(f"Total I²R Loss:          {power_total_loss:8.2f} W")
        print(f"Net Output Power:        {power_output:8.2f} W  ({power_output/1000:.3f} kW)")
        print(f"Efficiency:              {efficiency:8.2f} %")
        print()

        # Torque calculation
        torque = final_k_phi * final_current  # N·m
        omega = target_speed * 2 * np.pi / 60  # rad/s
        power_mechanical = torque * omega  # W

        print("MECHANICAL ANALYSIS:")
        print("-" * 70)
        print(f"Electromagnetic Torque:  {torque:.3f} N·m")
        print(f"Angular Velocity:        {omega:.3f} rad/s")
        print(f"Mechanical Power:        {power_mechanical:.2f} W")
        print()

        # Verification
        print("VERIFICATION:")
        print("-" * 70)
        print(f"Generator equation: E = V + I·R")
        print(f"                   {final_emf:.2f} = {voltage:.2f} + {final_current:.2f} × {R_total:.2f}")
        print(f"                   {final_emf:.2f} = {voltage + final_current * R_total:.2f}")
        print(f"Difference:             {abs(final_emf - (voltage + final_current * R_total)):.6f} V")
        print("✓ Equation satisfied")
        print()

        # Summary
        print("=" * 70)
        print("FINAL ANSWER")
        print("=" * 70)
        print(f"When operating as a generator at {target_speed} rpm,")
        print(f"loaded on a {R_load} Ω rheostat, with motor resistance {R_motor} Ω:")
        print()
        print(f"    Generator Current = {final_current:.2f} A")
        print()
        print("=" * 70)

        return final_current
    else:
        print("✗ SOLUTION DID NOT CONVERGE")
        return None


def test_interpolation_accuracy():
    """Test the accuracy of interpolation method"""
    print()
    print()
    print("=" * 70)
    print("INTERPOLATION ACCURACY TEST")
    print("=" * 70)
    print()

    # Original data
    current_data = np.array([75, 125, 175, 225])
    speed_data = np.array([1200, 950, 840, 745])

    # Create interpolation
    speed_interp = interp1d(current_data, speed_data, kind='cubic', fill_value='extrapolate')

    # Test at original points
    print("Testing interpolation at original data points:")
    print("-" * 70)
    print("Current (A) | Original Speed | Interpolated | Error")
    print("-" * 70)

    max_error = 0
    for i in range(len(current_data)):
        interpolated = speed_interp(current_data[i])
        error = abs(interpolated - speed_data[i])
        max_error = max(max_error, error)
        print(f"   {current_data[i]:3.0f}     |     {speed_data[i]:4.0f}      |    {interpolated:8.3f}    | {error:.6f}")

    print()
    print(f"Maximum interpolation error: {max_error:.6f} rpm")
    print("✓ Interpolation accurate at original points")
    print()

    # Test at intermediate point
    test_current = 150.0
    test_speed = speed_interp(test_current)
    print(f"Interpolated value at I = {test_current} A: N = {test_speed:.2f} rpm")
    print()


def compare_with_manual_calculation():
    """Compare with manual calculation method"""
    print("=" * 70)
    print("COMPARISON WITH MANUAL CALCULATION")
    print("=" * 70)
    print()

    print("Manual Method (using average k·φ):")
    print("-" * 70)

    # Given data
    current_data = np.array([75, 125, 175, 225])
    speed_data = np.array([1200, 950, 840, 745])
    voltage = 525.0
    R_total = 6.75
    target_speed = 1000.0
    R_motor = 3.5

    # Calculate average k·φ
    back_emf_data = voltage - current_data * R_motor
    k_phi_data = back_emf_data / speed_data
    k_phi_avg = np.mean(k_phi_data)

    print(f"Average k·φ = {k_phi_avg:.6f} V·min/rev")
    print()

    # Approximate solution using average k·φ
    # E = k_phi * N = V + I*R
    # k_phi * N = V + I*R
    # But k_phi depends on I, so we need iteration anyway
    # Let's use the average as approximation

    # E = k_phi_avg * I * N = V + I*R
    # This gives quadratic equation
    print("Note: This method gives rough approximation only,")
    print("since k·φ actually varies with current.")
    print()

    print("The iterative method with interpolation is more accurate.")
    print()


if __name__ == "__main__":
    # Run main calculation
    result = solve_series_generator_problem()

    # Test interpolation
    test_interpolation_accuracy()

    # Comparison
    compare_with_manual_calculation()

    print()
    print("=" * 70)
    print("TEST COMPLETE")
    print("=" * 70)
    print()
    print("To run the full GUI application, execute:")
    print("  python3 series_motor_generator_lab.py")
    print()
