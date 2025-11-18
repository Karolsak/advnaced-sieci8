"""
Test script to verify capacitor calculation without GUI
This script can run without matplotlib/tkinter
"""

import cmath
import math

def calculate_quadrature_capacitor(V, f, Rm, Xm, Ra, Xa):
    """
    Calculate the capacitor value needed for quadrature condition

    Parameters:
    V: Voltage (V)
    f: Frequency (Hz)
    Rm: Main winding resistance (ohm)
    Xm: Main winding reactance (ohm)
    Ra: Auxiliary winding resistance (ohm)
    Xa: Auxiliary winding reactance (ohm)

    Returns:
    C: Capacitance (F)
    C_uF: Capacitance (microfarads)
    results: Dictionary with detailed results
    """

    omega = 2 * math.pi * f

    # Main winding impedance
    Zm = complex(Rm, Xm)

    # Main winding current
    Im = V / Zm
    angle_Im = cmath.phase(Im)
    angle_Im_deg = math.degrees(angle_Im)

    # Main winding angle
    theta_m = math.atan2(Xm, Rm)
    theta_m_deg = math.degrees(theta_m)

    # For quadrature: auxiliary current should lead main by 90 degrees
    # This means auxiliary impedance angle = main impedance angle - 90°
    theta_a_desired = theta_m - math.pi/2
    theta_a_desired_deg = math.degrees(theta_a_desired)

    # Calculate required capacitive reactance
    # Za_total = Ra + j(Xa - Xc)
    # tan(theta_a) = (Xa - Xc) / Ra
    # Xc = Xa - Ra * tan(theta_a)

    Xc_required = Xa - Ra * math.tan(theta_a_desired)

    # Calculate capacitance
    # Xc = 1 / (omega * C)
    # C = 1 / (omega * Xc)

    if Xc_required > 0:
        C = 1.0 / (omega * Xc_required)
        C_uF = C * 1e6  # Convert to microfarads

        # Verify quadrature condition
        Za_total = complex(Ra, Xa - Xc_required)
        Ia = V / Za_total

        angle_Ia = cmath.phase(Ia)
        angle_Ia_deg = math.degrees(angle_Ia)

        angle_diff = abs(angle_Im - angle_Ia)
        angle_diff_deg = math.degrees(angle_diff)

        results = {
            'C_F': C,
            'C_uF': C_uF,
            'Xc_ohm': Xc_required,
            'Im_magnitude': abs(Im),
            'Im_angle_deg': angle_Im_deg,
            'Ia_magnitude': abs(Ia),
            'Ia_angle_deg': angle_Ia_deg,
            'phase_diff_deg': angle_diff_deg,
            'theta_m_deg': theta_m_deg,
            'theta_a_deg': theta_a_desired_deg,
            'Zm_magnitude': abs(Zm),
            'Za_magnitude': abs(Za_total),
        }

        return C, C_uF, results
    else:
        raise ValueError("Negative capacitive reactance required. Check winding parameters.")

def print_results(V, f, Rm, Xm, Ra, Xa):
    """Print formatted results"""
    print("="*70)
    print("CAPACITOR-START MOTOR ANALYSIS")
    print("="*70)
    print(f"\nMotor Parameters:")
    print(f"  Voltage: {V} V")
    print(f"  Frequency: {f} Hz")
    print(f"  Main winding:      Zm = ({Rm} + j{Xm}) Ω")
    print(f"  Auxiliary winding: Za = ({Ra} + j{Xa}) Ω")

    try:
        C, C_uF, results = calculate_quadrature_capacitor(V, f, Rm, Xm, Ra, Xa)

        print(f"\n{'='*70}")
        print("QUADRATURE CAPACITOR CALCULATION RESULTS")
        print(f"{'='*70}")
        print(f"\nStarting Capacitor Value:")
        print(f"  C = {C_uF:.2f} µF  ({C*1e6:.4f} µF)")
        print(f"  C = {C:.6e} F")
        print(f"  Xc = {results['Xc_ohm']:.2f} Ω")

        print(f"\nImpedance Angles:")
        print(f"  Main winding angle:      θm = {results['theta_m_deg']:.2f}°")
        print(f"  Auxiliary winding angle: θa = {results['theta_a_deg']:.2f}°")

        print(f"\nCurrent Analysis:")
        print(f"  Main winding current:      |Im| = {results['Im_magnitude']:.2f} A ∠ {results['Im_angle_deg']:.2f}°")
        print(f"  Auxiliary winding current: |Ia| = {results['Ia_magnitude']:.2f} A ∠ {results['Ia_angle_deg']:.2f}°")

        print(f"\nQuadrature Verification:")
        print(f"  Phase difference: {results['phase_diff_deg']:.2f}°")

        if abs(results['phase_diff_deg'] - 90) < 1.0:
            print(f"  ✓ QUADRATURE CONDITION SATISFIED (within 1°)")
        else:
            print(f"  ✗ Warning: Phase difference deviates from 90°")

        print(f"\nImpedance Magnitudes:")
        print(f"  |Zm| = {results['Zm_magnitude']:.2f} Ω")
        print(f"  |Za| (with capacitor) = {results['Za_magnitude']:.2f} Ω")

        print(f"\n{'='*70}")

        # Power calculations
        P_in = V * (results['Im_magnitude'] + results['Ia_magnitude'])  # Simplified
        print(f"\nApproximate Power Analysis:")
        print(f"  Apparent input power: ~{P_in:.2f} VA")

        print(f"\n{'='*70}\n")

    except ValueError as e:
        print(f"\nError: {e}")
        print()

# Test with given problem parameters
if __name__ == "__main__":
    # Problem parameters
    POWER = 250.0    # W
    VOLTAGE = 230.0  # V
    FREQ = 50.0      # Hz
    RM = 4.5         # Ω
    XM = 3.7         # Ω
    RA = 9.5         # Ω
    XA = 3.5         # Ω

    print_results(VOLTAGE, FREQ, RM, XM, RA, XA)

    # Test with some variations
    print("\n" + "="*70)
    print("TESTING WITH DIFFERENT FREQUENCIES")
    print("="*70 + "\n")

    for freq in [40, 50, 60]:
        print(f"\nFrequency = {freq} Hz:")
        print("-" * 40)
        try:
            C, C_uF, results = calculate_quadrature_capacitor(VOLTAGE, freq, RM, XM, RA, XA)
            print(f"  Required capacitor: {C_uF:.2f} µF")
            print(f"  Phase difference: {results['phase_diff_deg']:.2f}°")
        except ValueError as e:
            print(f"  Error: {e}")
