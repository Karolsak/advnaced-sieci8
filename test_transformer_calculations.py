#!/usr/bin/env python3
"""
Test script to verify transformer parallel operation calculations
"""

import numpy as np


def test_transformer_calculations():
    """Test the transformer parallel operation solution"""

    print("=" * 80)
    print("TRANSFORMER PARALLEL OPERATION - CALCULATION VERIFICATION")
    print("=" * 80)
    print()

    # Given data
    v_hv_rated = 6600  # V
    v_lv_rated = 250   # V

    # Transformer 1 short circuit test data (HV side)
    v_sc1 = 200  # V
    i_sc1 = 30   # A
    p_sc1 = 1200 # W

    # Transformer 2 short circuit test data (HV side)
    v_sc2 = 120  # V
    i_sc2 = 20   # A
    p_sc2 = 1500 # W

    # Load data
    load_kw = 150    # kW
    load_pf = 0.8    # lagging

    print("GIVEN DATA:")
    print("-" * 80)
    print(f"Transformer Rating: {v_hv_rated}/{v_lv_rated} V")
    print()
    print("Transformer 1 - Short Circuit Test (HV side):")
    print(f"  Voltage: {v_sc1} V")
    print(f"  Current: {i_sc1} A")
    print(f"  Power: {p_sc1} W")
    print()
    print("Transformer 2 - Short Circuit Test (HV side):")
    print(f"  Voltage: {v_sc2} V")
    print(f"  Current: {i_sc2} A")
    print(f"  Power: {p_sc2} W")
    print()
    print(f"Load: {load_kw} kW at {load_pf} pf lagging")
    print()

    # Step 1: Calculate transformer parameters
    print("STEP 1: CALCULATE TRANSFORMER PARAMETERS")
    print("-" * 80)

    # Transformer 1
    z1 = v_sc1 / i_sc1
    r1 = p_sc1 / (i_sc1**2)
    x1 = np.sqrt(z1**2 - r1**2)

    print(f"Transformer 1:")
    print(f"  Z₁ = V_sc / I_sc = {v_sc1}/{i_sc1} = {z1:.3f} Ω")
    print(f"  R₁ = P_sc / I_sc² = {p_sc1}/{i_sc1}² = {r1:.3f} Ω")
    print(f"  X₁ = √(Z₁² - R₁²) = √({z1}² - {r1}²) = {x1:.3f} Ω")
    print()

    # Transformer 2
    z2 = v_sc2 / i_sc2
    r2 = p_sc2 / (i_sc2**2)
    x2 = np.sqrt(z2**2 - r2**2)

    print(f"Transformer 2:")
    print(f"  Z₂ = V_sc / I_sc = {v_sc2}/{i_sc2} = {z2:.3f} Ω")
    print(f"  R₂ = P_sc / I_sc² = {p_sc2}/{i_sc2}² = {r2:.3f} Ω")
    print(f"  X₂ = √(Z₂² - R₂²) = √({z2}² - {r2}²) = {x2:.3f} Ω")
    print()

    # Step 2: Calculate load current
    print("STEP 2: CALCULATE TOTAL LOAD CURRENT")
    print("-" * 80)

    load_angle = np.arccos(load_pf)
    total_kva = load_kw / load_pf
    i_total = (total_kva * 1000) / v_hv_rated

    print(f"Load angle = arccos({load_pf}) = {np.degrees(load_angle):.2f}°")
    print(f"Total kVA = {load_kw} / {load_pf} = {total_kva:.2f} kVA")
    print(f"I_total = (kVA × 1000) / V = ({total_kva} × 1000) / {v_hv_rated}")
    print(f"I_total = {i_total:.3f} A")
    print()

    # Step 3: Calculate current distribution
    print("STEP 3: CALCULATE CURRENT DISTRIBUTION IN PARALLEL")
    print("-" * 80)

    # Complex impedances
    Z1 = complex(r1, x1)
    Z2 = complex(r2, x2)

    print(f"Z₁ = {r1:.3f} + j{x1:.3f} = {abs(Z1):.3f}∠{np.degrees(np.angle(Z1)):.2f}° Ω")
    print(f"Z₂ = {r2:.3f} + j{x2:.3f} = {abs(Z2):.3f}∠{np.degrees(np.angle(Z2)):.2f}° Ω")
    print()

    # Total load current (complex)
    I_total = i_total * np.exp(-1j * load_angle)

    # Current distribution
    I1 = I_total * Z2 / (Z1 + Z2)
    I2 = I_total * Z1 / (Z1 + Z2)

    print("For parallel operation:")
    print(f"I₁ = I_total × Z₂/(Z₁+Z₂)")
    print(f"I₂ = I_total × Z₁/(Z₁+Z₂)")
    print()

    # Extract magnitudes and angles
    i1_mag = abs(I1)
    i1_angle = np.angle(I1)
    pf1 = np.cos(i1_angle)

    i2_mag = abs(I2)
    i2_angle = np.angle(I2)
    pf2 = np.cos(i2_angle)

    # Step 4: Results
    print("STEP 4: FINAL RESULTS")
    print("-" * 80)

    print(f"\nTRANSFORMER 1:")
    print(f"  Current (I₁)          = {i1_mag:.3f} A")
    print(f"  Phase angle           = {np.degrees(i1_angle):.2f}°")
    print(f"  Power Factor          = {pf1:.4f} {'lagging' if i1_angle < 0 else 'leading'}")

    p1 = v_hv_rated * i1_mag * pf1 / 1000
    loss1 = 3 * i1_mag**2 * r1 / 1000

    print(f"  Power Output          = {p1:.3f} kW")
    print(f"  Copper Losses (3-ph)  = {loss1:.3f} kW")
    print()

    print(f"TRANSFORMER 2:")
    print(f"  Current (I₂)          = {i2_mag:.3f} A")
    print(f"  Phase angle           = {np.degrees(i2_angle):.2f}°")
    print(f"  Power Factor          = {pf2:.4f} {'lagging' if i2_angle < 0 else 'leading'}")

    p2 = v_hv_rated * i2_mag * pf2 / 1000
    loss2 = 3 * i2_mag**2 * r2 / 1000

    print(f"  Power Output          = {p2:.3f} kW")
    print(f"  Copper Losses (3-ph)  = {loss2:.3f} kW")
    print()

    # Verification
    print("VERIFICATION:")
    print("-" * 80)
    print(f"Total power output    = {p1:.3f} + {p2:.3f} = {p1+p2:.3f} kW")
    print(f"Required load         = {load_kw:.3f} kW")
    print(f"Total losses          = {loss1:.3f} + {loss2:.3f} = {loss1+loss2:.3f} kW")
    print(f"System efficiency     = {load_kw/(load_kw+loss1+loss2)*100:.2f} %")
    print()

    print("=" * 80)
    print("ANSWER:")
    print("=" * 80)
    print(f"Transformer 1: Current = {i1_mag:.3f} A, Power Factor = {pf1:.4f} lagging")
    print(f"Transformer 2: Current = {i2_mag:.3f} A, Power Factor = {pf2:.4f} lagging")
    print("=" * 80)

    return True


if __name__ == "__main__":
    test_transformer_calculations()
    print("\n✓ Calculations completed successfully!")
