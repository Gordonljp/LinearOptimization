"""Model parameters for the MO-EVDRP (Zhang et al., 2027, ESWA 332:133716).

Values marked "Table 1" are taken from the paper. The environmental and
aerodynamic coefficients used by Eqs. (1)-(9) are NOT listed in the paper;
the defaults below are placeholder assumptions and should be calibrated.
"""
from dataclasses import dataclass, asdict


@dataclass
class VehicleParams:
    speed_kmh: float = 55.0          # s^v            (Table 1)
    battery_kwh: float = 60.0        # B^v_max        (Table 1)
    load_cap_kg: float = 1000.0      # L^v_max        (Table 1)
    curb_mass_kg: float = 2000.0     # m^v_0          (Table 1)
    recharge_kw: float = 300.0       # r              (Table 1)
    fixed_cost: float = 400.0        # c^v_f          (Table 1)
    cost_per_km: float = 20.0        # c^v            (Table 1)
    recharge_cost_per_kwh: float = 5.0  # c^v_r       (Table 1)
    # --- assumed (not given in the paper) ---
    rolling_coef: float = 0.01       # sigma^v
    drag_coef: float = 0.7           # xi^v
    frontal_area_m2: float = 4.0     # A^v
    eta_ref: float = 0.90            # eta^v
    temp_sens: float = 0.0005        # alpha^v  [1/degC^2]
    temp_opt_c: float = 25.0         # T^v_opt


@dataclass
class DroneParams:
    speed_kmh: float = 60.0          # s^d            (Table 1)
    battery_kwh: float = 2.0         # B^d_max        (Table 1)
    load_cap_kg: float = 15.0        # L^d_max        (Table 1)
    curb_mass_kg: float = 7.5        # m^d_0          (Table 1)
    mass_energy_coef: float = 0.0036  # alpha~^d kWh/(kg km) (Table 1)
    fixed_cost: float = 400.0        # c^d_f          (Table 1)
    cost_per_km: float = 1.0         # c^d            (Table 1)
    # --- assumed (not given in the paper) ---
    drag_coef: float = 1.0           # xi^d
    frontal_area_m2: float = 0.1     # A^d
    eta_ref: float = 0.85            # eta^d
    temp_sens: float = 0.0008        # alpha^d
    temp_opt_c: float = 20.0         # T^d_opt
    altitude_coef: float = 2.725e-6  # beta~^d kWh/(kg m)  (= g/3.6e6, assumed)


@dataclass
class EnvParams:
    rho0: float = 1.225              # standard air density kg/m^3
    humidity_coef: float = 0.05      # psi
    humidity_ref: float = 0.5        # H_0 (relative humidity, fraction)
    g: float = 9.81


def to_dict(*objs):
    return {type(o).__name__: asdict(o) for o in objs}
