from dataclasses import asdict, dataclass


@dataclass
class Truckdata:
    # Average travel speed of electric vehicle (s^v)   
    average_ev_speed: float = 55.0  # km/h
    # Battery energy capacity of electric vehicle (B_max^v)
    ev_battery_capacity: float = 60.0  # kWh
    # Load capacity of electric vehicle (L_max^v)
    ev_load_capacity: float = 1000.0  # kg    
    # Curb mass of electric vehicle (m_0^v)
    ev_curb_mass: float = 2000.0  # kg
    # Additional energy coefficient per unit mass and distance (a_tilde^d)
    additional_energy_coefficient: float = 0.0036  # kWh/(kg*km)

    # Battery recharging rate for electric vehicle (r)
    ev_recharging_rate: float = 300.0  # kW

    # Fixed cost of using an electric vehicle (c_f^v)
    ev_fixed_cost: float = 400.0  # monetary units

    # Operating cost per unit distance for electric vehicle (c^v)
    ev_operating_cost: float = 20.0  # monetary units/km

    # Recharging cost per unit energy for electric vehicle (c_r^v)
    ev_recharging_cost: float = 5.0  # monetary units/kWh        


class DroneData:
    # Average flight speed of drone (s^d)
    average_drone_speed: float = 60.0  # km/h
    # Battery energy capacity of drone (B_max^d)
    drone_battery_capacity: float = 2.0  # kWh
    # Load capacity of drone (L_max^d)
    drone_load_capacity: float = 15.0  # kg  

    # Curb mass of drone (m_0^d)
    drone_curb_mass: float = 7.5  # kg    
    # Fixed cost of each delivery of a drone (c_f^d)
    drone_fixed_cost: float = 400.0  # monetary units
    # Operating cost per unit distance for drone (c^d)
    drone_operating_cost: float = 1.0  # monetary units/km
    additional_energy_coefficient: float = 0.0036  # kWh/(kg*km)
    drone_area = 0.5 #m^2
    eta = 0.5 # Assumed aerodynamics coefficent
    efficency = 0.76 # Assumed drone efficency

@dataclass
class EnvParams:
    rho0: float = 1.225              # standard air density kg/m^3
    humidity_coef: float = 0.05      # psi
    humidity_ref: float = 0.5        # H_0 (relative humidity, fraction)
    g: float = 9.81


def to_dict(*objs):
    return {type(o).__name__: asdict(o) for o in objs}
