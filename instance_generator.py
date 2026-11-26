"""Random MO-EVDRP test instances (supply + demand + operating scenario).

Follows the set-up of Section 5.1 of the paper: one depot in the centre of a
square service area (50 x 50 km in the paper), customer demands between
5 and 1000 kg, traffic levels 0.25/0.50/0.75, 8-20 recharging stations and
per-instance temperature, humidity, wind speed/direction plus per-customer
traffic indicators. Elevation is optional (the paper sets it inactive).

Node indexing used everywhere in this package
    0                 depot (departure)
    1..n              customers                 N_c
    n+1..n+e          recharging stations       N_s
    n+e+1             dummy return depot  0'
"""
import json
import math
import random
from dataclasses import dataclass, field

from params import VehicleParams, DroneParams


@dataclass
class GeneratorConfig:
    n_customers: int = 10
    n_stations: int = 3
    n_drones: int = 3
    n_vehicles: int | None = None            # None -> derived from demand
    area_km: float = 20.0
    traffic_level: float = 0.50              # I_traf scenario: 0.25 / 0.50 / 0.75
    traffic_spread: float = 0.10             # std-dev of customer traffic indicators
    demand_range: tuple = (5.0, 1000.0)      # kg
    drone_parcel_share: float = 0.4          # share of customers with demand <= L^d_max
    temperature_range: tuple = (0.0, 35.0)   # degC
    humidity_range: tuple = (0.3, 0.9)
    wind_speed_range: tuple = (0.0, 10.0)    # m/s
    elevation_range: tuple | None = None     # e.g. (0, 50) m; None -> flat
    supply_margin: tuple = (0.05, 0.30)      # depot stock = demand * (1 + U(margin))
    seed: int | None = None
    vehicle: VehicleParams = field(default_factory=VehicleParams)
    drone: DroneParams = field(default_factory=DroneParams)


PAPER_PRESET = dict(area_km=50.0, demand_range=(5.0, 1000.0))


def generate_instance(cfg: GeneratorConfig) -> dict:
    rng = random.Random(cfg.seed)
    n, e, A = cfg.n_customers, cfg.n_stations, cfg.area_km
    elev = (lambda: rng.uniform(*cfg.elevation_range)) if cfg.elevation_range else (lambda: 0.0)
    depot = {"id": 0, "x": A / 2, "y": A / 2, "elev": 0.0}

    lo, hi = cfg.demand_range
    dcap = cfg.drone.load_cap_kg
    customers = []
    for i in range(1, n + 1):
        if rng.random() < cfg.drone_parcel_share:
            q = rng.uniform(lo, min(dcap, hi))          # drone-weight parcel
        else:
            q = rng.uniform(max(lo, dcap), hi)          # EV-only parcel
        customers.append({
            "id": i, "x": rng.uniform(0, A), "y": rng.uniform(0, A), "elev": elev(),
            "demand_kg": round(q, 1),
            "traffic": round(min(1.0, max(0.0, rng.gauss(cfg.traffic_level, cfg.traffic_spread))), 3),
        })

    # stations: spread over the area on a jittered grid so routes can reach them
    stations = []
    k = math.ceil(math.sqrt(e))
    cells = [(a, b) for a in range(k) for b in range(k)]
    rng.shuffle(cells)
    for s, (a, b) in enumerate(cells[:e]):
        stations.append({"id": n + 1 + s,
                         "x": (a + rng.uniform(0.2, 0.8)) * A / k,
                         "y": (b + rng.uniform(0.2, 0.8)) * A / k,
                         "elev": elev(), "traffic": cfg.traffic_level})

    total = sum(c["demand_kg"] for c in customers)
    heavy = sum(c["demand_kg"] for c in customers if c["demand_kg"] > dcap)
    n_veh = cfg.n_vehicles or max(1, math.ceil(heavy / cfg.vehicle.load_cap_kg) + 1)
    if n_veh * cfg.vehicle.load_cap_kg + cfg.n_drones * dcap < total * 0.999 and cfg.n_vehicles:
        print("warning: fleet capacity may be insufficient for total demand")

    scenario = {
        "temperature_c": round(rng.uniform(*cfg.temperature_range), 2),
        "humidity": round(rng.uniform(*cfg.humidity_range), 3),
        "wind_speed_ms": round(rng.uniform(*cfg.wind_speed_range), 2),
        "wind_dir_rad": round(rng.uniform(0, 2 * math.pi), 4),
        "traffic_level": cfg.traffic_level,
    }
    supply = round(total * (1 + rng.uniform(*cfg.supply_margin)), 1)
    depot["supply_kg"] = supply

    return {
        "name": f"C{n}_{cfg.traffic_level:.2f}_R{e}",
        "seed": cfg.seed, "area_km": A,
        "depot": depot, "customers": customers, "stations": stations,
        "fleet": {"n_vehicles": n_veh, "n_drones": cfg.n_drones},
        "scenario": scenario,
        "totals": {"demand_kg": round(total, 1), "supply_kg": supply},
    }


def save_instance(inst, path):
    with open(path, "w") as f:
        json.dump(inst, f, indent=2)


def load_instance(path):
    with open(path) as f:
        return json.load(f)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Generate random MO-EVDRP instances")
    ap.add_argument("--customers", type=int, default=10)
    ap.add_argument("--stations", type=int, default=3)
    ap.add_argument("--drones", type=int, default=3)
    ap.add_argument("--vehicles", type=int, default=None)
    ap.add_argument("--traffic", type=float, default=0.5)
    ap.add_argument("--area", type=float, default=20.0)
    ap.add_argument("--paper", action="store_true", help="50x50 km, demand 5-1000 kg")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    cfg = GeneratorConfig(n_customers=a.customers, n_stations=a.stations, n_drones=a.drones,
                          n_vehicles=a.vehicles, traffic_level=a.traffic, area_km=a.area, seed=a.seed)
    if a.paper:
        cfg.area_km, cfg.demand_range = PAPER_PRESET["area_km"], PAPER_PRESET["demand_range"]
    inst = generate_instance(cfg)
    out = a.out or f"{inst['name']}_s{a.seed}.json"
    save_instance(inst, out)
    print(f"wrote {out}: {inst['totals']}, fleet {inst['fleet']}")
