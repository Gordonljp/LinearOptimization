"""Exact objective evaluation of a decoded MO-EVDRP solution.

Used (a) to re-check MILP solutions with the true route-dependent payload
(Eq. 6 with l^v_ij = load still on board) and the actual makespan, and
(b) as the fitness function of the INSGA-II skeleton.

A solution is
    ev_routes : list of {"route": [0, ..., n+e+1], "recharge": {station_id: kWh}}
    drone_customers : list of customer ids served by drones (one per drone)
"""
import math

from energy import drone_energy, ev_energy_coeffs
from params import DroneParams, EnvParams, VehicleParams


def node_tables(inst):
    n, e = len(inst["customers"]), len(inst["stations"])
    ret = n + e + 1
    dep, sc = inst["depot"], inst["scenario"]
    pos = {0: (dep["x"], dep["y"]), ret: (dep["x"], dep["y"])}
    elev = {0: dep.get("elev", 0.0), ret: dep.get("elev", 0.0)}
    traf = {0: sc["traffic_level"], ret: sc["traffic_level"]}
    q = {}
    for c in inst["customers"]:
        pos[c["id"]], elev[c["id"]], traf[c["id"]], q[c["id"]] = (c["x"], c["y"]), c.get("elev", 0.0), c["traffic"], c["demand_kg"]
    for s in inst["stations"]:
        pos[s["id"]], elev[s["id"]], traf[s["id"]] = (s["x"], s["y"]), s.get("elev", 0.0), s.get("traffic", sc["traffic_level"])
    return dict(n=n, e=e, ret=ret, pos=pos, elev=elev, traf=traf, q=q,
                Nc=[c["id"] for c in inst["customers"]], Ns=[s["id"] for s in inst["stations"]])


def evaluate(inst, ev_routes, drone_customers, ev=None, dr=None, env=None, tab=None):
    ev, dr, env = ev or VehicleParams(), dr or DroneParams(), env or EnvParams()
    tab = tab or node_tables(inst)
    P, sc, q = tab["pos"], inst["scenario"], tab["q"]
    viol, f1, f2, makespan, details = [], 0.0, 0.0, 0.0, []

    served = [i for r in ev_routes for i in r["route"] if i in q] + list(drone_customers)
    if sorted(served) != sorted(tab["Nc"]):
        viol.append("each customer must be served exactly once (17)")
    if len(ev_routes) > inst["fleet"]["n_vehicles"]:
        viol.append("too many electric vehicles")
    if len(drone_customers) > inst["fleet"]["n_drones"]:
        viol.append("too many drone trips (16)")

    for k, r in enumerate(ev_routes):
        route, rech = r["route"], r.get("recharge", {})
        load = sum(q[i] for i in route if i in q)
        if load > ev.load_cap_kg + 1e-9:
            viol.append(f"EV {k}: load {load:.1f} > capacity (24)")
        B, t, stops = ev.battery_kwh, 0.0, []
        f2 += ev.fixed_cost
        for i, j in zip(route, route[1:]):
            if i in rech:                                   # partial recharge at station i
                g = rech[i]
                if g > ev.battery_kwh - B + 1e-9:
                    viol.append(f"EV {k}: recharge at {i} exceeds capacity (28)")
                B += g; t += g / ev.recharge_kw; f2 += ev.recharge_cost_per_kwh * g
            a, b = ev_energy_coeffs(P[i], P[j], ev, env, sc, 0.5 * (tab["traf"][i] + tab["traf"][j]))
            e_ij = a + b * load
            dM = abs(P[i][0] - P[j][0]) + abs(P[i][1] - P[j][1])
            B -= e_ij; t += dM / ev.speed_kmh; f1 += e_ij; f2 += ev.cost_per_km * dM
            if B < -1e-9:
                viol.append(f"EV {k}: battery depleted before node {j} (26)-(27)")
            if j in q:
                load -= q[j]
            stops.append({"node": j, "arrival_h": round(t, 4), "battery_kwh": round(B, 3)})
        makespan = max(makespan, t)
        details.append({"route": route, "stops": stops})

    for i in drone_customers:
        e_out = drone_energy(P[0], P[i], q[i], dr, env, sc, tab["elev"][0], tab["elev"][i])
        e_back = drone_energy(P[i], P[0], 0.0, dr, env, sc, tab["elev"][i], tab["elev"][0])
        if q[i] > dr.load_cap_kg or e_out + e_back > dr.battery_kwh:
            viol.append(f"drone customer {i} not eligible (22)")
        d0 = math.dist(P[0], P[i])
        f1 += e_out + e_back
        f2 += dr.fixed_cost + 2 * dr.cost_per_km * d0
        makespan = max(makespan, 2 * d0 / dr.speed_kmh)

    return {"f1": f1, "f2": f2, "f3": makespan, "feasible": not viol, "violations": viol, "routes": details}
