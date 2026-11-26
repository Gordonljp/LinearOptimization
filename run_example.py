"""End-to-end demo: random instance -> MILP (constraints 13-28) -> Pareto points.

    python run_example.py                         # 6 customers, 2 stations, HiGHS
    python run_example.py --customers 8 --solver lpsolve --time 120
    python run_example.py --instance my_instance.json --payload exact
"""
import argparse
import json

from instance_generator import GeneratorConfig, generate_instance, load_instance, save_instance
from milp_model import EVDRPModel


def fmt(o):
    return "f1={f1:8.3f} kWh  f2={f2:9.1f}  f3={f3:6.3f} h".format(**o) if o else "no solution"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--instance", help="JSON instance file (otherwise one is generated)")
    ap.add_argument("--customers", type=int, default=6)
    ap.add_argument("--stations", type=int, default=2)
    ap.add_argument("--drones", type=int, default=2)
    ap.add_argument("--vehicles", type=int, default=None)
    ap.add_argument("--traffic", type=float, default=0.5)
    ap.add_argument("--area", type=float, default=20.0)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--solver", default="highs", choices=["highs", "cbc", "lpsolve"])
    ap.add_argument("--payload", default="half", help='"half"|"full"|"empty"|"exact"|<kg>')
    ap.add_argument("--time", type=float, default=60, help="time limit per MILP solve [s]")
    ap.add_argument("--grid", type=int, default=3, help="eps-constraint grid size per objective")
    ap.add_argument("--out", default="results.json")
    a = ap.parse_args()

    if a.instance:
        inst = load_instance(a.instance)
    else:
        inst = generate_instance(GeneratorConfig(n_customers=a.customers, n_stations=a.stations,
                                                 n_drones=a.drones, n_vehicles=a.vehicles,
                                                 traffic_level=a.traffic, area_km=a.area, seed=a.seed))
        save_instance(inst, f"{inst['name']}_s{a.seed}.json")
    print(f"Instance {inst['name']}: demand {inst['totals']['demand_kg']} kg, depot supply "
          f"{inst['totals']['supply_kg']} kg, fleet {inst['fleet']}")
    assert inst["totals"]["supply_kg"] >= inst["totals"]["demand_kg"], "depot supply < demand"

    payload = a.payload if a.payload in ("half", "full", "empty", "exact") else float(a.payload)
    M = EVDRPModel(inst, payload_mode=payload)
    print("Model size:", M.size(), "| drone-eligible customers:", M.C_elig)
    kw = dict(solver=a.solver, time_limit=a.time)

    print("\n-- payoff table (min each objective) --")
    ideal, nadir, table = M.payoff_table(**kw)
    for k, o in table.items():
        print(f"min {k}: {fmt(o)}")

    print("\n-- normalised weighted sum (1/3 each) --")
    ws = M.solve_weighted({"f1": 1, "f2": 1, "f3": 1}, ideal, nadir, **kw)
    print(ws["status"], fmt(ws["objectives"]))

    print(f"\n-- augmented eps-constraint ({a.grid}x{a.grid} grid on f2, f3) --")
    front = M.epsilon_constraint(ideal, nadir, primary="f1", grid=a.grid, **kw)
    for s in sorted(front, key=lambda s: s["objectives"]["f1"]):
        print(s["status"][:8].ljust(8), fmt(s["objectives"]))

    best = ws["solution"]
    print("\nWeighted-sum solution:")
    for r in best["ev_routes"]:
        print(f"  EV {r['vehicle']}: {r['route']}  load {r['load_kg']} kg  recharge % {r['recharge_pct']}")
    for d in best["drone_trips"]:
        print(f"  Drone {d['drone']}: depot -> {d['customer']} -> depot  ({d['energy_kwh']} kWh)")
    print("  exact-payload re-evaluation:", {k: round(v, 3) if isinstance(v, float) else v
                                             for k, v in ws["exact_payload_check"].items()})

    with open(a.out, "w") as f:
        json.dump({"instance": inst["name"], "ideal": ideal, "nadir": nadir, "payoff": table,
                   "weighted": ws, "pareto": front}, f, indent=2, default=str)
    print(f"\nSaved {a.out}")


if __name__ == "__main__":
    main()
