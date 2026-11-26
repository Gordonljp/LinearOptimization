"""MILP of the MO-EVDRP, Section 3.3 of Zhang et al. (2027).

Objectives (10)-(12) and constraints (13)-(28) are implemented one-to-one.
The model is built with PuLP and can be solved with
  * HiGHS   (pip install highspy)            solver="highs"   (default)
  * CBC     (bundled with PuLP 2.x)          solver="cbc"
  * lp_solve 5.5 command line (via free MPS)  solver="lpsolve"

Multi-objective handling (the paper solves the model with INSGA-II, the
MILP itself only states three objectives):
  * solve_single(k)          -> min f_k
  * payoff_table()           -> ideal / nadir points
  * solve_weighted(w)        -> normalised weighted sum
  * epsilon_constraint(...)  -> augmented eps-constraint Pareto set
"""
import math
import os
import re
import shutil
import subprocess
import tempfile
import time

import pulp

from energy import drone_energy, ev_energy_coeffs
from evaluate import evaluate
from params import DroneParams, EnvParams, VehicleParams

OBJ = ("f1", "f2", "f3")


class EVDRPModel:
    def __init__(self, inst, ev=None, dr=None, env=None, payload_mode="half",
                 symmetry_breaking=True, tighten_recharge=True):
        """payload_mode: how e^v_ij (Eq. 6) treats payload l^v_ij
            "half"/"full"/"empty"/<float kg>: fixed nominal payload -> e_ij is a
                  pure input parameter, exactly as in the paper's MILP;
            "exact": adds load-flow variables f_ij so that e_ij = a_ij x_ij + b_ij f_ij
                  (exact, still linear, because Eq. 1 is linear in mass)."""
        self.inst = inst
        self.ev, self.dr, self.env = ev or VehicleParams(), dr or DroneParams(), env or EnvParams()
        self.payload_mode = payload_mode
        self.symmetry_breaking, self.tighten_recharge = symmetry_breaking, tighten_recharge
        self._prepare()
        self._build()

    # ------------------------------------------------------------------ data
    def _prepare(self):
        I, ev, dr = self.inst, self.ev, self.dr
        cust, stat = I["customers"], I["stations"]
        self.n, self.e = len(cust), len(stat)
        self.Nc = [c["id"] for c in cust]
        self.Ns = [s["id"] for s in stat]
        self.ret = self.n + self.e + 1                      # dummy return node 0'
        self.V = list(range(I["fleet"]["n_vehicles"]))
        self.D = list(range(I["fleet"]["n_drones"]))
        dep = I["depot"]
        self.pos = {0: (dep["x"], dep["y"]), self.ret: (dep["x"], dep["y"])}
        self.elev = {0: dep.get("elev", 0.0), self.ret: dep.get("elev", 0.0)}
        self.traf = {0: I["scenario"]["traffic_level"], self.ret: I["scenario"]["traffic_level"]}
        self.q = {}
        for c in cust:
            self.pos[c["id"]] = (c["x"], c["y"]); self.elev[c["id"]] = c.get("elev", 0.0)
            self.traf[c["id"]] = c["traffic"]; self.q[c["id"]] = c["demand_kg"]
        for s in stat:
            self.pos[s["id"]] = (s["x"], s["y"]); self.elev[s["id"]] = s.get("elev", 0.0)
            self.traf[s["id"]] = s.get("traffic", I["scenario"]["traffic_level"])

        # arc set A (Section 3.2)
        A = [(0, j) for j in self.Nc]
        for i in self.Nc + self.Ns:
            for j in self.Nc + self.Ns + [self.ret]:
                if i != j:
                    A.append((i, j))
        self.A = A

        P, sc = self.pos, I["scenario"]
        self.dM = {(i, j): abs(P[i][0] - P[j][0]) + abs(P[i][1] - P[j][1]) for i, j in A}
        self.ea, self.eb = {}, {}
        for i, j in A:
            tr = 0.5 * (self.traf[i] + self.traf[j])          # arc traffic indicator
            self.ea[i, j], self.eb[i, j] = ev_energy_coeffs(P[i], P[j], ev, self.env, sc, tr)

        # drone legs (Eq. 9): loaded out, empty back
        self.dE0 = {i: math.dist(P[0], P[i]) for i in self.Nc}
        self.ed_out = {i: drone_energy(P[0], P[i], self.q[i], dr, self.env, sc, self.elev[0], self.elev[i]) for i in self.Nc}
        self.ed_back = {i: drone_energy(P[i], P[self.ret], 0.0, dr, self.env, sc, self.elev[i], self.elev[self.ret]) for i in self.Nc}
        # Constraint (22): drone-eligible customers
        self.C_elig = [i for i in self.Nc
                       if self.ed_out[i] + self.ed_back[i] <= dr.battery_kwh and self.q[i] <= dr.load_cap_kg]

        L = ev.load_cap_kg
        pm = self.payload_mode
        self.l_nom = {"half": L / 2, "full": L, "empty": 0.0}.get(pm, pm if pm != "exact" else None)
        e_max = max(self.ea[a] + self.eb[a] * L for a in A)
        self.M_B = ev.battery_kwh + e_max
        self.M_T = ((self.n + self.e + 1) * max(self.dM.values()) / ev.speed_kmh
                    + self.e * ev.battery_kwh / ev.recharge_kw
                    + max(2 * d for d in self.dE0.values()) / dr.speed_kmh + 1.0)

    # ----------------------------------------------------------------- model
    def _build(self):
        ev, dr = self.ev, self.dr
        V, D, A, Nc, Ns, ret = self.V, self.D, self.A, self.Nc, self.Ns, self.ret
        m = pulp.LpProblem("MO_EVDRP", pulp.LpMinimize)
        Bmax, L = ev.battery_kwh, ev.load_cap_kg

        z = {v: pulp.LpVariable(f"z_{v}", cat="Binary") for v in V}
        x = {(v, i, j): pulp.LpVariable(f"x_{v}_{i}_{j}", cat="Binary") for v in V for (i, j) in A}
        y = {(d, i): pulp.LpVariable(f"y_{d}_{i}", cat="Binary") for d in D for i in self.C_elig}
        u = {(v, i): pulp.LpVariable(f"u_{v}_{i}", 1, self.n) for v in V for i in Nc}
        r = {(v, s): pulp.LpVariable(f"r_{v}_{s}", 0, Bmax) for v in V for s in Ns}
        nodes = Nc + Ns + [ret]
        tv = {(v, j): pulp.LpVariable(f"tv_{v}_{j}", 0, self.M_T) for v in V for j in nodes}
        td = {d: pulp.LpVariable(f"td_{d}", 0) for d in D}
        B = {(v, j): pulp.LpVariable(f"B_{v}_{j}", 0, Bmax) for v in V for j in nodes}
        T = pulp.LpVariable("Tmax", 0)
        tvar = lambda v, i: 0.0 if i == 0 else tv[v, i]
        Bvar = lambda v, i: Bmax if i == 0 else B[v, i]          # (25) B^v_0 = B^v_max

        # EV arc energy expression e^v_ij x^v_ij
        if self.payload_mode == "exact":
            f = {(v, i, j): pulp.LpVariable(f"f_{v}_{i}_{j}", 0, L) for v in V for (i, j) in A}
            E = {k: self.ea[k[1:]] * x[k] + self.eb[k[1:]] * f[k] for k in x}
        else:
            f = None
            E = {k: (self.ea[k[1:]] + self.eb[k[1:]] * self.l_nom) * x[k] for k in x}

        out_arcs = {i: [] for i in [0] + nodes}
        in_arcs = {j: [] for j in [0] + nodes}
        for (i, j) in A:
            out_arcs[i].append(j); in_arcs[j].append(i)

        # ---------------- objectives (10)-(12)
        f1 = pulp.lpSum(E.values()) + pulp.lpSum((self.ed_out[i] + self.ed_back[i]) * y[d, i] for (d, i) in y)
        f2 = (pulp.lpSum(ev.fixed_cost * z[v] for v in V)
              + pulp.lpSum(self.dM[i, j] * ev.cost_per_km * x[v, i, j] for (v, i, j) in x)
              + pulp.lpSum(ev.recharge_cost_per_kwh * r[k] for k in r)
              + pulp.lpSum((dr.fixed_cost + 2 * dr.cost_per_km * self.dE0[i]) * y[d, i] for (d, i) in y))
        f3 = T
        for v in V:
            m += T >= tv[v, ret], f"f3_ev_{v}"
        for d in D:
            m += T >= td[d], f"f3_dr_{d}"

        # ---------------- constraints
        for v in V:
            m += pulp.lpSum(x[v, 0, j] for j in Nc) == z[v], f"c13_{v}"
            m += pulp.lpSum(x[v, i, ret] for i in Nc + Ns) == z[v], f"c14_{v}"
            for j in Nc + Ns:                                                     # (15)
                m += (pulp.lpSum(x[v, i, j] for i in in_arcs[j])
                      - pulp.lpSum(x[v, j, k] for k in out_arcs[j]) == 0), f"c15_{v}_{j}"
            for i in Nc:                                                          # (18) MTZ
                for j in Nc:
                    if i != j:
                        m += u[v, i] - u[v, j] + 1 <= self.n * (1 - x[v, i, j]), f"c18_{v}_{i}_{j}"
            m += pulp.lpSum(self.q[j] * x[v, i, j] for j in Nc for i in in_arcs[j]) <= L, f"c24_{v}"
            for (i, j) in A:
                xv = x[v, i, j]
                if i in Ns:
                    # (21) time incl. recharging, (27) battery after partial recharge
                    m += tv[v, j] >= tvar(v, i) + self.dM[i, j] / ev.speed_kmh + (1.0 / ev.recharge_kw) * r[v, i] - self.M_T * (1 - xv), f"c21_{v}_{i}_{j}"
                    m += B[v, j] - B[v, i] - r[v, i] + E[v, i, j] <= self.M_B * (1 - xv), f"c27a_{v}_{i}_{j}"
                    m += B[v, j] - B[v, i] - r[v, i] + E[v, i, j] >= -self.M_B * (1 - xv), f"c27b_{v}_{i}_{j}"
                else:
                    # (20) time propagation, (26) battery update
                    m += tv[v, j] >= tvar(v, i) + self.dM[i, j] / ev.speed_kmh - self.M_T * (1 - xv), f"c20_{v}_{i}_{j}"
                    m += B[v, j] - Bvar(v, i) + E[v, i, j] <= self.M_B * (1 - xv), f"c26a_{v}_{i}_{j}"
                    m += B[v, j] - Bvar(v, i) + E[v, i, j] >= -self.M_B * (1 - xv), f"c26b_{v}_{i}_{j}"
            for s in Ns:
                m += r[v, s] <= Bmax - B[v, s], f"c28_{v}_{s}"                    # (28)
                if self.tighten_recharge:   # valid cut: no energy gained at unvisited stations
                    m += r[v, s] <= Bmax * pulp.lpSum(x[v, s, k] for k in out_arcs[s]), f"cut_r_{v}_{s}"
            if f is not None:               # exact payload load-flow
                for (i, j) in A:
                    m += f[v, i, j] <= L * x[v, i, j], f"flow_cap_{v}_{i}_{j}"
                for j in Nc:
                    m += (pulp.lpSum(f[v, i, j] for i in in_arcs[j]) - pulp.lpSum(f[v, j, k] for k in out_arcs[j])
                          == self.q[j] * pulp.lpSum(x[v, i, j] for i in in_arcs[j])), f"flow_c_{v}_{j}"
                for s in Ns:
                    m += (pulp.lpSum(f[v, i, s] for i in in_arcs[s]) == pulp.lpSum(f[v, s, k] for k in out_arcs[s])), f"flow_s_{v}_{s}"
                for i in in_arcs[ret]:
                    f[v, i, ret].upBound = 0
        if self.symmetry_breaking:
            for a, b in zip(V, V[1:]):
                m += z[a] >= z[b], f"sym_{a}"

        for d in D:
            m += pulp.lpSum(y[d, i] for i in self.C_elig) <= 1, f"c16_{d}"       # (16)
            for i in self.C_elig:                                                 # (19)
                m += td[d] >= 2 * self.dE0[i] / dr.speed_kmh - self.M_T * (1 - y[d, i]), f"c19_{d}_{i}"
        for j in Nc:                                                              # (17)
            m += (pulp.lpSum(x[v, i, j] for v in V for i in in_arcs[j])
                  + pulp.lpSum(y[d, j] for d in D if (d, j) in y) == 1), f"c17_{j}"
        # (22)-(23): y^d_i only exists for i in C_elig (equivalent to fixing the rest to 0)

        self.m = m
        self.vars = dict(z=z, x=x, y=y, u=u, r=r, tv=tv, td=td, B=B, T=T, f=f)
        self.obj = {"f1": f1, "f2": f2, "f3": f3}

    # ----------------------------------------------------------------- solve
    def _set_objective(self, expr):
        self.m.setObjective(expr)

    def _set_eps(self, eps):
        for k in OBJ:
            name = f"eps_{k}"
            if name in self.m.constraints:
                del self.m.constraints[name]
            if eps and eps.get(k) is not None:
                self.m += self.obj[k] <= eps[k], name

    def _run(self, solver="highs", time_limit=60, gap=1e-4, msg=False):
        t0 = time.time()
        if solver == "highs":
            status = pulp.LpStatus[self.m.solve(pulp.HiGHS(msg=msg, timeLimit=time_limit, gapRel=gap))]
        elif solver == "cbc":
            status = pulp.LpStatus[self.m.solve(pulp.PULP_CBC_CMD(msg=msg, timeLimit=time_limit, gapRel=gap))]
        if solver in ("highs", "cbc"):
            # sol_status 2 = integer feasible but not proven optimal (time limit)
            if self.m.sol_status == pulp.LpSolutionIntegerFeasible:
                status = "Feasible (time limit)"
        elif solver == "lpsolve":
            status = self._solve_lpsolve(time_limit, gap)
        else:
            raise ValueError(solver)
        return status, time.time() - t0

    def _solve_lpsolve(self, time_limit, gap):
        exe = shutil.which("lp_solve")
        if exe is None:
            raise RuntimeError("lp_solve executable not found on PATH (apt install lp-solve / brew install lp_solve)")
        with tempfile.TemporaryDirectory() as tmp:
            mps = os.path.join(tmp, "model.mps")
            self.m.writeMPS(mps)
            cmd = [exe, "-fmps", mps, "-S4", "-e", "1e-7", "-ga", "1e-9", "-gr", str(gap), "-timeout", str(int(time_limit))]
            out = subprocess.run(cmd, capture_output=True, text=True).stdout
        if "infeasible" in out.lower():
            return "Infeasible"
        vals = {}
        block = out.split("Actual values of the variables:")[-1].split("Actual values of the constraints:")[0]
        for line in block.strip().splitlines():
            parts = line.split()
            if len(parts) == 2:
                try:
                    vals[parts[0]] = float(parts[1])
                except ValueError:
                    pass
        if not vals:
            return "Not Solved"
        for v in self.m.variables():
            v.varValue = vals.get(v.name, 0.0)
        return "Optimal" if "SUBOPTIMAL" not in out else "Feasible (time limit)"

    def objective_values(self):
        return {k: pulp.value(e) for k, e in self.obj.items()}

    def solve_single(self, k, eps=None, **kw):
        self._set_eps(eps)
        self._set_objective(self.obj[k])
        status, t = self._run(**kw)
        return self._result(status, t)

    def payoff_table(self, **kw):
        """Min each objective separately; return ideal, nadir and the table."""
        table = {}
        for k in OBJ:
            res = self.solve_single(k, **kw)
            table[k] = res["objectives"]
        ideal = {k: table[k][k] for k in OBJ}
        nadir = {k: max(table[j][k] for j in OBJ) for k in OBJ}
        self._set_eps(None)
        return ideal, nadir, table

    def solve_weighted(self, weights, ideal=None, nadir=None, **kw):
        """min sum_k w_k (f_k - ideal_k)/(nadir_k - ideal_k)  (plain weighted sum if no ideal/nadir)."""
        self._set_eps(None)
        terms = []
        for k, w in weights.items():
            if ideal and nadir:
                rng = max(nadir[k] - ideal[k], 1e-9)
                terms.append(w / rng * self.obj[k])
            else:
                terms.append(w * self.obj[k])
        self._set_objective(pulp.lpSum(terms))
        status, t = self._run(**kw)
        return self._result(status, t)

    def epsilon_constraint(self, ideal, nadir, primary="f1", grid=4, delta=1e-3, **kw):
        """Augmented eps-constraint: min f_primary + delta*sum(normalised others)
        s.t. other objectives <= eps on a grid x grid lattice. Returns the
        non-dominated solutions found."""
        others = [k for k in OBJ if k != primary]
        rng = {k: max(nadir[k] - ideal[k], 1e-9) for k in OBJ}
        sols = []
        levels = lambda k: [ideal[k] + rng[k] * g / (grid - 1) for g in range(grid)] if grid > 1 else [nadir[k]]
        for e1 in levels(others[0]):
            for e2 in levels(others[1]):
                self._set_eps({others[0]: e1 + 1e-6, others[1]: e2 + 1e-6})
                self._set_objective(self.obj[primary] + delta * pulp.lpSum((1.0 / rng[k]) * self.obj[k] for k in others))
                status, t = self._run(**kw)
                if status.startswith(("Optimal", "Feasible")):
                    res = self._result(status, t)
                    res["eps"] = {others[0]: e1, others[1]: e2}
                    sols.append(res)
        self._set_eps(None)
        return pareto_filter(sols)

    # -------------------------------------------------------------- results
    def _result(self, status, t):
        if not status.startswith(("Optimal", "Feasible")):
            return {"status": status, "time_s": round(t, 2), "objectives": None}
        sol = self.extract_solution()
        obj = self.objective_values()
        # T (and t^v) are only lower-bounded, so when f3 is not the objective the
        # solver may leave slack in them: report the simulated makespan instead.
        chk = evaluate(self.inst, sol["ev_routes"], [d["customer"] for d in sol["drone_trips"]],
                       self.ev, self.dr, self.env)
        obj["f3"] = chk["f3"]
        for r, det in zip(sol["ev_routes"], chk["routes"]):
            r["stops"] = det["stops"]
        return {"status": status, "time_s": round(t, 2), "objectives": obj, "solution": sol,
                "exact_payload_check": {"f1": chk["f1"], "f2": chk["f2"], "f3": chk["f3"],
                                        "feasible": chk["feasible"], "violations": chk["violations"]}}

    def extract_solution(self):
        V, x, r = self.V, self.vars["x"], self.vars["r"]
        val = lambda var: var.varValue or 0.0
        routes = []
        for v in V:
            if val(self.vars["z"][v]) < 0.5:
                continue
            route, cur = [0], 0
            succ = {i: j for (vv, i, j), var in x.items() if vv == v and val(var) > 0.5}
            while cur != self.ret and cur in succ and len(route) <= len(self.A):
                cur = succ[cur]
                route.append(cur)
            rech = {s: val(r[v, s]) for s in route if s in self.Ns and val(r[v, s]) > 1e-6}
            routes.append({"vehicle": v, "route": route, "recharge": rech,
                           "recharge_pct": {s: round(100 * g / self.ev.battery_kwh, 1) for s, g in rech.items()},
                           "load_kg": round(sum(self.q[i] for i in route if i in self.q), 1)})
        drones = [{"drone": d, "customer": i, "return_h": round(2 * self.dE0[i] / self.dr.speed_kmh, 3),
                   "energy_kwh": round(self.ed_out[i] + self.ed_back[i], 4)}
                  for (d, i), var in self.vars["y"].items() if val(var) > 0.5]
        return {"ev_routes": routes, "drone_trips": drones}

    def size(self):
        return {"variables": len(self.m.variables()), "constraints": len(self.m.constraints),
                "binaries": sum(1 for v in self.m.variables() if v.cat == "Integer"),
                "drone_eligible": len(self.C_elig)}


def dominates(a, b):
    return all(a[k] <= b[k] + 1e-9 for k in OBJ) and any(a[k] < b[k] - 1e-9 for k in OBJ)


def pareto_filter(sols):
    uniq = []
    for s in sols:
        o = s["objectives"]
        if any(all(abs(o[k] - u["objectives"][k]) < 1e-6 for k in OBJ) for u in uniq):
            continue
        uniq.append(s)
    return [s for s in uniq if not any(dominates(t["objectives"], s["objectives"]) for t in uniq if t is not s)]
