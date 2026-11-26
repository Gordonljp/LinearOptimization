"""INSGA-II skeleton (Section 4 of the paper) — structure, not a full solver.

Implemented: chromosome container, adaptive probabilities (Eqs. 33-34),
PMX / OX crossover, swap / reverse mutation, fast non-dominated sort,
crowding distance, Algorithm 1-2 initialisation, evaluation via evaluate.py.
Left as TODO: Algorithm 3 station insertion / route split repair,
feasibility repair after variation, VNS neighbourhoods N1 and N2.
See METAHEURISTIC.md for the full outline.
"""
import math
import random
from dataclasses import dataclass, field

from evaluate import evaluate, node_tables
from params import DroneParams, EnvParams, VehicleParams


@dataclass
class Chromosome:
    ev_routes: list                     # layer 1: [[0, c.., s.., 0'], ...]
    drone_routes: list                  # layer 2: [[0, i, 0'], ...]
    charging: dict = field(default_factory=dict)   # layer 3: {(route_idx, pos): rho in [0,1]}
    objectives: tuple | None = None
    rank: int = 0
    crowding: float = 0.0


# ---------------------------------------------------------------- Eqs. 33-34
def crossover_prob(tau, tau_max, pc_max=0.95, pc_min=0.7):
    return pc_max - (pc_max - pc_min) * tau / tau_max


def mutation_prob(tau, tau_max, pm_max=0.2, pm_min=0.03):
    return pm_max - (pm_max - pm_min) * (tau / tau_max) ** 2


# ------------------------------------------------------ operators (Figs. 4-5)
def pmx(p1, p2, rng):
    n = len(p1); a, b = sorted(rng.sample(range(n), 2))
    def child(x, y):
        c = x[:]; c[a:b + 1] = y[a:b + 1]
        mapping = {y[k]: x[k] for k in range(a, b + 1)}
        for k in list(range(a)) + list(range(b + 1, n)):
            g = c[k]
            while g in c[a:b + 1]:
                g = mapping[g]
            c[k] = g
        return c
    return child(p1, p2), child(p2, p1)


def ox(p1, p2, rng):
    n = len(p1); a, b = sorted(rng.sample(range(n), 2))
    def child(x, y):
        seg = x[a:b + 1]; rest = [g for g in y if g not in seg]
        return rest[:a] + seg + rest[a:]
    return child(p1, p2), child(p2, p1)


def swap_mut(s, rng):
    s = s[:]; i, j = rng.sample(range(len(s)), 2); s[i], s[j] = s[j], s[i]; return s


def reverse_mut(s, rng):
    s = s[:]; i, j = sorted(rng.sample(range(len(s)), 2)); s[i:j + 1] = s[i:j + 1][::-1]; return s


# --------------------------------------------------------- NSGA-II machinery
def dominates(a, b):
    return all(x <= y for x, y in zip(a, b)) and any(x < y for x, y in zip(a, b))


def fast_non_dominated_sort(pop):
    S = {i: [] for i in range(len(pop))}; n = [0] * len(pop); fronts = [[]]
    for i, p in enumerate(pop):
        for j, q in enumerate(pop):
            if dominates(p.objectives, q.objectives): S[i].append(j)
            elif dominates(q.objectives, p.objectives): n[i] += 1
        if n[i] == 0:
            p.rank = 0; fronts[0].append(i)
    k = 0
    while fronts[k]:
        nxt = []
        for i in fronts[k]:
            for j in S[i]:
                n[j] -= 1
                if n[j] == 0:
                    pop[j].rank = k + 1; nxt.append(j)
        k += 1; fronts.append(nxt)
    return [[pop[i] for i in f] for f in fronts[:-1]]


def crowding_distance(front):
    for p in front: p.crowding = 0.0
    for m in range(len(front[0].objectives)):
        front.sort(key=lambda p: p.objectives[m])
        front[0].crowding = front[-1].crowding = math.inf
        span = (front[-1].objectives[m] - front[0].objectives[m]) or 1.0
        for k in range(1, len(front) - 1):
            front[k].crowding += (front[k + 1].objectives[m] - front[k - 1].objectives[m]) / span


# ------------------------------------------------------------------ INSGA-II
class INSGA2:
    def __init__(self, inst, pop_size=100, iters=500, k_rho=5, seed=None,
                 ev=None, dr=None, env=None):
        self.inst, self.N, self.T, self.k_rho = inst, pop_size, iters, k_rho
        self.ev, self.dr, self.env = ev or VehicleParams(), dr or DroneParams(), env or EnvParams()
        self.rng = random.Random(seed)
        self.tab = node_tables(inst)
        self.C_elig = self._drone_eligible()          # Constraint (22)

    def _drone_eligible(self):
        from energy import drone_energy
        P, q, sc = self.tab["pos"], self.tab["q"], self.inst["scenario"]
        return [i for i in self.tab["Nc"] if q[i] <= self.dr.load_cap_kg and
                drone_energy(P[0], P[i], q[i], self.dr, self.env, sc) +
                drone_energy(P[i], P[0], 0, self.dr, self.env, sc) <= self.dr.battery_kwh]

    # Algorithm 1 + 2 ----------------------------------------------------
    def heuristic_individual(self):
        nd = self.inst["fleet"]["n_drones"]
        Cd = self.rng.sample(self.C_elig, min(nd, len(self.C_elig)))          # Eq. 29
        Cv = [i for i in self.tab["Nc"] if i not in Cd]                        # Eq. 30
        routes = self.nearest_neighbour_routes(Cv)                             # Alg. 2
        if routes is None:
            return None
        routes, charging = self.insert_recharging(routes)                      # Alg. 3 (TODO)
        ret = self.tab["ret"]
        return Chromosome(routes, [[0, i, ret] for i in Cd], charging)         # Eqs. 31-32

    def nearest_neighbour_routes(self, Cv):
        P, q, ret, L = self.tab["pos"], self.tab["q"], self.tab["ret"], self.ev.load_cap_kg
        dM = lambda a, b: abs(P[a][0] - P[b][0]) + abs(P[a][1] - P[b][1])
        unvisited, routes = set(Cv), []
        while unvisited and len(routes) < self.inst["fleet"]["n_vehicles"]:
            r, cur, load = [0], 0, 0.0
            while True:
                F = [j for j in unvisited if load + q[j] <= L]
                if not F: break
                j = min(F, key=lambda j: dM(cur, j)); r.append(j); unvisited.remove(j); load += q[j]; cur = j
            routes.append(r + [ret])
        return None if unvisited else routes

    def insert_recharging(self, routes):
        """TODO Algorithm 3: simulate battery, at first deficit walk back, insert the
        accessible station with min detour, sample rho ~ U[rho_min, rho_max] (Eqs. 36-37),
        split route if no station works."""
        return routes, {}

    # evaluation ----------------------------------------------------------
    def evaluate(self, ch):
        recharge_by_route = [{} for _ in ch.ev_routes]
        for (ri, pos), rho in ch.charging.items():
            recharge_by_route[ri][ch.ev_routes[ri][pos]] = rho * self.ev.battery_kwh
        res = evaluate(self.inst, [{"route": r, "recharge": g} for r, g in zip(ch.ev_routes, recharge_by_route)],
                       [r[1] for r in ch.drone_routes], self.ev, self.dr, self.env, self.tab)
        penalty = 0 if res["feasible"] else 1e6
        ch.objectives = (res["f1"] + penalty, res["f2"] + penalty, res["f3"] + penalty)
        return ch

    # variation ----------------------------------------------------------
    def vary(self, p1, p2, tau):
        """Crossover/mutation act on the giant customer sequence of layer 1."""
        def flat(ch): return [g for r in ch.ev_routes for g in r[1:-1] if g in self.tab["q"]]
        s1, s2 = flat(p1), flat(p2)
        if len(s1) >= 2 and sorted(s1) == sorted(s2) and self.rng.random() < crossover_prob(tau, self.T):
            s1, s2 = (pmx if self.rng.random() < 0.5 else ox)(s1, s2, self.rng)
        kids = []
        for s, parent in ((s1, p1), (s2, p2)):
            if len(s) >= 2 and self.rng.random() < mutation_prob(tau, self.T):
                s = (swap_mut if self.rng.random() < 0.5 else reverse_mut)(s, self.rng)
            kids.append(self.repair(s, parent))
        return [k for k in kids if k is not None]

    def repair(self, seq, parent):
        """TODO: split seq into load-feasible routes, re-insert stations (Alg. 3), and
        update layer 3. Current placeholder: capacity split only, no stations."""
        routes = self.nearest_split(seq)
        if routes is None:
            return None
        return Chromosome(routes, [r[:] for r in parent.drone_routes], {})

    def nearest_split(self, seq):
        q, L, ret, routes, cur = self.tab["q"], self.ev.load_cap_kg, self.tab["ret"], [], [0]
        load = 0.0
        for c in seq:
            if load + q[c] > L:
                routes.append(cur + [ret]); cur, load = [0], 0.0
            cur.append(c); load += q[c]
        routes.append(cur + [ret])
        return routes if len(routes) <= self.inst["fleet"]["n_vehicles"] else None

    def vns(self, ch):
        """TODO Algorithm 4: N1 customer relocation between EV routes (Eq. 35),
        N2 charging-percentage adjustment with K_rho candidates (Eqs. 38-39),
        first-improvement by non-domination rank then crowding distance."""
        return ch

    # main loop (Fig. 2) --------------------------------------------------
    def run(self):
        P = []
        while len(P) < self.N:
            ind = self.heuristic_individual()
            if ind is not None:
                P.append(self.evaluate(ind))
        for tau in range(1, self.T + 1):
            fronts = fast_non_dominated_sort(P)
            P1 = []
            while len(P1) < self.N:
                a, b = self.rng.sample(P, 2)
                P1 += [self.evaluate(k) for k in self.vary(a, b, tau)]
            P2 = [self.evaluate(self.vns(ch)) for ch in fronts[0]]
            R, P = P + P1 + P2, []
            for f in fast_non_dominated_sort(R):
                crowding_distance(f)
                if len(P) + len(f) <= self.N:
                    P += f
                else:
                    P += sorted(f, key=lambda p: -p.crowding)[:self.N - len(P)]
                    break
        front, seen = [], set()
        for ch in fast_non_dominated_sort(P)[0]:     # drop duplicate objective vectors
            key = tuple(round(o, 6) for o in ch.objectives)
            if key not in seen:
                seen.add(key); front.append(ch)
        return front


if __name__ == "__main__":
    from instance_generator import GeneratorConfig, generate_instance
    inst = generate_instance(GeneratorConfig(n_customers=12, n_stations=3, seed=2))
    front = INSGA2(inst, pop_size=30, iters=30, seed=0).run()
    for ch in sorted(front, key=lambda c: c.objectives):
        print([round(o, 3) for o in ch.objectives], ch.ev_routes, ch.drone_routes)
