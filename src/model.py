from gurobipy import GRB
import gurobipy as gp
import math
import numpy as np

# EV 0 - kWh, EV 1 - kg
input = {"EV": [10, 2000], "DRONE": 1} # Input values for the model

objective = {"f1": 0, "f2": 0, "f3": 0}

def ev_energy_coeffs(pos_i, pos_j, ev, env, sc, tr):
    # Calculate the energy coefficients for electric vehicles based on the positions of nodes i and j
    # This is a placeholder function and should be implemented with the actual logic for calculating energy coefficients
    return 1.0, 1.0


def drone_energy(pos_i, pos_j, load, dr, env, sc, elev_i, elev_j):
    # Calculate the energy consumption for drones based on the positions of nodes i and j, load, and other parameters
    # This is a placeholder function and should be implemented with the actual logic for calculating drone energy consumption
    return 1.0

class Model:
    def __init__(self, input, objective, instance, env):
        self.EV = input["EV"]
        self.DRONE = input["DRONE"]

        self.inst = instance
        self.env = env
        self.objective = objective
        self.payload_mode = instance["scenario"].get("payload_mode", "half")

        self._prepare()
        self._build()

    def _prepare(self):
        I, ev, dr = self.inst, self.EV, self.DRONE
        cust, stat = I["customers"], I["stations"]
        dep = I["depot"]

        # Prepare the model parameters based on the input instance
        self.n, self.e = len(cust), len(stat)
        self.Nc = [c["id"] for c in cust]
        self.Ns = [s["id"] for s in stat]
        self.ret = self.n + self.e + 1                      # dummy return node 0'
        self.V = list(range(I["fleet"]["n_vehicles"]))
        self.D = list(range(I["fleet"]["n_drones"]))
        
        # Prepare the model parameters based on the input instance
        self.pos = {0: (dep["x"], dep["y"]), self.ret: (dep["x"], dep["y"])}
        self.elev = {0: dep.get("elev", 0.0), self.ret: dep.get("elev", 0.0)}
        self.traf = {0: I["scenario"]["traffic_level"], self.ret: I["scenario"]["traffic_level"]}
        self.q = {}

        # Populate the position, elevation, traffic, and demand for each customer and station
        for c in cust:
            self.pos[c["id"]] = (c["x"], c["y"])
            self.elev[c["id"]] = c.get("elev", 0.0)
            self.traf[c["id"]] = c.get("traffic", I["scenario"]["traffic_level"])
            self.q[c["id"]] = c["demand_kg"]
        for s in stat:
            self.pos[s["id"]] = (s["x"], s["y"])
            self.elev[s["id"]] = s.get("elev", 0.0)
            self.traf[s["id"]] = s.get("traffic", I["scenario"]["traffic_level"])

        # arc set A (Section 3.2)
        A = [(0, j) for j in self.Nc]
        for i in self.Nc + self.Ns:
            for j in self.Nc + self.Ns + [self.ret]:
                if i != j:
                    A.append((i, j))
        self.A = A

        # Compute the distance and energy coefficients for each arc in A
        P, sc = self.pos, I["scenario"]
        self.dM = {(i, j): abs(P[i][0] - P[j][0]) + abs(P[i][1] - P[j][1]) for i, j in A}
        self.ea, self.eb = {}, {}
        for i, j in A:
            tr = 0.5 * (self.traf[i] + self.traf[j])          # arc traffic indicator
            # Energy coefficient for EV eq. 6, vehicle coefficient, payload coefficient
            self.ea[i, j], self.eb[i, j] = ev_energy_coeffs(P[i], P[j], ev, self.env, sc, tr)

        # drone energy consumptions (Eq. 9): loaded out, empty back
        self.dE0 = {i: math.dist(P[0], P[i]) for i in self.Nc}
        self.ed_out = {i: drone_energy(P[0], P[i], self.q[i], dr, self.env, sc, self.elev[0], self.elev[i]) for i in self.Nc}
        self.ed_back = {i: drone_energy(P[i], P[self.ret], 0.0, dr, self.env, sc, self.elev[i], self.elev[self.ret]) for i in self.Nc}
        
        # Constraint (22): drone-eligible customers
        self.C_elig = [i for i in self.Nc
                       if self.ed_out[i] + self.ed_back[i] <= dr.battery_kwh and self.q[i] <= dr.load_cap_kg]

        # Constraint (24): EV payload capacity
        L = ev.load_cap_kg

        # Determine the nominal load based on the payload mode, when payload is modelled use exact payload, otherwise use half, full or empty load
        pm = self.payload_mode
        self.l_nom = {
            "half": L / 2, 
            "full": L, 
            "empty": 0.0
            }.get(pm, pm if pm != "exact" else None)
        
        # Compute the energy required for EVs based on the energy coefficients and nominal load
        e_max = max(self.ea[a] + self.eb[a] * L for a in A)

        # Use the big-M method, compute the big-M values for battery and time constraints
        self.M_B = ev.battery_kwh + e_max
        self.M_T = ((self.n + self.e + 1) * max(self.dM.values()) / ev.speed_kmh
                    + self.e * ev.battery_kwh / ev.recharge_kw
                    + max(2 * d for d in self.dE0.values()) / dr.speed_kmh + 1.0)

    def _build(self):
        # Build the model using the input and objective
        # Underscore methods are used to indicate that these methods are intended for internal use within the class
        ev, dr = self.EV, self.DRONE

        # Define the sets (Section 3.2)
        A = self.A # set of feasible arcs
        Nc = self.Nc # set of customer nodes
        Ns = self.Ns # set of recharge station nodes
        N0 = [0, self.ret] # set of depot nodes
        V = self.V # set of all electric vehicles
        D = self.D # set of all drones
        N = [0] + Nc + Ns + [self.ret] # set of all customer, recharge station and depot nodes
        Nd = Nc # set of potential drone delivery nodes

        # Map node IDs to matrix indices and identify feasible arcs
        ni = {i: k for k, i in enumerate(N)}
        ci = {i: k for k, i in enumerate(Nc)}
        si = {i: k for k, i in enumerate(Ns)}
        arc_ub = np.zeros((len(N), len(N)))
        for i, j in A:
            arc_ub[ni[i], ni[j]] = 1

        in_arcs = {j: [] for j in N}
        out_arcs = {i: [] for i in N}
        for i, j in A:
            in_arcs[j].append(i)
            out_arcs[i].append(j)

        # Battery capacity and load capacity for electric vehicles
        Bmax, L = ev.battery_kwh, ev.load_cap_kg

        # Create a Gurobi model

        self.model = gp.Model()

        # Create decision variables
        # EV activation - z[v] = 1 if EV v is activated, 0 otherwise
        z = self.model.addMVar(len(V), vtype=GRB.BINARY, name="z")
        # EV routing - x[v, i, j] = 1 if EV v travels from node i to node j, 0 otherwise
        x = self.model.addMVar((len(V), len(N), len(N)), ub=arc_ub, vtype=GRB.BINARY, name="x")
        # EV energy gained - r[v, i] = energy gained by EV v at node i
        r = self.model.addMVar((len(V), len(Ns)), ub=Bmax, vtype=GRB.CONTINUOUS, name="r")
        # Position of EV - u[v, i] = position of node i in the route of EV v
        u = self.model.addMVar((len(V), len(Nc)), lb=1, ub=self.n, vtype=GRB.CONTINUOUS, name="u")
        # Drone routing - y[d, i] = 1 if drone d travels from depot to node i, 0 otherwise
        y = self.model.addMVar((len(D), len(Nd)), vtype=GRB.BINARY, name="y")

        # Create intermediate variables
        # EV arrival time - tv[v, i] = time at which EV v reaches node i
        tv = self.model.addMVar((len(V), len(N)), ub=self.M_T, vtype=GRB.CONTINUOUS, name="tv")
        # Drone completion time - td[d] = time at which drone d returns to the depot
        td = self.model.addMVar(len(D), ub=self.M_T, vtype=GRB.CONTINUOUS, name="td")
        # EV battery energy - B[v, i] = remaining battery energy of EV v at node i
        B = self.model.addMVar((len(V), len(N)), ub=Bmax, vtype=GRB.CONTINUOUS, name="B")
        # Maximum completion time - T = time at which the last EV or drone returns
        T = self.model.addVar(ub=self.M_T, vtype=GRB.CONTINUOUS, name="T")

        # To call the Mvars effectively, we need to create dictionaries that map the decision variables to their corresponding indices
        # .item() is used to extract the value of the decision variable from the Gurobi model after optimization
        # z is not indexed by i, j, so we can directly use z[v].item() to get the value of the decision variable for EV v
        xv = {(v, i, j): x[v, ni[i], ni[j]].item() for v in V for i, j in A}
        rv = {(v, i): r[v, si[i]].item() for v in V for i in Ns}
        uv = {(v, i): u[v, ci[i]].item() for v in V for i in Nc}
        yd = {(d, i): y[d, ci[i]].item() for d in D for i in Nd}
        t = {(v, i): tv[v, ni[i]].item() for v in V for i in N}
        b = {(v, i): B[v, ni[i]].item() for v in V for i in N}

        # Energy consumption for EVs
        #----------------------------------------------------
        # Placeholder
        #----------------------------------------------------
        rv = self.model.addMVar((len(V), len(Ns)), vtype=GRB.CONTINUOUS, name="rv")
        E = {}

        # Create objective functions
        # Objective (10): minimize total energy consumption
        f1 = (gp.quicksum(E.values())
              + gp.quicksum((self.ed_out[i] + self.ed_back[i]) * yd[d, i] for d in D for i in Nd))

        # Objective (11): minimize total operational cost
        f2 = (gp.quicksum(ev.fixed_cost * z[v].item() for v in V)
              + gp.quicksum(ev.cost_per_km * self.dM[i, j] * xv[v, i, j] for v in V for i, j in A)
              + gp.quicksum(ev.recharge_cost_per_kwh * rv[v, i] for v in V for i in Ns)
              + gp.quicksum((dr.fixed_cost + 2 * dr.cost_per_km * self.dE0[i]) * yd[d, i]
                            for d in D for i in Nd))

        # Objective (12): minimize total completion time
        #-----------------------------------------------------
        # Todo add the completion time objective function
        #-----------------------------------------------------
        f3 = T

        # Create constraints - Use addConstr for each constraint in the model
        # Each constraint line adds constraints named c13_0, c13_1... numbered according to the EV index and constraint number
        # Constraint (13): each activated EV departs from the depot - quicksum is same as lpsum
        for v in V:
            self.model.addConstr(gp.quicksum(xv[v, 0, j] for j in Nc) == z[v].item(), name=f"c13_{v}")

        # Constraint (14): each activated EV returns to the depot
        for v in V:
            self.model.addConstr(gp.quicksum(xv[v, i, self.ret] for i in Nc + Ns) == z[v].item(), name=f"c14_{v}")

        # Constraint (15): balance the inflow and outflow at each customer and recharge station
        for v in V:
            for j in Nc + Ns:
                self.model.addConstr(gp.quicksum(xv[v, i, j] for i in in_arcs[j])
                                     == gp.quicksum(xv[v, j, k] for k in out_arcs[j]), name=f"c15_{v}_{j}")

        pass