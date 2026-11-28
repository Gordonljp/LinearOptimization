from gurobipy import GRB
import gurobipy as gp
import math
import numpy as np
from data.EV_data import Truckdata, DroneData, EnvParams

# EV 0 - kWh, EV 1 - kg
input = {"EV": [10, 2000], "DRONE": 1} # Input values for the model

objective = {"f1": 0, "f2": 0, "f3": 0}

def ev_energy_coeffs(pos_i, pos_j, ev, env, sc, tr):
    """
    Energy coefficients of the EV on arc i -> j (Zhang et al., Eq. 1-6).
    Returns a_ij [kWh] (empty vehicle) and b_ij [kWh/kg] (extra energy per kg payload),
    so that e_ij = a_ij + b_ij * payload.
    pos_i, pos_j : (x, y) in km;  tr : traffic indicator of the arc (0-1).
    """
    dx = (pos_j[0] - pos_i[0]) * 1000.0                  # horizontal segment [m]
    dy = (pos_j[1] - pos_i[1]) * 1000.0                  # vertical segment [m]
    d_M = abs(dx) + abs(dy)                              # Manhattan distance [m]

    # Relative air speed per segment (Eq. 3)
    s = ev.speed_kmh / 3.6                               # km/h -> m/s
    w = sc.wind_speed                                    # [m/s]
    theta_w = math.radians(sc.wind_direction)            # degrees -> rad
    theta_x = 0.0 if dx >= 0 else math.pi                # driving right / left
    theta_y = math.pi / 2 if dy >= 0 else 3 * math.pi / 2  # driving up / down
    s_rel2_x = s**2 + w**2 - 2 * s * w * math.cos(theta_x - theta_w)   # already squared
    s_rel2_y = s**2 + w**2 - 2 * s * w * math.cos(theta_y - theta_w)

    # Environment effects
    rho = env.rho0 * (1 - env.humidity_coef * (sc.humidity - env.H0))           # Eq. 2
    eta = ev.eta * math.exp(-ev.alpha * (sc.temperature - ev.T_opt) ** 2)       # Eq. 4
    lam = 1 + tr                                                                # Eq. 5

    # Forces times distance [J]
    rolling_per_kg = ev.sigma * env.g * d_M                                     # Eq. 1, per kg
    drag = 0.5 * rho * ev.xi * ev.area * (s_rel2_x * abs(dx) + s_rel2_y * abs(dy))  # Eq. 3

    # Eq. 6, split into empty-vehicle part and payload part, converted J -> kWh
    a_ij = lam / eta * (rolling_per_kg * ev.curb_mass_kg + drag) / 3.6e6
    b_ij = lam / eta * rolling_per_kg / 3.6e6
    return a_ij, b_ij


def drone_energy(pos_i, pos_j, load, dr, env, sc, elev_i, elev_j):
    d_km = math.dist(pos_i, pos_j)                                   # Euclidean distance [km]
    theta = math.atan2(pos_j[1] - pos_i[1], pos_j[0] - pos_i[0])     # travel angle [rad]
    s = dr.average_drone_speed / 3.6                                 # km/h -> m/s
    w = sc.wind_speed                                                # [m/s]
    s_rel2 = s**2 + w**2 - 2*s*w*math.cos(theta - math.radians(sc.wind_direction))  # already squared
    rho = env.rho0 * (1 - env.humidity_coef * (sc.humidity - env.H0))           # Eq. 2
    eta = dr.eta * math.exp(-dr.alpha * (sc.temperature - dr.T_opt)**2)         # Eq. 8
    drag_kwh  = 0.5 * rho * dr.xi * dr.drone_area * s_rel2 / eta * d_km * 1000 / 3.6e6
    mass_kwh  = dr.additional_energy_coefficient * (dr.drone_curb_mass + load) * d_km
    climb_kwh = dr.beta * (dr.drone_curb_mass + load) * max(0.0, elev_j - elev_i)
    return drag_kwh + mass_kwh + climb_kwh

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
        # Underscore methods are used to indicate that these methods are intended for internal use within the class
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
        # Placeholder Energy consumption calculation for EVs, to be replaced with actual logic based on the model's requirements
        #----------------------------------------------------
        f = None
        if f is not None:
            fv = self.model.addMVar((len(V), len(Ns)), vtype=GRB.CONTINUOUS, name="fv")
        e = {(i, j): self.ea[i, j] + self.eb[i, j] * self.l_nom for i, j in A}
        E = {(v, i, j): e[i, j] * xv[v, i, j] for v in V for i, j in A}

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
        f3 = T
        for v in V:
            self.model.addConstr(T >= tv[v, N0[1]], name=f"f3_{v}") # N0[1] is the return depot node, should use dummy if no return depot
            self.model.addConstr(t[v, N0[0]] == 0, name=f"f3_start_{v}") # N0[0] is the start depot node
        for d in D:
            self.model.addConstr(T >= td[d].item(), name=f"f3_{d}")

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

        # Constraint (16): each drone can only serve one customer at a time
        for d in D:
            self.model.addConstr(gp.quicksum(yd[d, i] for i in Nd) <= 1, name=f"c16_{d}")

        # Constraint (17): each customer is served by either an EV or a drone
        for i in Nc:
            self.model.addConstr(gp.quicksum(xv[v, j, i] for v in V for j in in_arcs[i])
                                 + gp.quicksum(yd[d, i] for d in D) == 1, name=f"c17_{i}")
            
        # Constraint (18): eliminate subtours for EVs using the Miller-Tucker-Zemlin (MTZ) formulation
        for v in V:
            for i in Nc:
                for j in Nc:
                    if i != j:
                        self.model.addConstr(uv[v, i] - uv[v, j] + 1 <= self.n * (1 - xv[v, i, j]), name=f"c18_{v}_{i}_{j}")

        # Constraint (19): drone completion time based on round trip time to the customer and back to the depot
        for d in D:
            for i in Nd:
                self.model.addConstr(td[d].item() >= 2 * self.dE0[i] / dr.speed_kmh
                                     - self.M_T * (1 - yd[d, i]), name=f"c19_{d}_{i}")
                
        # Constraints (20)-(21): EV time propagation, including recharging time at stations
        for v in V:
            for i, j in A:
                travel_time = self.dM[i, j] / ev.speed_kmh
                if i in Ns:
                    self.model.addConstr(t[v, j] >= t[v, i] + travel_time + rv[v, i] / ev.recharge_kw
                                         - self.M_T * (1 - xv[v, i, j]), name=f"c21_{v}_{i}_{j}")
                else:
                    self.model.addConstr(t[v, j] >= t[v, i] + travel_time
                                         - self.M_T * (1 - xv[v, i, j]), name=f"c20_{v}_{i}_{j}")

        # Constraints (22)-(23): only drone-eligible customers may be assigned to a drone
        # The eligible set is calculated in _prepare()
        for d in D:
            for i in Nd:
                if i not in self.C_elig:
                    self.model.addConstr(yd[d, i] == 0, name=f"c23_{d}_{i}")

        # Constraint (24): EV payload capacity
        for v in V:
            self.model.addConstr(gp.quicksum(self.q[j] * xv[v, i, j] for j in Nc for i in in_arcs[j])
                                 <= L, name=f"c24_{v}")

        # Constraint (25): each EV departs from the depot with a fully charged battery
        for v in V:
            self.model.addConstr(b[v, 0] == Bmax, name=f"c25_{v}")

        # Constraints (26)-(27): EV battery update, including energy gained at stations
        for v in V:
            for i, j in A:
                energy = E[v, i, j] if f is not None else e[i, j]
                battery_balance = b[v, j] - b[v, i] + energy
                constraint = "c26"
                if i in Ns:
                    battery_balance -= rv[v, i]
                    constraint = "c27"

                self.model.addConstr(battery_balance <= self.M_B * (1 - xv[v, i, j]),
                                     name=f"{constraint}_upper_{v}_{i}_{j}")
                self.model.addConstr(battery_balance >= -self.M_B * (1 - xv[v, i, j]),
                                     name=f"{constraint}_lower_{v}_{i}_{j}")

        # Constraint (28): EV partial recharge without exceeding battery capacity
        for v in V:
            for i in Ns:
                self.model.addConstr(rv[v, i] <= Bmax - b[v, i], name=f"c28_{v}_{i}")
        

        # Store the decision variables in a dictionary for easy access
        self.vars = dict(z=z, x=x, r=r, u=u, y=y, tv=tv, td=td, B=B, T=T)
        self.obj = {"f1": f1, "f2": f2, "f3": f3}
        self.node_index, self.customer_index, self.station_index = ni, ci, si

        # Set the weighted sum of the objectives as the model's objective function
        weights = [self.objective.get(k, 0) for k in self.obj]
        if not any(weights):
            weights = [1] * len(self.obj)  # Default to equal weights if none are provided
        self.model.setObjective(gp.quicksum(w * self.obj[k] for w, k in zip(weights, self.obj)), GRB.MINIMIZE)
        self.model.update()  # Update the model to integrate the new objective function

    def solve(self, time_limit=None):
        # Solve the optimization model using Gurobi
        if time_limit is not None:
            self.model.setParam('TimeLimit', time_limit)  # Set a time limit for the solver
        self.model.optimize()  # Optimize the model

        # Check if the optimization was successful
        if self.model.status == GRB.OPTIMAL:
            print("Optimal solution found.")
            self.model.write("model_solution.sol")  # Save the solution to a file
        elif self.model.status == GRB.TIME_LIMIT:
            print("Time limit reached. Best solution found within the time limit.")
        else:
            print(f"Optimization ended with status {self.model.status}.")