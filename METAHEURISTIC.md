# INSGA-II for the MO-EVDRP — outline

Source: Zhang, Wu, Chen & Chen (2027), *Multi-objective electric vehicle-drone routing problem: Incorporating multi-factor energy-consumption inputs and a partial recharging strategy*, Expert Systems With Applications 332, 133716, Section 4.

INSGA-II is NSGA-II (Deb et al., 2002) with four changes made for this problem:

1. A three-layer chromosome.
2. Heuristic initialisation.
3. Adaptive crossover and mutation probabilities.
4. A VNS local search applied to the first Pareto front.

It minimises the three MILP objectives at the same time: f1 (energy, Eq. 10), f2 (cost, Eq. 11) and f3 (makespan, Eq. 12).

## 1. Main loop (Fig. 2)

```
P  <- HeuristicInit(N_pop)                      # Alg. 1-3
evaluate(P)
for tau = 1 .. tau_max:
    fast_non_dominated_sort(P)
    P1 <- AdaptiveCrossoverMutation(P, tau)      # Eqs. 33-34, PMX/OX + swap/reverse
    repair + evaluate(P1)
    P2 <- VNS(first_front(P))                    # Alg. 4, neighbourhoods N1, N2
    R  <- P ∪ P1 ∪ P2
    P  <- best N_pop of R by (rank, crowding distance)
return first_front(P)
```

## 2. Three-layer coding scheme (Section 4.2.1, Fig. 3)

| Layer | Content | Example |
|---|---|---|
| 1 – EV routes | node sequence per EV: 0 → customers / stations → 0' | `[0,4,6,8,10] [0,5,9,7,10]` |
| 2 – Drone routes | one customer per drone flight | `[0,1,10] [0,2,10] [0,3,10]` |
| 3 – Charging | `[station, gained-energy %]` for **each station visit in layer 1** | `[8,20%] [9,65%]` |

- The gained energy in absolute terms is ρ(s)·B^v_max.
- Layer 3 is a *dependent* layer. When a station visit is removed from layer 1, its gene is deleted. When a visit is inserted by repair, a new gene is sampled inside the feasible interval [ρ_min, ρ_max] (Eqs. 36–37).

## 3. Heuristic initialisation (Section 4.2.2)

**Algorithm 1 – overall**

1. Assign customers to drones or EVs. Build C^d ⊆ C^d_elig with |C^d| ≤ |D| (Eq. 29), and C^v = N_c \ C^d (Eq. 30). If |C^d_elig| > |D|, pick |D| of them at random.
2. Construct the EV routes (Alg. 2).
3. Optimise those routes for battery feasibility (Alg. 3).
4. Build the drone routes: r_i^d = [0, i, 0'] for each i in C^d (Eqs. 31–32).

**Algorithm 2 – nearest-neighbour construction**

- Start at the depot. Repeatedly append the nearest unvisited customer (by Manhattan distance d^M) whose demand still fits under L^v_max. Close the route at 0' when no customer fits.
- Discard and regenerate the individual if more than |V| routes are needed.

**Algorithm 3 – insert recharging stations**

1. Simulate the battery level U_k along the route with no stations.
2. At the first deficit node j, walk backward over the predecessors p.
3. A station s is accessible from p if e_ps ≤ U_p and the cumulative energy e_route(s, j) ≤ B^v_max. Insert the accessible station with the smallest detour d^M_ps.
4. Sample the gained energy ρ(s) ~ U[ρ_min, ρ_max]:
   - ρ_min = max(0, (e_route(s, j) − U_s)/B_max)
   - ρ_max = (B_max − U_s)/B_max
5. Re-simulate the route and repeat until it has no deficit.
6. If no station can be inserted, split the route at the deficit segment. Discard the individual if the split needs more than |V| EVs.

## 4. Adaptive genetic operators (Section 4.2.3)

**Crossover probability (linear decay, Eq. 33):**
P_c(τ) = P_c^max − (P_c^max − P_c^min)·τ/τ_max, with 0.9 ≤ P_c^max ≤ 1.0 and 0.6 ≤ P_c^min ≤ 0.8.

**Mutation probability (quadratic decay, Eq. 34):**
P_m(τ) = P_m^max − (P_m^max − P_m^min)·(τ/τ_max)², with 0.1 ≤ P_m^max ≤ 0.3 and 0.01 ≤ P_m^min ≤ 0.05.

**Where the operators apply:**

- The operators act only on the **customer sequence of layer 1**:
  - crossover: PMX or OX, chosen at random (Fig. 4);
  - mutation: swap or reverse (inversion), chosen at random (Fig. 5).
- Depot genes stay fixed. Layer 2 changes little because each drone serves a single customer.
- Layer 3 is not touched by these operators. After a layer-1 change, a feasibility repair updates layer 3 (load split, station re-insertion as in Alg. 3), and the VNS then tunes the charging percentages.

## 5. VNS local search (Section 4.2.4, Algorithm 4)

Solution S = (R^v, R^d, G). The search uses sequential neighbourhoods with first improvement:

```
S* <- S ; k <- 1
while k <= 2:
    find first S' in N_k(S*) that improves S*
    if found: S* <- S' ; k <- 1   else: k <- k + 1
return S*
```

**N1 – customer relocation between EV routes (Eq. 35)**

- Remove customer i from route r_a and insert it at a feasible position q in another route r_b.
- Depot nodes 0 and 0' and the stations are not moved.
- Re-simulate the battery and update layer 3.

**N2 – charging adjustment (Eqs. 36–39)**

- For each station visit s, build the candidate set Ω(s) = {ρ(s), ρ_1 … ρ_Kρ}, with ρ_h ~ U[ρ_min(s), ρ_max(s)] and K_ρ = 5.
- Change only that one layer-3 gene. Keep each candidate if it is feasible.

**Improvement test**

- S' improves S* if it gets a better non-domination rank when the two are sorted together.
- If S' and S* tie on rank, the one with the larger crowding distance wins.

## 6. Evaluation

- Decode the three layers, then simulate each EV route with the **actual remaining payload** (Eq. 6) and the partial recharges.
- Compute f1, f2 and f3. Solutions that are infeasible are repaired, or discarded if repair fails.
- `evaluate.py` in this package implements this decoder and evaluator. It returns the same numbers as the MILP when the MILP is run with `payload_mode="exact"`.

## 7. Experimental settings (Section 5, Tables 1–2)

**Algorithm settings**

| Setting | Value |
|---|---|
| Population size | 100 |
| Iterations | 500 |
| Crossover / mutation | adaptive |
| Charging candidates K_ρ | 5 |
| Independent runs per instance | 30 |

**Benchmark instances**

- Instances are adapted from Schneider et al. (2014).
- 30, 60 or 90 customers.
- 8–20 recharging stations.
- I_traf ∈ {0.25, 0.50, 0.75}.
- 50 × 50 km area with a central depot.
- Demand of 5–1000 kg.
- Elevation term inactive.

**Performance metrics**

- Hypervolume (HV, higher is better).
- Inverted generational distance (IGD, lower is better).
- Runtime.
- Two-sided Wilcoxon rank-sum test at the 5 % level.

**Baselines:** NSGA-II, SPEA2, R-MOEA/D and AMA.
