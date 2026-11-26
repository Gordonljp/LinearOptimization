# MO-EVDRP — MILP + INSGA-II outline

Python implementation of the multi-objective electric vehicle–drone routing problem of
Zhang et al. (2027), *Expert Systems With Applications* 332, 133716.

| File | What it is |
|---|---|
| `params.py` | Table 1 parameters (+ assumed aerodynamic/environment coefficients) |
| `energy.py` | Multi-factor energy inputs, Eqs. (1)–(9) |
| `instance_generator.py` | Random supply/demand/scenario instances (Section 5.1 style), JSON I/O, CLI |
| `milp_model.py` | MILP: objectives (10)–(12), constraints (13)–(28); HiGHS / CBC / lp_solve backends; payoff table, weighted sum, augmented ε-constraint |
| `evaluate.py` | Exact decoder/evaluator (true payload along route) – used for checking and by the GA |
| `insga2_skeleton.py` | INSGA-II structure: operators, adaptive probabilities, NSGA-II sorting, Alg. 1–2 init; Alg. 3 and VNS marked TODO |
| `METAHEURISTIC.md` | Outline of the INSGA-II algorithm from Section 4 |
| `run_example.py` | End-to-end demo |

```bash
pip install -r requirements.txt
python instance_generator.py --customers 30 --stations 8 --traffic 0.75 --paper --seed 1
python run_example.py --customers 6 --stations 2 --solver highs --time 30
python run_example.py --solver lpsolve            # needs lp_solve on PATH
python run_example.py --payload exact             # exact payload-dependent EV energy
```

## Modelling notes
* Node indices: `0` depot, `1..n` customers, `n+1..n+e` stations, `n+e+1` dummy return 0'.
  Arc set A excludes depot→station and depot→0' (as in the paper).
* **Payload in Eq. (6).** The paper treats e^v_ij as precomputed input. `payload_mode="half"`
  (default) uses a nominal payload L/2; `"exact"` adds load-flow variables f_ij so that
  e_ij = a_ij·x_ij + b_ij·f_ij, which is exact and still linear (only rolling resistance depends on mass).
* f3 = max return time is linearised with `T ≥ t_0'` for every EV and drone.
* Valid cuts added (switchable): r_s ≤ B_max·(visits of s) and z_v ≥ z_{v+1}.
* ρ0, ψ, H0, σ, ξ, A, η, α, T_opt and β̃ are **not given in the paper**; the defaults in `params.py` are assumptions.
* The exact MILP is practical only for small instances (≈ ≤ 8–10 customers, 2–3 vehicles);
  the 30/60/90-customer instances are what the metaheuristic is for.
