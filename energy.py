"""Multi-factor energy-consumption inputs, Eqs. (1)-(9) of the paper.

All energies are returned in kWh. Coordinates are in km, speeds in km/h
(converted to m/s for the drag terms), wind speed in m/s, temperature in
degC, humidity as a fraction in [0, 1].

Forces are in N; N * km = kJ, so dividing by 3600 gives kWh.
"""
import math

KJ_PER_KWH = 3600.0


def air_density(env, humidity):
    """Eq. (2): rho_(H) = rho0 [1 - psi (H - H0)]."""
    return env.rho0 * (1.0 - env.humidity_coef * (humidity - env.humidity_ref))


def temp_efficiency(eta_ref, alpha, temp, temp_opt):
    """Eqs. (4) and (8): eta(T) = eta * exp(-alpha (T - T_opt)^2)."""
    return eta_ref * math.exp(-alpha * (temp - temp_opt) ** 2)


def relative_speed(s_vehicle_ms, s_wind_ms, theta_travel, theta_wind):
    """Law-of-cosines relative air speed used in Eqs. (3) and (7)."""
    val = s_vehicle_ms ** 2 + s_wind_ms ** 2 - 2 * s_vehicle_ms * s_wind_ms * math.cos(theta_travel - theta_wind)
    return math.sqrt(max(val, 0.0))


def ev_energy_coeffs(pi, pj, ev, env, scen, traffic):
    """EV energy on arc i->j as e(l) = a + b*l  (l = payload in kg).

    Eq. (6) is linear in payload because only rolling resistance (Eq. 1)
    depends on mass. Returning (a, b) lets the MILP either use a fixed
    nominal payload (paper: e_ij is an input parameter) or an exact
    load-flow linearisation.
    """
    dx, dy = pj[0] - pi[0], pj[1] - pi[1]
    d_man = abs(dx) + abs(dy)
    if d_man == 0:
        return 0.0, 0.0
    lam = 1.0 + traffic                                   # Eq. (5)
    eta = temp_efficiency(ev.eta_ref, ev.temp_sens, scen["temperature_c"], ev.temp_opt_c)
    rho = air_density(env, scen["humidity"])
    sv = ev.speed_kmh / 3.6
    sw, thw = scen["wind_speed_ms"], scen["wind_dir_rad"]
    th_x = 0.0 if dx >= 0 else math.pi                    # horizontal segment
    th_y = math.pi / 2 if dy >= 0 else 3 * math.pi / 2    # vertical segment
    f_air_x = 0.5 * rho * ev.drag_coef * ev.frontal_area_m2 * relative_speed(sv, sw, th_x, thw) ** 2  # Eq. (3)
    f_air_y = 0.5 * rho * ev.drag_coef * ev.frontal_area_m2 * relative_speed(sv, sw, th_y, thw) ** 2
    k = lam / eta / KJ_PER_KWH
    a = k * (ev.rolling_coef * ev.curb_mass_kg * env.g * d_man + f_air_x * abs(dx) + f_air_y * abs(dy))
    b = k * ev.rolling_coef * env.g * d_man               # per kg of payload (Eq. 1)
    return a, b


def drone_energy(pi, pj, payload, dr, env, scen, hi=0.0, hj=0.0):
    """Eq. (9): drone energy on the Euclidean leg i->j carrying `payload` kg."""
    dx, dy = pj[0] - pi[0], pj[1] - pi[1]
    d_euc = math.hypot(dx, dy)
    if d_euc == 0:
        return 0.0
    rho = air_density(env, scen["humidity"])
    theta = math.atan2(dy, dx)
    s_rel = relative_speed(dr.speed_kmh / 3.6, scen["wind_speed_ms"], theta, scen["wind_dir_rad"])
    f_air = 0.5 * rho * dr.drag_coef * dr.frontal_area_m2 * s_rel ** 2          # Eq. (7)
    eta = temp_efficiency(dr.eta_ref, dr.temp_sens, scen["temperature_c"], dr.temp_opt_c)  # Eq. (8)
    mass = dr.curb_mass_kg + payload
    dh = max(0.0, hj - hi)
    return d_euc * (f_air / eta / KJ_PER_KWH + dr.mass_energy_coef * mass) + dr.altitude_coef * mass * dh
