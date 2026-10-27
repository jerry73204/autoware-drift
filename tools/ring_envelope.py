#!/usr/bin/env python3
"""The speeds, accelerations and drift equilibria of drift_mustang on Town05's ring.

    tools/ring_envelope.py [--mu 1.12] [--sliding 0.8] [--radius 105]

Pure arithmetic from the car's spec (sim_vehicle_spec drift_mustang.yaml) and the ring's lane
(1540 m; four 90 deg bends of 102-106 m minimum lane radius, 165-170 m of arc each; straights of
196, 196, 238 and 230 m), measured on the lanelet map in Autoware's frame. docs/ring-envelope.md
explains it.

Steady drift, after Velenis, Frazzoli and Tsiotras (2010), single track, steady (R, V, beta):
  rear lateral    f_Ry = m a cos(beta) l_F / L                      (a = V^2 / R)
  rear thrust     f_Rx = m a sin(beta) + drag + front rolling       (the front tire rolls free)
  rear load       f_Rz = (m g l_F + m h a sin(beta)) / L
  rear saturated  sqrt(f_Rx^2 + f_Ry^2) = mu_slide f_Rz             (sliding: mu sliding share)
  front           f_Fy = m a cos(beta) l_R / L  <=  mu f_Fz,  f_Fz = (m g l_R - m h a sin(beta)) / L
which, without drag, gives a = mu_slide g l_F / (sqrt(sin^2 beta + (cos beta l_F / L)^2)
                                                  - mu_slide h sin(beta) / L).
"""
import argparse
import math

G = 9.81
CAR = dict(m=1450.0, L=2.886, front=0.50, h=0.48, P=330e3, T_wheel=700.0 * 6.0, r=0.320,
           cda=0.45 * 1.9, rho=1.2, crr=0.012)
STRAIGHTS = [230.0, 196.0, 238.0, 196.0]   # m, from the NE bend's exit round to it again
BEND_ARC = 168.0                            # m


def drag(v, c=CAR):
    return 0.5 * c["rho"] * c["cda"] * v * v + c["crr"] * c["m"] * G


def drift_speed(beta, R, mu, sliding, c=CAR):
    """V of the steady drift at sideslip beta on radius R (rear sliding, front free rolling),
    with drag; and the front's friction use, the rear thrust and the power it takes."""
    m, L, h = c["m"], c["L"], c["h"]
    lF, lR = L * (1 - c["front"]), L * c["front"]       # CoM to front axle, to rear axle
    mu_r = mu * sliding
    v = 25.0
    for _ in range(50):
        a = v * v / R
        fry = m * a * math.cos(beta) * lF / L
        frx = m * a * math.sin(beta) + drag(v)
        frz = (m * G * lF + m * h * a * math.sin(beta)) / L
        # scale a so the rear is just saturated
        need = math.hypot(frx, fry) / (mu_r * frz)
        v = v / need ** 0.5
    a = v * v / R
    ffz = (m * G * lR - m * h * a * math.sin(beta)) / L
    ffy = m * a * math.cos(beta) * lR / L
    frx = m * a * math.sin(beta) + drag(v)
    return v, a, ffy / ffz / mu, frx, frx * v


def accel_limit(v, mu, c=CAR):
    """Forward acceleration: the rear's traction (with load transfer) or the motor's power."""
    m, L, h = c["m"], c["L"], c["h"]
    lF = L * (1 - c["front"])
    traction = mu * G * lF / L / (1 - mu * h / L)       # a = mu (m g lF + m a h) / L / m
    power = min(c["P"] / max(v, 1.0), c["T_wheel"] / c["r"]) / m
    return min(traction, power) - drag(v) / m


def straight_peak(v_in, v_out, length, acc, dec):
    """Peak speed on a straight entered at v_in, left at v_out, at constant acc and dec."""
    v2 = (2 * length * acc * dec + dec * v_in ** 2 + acc * v_out ** 2) / (acc + dec)
    return math.sqrt(max(v2, max(v_in, v_out) ** 2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mu", type=float, default=1.12, help="peak friction (asphalt's, swb)")
    ap.add_argument("--sliding", type=float, default=0.8, help="sliding share past the peak")
    ap.add_argument("--radius", type=float, default=105.0, help="m, the bends' lane radius")
    args = ap.parse_args()
    mu, R = args.mu, args.radius

    v_grip = math.sqrt(mu * G * R)
    print(f"grip limit on R {R:.0f} m: {v_grip:.1f} m/s ({v_grip * 3.6:.0f} km/h), a_y {mu * G:.1f} m/s^2")
    print("steady drift (rear sliding at mu x sliding, front rolling):")
    print("  beta   V m/s  km/h  a_y   front use  rear thrust  power   yaw rate")
    for deg in (5, 10, 15, 20, 25, 30, 40):
        v, a, fu, frx, p = drift_speed(math.radians(deg), R, mu, args.sliding)
        print(f"  {deg:3d}   {v:5.1f}  {v * 3.6:5.0f}  {a:4.1f}   {fu:5.2f}     {frx:6.0f} N   {p / 1e3:4.0f} kW"
              f"  {v / R:.2f} rad/s")
    print("acceleration the car can make (rear traction or power, less drag):")
    print("  " + "  ".join(f"{v:.0f} m/s {accel_limit(v, mu):.1f}" for v in (5, 10, 20, 27.8, 30, 35, 40)))
    print(f"braking, no ABS: {mu * G:.1f} m/s^2 at the peak, {mu * args.sliding * G:.1f} locked")
    print("peak speed on each straight (entered and left at the drift speed of beta 20 deg):")
    vd = drift_speed(math.radians(20), R, mu, args.sliding)[0]
    for name, acc, dec in (("Autoware's limits now (common.param 3.0 / -2.5)", 3.0, 2.5),
                           ("Autoware's upstream (1.0 / -1.0)", 1.0, 1.0),
                           ("the car's (traction ~6 / brake 8)", 6.0, 8.0)):
        peaks = [straight_peak(vd, vd, s, acc, dec) for s in STRAIGHTS]
        print(f"  {name}: " + ", ".join(f"{p:.1f}" for p in peaks) + " m/s")


if __name__ == "__main__":
    main()
