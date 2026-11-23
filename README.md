# encs4380-dynamics

# Dynamic Characteristics & PID Control Toolkit — ENCS4380 Homeworks 1 and 2

Interactive web toolkit for sensor dynamic characteristics (Homework 1) and feedback control (Homework 2, the "Control" section, linked from the landing page) (ENCS4380 Interfacing Techniques, Birzeit University, Dr. Wasel Ghanem).

**Live page:** [https://dalia-00.github.io/encs4380-dynamics/](https://dalia-00.github.io/encs4380-dynamics/)

## Team

| Member | Module |
|---|---|
| Dalia Abu Baker| Module 1 — First-order time response; HW2 P2 — A slow sensor in the loop |
| Mohammad Obeidallah | Module 2 — First-order frequency response; HW2 P3 — Digital PID |
| Ghaleb Shakhsheer | HW1 Module 3 — Second-order response + state-space panel; HW2 P1 — PID playground |
| Hala Sarsour | Module 4 — "Guess the sensor" challenge; HW2 P4 — Tuning challenge |
| Whole team (T21) | HW2 "My system" — micromouse drive-motor speed loop |

## Module 1 — First-order time response (thermometer)

Model: τ·dy/dt + y = K·x(t), G(s) = K/(τs + 1)

- Sliders for K, τ, step size A (or ramp rate q̇) and initial reading y₀; Step / Ramp switch.
- Step plot with the 63.2 % point at t = τ and the 95 % point at t = 3τ, the ideal (instant) sensor, and the initial-slope line.
- Error-fraction plot on a log axis: ln eₘ = −t/τ, a straight line with slope −1/τ (τ is recovered from the slope).
- Ramp plot showing the steady-state time lag (= τ) and steady-state error (= q̇τ), plus the error-vs-time curve.
- "Predict, then check" box with random questions and a session score.
- Validation table (expected value, our value, PASS/FAIL), with values read off the plotted curve.

Closed-form responses used (from the lecture slides):
- Step: y(t) = KA + (y₀ − KA)·e^(−t/τ)
- Ramp: y(t) = K·q̇·(τ·e^(−t/τ) + t − τ)
## Screenshots

| Module 1 | Module 2 |
|---|---|
| ![Module 1](screenshots/module_1.png) | ![Module 2](screenshots/module_2.png) |

| Module 3 | Module 4 |
|---|---|
| ![Module 3](screenshots/module_3.png) | ![Module 4](screenshots/module_4.png) |

## Validation checks

| # | Check | Expected | Status |
|---|---|---|---|
| 1 | First-order step | y(τ) = 0.632 KA, y(3τ) = 0.95 KA | PASS (Module 1) |
| 2 | τ = 0.2 s, input sin 2t | M = 0.93, lag 21.8° | Module 2 |
| 3 | 100 Hz, ≤ 5 % dynamic error | τmax ≈ 0.52 ms, 9.3° at 50 Hz | Module 2 |
| 4 | ζ = 0.5 | overshoot 16.3 % | Module 3 |
| 5 | ζ = 1 | no overshoot | Module 3 |
| 6 | m = 1, b = 2, k = 4 | eigenvalues −1 ± j1.732 | Module 3 |
| 7 | HW2: P only, Kp = 9, K = 1, τ = 1 s | τcl = 0.1 s, ess = 10 % | PASS (Validation page, closed-loop Euler simulation) |
| 8 | HW2: PI, Kp = 1, Ki = 1 | ωn = 1 rad/s, ζ = 1 | PASS (also y(1 s) = 1 − e⁻¹, no overshoot) |
| 9 | HW2: any PI | zero steady-state error for a step | PASS (five gain sets) |
## Module 2 — First-order frequency response

Model: G(s) = K/(τs + 1), M = 1/√(1 + (ωτ)²), φ = −tan⁻¹(ωτ)

- Sliders for τ, input frequency ω, gain K and input amplitude A.
- Input, ideal output and sensor output sines on one plot.
- Live amplitude ratio M, phase lag, and cutoff frequency ω_c = 1/τ.
- Bode plot (magnitude and phase) with the cutoff marked and a dot that follows the selected ω.
- Dynamic-error calculator (δ = M − 1) and a button that applies the "100 Hz, at most 5% error" design limit.
- "Predict, then check" box and a validation table.

## Module 3 — Second-order response and state-space panel

Model: G(s) = Kωₙ² / (s² + 2ζωₙs + ωₙ²), with ωₙ = √(k/m) and ζ = b / (2√(mk))

- Sliders for ζ and ωₙ, or for m, b and k. The two sets stay synchronised.
- Underdamped, critically damped and overdamped step responses (preset buttons), with reference curves.
- Rise time, peak time, overshoot and settling time read live off the sampled curve.
- Poles plotted in the s-plane.
- State-space panel: A and B update live with m, b and k, the eigenvalues of A are shown next to the poles, and the (x₁, x₂) path is drawn in the phase plane.
- "Predict, then check" box and a validation table.

## Module 4 — "Guess the sensor"

- The page picks a hidden sensor at random: first order step (random K and τ), first order with a sine input (K = 1, random τ), or second order step (K = 1, random ζ and ωₙ). It adds noise to the response.
- You read the curve, estimate the parameters, and submit.
- The page reveals the true values, overlays the true curve (green) and your guess (red dashed) on the data, and scores the round.
- Scoring: within 10% error on a parameter is full marks, falling linearly to zero at 50% error. A round is worth 100 points, split across its parameters.
- Hints on request (one at a time), a "New sensor" button, and a running score for the session.
- The methods section on the page lists the equations needed: the 63.2% point and the log method for first order, and overshoot and peak time for second order.

## Tech

Static HTML + JavaScript, plots with [Plotly.js](https://plotly.com/javascript/). No server needed; works on phones.


<!-- add screenshots/module1-step.png and screenshots/module1-ramp.png -->

## AI use

An AI assistant (Claude) was used to help write the Module 1 page code and this README. All formulas were checked against the lecture slides and the validation numbers.

HW2 "My system": Claude (Anthropic) was used to help write the My system page (`sys.js`, `sys.css`, its section of `index.html`), the HW2 rows of the Validation page and this README update. The motor numbers come from our own parts order; the τ estimate, the PI pole-placement formulas and the generated ESP32 code were checked against the lecture formulas and the validation checks.

HW2 P2: Claude (Anthropic) was used to help write the P2 page (`p2.js`, `p2.css`, the P2 section of `index.html`) and its README entry. The Routh–Hurwitz threshold and the P-only damping formula come from the closed-loop characteristic equation and are checked by the validation table on the page.


---

## Homework 2 — Control

- **P1 — PID Playground (Ghaleb Shakhsheer):** [Open P1](https://dalia-00.github.io/encs4380-dynamics/#p1).
- Reuses HW1 Module 1 first-order model `G(s) = K/(τs+1)` and Module 3 normalized second-order model `G(s) = Kωₙ²/(s²+2ζωₙs+ωₙ²)`.
- Live plant selector, PID gains `Kp/Ki/Kd`, setpoint step, and symmetric actuator limit `±Umax` (normalized units).
- Differential-equation simulation with adaptive-step **forward Euler**; it does not use closed-form output curves.
- Plots of output versus setpoint, control signal and saturation limits, plus rise time (10–90%), overshoot, ±2% settling time and final simulated error.
- Uses **derivative on measurement** (`-Kd·dy/dt`) to prevent derivative kick from a setpoint step; conditional integral anti-windup prevents accumulation beyond actuator limits.
- Includes a “Predict, then check” exercise and numerical validation checks for P-only (`Kp = 9`) and PI (`Kp = 1, Ki = 1`, plus another PI setting).

**P1 source files:** `p1.js` (simulation, plots, metrics, validation), `p1.css` (styles), plus the new P1 page/navigation and script link in `index.html`.

**Note:** P1 uses normalized actuator commands. The real Micromouse motor model and the physical 0–100% PWM constraint belong in the team’s separate *My System* page. P2, P3 and P4 are separate teammate modules; they are not included in this P1 branch.

- **P2 — A slow sensor in the loop (Dalia Abu Baker):** [Open P2](https://dalia-00.github.io/encs4380-dynamics/#p2). Issue #7.
- A first-order sensor `1/(τs·s + 1)` sits in the feedback path. The controller only sees the sensor reading `ym`: `e = r − ym`, `u = Kp·e + Ki·∫e dt − Kd·dym/dt`.
- Same plants as P1 (HW1 Module 1 first order, Module 3 second order), sensor time constant `τs` slider, PID gains, setpoint, actuator limit and simulation time.
- **True output `y` and sensor reading `ym` on the same plot**, plus the control signal. A "Slow the sensor down" button animates `τs` from 0 to 3 s so the move to oscillation is visible.
- **Overshoot vs `τs` plot:** each point is a full simulation; the range where the loop no longer settles is shaded, with its edge from Routh–Hurwitz. For the first-order plant: `τ·τs·s³ + (τ + τs + K·Kd)s² + (1 + K·Kp)s + K·Ki = 0`, so the loop oscillates once `τs > (τ + K·Kd)(1 + K·Kp) / (τ·K·Ki − (1 + K·Kp))`.
- Live metrics: true overshoot, the overshoot the sensor reports, the hidden difference, true settling time, steady-state error, loop behaviour (settles / sustained / growing oscillation).
- "i" info buttons on every P2 card (same idea as Module 1): each opens an explanation written from the current settings, e.g. the Routh threshold with your own numbers substituted.
- Forward Euler simulation of plant, sensor and integral together (no closed-form curves). Derivative on the sensor reading, conditional-integration anti-windup.
- "Predict, then check" questions (final value, critical `τs`, true overshoot) and 5 validation checks computed by the simulator: `τs = 0` reproduces P1's `Kp = 9` case; a slow sensor does not change the steady state; with P only the sensor reading is a pure second-order response, so its overshoot matches `exp(−πζ/√(1−ζ²))` with `ζ = (τ + τs)/(2√(τ·τs(1 + K·Kp)))`; the true output overshoots more than the sensor shows; and for PI `Kp = 4, Ki = 8` the loop settles below and oscillates above `τs,crit = 5/3 s`.

**P2 source files:** `p2.js` (simulation, Routh analysis, plots, metrics, questions, validation), `p2.css` (styles scoped to `#p2`), plus the P2 page, menu link, home-page card and script link in `index.html`. P1's files are unchanged.


- **P3 — Digital PID (Mohammad Obeidallah):** [Open P3](https://dalia-00.github.io/encs4380-dynamics/#p3). Issue #10.
- Implements a discrete-time PID controller with adjustable sampling time Ts and gains Kp, Ki, and Kd.
- Supports first-order and second-order plants from Homework 1, with adjustable plant parameters and setpoint.
- Simulates the closed-loop response using forward Euler integration and zero-order hold between sampling instants.
- Displays the plant output versus setpoint, sampled measurements, and stair-step control signal with actuator limits.
- Shows the effect of increasing Ts on overshoot, tracking error, and stability.
- Includes an Anti-Windup ON/OFF option to demonstrate the effect of integral accumulation during actuator saturation.
- Displays live performance metrics: rise time, overshoot, settling time, end-of-run error, and saturated samples.
- Includes a "Predict, then check" activity with interactive questions and a session score.
- Generates Arduino PID loop code using the current controller settings, with a Copy Code button.

**P3 source files:** `p3.js` (digital PID simulation, plots, performance metrics, Arduino code generation), `p3.css` (styles), and the P3 section/navigation in `index.html`.

**GitHub:** Issue #10, Pull Request #11.

- **P4 — Tuning challenge (Hala Sarsour):** [Open P4](https://dalia-00.github.io/encs4380-dynamics/#p4).
- A hidden random plant is drawn each round (first or second order, reusing the HW1 Module 4 idea), and the page builds a spec from a reference design: overshoot limit, 2 % settling time, zero steady-state error and a control-signal peak limit.
- Kp, Ki, Kd sliders; the closed loop is stepped in time with RK4 (filtered derivative).
- PASS/FAIL for each part of the spec, a score per round and a running session score, hints, and a "Predict, then check" question about how a gain change moves the overshoot.

**P4 source files:** `p4.js`, `p4.css`, and the P4 section in `index.html`.

## Homework 2 — My system: micromouse drive-motor speed loop

[Open the page](https://dalia-00.github.io/encs4380-dynamics/#sys). Our project is the micromouse (team T21), so the plant is one **drive wheel**: PWM duty u (0–100 %) in, wheel speed v (m/s) out, G(s) = K/(τs + 1).

**Plant numbers and sources**

| Quantity | Value | Source |
|---|---|---|
| Motor | JGA12-N20, 6 V, 500 RPM, 7 PPR Hall encoder | our parts order (option "500 RPM / 6V") |
| Wheel | 43 mm rubber wheel | our parts order |
| Driver / supply | DRV8833; 2S LiPo through an LM2596 set to 6 V | our M0 bill of materials |
| K | 500 RPM · π · 0.043 m / 60 / 100 % ≈ **0.0113 (m/s)/%** | datasheet (no-load) |
| τ | R·J/(KtKe) ≈ **0.039 s** (R = 8 Ω, 1:30 gearbox, 200 g robot assumed) | estimate, shown step by step on the page |

τ is an estimate until we run the step test. The page has a box where a logged `t, speed` step test can be pasted; it fits K (final speed / PWM step) and τ (63.2 % time) and redesigns the controller from the measured plant.

**Spec (and why):** ζ between 0.6 and 0.8 (lecture range), overshoot ≤ 10 % (a speed spike makes the wheel slip and the encoder over-count), 2 % settling time ≤ 0.15 s (at 0.5 m/s a 180 mm cell takes 0.36 s), zero steady-state error (both wheels must hold the same speed or the mouse curves), and PWM inside 0–100 %.

**Design:** PI by pole placement, ζ = 0.7 and ts = 0.15 s → **Kp ≈ 94 %/(m/s), Ki ≈ 4975 %/(m/s·s)**, Ts = 5 ms. Simulated (digital PI, ZOH, forward Euler): 7.1 % overshoot, 0.13 s settling, 0 % error, peak PWM 64 %: every part of the spec passes. Dashed curves show the same controller if the real τ is 0.5× or 1.5× the estimate.

**Generated code:** an ESP32 (Arduino core) sketch with x4 quadrature encoder interrupts, the same PI and anti-windup as the simulation, DRV8833 PWM and `t, v, u` serial logging, so the real response can be compared with the simulation (bonus). Pins are placeholders until `robot_config.h` is filled in.

**Source files:** `sys.js` (model, design, simulation, spec check, step-test fit, code generator, HW2 validation checks), `sys.css`, and the My system section, menu link and home card in `index.html`.
