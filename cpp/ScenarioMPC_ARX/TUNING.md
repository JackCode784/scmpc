# Tuning the SCMPC hyperparameters

This note records how the hyperparameters in `setup.h` (the `HP_DEFAULT_*`
blocks) were chosen, and turns the results into a **policy**: a way to guess
good values for a new system before sweeping, then refine them. The raw sweep
data is in `tuning/results/`; every number below can be regenerated with
`tuning/sweep.py`.

Notation: a system is the ARX model
`y(k) = θ1·y(k-1) + ... + θna·y(k-na) + θna+1·u(k-nk) + ...`, sampled every
`Ts`. With `NRMLZ`, the controller works in normalized units where the
physical ranges `[YMINPHYS, YMAXPHYS]` and `[UMINPHYS, UMAXPHYS]` map to
`[-1, 1]` (gains `gy = yNormGain`, `gu = uNormGain`).

## 1. The hyperparameters

| Name (`HP_*`) | Symbol | Meaning |
|---|---|---|
| `NHOR` | N | prediction horizon (samples) |
| `NHORU` | Nu | control horizon: free input moves; the input is held from step Nu to N. Also the number of MADS variables d |
| `LOG2NSCEN` | log2 S | S scenarios (θ drawn from the zonotope each call), plus the nominal model |
| `MADS_ITER` | I | MADS iterations per controller call |
| `D0` | D0 | initial MADS frame exponent: the first poll step is 2^D0 (normalized input units) |
| `FRAME_EXP_MIN` | | smallest frame exponent |
| `LOG2Q`, `LOG2P`, `LOG2R` | | stage output, terminal output and input-increment weights, 2^LOG2X (normalized units) |
| `RBASELINE`, `LOG2R_RAW` | | input weight without `NRMLZ` (physical units) |

The stage cost is `Q·(y - r)² + R·(Δu)²`, summed over the horizon, plus
`P·(y(N) - r)²`. Note R weights the input **increment** Δu, not u.

## 2. How a configuration is scored

`tuning/sweep.py` builds the C simulation per configuration and runs it on
`--runs` plants. With `--plant random` each run draws a new true plant
(`plant.h`); with `--plant true` every run uses `THETA_TRUE_INIT` and only
the measurement noise differs. The same seed gives every configuration the
same plants and noise, so all comparisons are **paired**. Per configuration
it reports:

- **Output-bound violations** (`runsViolY`, `samplesViolY`, `maxViolY`):
  y outside `[YMIN, YMAX]`. Hard requirement: none.
- **Rate violations** (`runsViolDy`, `samplesViolDy`, `maxViolDy`):
  |y(k) − y(k−1)| > `DELTAY`, and by how much.
- **Tracking ratio** `rmse / rmseFloor` (mean and worst over runs). The
  floor is the RMSE of the fastest output the constraints allow: the first
  nk samples as measured (no input reaches them), then a step towards the
  reference of at most `DELTAY` per sample, inside `[YMIN, YMAX]`. It ignores
  the plant's dynamics and input limits, so it is a lower bound that is not
  always reachable, but it makes tracking comparable across systems and
  references: 1.10 means 10% worse than any constraint-respecting output
  could be. (A ratio below 1 is possible only by violating `DELTAY`.)
- **Input activity** `rmsDu`: RMS of Δu, normalized to the input range
  (2 = full swing every sample).
- **Estimated latency** `estCycles` (Section 3), and with `--ts` its share of
  the sampling period.

Measured violations are on the plant output the test bench records, so some
are below what the controller can observe. For the pendulum, one ADC step is
1.8/4095 = 4.4·10⁻⁴ rad. Rate excesses smaller than that are below the
sensor's resolution, so the violation *counts* are best read as a relative
indicator.

Tools:

```sh
# grid sweep (cartesian product of --grid), with and without scenarios
python3 tuning/sweep.py --system pendulum --runs 20 --ts 0.01 \
    --grid HP_NHOR=10,12,15 --grid HP_D0=-2,-4 --out my.csv
# pivot one metric over two hyperparameters
python3 tuning/table.py my.csv --rows HP_NHOR --cols HP_D0 --metric meanRatio --scen scen
# fixed point (ap_fixed headers from Vitis or Xilinx/HLS_arbitrary_Precision_Types)
python3 tuning/sweep.py --system pendulum --fixed --ap-include <dir> ...
```

## 3. The hardware budget comes first

All the work is in the cost function: each MADS iteration polls 2·Nu
points, and each poll simulates N steps of S+1 models. Two Vitis HLS
2021.1 syntheses with `PRAGMA_PROFILE_BALANCED` fit

    cycles ≈ I · 2Nu · N · (14.5 + 2.0·S)

| | I | Nu | N | S | measured | model |
|---|---|---|---|---|---|---|
| buck | 7 | 3 | 5 | 4 | 4706 | 4725 |
| pendulum | 10 | 3 | 15 | 16 | 41470 | 41850 |

Two points determine the two coefficients, so treat the model as a guide
and re-fit it as syntheses accumulate. The 2 cycles per scenario come from
the serial accumulations in `costFunctionArx`. The scenario loop is
unrolled, but each unrolled copy adds to the same `scenariosContrib` and
`cost[1]` (through `updateConstraintViolation`). Those are saturating
`cost_type` additions, which are not associative, so HLS cannot rebalance
them into a tree. The budget is `f_clk · Ts` (10⁶ cycles for the pendulum
at 100 MHz and 10 ms); keep some margin for I/O and the PL/AL update.

## 4. The policy, parameter by parameter

### 4.1 Weights (normalized, powers of two)

With `NRMLZ`, errors and input moves are dimensionless, of order 1 at full
range, so weights carry over between systems. `FIXED` applies them as shifts,
so they are powers of two: Q = 2^LOG2Q, P = 2^LOG2P and R = 2^LOG2R (since
this branch, R is defined from `LOG2R` directly, so floating point and `FIXED`
use the same R by construction).

- **Start** with Q = P = 16 and R = Q/4 (LOG2Q = LOG2P = 4, LOG2R = 2). This
  is Bryson's rule in normalized units, w = 1/(acceptable value)². An error of
  1/4 of the range costs the same as an input increment of 1/2 of the range.
- **Then** raise R until the input activity is acceptable, as long as the
  tracking ratio rises by less than about 5%.
- **What the sweeps show** (pendulum, 48 combinations): the tracking ratio
  stays within 1.09–1.18, and rate violations stay near zero. R mainly sets the
  input activity: rmsDu falls about 3× from LOG2R = −2 to 4. But too large an
  R makes the loop sluggish. Without scenarios, R = 16 made the pendulum
  oscillate (the loop never settled within 0.02 rad, steady-state jitter 0.034
  vs 0.003), while R = 4 was fine. So R = Q/4 stays the pendulum's value.
- **For the bucks** the weights hardly matter: ratio 1.05–1.07 (BUCK_LOSS) and
  1.20–1.23 (BUCK_ALBERTO) over 9 combinations; rate violations come from
  other causes (Section 5.2).

### 4.2 Prediction horizon N: long enough to brake

To respect |Δy| ≤ `DELTAY`, the controller must see, within the horizon, the
moment it has to start slowing down. A rule from the plant:

    N ≳ nk + 1.5 · t_stop,   t_stop = DELTAY / a_max

Here a_max is the largest output "deceleration" |Δ²y| the input can produce,
in the worst case over the operating range. For an ARX model, Δ²y is
linear in u and y, so a_max is easy to bound.

- **Pendulum.** Linearized, Δ²y = (θ2+1)·y + θ3·u = 1.18·10⁻³·y +
  7.5·10⁻⁴·u. Near the target (|y| = 0.78) gravity pushes away from upright,
  so a_max = 7.5·10⁻⁴·3 − 1.18·10⁻³·0.78 = 1.33·10⁻³ rad/sample². Then
  t_stop = 0.01/1.33·10⁻³ = 7.5 samples, and N ≳ 2 + 11 = 13.
- **What the sweeps show** (with scenarios, Nu = 3):

  | | N = 10 | 12 | 15 | 20 |
  |---|---|---|---|---|
  | rate violations (I = 50) | 13 | 4 | 0 | 0 |
  | tracking ratio | 1.08 | 1.13 | 1.26 | 1.53 |

- **Why not longer.** Beyond the braking horizon, a longer N only slows
  tracking down. With Nu = 3 the input is frozen for the last N − 3 steps.
  For the unstable pendulum (pole at 1.034 per sample) a frozen torque makes
  the predicted angle run away, so the optimizer prefers timid first moves.
  The plant itself is not the limit: full torque reaches the 0.01 rad/sample
  rate in about 8 samples, yet N = 15 took 230 samples to cross +0.7 rad,
  against about 150 at the floor.
- **For stable plants** use the step-response rise time instead of t_stop.
  For BUCK_LOSS's nominal model (poles 0.934∠0.23 rad, time constant 14.5
  samples, 63% of the step after 8 samples) that gives N ≈ 2 + 12 = 14. The
  synthesized N = 5 was chosen for latency, and the sweeps show the
  trade-off: BUCK_ALBERTO's rate violations at I = 7 fall from 15 (N = 5) to
  1 (N = 12) samples, while the tracking ratio rises from 1.20 to 1.28.

### 4.3 Control horizon Nu and scenarios S: scenario theory

Nu is the number of decision variables d of the scenario program. For a
convex scenario program with d variables and S scenarios, Campi & Garatti
(2008) bound the probability β that the solution violates the constraints
with probability more than ε:

    P[ V > ε ] ≤ Σ_{i=0}^{d-1} C(S,i) ε^i (1−ε)^(S−i)

With β = 0.1, the ε this certifies is:

| S | d = 1 | 2 | 3 | 4 |
|---|---|---|---|---|
| 4 | 0.44 | 0.68 | 0.86 | 0.97 |
| 8 | 0.25 | 0.41 | 0.54 | 0.66 |
| 16 | 0.13 | 0.22 | 0.30 | 0.37 |
| 32 | 0.07 | 0.12 | 0.16 | 0.20 |
| 64 | 0.04 | 0.06 | 0.08 | 0.10 |

Our problem is not convex (MADS), the scenarios are redrawn every call, and a
"violation" is any predicted sample out of bounds, so these are not
guarantees. But the trend they predict is exactly what the sweeps show: **for
fixed S, more freedom (larger Nu) gives more violations**. The optimizer
exploits the finite set of scenarios it sees.

- **Pendulum** (N = 12, I = 20, D0 = −4), rate violation samples:

  | S \ Nu | 3 | 4 | 6 |
  |---|---|---|---|
  | 4 | 33 | 29 | 45 |
  | 8 | 17 | 5 | 7 |
  | 16 | **0** | 4 | 5 |
  | 32 | 3 | 1 | 1 |
  | 64 | 5 | **0** | **0** |

  More Nu gives faster tracking (ratio 1.18 → 1.09 from Nu = 3 to 4 at
  N = 12, I = 20, D0 = −8), and Nu = 1–2 never violated but was slow
  (1.29–1.36).
- **Policy.** Start at Nu = 3. Pick S as the smallest power of two with
  ε(S, Nu) ≲ 0.3: S = 16 for Nu = 3, S = 32–64 for Nu = 4. Then check the
  latency budget; S costs 2 cycles per step and Nu costs a factor of Nu.
- **BUCK_ALBERTO.** S = 8 removes all rate violations at every Nu, for
  ratio 1.19–1.27 (vs 1.17–1.20 at S = 4).
- **BUCK_LOSS** needs S = 16 (rate violation samples 422 → 89 at Nu = 3),
  but then tracks 3–4× slower than the floor (Section 5.2).

### 4.4 MADS: D0 and iterations go together

The first poll step is 2^D0 in normalized input units. It at most doubles
on each successful iteration and halves on each failure. A call that has to
move the input by Δ therefore needs about log2(Δ) − D0 successful
iterations before it can take that step. With I = 10 and D0 = −8
(Δ0 = 0.004), the optimizer cannot follow a transient.

- **Policy.** D0 ≈ log2 of the typical per-call input change during a
  transient, in normalized units. Then I ≥ 2·|D0| or so, so that the frame
  can also shrink to refine. For the pendulum, the gravity-compensation input
  alone changes by 2·1.22/3 = 0.82 (normalized) between the two equilibria,
  over a transient of tens of samples, giving D0 = −2 to −4.
- **What the sweeps show** (pendulum, N = 12, with scenarios):

  | D0 \ I | 10 | 20 | 50 |
  |---|---|---|---|
  | −8 | 1.245 (20) | 1.182 (32) | 1.127 (4) |
  | −4 | 1.131 (2) | 1.113 (0) | 1.093 (5) |
  | −2 | 1.139 (0) | 1.123 (0) | 1.081 (1) |

  Entries are the tracking ratio, with rate violation samples in brackets.
  A good D0 at I = 10 matches D0 = −8 at I = 50, at a fifth of the latency.
- **Price.** A larger first step makes the input busier during transients
  (rmsDu 0.17 vs 0.04). In steady state it is not worse: the MADS frame
  shrinks within the call, and the measured steady-state jitter was 0.0066
  (D0 = −2) vs 0.0095 (D0 = −8).
- **`FRAME_EXP_MIN`** only matters if the frame shrinks below the input's
  resolution. In `FIXED`, a step below one LSB of `norm_input_type` (2⁻¹⁶)
  polls the same point twice, so −16 is the useful floor. With D0 = −2 and
  I ≤ 20 the frame never gets there, so the defaults were left as they were.

## 5. Results per system

### 5.1 Inverted pendulum (Ts = 10 ms, budget 10⁶ cycles)

Sweeps: 20 random true plants (l, m within ±20% of nominal, nonlinear
simulation), float, `NRMLZ` + `CONVERSIONS_MODE`, seed 1. The finalists were
then re-checked on **50 new plants (seed 2)**, 400 samples, so the choice is
not fitted to the 20 tuning plants:

| | N | I | D0 | LOG2R | ratio mean/worst | rate viol. runs/samples (max) | settling* mean/max | steady jitter | cycles |
|---|---|---|---|---|---|---|---|---|---|
| C0 previous | 15 | 10 | −8 | 2 | 1.292 / 1.795 | 7 / 74 (4.2e-4) | 249 / never | 0.0095 | 41 850 |
| C3 | 15 | 10 | −4 | 4 | 1.163 / 1.413 | 9 / 45 (2.0e-4) | 225 / never | 0.0044 | 41 850 |
| C4 | 12 (Nu 4, S 32) | 20 | −4 | 4 | 1.075 / 1.146 | 11 / 73 (2.1e-4) | 202 / 301 | 0.0050 | 150 720 |
| C5 | 12 | 10 | −2 | 4 | 1.107 / 1.213 | 8 / 14 (1.2e-4) | 203 / 314 | 0.0057 | 33 480 |
| **C8 chosen** | **12** | **10** | **−2** | **2** | **1.125 / 1.213** | **5 / 9 (1.4e-4)** | **190 / 240** | **0.0066** | **33 480** |
| C11 | 15 | 10 | −2 | 2 | 1.131 / 1.227 | 3 / 5 (8.2e-5) | 215 / never | 0.0096 | 41 850 |

\*samples until |y − r| < 0.02 rad for good; "never" means at least one run
did not settle within 400 samples. Steady jitter is the RMS normalized Δu
over samples 300–400. Nu = 3, S = 16 and LOG2Q = LOG2P = 4 unless shown.
All rows are with scenarios; `tuning/results/pendulum_candidates_seed2.csv`
has the no-scenario rows too.

**Chosen: N = 12, D0 = −2**, everything else unchanged. It is better than the
previous defaults on every closed-loop metric, and 20% cheaper.

- **No-scenario baseline.** It stays sound with these values (ratio 1.057,
  settles in 177/202 samples), which matters for the scenarios-vs-no-scenarios
  comparisons. C5's R = 16 was rejected because without scenarios it
  oscillates.
- **Fixed point** (50 plants, seed 2) reproduces float: ratio 1.102 vs 1.125,
  rate violations in 2 vs 5 runs (`pendulum_final_*.csv`).
- **Budget.** At 33 480 cycles (3.3% of Ts) there is room for more. C4
  (Nu 4, S 32, I 20) tracks best (1.075) at 15% of Ts. Use it if tracking
  speed matters more than area.

### 5.2 Buck converters (BUCK_LOSS, BUCK_ALBERTO)

Swept with `--plant true` (10 runs, differing by the measurement noise only;
see the caveat below on random plants). Their defaults are left at the
synthesized values (N 5, Nu 3, S 4, I 7, D0 −8, weights 4/4, R = 2⁻³)
because their sampling period, hence their cycle budget, is not known here.

- **BUCK_ALBERTO.** Rate violations (35 samples, up to 0.46 V at N 5,
  I 50) disappear with **S = 8** (`HP_LOG2NSCEN 3`), at every Nu, for a
  tracking ratio of 1.19–1.27. Estimated cost: 6.4k instead of 4.7k cycles.
  Recommended if Ts allows it.
- **BUCK_LOSS** violates the rate limit in about 15% of samples in every
  configuration, with or without scenarios. Two causes:
  - Its true plant has 1.5× the nominal input gain (θ3 = 0.683 vs 0.458),
    so the nominal prediction underestimates every move.
  - With S = 4 the input jitters in steady state (rmsDu 0.21 with scenarios
    vs 0.06 without). The scenarios are redrawn every call, so the optimum
    moves from one sample to the next.

  S = 16 brings the largest excess from 0.33 V to 0.04–0.08 V (the noise
  level is 0.04 V) but tracks 3–4× slower than the floor. Its zonotope is too wide for robust
  control without learning. PL/AL, which shrink the zonotope online, are the
  more promising route for this system.
- **Output-bound violations.** The ones reported for both bucks (0.05 V and
  0.46 V) are the measurement noise around y = 0 in the first samples, not
  control errors.

## 6. Recipe for a new system

1. **Budget.** cycles_max = f_clk · Ts minus a margin. Use the latency model
   (Section 3) to see what fits.
2. **Normalize** (`NRMLZ`). Start with LOG2Q = LOG2P = 4, LOG2R = 2.
3. **Horizon.** Compute t_stop (or the rise time for a stable plant) from
   the nominal ARX model, and set N ≈ nk + 1.5·t_stop.
4. **Nu = 3 and S from the ε table** (S = 16 for ε ≈ 0.3). Check the budget.
5. **D0** from the typical per-call input change in a transient, with
   I ≈ 2|D0|–4|D0|.
6. **Sweep.** Run `sweep.py` around this point (N ± 3, D0 ± 2, I × 2, S × 2)
   on random or true plants, with and without scenarios. Choose by:
   - no output-bound violations;
   - then rate violations;
   - then the tracking ratio;
   - then input activity and latency.
7. **Re-check** the winner on a different `--seed` with more runs, and in
   `--fixed`.
8. **Record** the values in a `HP_DEFAULT_*` block and the CSVs in
   `tuning/results/`. When a new synthesis report arrives, add it to
   Section 3 and re-fit the latency model.

## 7. Caveats

- **Random buck plants.** With `RANDOM_TRUE_PLANT 1`, the bucks draw θ
  uniformly from the zonotope. About 4–5% of those plants are unstable, and
  the DC gain θ3/(1 − θ1 − θ2), whose nominal denominator is only 0.054,
  varies by large factors. Such plants are not realistic buck converters. A
  physically parameterized draw (as for the pendulum: R, L, C within a
  tolerance) would give a fairer multi-plant buck experiment.
- **Fixed point without `NRMLZ`** does not compile with the open-source
  `ap_fixed` headers: a `?:` between signed and unsigned types in
  `computeArxOutput.cpp` and, with `CONVERSIONS_MODE`, ADC/DAC types used
  before they are declared in `types.h`. Both predate this work.
- **Possible latency improvement.** Summing the scenario terms in a tree,
  over a wide non-saturating type and rounding once, would make the
  per-scenario latency cost logarithmic instead of linear in S. That is
  about 2× fewer cycles for the pendulum at S = 16. It has not been
  implemented or synthesized.
