# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repository is

Scenario-based MPC (SCMPC) for ARX systems with a zonotopic parameter-uncertainty set, solved by MADS (Mesh Adaptive Direct Search) with a progressive barrier, and implemented on an FPGA with Vitis HLS.

- `cpp/ScenarioMPC_ARX/`: the active project. A single C++ source tree is both:
  - the PC closed-loop simulation (`double` types), and
  - the Vitis HLS source (`ap_fixed` types).

  Target: xc7z020-clg484-1, 10 ns clock, top function `controller`.
- `vitis/scmpc-controller2026/`: the Vitis 2026.1 (Unified IDE) HLS component. Only `hls_config.cfg` and `vitis-comp.json` are tracked. The config lists source files by absolute Windows paths on the author's machine. Build outputs live in `vitis/scmpc-controller2026/scmpc-controller2026/` and are gitignored. `vitis/scmpc-arx/` is the old Vitis HLS 2021.1 project.
- `cpp/MADS_ISCAS23/`: an earlier, separate MADS-MPC HLS design, built on its own.
- `matlab/`: reference and prototype implementations (zonotopes, passive learning, SCMPC). They need the CORA toolbox (gitignored), YALMIP and quadprog.

`README.md` (on `main`) states the control problems in mathematical form (MPC, SCMPC, passive and active learning) and keeps the roadmap ("Past, present, future"). Read it for the theory and the planned work.

## Notation: README/paper ↔ code

| README / paper | Code |
|---|---|
| $N_y$ (prediction horizon), $N_u$ (control horizon) | `mpc::predHrzn`, `mpc::ctrlHrzn` (= `nOpt`) |
| $S$ scenarios, $I$ MADS iterations | `scen::nScen` (= `1 << scen::log2nScen`), `mads::iter` |
| $n_a$, $n_b$, $n_d$ (input delay) | `na`, `nb`, `nd` |
| zonotope centre $c$ (= $\theta^{(0)}$, nominal model), generators $G$, $n_g$ | `thetaCenter`, `thetaGens`, `nGens`; only the `nUnc` uncertain parameters have rows (`UNC_MAP`) |
| $\xi$ | `coeffs` (`generateScenarios`, `pseudoRandArx`) |
| $Q$, $R$ | `mpc::weightQ`, `mpc::weightR` (= 2^`mpc::log2Q`, 2^`mpc::log2R` in normalized units; see "Cost weights" below) |
| cost and constraint violation | `cost[0]`, `cost[1]` of `costFunctionArx` |
| MADS (Ravera et al.): Householder scale $2^{\gamma}$, frame update $-\tau$, initial frame exponent | `mads::gamma` (`householderClamp` = 2^−γ), `mads::frameExpStep` (= −τ > 0), `mads::d0` |

Where the code differs from the README's formulation:
- The last prediction step has its own weight, `mpc::weightP` (= 2^`mpc::log2P`).
- The reference is held constant over the horizon: every error is against the current `yref`.
- Constraints are not hard: MADS's progressive barrier ranks candidates by a violation measure, and the nominal model is always constrained.
- With `DELAY_COMPENSATION`, the decision variables are $u_{k+1}..u_{k+N_u}$ and $u_k$ is known; without it they are $u_k..u_{k+N_u-1}$.

## Building and running the simulation

There is no build system and no unit-test framework. The closed-loop test bench `testMain.cpp` is the test: it simulates the controller against a plant, reports constraint violations and RMSE, and writes trajectories.

```bash
cd cpp/ScenarioMPC_ARX
# default setup.h (float + CONVERSIONS_MODE + NRMLZ): plain g++
g++ -std=c++14 -O2 -I . -o sim *.cpp && ./sim
# fixed point (FIXED in setup.h): needs the Xilinx ap_types headers
# (github.com/Xilinx/HLS_arbitrary_Precision_Types, include/) and a stub
# hls_math.h (it is included, nothing from it is used):
#   printf '#pragma once\n#include <cmath>\n' > <stub_dir>/hls_math.h
g++ -std=c++14 -O2 -I <stub_dir> -I <ap_types>/include -I . -o sim *.cpp && ./sim
```

- `passiveLearning/` is not part of the build: it holds unused code. `systems/` holds headers only (one per system, see "Systems and plants" below).
- The default `setup.h` is floating point with `CONVERSIONS_MODE`, `NRMLZ`, `USE_SCENS_COST` and `USE_SCENS_CONSTR`. For hardware, also uncomment `FIXED`.
- The pendulum in fixed point requires `NRMLZ` (there is an `#error` otherwise).
- **Output location.** Floating-point builds write into `FloatSimulations/` (created if missing). Fixed-point builds write into the working directory, which in Vitis is `.../hls/csim/build`.
  - `testbench_settings::nRuns == 1` writes `output.txt` (plotted by `plotOutputCpp.m`).
  - `nRuns > 1` writes `runs_<ctrl>_<plant>_<NNN>.txt` plus `..._summary.txt` (plotted by `plotMultipleOutputsCpp.m`; `opts.isVitisSim` selects the Vitis folder).
  - `<ctrl>` is `scen`, `scencost`, `scenconstr` or `noscen`, plus `_dc` (delay compensation) or `_latNN` (emulated latency). `<plant>` is `nl` or `arx`.

### Configuration is by editing headers, not compiler flags

Mode flags and selectors are `#define`s; constants are `constexpr`. Only what the preprocessor itself must see stays a macro: what is tested in `#if`, `ACTIVE_SYSTEM` and the system ids, and the arguments of `HLS_PRAGMA(...)`.

- `setup.h`, the "TARGET SELECTION" block and below:
  - mode flags: `FIXED`, `CONVERSIONS_MODE`, `NRMLZ`, `USE_SCENS_COST`, `USE_SCENS_CONSTR`, `DELAY_COMPENSATION`;
  - selectors: `ACTIVE_SYSTEM`, `CTRL_MODE` (SCMPC / passive learning / active-learning stub), `PRAGMA_PROFILE`;
  - hyperparameters, grouped in namespaces and always used qualified (no aliases): `mpc::predHrzn`, `mpc::ctrlHrzn`, `mpc::log2Q/log2P/log2R`; `scen::log2nScen`; `mads::iter`, `mads::frameExpStep`, `mads::gamma`, `mads::frameExpMin`, `mads::d0`. These are global for now, shared by every system.
- `systems/system_<name>.h`, per system: everything listed at the top of `system_configs.h` (dimensions, physical constants, initial values, fixed-point types, optional test-bench section). See "Systems and plants".
- `testbench_config.h` (test bench only): namespace `testbench_settings` (`nRuns`, `nSim`, `randomTruePlant`, `testbenchSeed`, `computeDelayFraction`), and the plant selector `PLANT_MODEL` (`PLANT_MODEL_ARX` or `PLANT_MODEL_PHYSICS`). It is kept out of `setup.h` on purpose: everything in `setup.h` may change the RTL, nothing in the test-bench headers can. `PLANT_MODEL` stays a macro because it selects code with `#if` and the `PLANT_TAG` file-name literal.

### Vitis flow (on the author's machine; not available in this environment)

C simulation, then synthesis, then C/RTL co-simulation. Vitis reports co-simulation "Pass" whenever `main()` returns 0, so the test bench does the real check:

1. C simulation writes `golden_u.txt` (run 0's applied inputs).
2. Co-simulation (`__RTL_SIMULATION__`) compares the RTL's inputs against it and returns 1 on any mismatch, or if the file is missing.

So run C simulation first, and set `testbench_settings::nRuns = 1` for co-simulation: only run 0 is compared, and the RTL's state is not reset between runs.

## Verifying changes

The established check for refactors and pragma changes is that **C simulation output is byte-identical to the previous commit**:

1. Build the old tree (`git archive <rev> cpp/ScenarioMPC_ARX`, which includes `systems/`) and the new tree with the same configuration (`nRuns` reduced, e.g. 3). Set every mode flag explicitly in both trees: the committed defaults have changed over time (before `qol/log2-weights`, `CONVERSIONS_MODE` and `NRMLZ` were off).
   - Fixed-point output changed on purpose with the per-system files (47f89ec and after): the bounds became `constexpr double`, no longer rounded to `output_type`/`input_type` (the pendulum had `yMax` = 0.899994, `deltaY` = 0.010010). Comparisons of fixed point across that point differ; float is identical.
2. Run both and `cmp` every output file.
3. Do this across a matrix of configurations: systems (pendulum, buck converters), fixed and float, the three pragma profiles, with and without scenarios, delay compensation, passive learning.

Pragmas never change C simulation, so any difference is a behaviour change. Hardware effects (timing, latency, instance counts) can only be checked from Vitis reports (`csynth.rpt`, `hls_compile.log`, implementation reports), which the user provides.

## Architecture

**Header chain.** `setup.h` → `system_configs.h` → `systems/system_<name>.h`, then `types.h`.
- `system_configs.h`: the registry. System ids (macros, defined before anything tests `ACTIVE_SYSTEM`), `WORD_LENGTH`, the dispatch that sets `SYSTEM_FILE` to the active system's file and includes it, then `types.h`, then `static_assert`s on the system's constants (constraints inside the physical ranges, `DELTAY > 0`, ...). Its header comment lists what every system file must define.
- `systems/system_<name>.h`: the controller's *model* of one system (`system_buck.h` holds `BUCK` and `BUCK_LOSS`). Dimensions (`constexpr int`); physical constants `UMINPHYS`..`YMAX`, `DELTAY`, `SIGMA_UNNORM`, `Ts` and the `*Init` arrays (`constexpr double`); under `#ifdef FIXED`, the fixed-point types whose range depends on the system. A floating-point-only system (`SIMPLE`, `BENCHMARK`, `MILANO`, `GAIN_DEMO`) stops a `FIXED` build with `#error`. A system without a rate limit sets `DELTAY = YMAXPHYS − YMINPHYS`.
  - `Ts` must be the one the ARX model was discretized at (10 µs for the buck converters, 10 ms for the pendulum, a convention for the abstract systems). The test bench logs it as the last column of the output files, and the MATLAB plots use it for the time axis. The pendulum plant derives l and m from θ and `Ts`, so changing its `Ts` silently changes the pendulum.
- `types.h`: the system-independent types (normalized quantities, 12-bit codes, MADS internals) and the aliases that refer to the system's types (`norm_*_type` without `NRMLZ`, `digital_*_type` without `CONVERSIONS_MODE`); everything is `double` without `FIXED`. It must come after the system file.
- `setup.h`: mode flags and selectors, the hyperparameter namespaces `mpc`, `scen`, `mads`, the conversion/normalization constants (namespaces `dig`, `nrmlz`, `conv`, each defined in every mode, as the identity when its mode is off), `StripCoeffs`, pragma profiles, every prototype, and the `extern` declarations of the controller state.
- `plant.h`: the *true* plant for the test bench. It is never synthesized. See "Systems and plants".

**Systems and plants.** One file per system holds both its model (synthesized) and, optionally, its test-bench code. `plant.h` includes the active system file a second time:
- First pass (every file, via `setup.h`): only the model section.
- `plant.h` defines `TbRng` and `ArxPlant` (with `plantSampleTrue`, `plantReset`, `plantNextOutput`, `plantPrintParams`, `plantPush`), then `#define SYSTEM_TESTBENCH_PASS` and `#include SYSTEM_FILE`.
- Second pass: the system's test-bench section, which can build on `ArxPlant`. A system file with such a section has the form `#ifndef SYSTEM_TESTBENCH_PASS` (model) `#else` (test bench) `#endif`, and must not have `#pragma once` (it would skip the second pass: with it, the pendulum silently ran the ARX plant). Each pass happens once because `system_configs.h` and `plant.h` have `#pragma once`. A system with only the ARX plant keeps `#pragma once`; its second pass adds nothing.
- A system file includes nothing. The test-bench section must not be reachable from synthesized files: check with `g++ -E` on a synthesized `.cpp` (no `TbRng`/`ArxPlant`).
- A model other than ARX is a struct derived from `ArxPlant` (`PendulumPlant : ArxPlant`, with l, m and the ODE state) plus overloads of only the functions that differ; overload resolution prefers the derived overload and falls back to the `ArxPlant&` one (`plantPush` is shared). The section sets `using Plant = ...;` and `PLANT_TAG` for each `PLANT_MODEL` it implements; otherwise `plant.h` falls back to `ArxPlant` and `"arx"`. `testMain.cpp` declares `Plant plant;`.
- Computational delay (`computeDelayFraction`): `plantPush` stores the interval's average input in `uHist` (all a discrete-time ARX plant can represent) and `uEarly`/`uLate`/`fEarly` for models that apply the switch exactly (the pendulum's Euler sub-steps hold `uLate`).
- The default ARX plant draws θ from the controller's initial zonotope, a wider population than the pendulum's l, m ± 20 % (input gain 0.39–1.61 vs 0.58–1.95 times nominal); with it, low-gain draws starting at y = −0.78 can be lost (run 6 of seed 1 diverges).
- ARX plants are equation-error models: they regress on the *measured* outputs (`p.yMeasHist`), i.e. the noise acts as a process disturbance. This is the noise model the passive-learning strip assumes (Bravo, Alamo, Camacho 2006, eq. 1): |y_k − φ_kᵀθ| ≤ σ holds for the true θ. With pure sensor noise, the residual bound would be σ(1 + Σ|aᵢ|) instead.
- `testMain.cpp` naming: `yTrue` (plant output), `yMeas` = `yTrue` + noise (what the ADC sees), `uCtrl` (what `controller()` returned), `uApplied` (what the DAC holds at the start of the interval). Output-file columns: `uAppliedLog yMeasLog yref … uAppliedDigLog yMeasDigLog vol Ts`, read by name by the MATLAB scripts.
- `testbench_config.h`: the experiment settings (`testbench_settings`), included by `testMain.cpp` only.

**`controller()` (`controller.cpp`)** is the HLS top level, with `ap_ctrl_hs` control, 12-bit ADC inputs `yCurrDig`/`yrefDig`, and the DAC value as return. State persists across calls in file-scope globals defined without `static`: zonotope `thetaCenter`/`thetaGens`, `yHistCtrl`/`uHistCtrl`, `uOptPrev` (warm start), `uAppliedCtrl` (delay compensation). `initControllerState` sets them from the `*Init` arrays; the warm start and `uAppliedCtrl` start from `uHistInit[0]`. The step order documented at the top of the file matters (time k vs k−1 data). A call does:

1. Convert and normalize the inputs.
2. Warm start: shift the previous solution.
3. Passive learning only: strip–zonotope intersection (`boundStripZonotopeIntersection`).
4. `generateScenarios`: xorshift32 PRNG, then θᵢ = c + Gξᵢ.
5. `computeArxCoeffs`: fold the normalization into per-model coefficients; row `scen::nScen` is the nominal model.
6. `MADSARX`: `mads::iter` iterations. Each runs `generatePollMatrixArx` (Householder directions), then `progressiveBarrierPollingArx`, which evaluates 2·`mpc::ctrlHrzn` poll points and selects by (violation, cost).
7. Denormalize and convert the output.

The starting point is evaluated through the same poll call site, as an iteration −1 (`startOnly`).

**`costFunctionArx`** is the hot spot: `mpc::predHrzn` prediction steps (a true recurrence) × (`scen::nScen`+1) independent models. It returns `cost[0]` (tracking + input) and `cost[1]` (constraint violation).

With `DELAY_COMPENSATION`, a call at k computes u(k+1) and treats u(k) as known (`DC_STEPS`, `nUKnown`).

## HLS conventions and pitfalls

- **How pragmas are written.**
  - Every pragma sits inside `#ifdef PRAGMAS`, which `FIXED` defines.
  - Profile-dependent pragmas come from the `PRAGMA_*` macros in `setup.h`.
  - A pragma whose argument is a macro must use `HLS_PRAGMA(...)`, because the HLS pragma parser does not expand macros.
  - Each pragma has a comment explaining why it is there; keep that convention and the existing long rationale comments.
- **Profiles.** `PRAGMA_PROFILE` is AREA, BALANCED or LATENCY. All three compute identical numbers; they differ only in the model-loop pipelining, the ARX term unrolling, and the number of `costFunctionArx` instances (`ALLOCATION` in `MADSARX.cpp`). BALANCED is the one used in the paper.
- **Recent Vitis versions pipeline loops and inline functions on their own,** hence the explicit `PIPELINE off` / `INLINE off`. A `PIPELINE` pragma written before a loop applies to the enclosing function, so put it inside the loop body.
- **Timing (10 ns).** In an II=1 loop, every loop-carried value must be a short operation.
  - `yPastCurr` is LUTRAM (partition on dim 2 only) so that `DEPENDENCE ... false` can apply. Complete partitioning into registers created a 21 ns recurrence.
  - The model-loop accumulators `ScenAcc` and `ViolAcc` are single additions, bit-identical to the rounding, saturating `cost_type` sums they replace. Don't revert them to `cost_type +=`.
- **Single call site.** `costFunctionArx` must keep a single call site. Otherwise HLS clones it (`costFunctionArx_1`) and ignores the `ALLOCATION` pragma (warning HLS 214-300).
- **Bit-exactness.** Fixed-point changes must keep C simulation bit-exact, or be deliberately validated. Types use `AP_RND_CONV`/`AP_SAT`, so regrouping additions is not free.
  - Exception: the weighted terms of `cost[0]` (the input term ·2^`log2R`, the output terms ·2^`log2Q`/2^`log2P`). A term is a square times 2^r, so the bits it loses when rounded to `cost_type` can be exactly half an LSB (a tie, where the result depends on the running sum's parity) only if r + (fractional bits of `cost_type`) is odd. With every such sum even, each term rounds on its own, and the saturating sum of non-negative terms is min(total, max) in any order.
  - This is why moving the input term out of the k loop was bit-identical in fixed point: the pendulum has 20 fractional bits and r = 2, 4, 4. With an odd `log2R`, about 1% of random evaluations differ by 1–2 LSB.
  - In floating point the same reordering does change the cost by about 1 ULP (unit in the last place) in about 1% of evaluations. Trajectories change only if that flips a MADS comparison.
- **Cost weights are powers of two, defined by their exponents** (`mpc::log2Q`, `mpc::log2P`, `mpc::log2R` in `setup.h`; `pow2()` also handles negative exponents).
  - Fixed point applies `weightQ`/`weightP` as left shifts. It multiplies by `weightR`: a constant power-of-two multiply is only wiring, whereas an `ap_fixed` right shift keeps the operand's format and drops bits.
  - With `NRMLZ`, `weightR = 2^log2R` is in normalized units. Without it, `weightR` is the equivalent physical weight, 2^`log2R` · ((`YMAXPHYS`−`YMINPHYS`)/(`UMAXPHYS`−`UMINPHYS`))², which makes the two problems differ only by the positive factor g_y² (same minimizer). Use the `*PHYS` (normalization) ranges, not the constraint bounds `YMAX`/`UMAX`.
  - `static_assert`s on `width`/`iwidth` check that each 2^e fits its type (`output_weight_type` in `types.h`, the per-system `input_weight_type` in the system files). Without them an out-of-range weight would silently become 0: before the exponents were introduced, the buck converters had `weightR` = 0 in fixed point.
  - **Current state:** `log2R = 2` is the pendulum's value. The buck converters need −3, so their fixed-point build fails that assert until the exponents become per system.
- **`#ifdef` vs `constexpr`.** Use `#ifdef` only when the variants cannot both compile (different types, array sizes, or symbols that exist in one mode only). When they differ only in a number, use a `constexpr` value in ordinary code (e.g. `DC_STEPS`): both variants are then compiled in every build, and the folded condition gives the same hardware.
- **No `double` in the datapath.** `double` is legal in HLS but becomes floating-point cores (`dadd`, `dmul`, converters). It is fine on the right-hand side of a global constant's initializer and in functions run only at initialization or in the test bench (`initControllerState`, `computeStripCoeffs`, `plant.h`). Physical constants and the constants derived only to compute others (`nrmlz` gains, `dig::yDACGain`/`uADCGain`) are `constexpr double`; what the datapath reads is a `static const` of its `ap_fixed` type, rounded once from the double value (by that type's quantization mode: `AP_TRN` truncates, up to 1 LSB). Checks: disassemble the synthesized objects in a `FIXED` build (only static initializers and `initControllerState` contain floating point), and the Bind Op section of `csynth.rpt` (2026-11-09, BALANCED: only add, sub, mul, shifts, compares, logic, muxes). `double * ap_fixed` does not compile (ambiguous), but assignments and comparisons with a `double` do.
- **Compile-time constants.** Tables derived from the `*Init` arrays are `constexpr` variables built by `constexpr` functions returning a struct (`UNC_MAP`, `DgInit` in `setup.h`). A `constexpr` variable's initializer must be a constant expression, so these are guaranteed compile-time (C++14 has no `consteval`, and Vitis HLS does not support C++20). `ap_fixed` has no `constexpr` constructor, so `ap_fixed` constants are `static const` and initialized dynamically (see below).
- **Interval hull D_G** of the initial zonotope is `DgInit.Dg[i]` (`nTheta` entries, 0 for certain parameters, so divide only over `UNC_MAP` rows). `initControllerState` and `computeStripCoeffs` both use it. `computeStripCoeffs` works in double from the `*Init` arrays and rounds only into the `StripCoeffs` member types. Never route physical θ through `theta_type`: with `NRMLZ` it is sized for the normalized zonotope (`ap_fixed<18,2>` for the pendulum). Doing so used to saturate θ₁ = 2 and zero G₁₁ = −10⁻⁶, giving the fixed-point controller a model a few percent off the float one (fixed in f313012).
- **Header-defined references need `static`.** A namespace-scope `const`/`constexpr` *object* has internal linkage, but a reference does not: `static constexpr const double (&thetaTrueInit)[nTheta] = thetaNominalInit;` (the pendulum) needs the `static`, or every `.cpp` that includes the header defines the same symbol and the link fails. VS Code IntelliSense flags this line ("expression must have a constant value"); g++ and clang++ accept it, and it is a constant expression.
- **`static const ap_fixed` constants** are initialized dynamically. Vitis HLS 2026.1 co-simulation matched C simulation with them. The older toolchain produced constant-zero RTL, fixed on branch `fix/hw-constants-on-e6e7d01`.

## Conventions

- Commit subjects use prefixes like `feat:`, `fix:`, `refactor:`, `perf:`, `chore:`, `wip:`.
- Development happens on feature branches. `vitis2026-pragmas` carries the Vitis 2026.1 work; `main` holds `README.md`.
