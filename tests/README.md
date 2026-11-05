# Tests for `cpp/ScenarioMPC_ARX`

Everything is C++: a host tool, `scmpc_test`, builds the controller in the configurations it needs and checks the results. No other language or framework is required, only a C++17 compiler (for the tool itself), git and tar. The fixed-point configurations also need the Xilinx `ap_types` headers.

## Setup

From the repository root:

```bash
g++ -std=c++17 -O2 -o tests/scmpc_test tests/harness/scmpc_test.cpp
export SCMPC_AP_INCLUDE=/path/to/ap_types/include      # or pass --ap <dir>
```

- On Windows, the `g++` that ships with Vitis (or any MinGW g++ ≥ 9) works.
- `tar` is built into Windows 10 and later.
- The `ap_types` headers are in the Vitis installation (`include/`), or in github.com/Xilinx/HLS_arbitrary_Precision_Types.

In VS Code, the same commands are tasks: *Terminal → Run Task…* → `tests: …` (`.vscode/tasks.json`).

## The three checks

### `scmpc_test regress <old> <new>`: nothing changed

This is the byte-identity check from CLAUDE.md, automated.
- `<old>` and `<new>` are git revisions, or `WORKTREE` for the files on disk, uncommitted edits included.
- For every configuration of the matrix, the tool builds both trees, runs the test bench with 3 runs, and compares every output file.
- The matrix: pendulum float/fixed, the three pragma profiles, delay compensation, no scenarios, passive learning, and the buck converter in float.
- Use it for every refactor:

```bash
tests/scmpc_test regress HEAD WORKTREE          # my edits change nothing?
tests/scmpc_test regress main my-branch --quick # 3 configurations only
```

`--ignore-header` compares the files without their header line. Use it for a commit that only renames output columns.

### `scmpc_test unit`: each function does what it should

`tests/unit/unit_tests.cpp` is built against the controller sources, without `testMain.cpp`, in six configurations:
- pendulum float, with and without normalization/conversions;
- pendulum fixed, with and without delay compensation;
- buck float;
- buck float with passive learning.

Each test compares a function with an independent computation in double:

| Test | Property checked |
|---|---|
| `computeArxOutput` | prediction = offset + coeff·regressor |
| `computeArxCoeffs` | the normalized prediction is the normalization of the physical one, with θ = c₀ + D_G·θ̃ |
| `addConstraintViolation` | 0 inside the bounds, the exact excess outside (box and rate) |
| `costFunctionArx` | cost[0] and cost[1] equal a reference implementation of the cost (input term, horizon, weights, scenario mean, delay compensation) |
| `generateScenarios` | every scenario is inside the zonotope; same PRNG state, same scenarios |
| `pseudoRandArx` | range [−1, 1], reproducible after reset, mean and variance of U[−1, 1] |
| `zonotopeVolume` | \|det G\| for a square generator matrix |
| conversions | physical → ADC → normalized and back, against the exact affine maps |
| `MADSARX` | never returns a worse point than its start (lower violation first, then lower cost) |
| passive learning | closed loop with an equation-error ARX plant: the true θ never leaves the zonotope, and the volume never grows |

**Tolerances.**
- In floating point, a function and its reference must agree to rounding.
- In fixed point, each tolerance is built from the LSBs of the types involved, or from the actual error of the stored constants. It is never tuned until a test passes.
- Every test prints the largest error it saw, so an error that grows is visible before it fails.

### `scmpc_test closedloop`: the controller still controls

The tool runs the pendulum test bench, 10 runs by default, in float and in fixed point, and computes:
- the tracking RMSE;
- the number of runs with constraint violations;
- the RMS distance between the fixed-point and the float trajectories (y and u).

It compares them with `tests/closed_loop_limits.txt`. `--record` writes the current values plus a margin (25 %, or one run for violation counts) as the new limits. Record only after a change you have checked and accept. The limits are an alarm for regressions, not a proof of good control.

## Files

- `harness/scmpc_test.cpp`: the tool.
  - It copies the sources into `tests/build/` and edits `setup.h`/`testbench_config.h` there; the repository files are never modified.
  - Every edited setting must match exactly one line, otherwise the tool stops.
- `unit/unit_tests.cpp`: the unit tests, with a minimal `CHECK` macro and no external framework.
- `closed_loop_limits.txt`: the closed-loop acceptance limits.
