# Grid search over the controller's hyperparameters

`grid_search` runs the closed-loop test bench of `cpp/ScenarioMPC_ARX` for every combination of hyperparameter values in a grid file, and writes one CSV row per combination. `plotGridSearch.m` plots the CSV.

The hyperparameters are the entries of `cpp/ScenarioMPC_ARX/hyperparams.h`: `mpc::predHrzn`, `mpc::ctrlHrzn`, `mpc::log2Q`, `mpc::log2P`, `mpc::log2R`, `scen::log2nScen`, `mads::iter`, `mads::frameExpStep`, `mads::gamma`, `mads::frameExpMin`, `mads::d0`.

## Usage

From the repository root:

```bash
g++ -std=c++17 -O2 -pthread -o gridsearch/grid_search gridsearch/grid_search.cpp
gridsearch/grid_search gridsearch/example_grid.txt --jobs 4
```

Then, in MATLAB:

```matlab
T = plotGridSearch('results/example_grid.csv');
```

Options:
- `--jobs N`: N grid points in parallel (one per CPU core is a good choice).
- `--out file.csv`: the results file; by default `gridsearch/results/<grid file name>.csv`.
- `--keep`: keep the build directory of every point, not only of the failed ones.
- `--cxx <compiler>`: the compiler for the controller builds (default `g++`).

On Windows, the `g++` that ships with Vitis (or any MinGW g++ ≥ 9) builds both the tool and the controller.

## Grid file

One setting per line; `#` starts a comment. A setting that is not given keeps its value in the sources.

| Line | Meaning |
|---|---|
| `system SYSTEM_INVERTED_PENDULUM` | `ACTIVE_SYSTEM` in `setup.h` |
| `flag FIXED off` | a mode flag of `setup.h` (`FIXED`, `NRMLZ`, `CONVERSIONS_MODE`, ...): `on` or `off` |
| `runs 10` | `testbench_settings::nRuns`, at least 2 (the summary file exists only then) |
| `seed 1` | `testbench_settings::testbenchSeed`: the same plants and noise for every grid point |
| `param mpc::predHrzn 10 15 20` | a hyperparameter and the values to try |
| `ap_include <dir>` | the `ap_types` headers, needed with `FIXED on` |

The grid is the Cartesian product of the `param` lines: 3 × 2 × 2 = 12 points for `example_grid.txt`.

## What one grid point does

1. Copies the sources (the top-level `.cpp`/`.h` files and `systems/`) into `gridsearch/build/<grid name>/point_<k>/`. The repository files are never modified.
2. Edits the copy:
   - `hyperparams.h`: the line `X(ns, name, value, ...)` of every swept parameter;
   - `setup.h`: `ACTIVE_SYSTEM` and the flags;
   - `testbench_config.h`: `nRuns` and `testbenchSeed`.

   Every edit must match exactly one line. The tool checks all of them on a scratch copy before it starts, so a typo stops it instead of silently testing the default.
3. Builds with `g++ -std=c++14 -O2` and runs the test bench.
4. Reads the test bench's `*_summary.txt` and appends a row to the CSV.

A combination that the code rejects (for example `mpc::ctrlHrzn` too long for `mpc::predHrzn`: a `static_assert` in `setup.h`) is a row with status `build_failed` and the first error line. Failed points keep their build directory (`build.log`, `run.log`).

Each point also writes the test bench's `*_hyperparams.txt` next to its outputs, the values that point actually used.

## CSV columns

| Column | Meaning |
|---|---|
| `point` | index of the combination (the last `param` line varies fastest) |
| one column per `param` | its value |
| `status`, `message` | `ok`, or `build_failed` / `run_failed` / `error` with the reason |
| `runs` | number of closed-loop runs |
| `meanRmse`, `maxRmse` | tracking RMSE over the runs: mean and worst |
| `runsViolY`, `runsViolDy` | runs with at least one sample outside [yMin, yMax], with abs(Δy) > deltaY |
| `maxViolY`, `maxViolDy` | largest violation over all runs |
| `estCyclesBalanced`, `estLatencyUs` | latency estimate of one `controller()` call, see below |
| `Ts`, `latencyFitsTs` | the sampling time (from the output files) and whether the estimate is below it (1/0) |

## The latency estimate

A formula of the BALANCED profile's loop structure, calibrated on one `csynth.rpt`: the pendulum with $N_y = 15$, $N_u = 3$, $S = 16$, $I = 10$, 34208 cycles, which the formula matches exactly. In clock cycles of 10 ns:

$$
\begin{aligned}
c_\text{eval} &= N_y\,(S + 1 + 16) + 7 && \text{costFunctionArx: } S+1 \text{ models, II = 1}\\
c_\text{iter} &= 2N_u\,(c_\text{eval} + 4) + 55 && \text{one MADS iteration: } 2N_u \text{ poll points}\\
c &= (I + 1)\,c_\text{iter} + 1 + (11S + 1) + (S + 6) + 7 && \text{MADSARX, scenarios, coefficients, I/O}
\end{aligned}
$$

For other values it extrapolates that structure; it is not a synthesis result. It does not model area (DSPs, LUTs), and it is wrong for the AREA and LATENCY profiles. Check the chosen points with a real synthesis.

## Notes

- Every grid point is a separate build: the horizons and $S$ size arrays, so they are compile-time constants.
- The same `seed` gives every grid point the same true plants and noise, so differences between points come from the hyperparameters.
- `plotGridSearch.m` is written for MATLAB R2019b or later (data tips, `xline`). It has not been run: no MATLAB was available where it was written.
