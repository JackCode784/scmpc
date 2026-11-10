# SCMPC
This repository contains the work for my PhD programme, revolving around ***Scenario-based Model Predictive Control*** without (with ***passive*** and ***active***) learning. In particular, using Vitis 2026.1, this project's aim is to successfully implement the aforementioned algorithms on FPGA, working in *fixed point* arithmetic (target: xc7z020-clg484-1, `10ns` clock period).

A *controller* implementing MPC strategies generates appropriate inputs for a system - described via an ***AutoRegressive with eXogenous inputs*** (ARX) model - to follow a given reference output trajectory $y_k^*$. The ARX equation describing the system is:

```math
y_k = a_1\ y_{k-1} + \dots + a_{n_a}\ y_{k-n_a} + b_1\ u_{k-n_d}+\dots+b_{n_b}\ u_{k-n_d-n_b+1} = \varphi_k^\top\ \theta
```

where $\varphi_k = [y_{k-1}\ \dots\  y_{k-n_a} \  u_{k-n_d} \  \dots \  u_{k-n_d-n_b+1}]^\top$ and $\theta = [a_1 \  \dots \  a_{n_a} \  b_1 \  \dots \  b_{n_b}]^\top$. Notice $n_d$ is a *delay* between the input and output samples, in this project constrained to be $n_d \geq 1$ - meaning the most "recent" input sample that could contribute to the sample $y_k$ is $u_{k-n_d}$. $n_a, n_b$ are the number of output and input samples needed to compute the next output sample, respectively.

Due to system's parameters uncertainty, the control through nominal MPC could result in suboptimal tracking performance, because the nominal model of the system - known to the controller - could differ from the **true** - or *closest* - one. SCMPC is a form of *robust* MPC which can cope with such uncertainties.

The next sections contain brief summaries about the employed algorithms.

## No learning
### MPC
Nominal MPC uses a model for the system under control to compute $N_y$ (*prediction horizon*) predictions of the output's future behavior. Based on those, a - constrained - optimization problem is solved to optimize the sequence of future $N_u$ (*control horizon*) input samples $u_k, u_{k+1}, \dots, u_{k+N_u-1}$. Define, at instant $k$:

```math
\Delta u_k := u_k - u_{k-1}
```

```math
\Delta y_k := y_k - y_{k-1}
```

```math
e_{k+i|k} := \hat{y}_{k+i} - y_k^*
```

then the optimization problem solved at each time instant $k$ by MPC is:

```math
\begin{aligned}
    \min_{u_{k}, \dots, u_{k+N_u-1}} & \sum_{i=0}^{N_u-1} R \left(\Delta u_{k+i}\right)^2 +\sum_{i=1}^{N_y} Q\ e_{k+i|k}^2 \\
    \text{s.t.} \ & \hat{y}_{k+i} = \varphi_{k+i}^{\top} \theta, \ i \in \{1,\dots, N_y\} \\
    & \underline{y} \leq \hat{y}_{k+i} \leq \overline{y}, \ i \in \{1, \dots, N_y\} \\
    & \underline{u} \leq u_{k+i} \leq \overline{u}, \ i \in \{0,\dots, N_u-1\} \\
    & \underline{\Delta y} \leq \Delta y_{k+i} \leq \overline{\Delta y}, \ i \in \{1,\dots, N_y\} \\
    & u_{k+i} = u_{k+i-1}, \ i \in \{N_u, \ldots, N_y-1\}
\end{aligned}
```

where $R, Q$ are appropriate weight matrices (scalars in this ARX case) - the values don't have a theoretical optimum -, $\underline{(\cdot)}, \overline{(\cdot)}$ are the lower and upper constraints on the relevant variables. Last equation in the problem means input samples are kept unchanged after the control horizon.

From optimal control sequence $u_k^*, \dots, u_{k+N_u-1}^*$, only the first input sample ($u_k^*$) is actually applied to the system. At instant $k+1$, the system will output a new sample $y_{k+1}$ which will be recorded by the controller and used to solve the previous optimization all over again.

The optimization is solved through a *mesh adaptive direct search* (MADS) algorithm, using a *progressive barrier polling* approach to manage constraints without enforcing them as hard ones.

### SCMPC
SCMPC is a form of **robust optimal control** which stems from MPC. It solves a similar optimization problem as before, but also considers different *scenarios*, i.e. realizations of the system's parameters ($\theta$) uncertainty, to impose constraints on. This helps the controller choose a control action that is more likely to produce a constraints-compliant output.

#### Zonotopes
To this end, ***zonotopes*** (a particular class of convex polytopes) are employed as sets for uncertainty representation. They are described by a *center* $c\in\mathbb{R}^{n_a+n_b}$ and a *generator matrix* $G\in\mathbb{R}^{(n_a+n_b)\times n_g}$ whose $n_g$ columns are the *generators*. By definition,

```math
\mathcal{Z} = \bigl\{ \theta \in\mathbb{R}^{n_a + n_b} : \theta = c + G \xi, \ \xi \in [-1, 1]^{n_g} \bigr\}.
```

These sets make the sampling of scenarios easy and inexpensive: it's sufficient to randomly pick $n_g$ numbers in $[-1, 1]$ and use the zonotope's definition to find a point (i.e. a scenario) inside it.

---

Then, at instant $k$, after sampling $S$ scenarios $\theta^{(j)}, j = 1,\dots,S$ from $\mathcal{Z}$ and defining $\theta^{(0)} := c \in \mathcal{Z}$, SCMPC solves the following, new version of the optimization problem:

```math
\begin{aligned}
    \min_{u_{k}, \dots, u_{k+N_u-1}} & \sum_{i=0}^{N_u-1} R \left(\Delta u_{k+i}\right)^2 + \sum_{i=1}^{N_y} Q \left(e^{(0)}_{k+i|k}\right)^2 + \sum_{j=1}^S \sum_{i=1}^{N_y} \frac{Q}{S} \left(e^{(j)}_{k+i|k}\right)^2  \\
    \textrm{s.t.} \ & y^{(j)}_{k-i} = y_{k-i}, \ i \in \{0,\dots, n_a-1\}\\
    & y^{(j)}_{k+i} = \left(\varphi_{k+i}^{(j)}\right)^{\top} \theta^{(j)}, \ i \in \{1,\dots, N_y\} \\
    & \underline{y} \leq y^{(j)}_{k+i} \leq \overline{y}, \ i \in \{1, \dots, N_y\} \\
    & \underline{u} \leq u_{k+i} \leq \overline{u}, \ i \in \{0,\dots, N_u-1\} \\
    & \underline{\Delta y} \leq \Delta y^{(j)}_{k+i} \leq \overline{\Delta y}, \ i \in \{1,\dots, N_y\} \\
    & u_{k+i} = u_{k+i-1}, \ i \in \{N_u, \ldots, N_y-1\}
\end{aligned}
```

for all $j=0,\dots,S$ ($j=0$ corresponding to the nominal system) with trivial meaning for the $(\cdot)^{(j)}$ quantities. Notice that constraints are imposed on all scenarios, making the optimal control sequence more conservative, resulting in a possibly slower tracking performance.

### Delay compensation
The optimization described up to this point was employed until, at a certain point of the project, a *delay compensation* strategy was implemented as an additional mode of operation for the algorithm.

Delay compensation changes the previous problems - both with and without scenarios - such that instead of optimizing $u_k$ at instant $k$, the controller starts with it as an initial condition, and at each time step $k$ it optimizes $u_{k+1}$ instead.

This was done because, in real-time use, at instant $k$ (absolute time $k\ T_s$), after receiving output measurement $y_k$, the optimal input $u_k^*$ would be ready only after the controller's computation latency $\Delta t$ and thus would be applied only at absolute time $k \  T_s + \Delta t$ instead of, in the ideal case, $k\ T_s$. In that $\Delta t$ time, though, the system state changed and such optimal input $u_k^*$ would now be sub-optimal due to a already-updated system state - the shorter $\Delta t$, the less difference between system state at absolute time $k\ T_s$ and $k\ T_s + \Delta t$.

## Passive Learning
Passive learning (PL) consists in using the system's current output sample to *passively* shrink the size of a system's parameters uncertainty. [Algorithms](https://ieeexplore.ieee.org/abstract/document/1652874) that decrease a zonotope's size (in terms of **volume**) exist and are suitable in system control and for hardware implementation.

The optimization problem is exactly the same as in the SCMPC case.

## Active learning
While in passive learning zonotopes are only used to sample scenarios that will be used for the optimization of the control input $u^*_k$, with ***active learning*** (AL) the optimization directly takes into account the volume of the future zonotope - resulting from the use of a certain control action $u_k$ at the current step. In formulas:

```math
J_A(u_k, \dots, u_{k+N_u-1}) := J_P(u_k, \dots, u_{k+N_u-1}) + \alpha V(\hat{\mathcal{Z}}_{k+1})
```

where $J_P$ is the cost function optimized for SCMPC and $V(\hat{\mathcal{Z}}_{k+1})$ is the volume of the zonotope $\hat{\mathcal{Z}}_{k+1}$, i.e. the zonotope which would result from the prediction of $y_{k+1}$ computed inside the cost function.

# Getting started

## Repository structure
| Path | Content |
|---|---|
| `cpp/ScenarioMPC_ARX/` | The controller (active project). The same C++ sources are the PC simulation (floating point) and the Vitis HLS design (fixed point). |
| `vitis/scmpc-controller2026/` | Vitis 2026.1 HLS component: `hls_config.cfg` (part, clock, top function `controller`, source files) and `vitis-comp.json`. Build outputs are not tracked. |
| `cpp/MADS_ISCAS23/` | Earlier, separate MADS-based MPC HLS design. |
| `matlab/` | Reference and prototype implementations (zonotopes, passive learning, SCMPC). They need the CORA toolbox, YALMIP and `quadprog`. |

Main files in `cpp/ScenarioMPC_ARX/`:
- `setup.h`: experiment configuration and algorithm parameters.
- `system_configs.h`: the controller's model of each system (nominal parameters, uncertainty zonotope, constraints).
- `types.h`: fixed-point data types, per system.
- `controller.cpp`: the top-level function synthesized to hardware.
- `costFunctionArx.cpp`, `MADSARX.cpp`, `progressiveBarrierPollingArx.cpp`: cost evaluation and the MADS optimizer.
- `testMain.cpp`, `plant.h` and `testbench_config.h`: test bench only; `plant.h` holds the *true* plant the controller is tested against (e.g. the nonlinear inverted pendulum), `testbench_config.h` the experiment settings.

## Configuring an experiment
Experiments are configured by editing the headers (`#define` mode flags and `constexpr` constants); no compiler flags are needed.

| File | Setting | Meaning |
|---|---|---|
| `setup.h` | `FIXED` | fixed point (hardware) instead of floating point |
| | `CONVERSIONS_MODE`, `NRMLZ` | ADC/DAC conversions, normalization (the usual hardware configuration is `FIXED` + `CONVERSIONS_MODE` + `NRMLZ`; the inverted pendulum in fixed point requires `NRMLZ`) |
| | `USE_SCENS_COST`, `USE_SCENS_CONSTR` | scenarios in the cost and in the constraints (both off: nominal MPC) |
| | `DELAY_COMPENSATION` | the input computed at step $k$ is applied at $k+1$ |
| | `ACTIVE_SYSTEM`, `CTRL_MODE` | system; SCMPC, passive or active learning |
| | `PRAGMA_PROFILE` | hardware profile: `AREA`, `BALANCED` or `LATENCY` |
| | `Nhor`, `NhorU`, `Nscen`, `MADS_ITER` | $N_y$, $N_u$, $S$ (a power of 2, with `log2Nscen`), $I$ |
| `testbench_config.h` | `nRuns`, `randomTruePlant`, `testbenchSeed` (namespace `testbench_settings`) | number of closed-loop runs, a new random true plant per run, test-bench seed |
| | `computeDelayFraction` | controller latency emulated by the test bench, as a fraction of the sampling period |
| `plant.h` | `PENDULUM_NONLINEAR` | the pendulum's true plant: nonlinear equation (1) or its ARX model (0) |

## Running a simulation
Any C++14 compiler works. From `cpp/ScenarioMPC_ARX/`:
```bash
# floating point
g++ -std=c++14 -O2 -I . -o sim *.cpp && ./sim
# fixed point: also needs the Xilinx ap_types headers (shipped with Vitis, or
# github.com/Xilinx/HLS_arbitrary_Precision_Types) and Vitis's hls_math.h
g++ -std=c++14 -O2 -I <vitis_include_dir> -I . -o sim *.cpp && ./sim
```
`passiveLearning/` holds unused code and is not part of the build.

The test bench simulates the controller in closed loop and prints, for each run, the constraint violations and the tracking RMSE. Outputs:
- floating point: in `FloatSimulations/`; fixed point: in the working directory (in Vitis, `.../hls/csim/build`);
- `nRuns = 1`: `output.txt`, plotted by `plotOutputCpp.m`;
- `nRuns > 1`: one `runs_<ctrl>_<plant>_<run>.txt` per run plus `runs_<ctrl>_<plant>_summary.txt`, plotted by `plotMultipleOutputsCpp.m`. `<ctrl>` tells the controller configuration (e.g. `scen`, `noscen`, with `_dc` for delay compensation) and `<plant>` the true plant (`nl` nonlinear, `arx`).

In both MATLAB scripts, `opts.isVitisSim = true` reads the Vitis C-simulation folder instead of `FloatSimulations/`.

## Hardware synthesis with Vitis
1. Open the component `vitis/scmpc-controller2026` in Vitis 2026.1. `hls_config.cfg` lists the source files by absolute path: adjust them to your clone.
2. Set `FIXED` (and the other hardware options) in `setup.h`.
3. **C simulation**: runs the test bench in fixed point and writes `golden_u.txt`, the inputs applied in the first run.
4. **Synthesis**: `csynth.rpt` reports latency, resources and the estimated clock period. The estimate must stay within the clock period minus the uncertainty margin (7.3 ns for a 10 ns clock with the default 27 % uncertainty).
5. **C/RTL co-simulation**, with `nRuns = 1` in `testbench_config.h`: the test bench compares the RTL's inputs with `golden_u.txt`, so run C simulation first. Vitis's "Pass" only means that `main()` returned 0; the log line `CO-SIM CHECK PASSED` confirms that the RTL matches C simulation.
6. **Implementation**: post-route timing and resources, the final word on whether the design meets the clock.

## Checking a change
There is no unit-test suite. A change that should not alter the algorithm (a refactor, a pragma, a hardware optimization) must leave the C simulation output **byte-identical**: run the test bench before and after the change, with the same configuration, and compare the output files. Pragmas never affect C simulation.

# Past, present, future
This section keeps track of the project's past progress, current state and future work.

## Work history
The project started from the work already done and kept inside the `MADS_ISCAS23` directory. The MADS algorithm, cost, and progressive barrier polling functions were already available for use but have been adapted for ARX models.

MATLAB code has been developed to test the algorithm (no learning and passive learning) in a high-level and versatile environment. After that, C/C++ code has been developed starting from the MATLAB one. 

In time, some experiments and tests have also been run in MATLAB to check some properties and invent some algorithms that could prove useful. For instance, initially the passive learning zonotope update [algorithm](https://www.sciencedirect.com/science/article/pii/S0005109805000294) added one generator to $G$, which required a *reduction* of the zonotope (i.e. overapproximation with a bigger zonotope but with a smaller number of generators $n_g$). A [PCA-based technique](https://www.sciencedirect.com/science/article/pii/S000510981830298X) was used at first to this end, but it was found that the outer zonotope results way too big and that the overapproximation error grows as $e^{n^2}$.

After that, a new algorithm for the zonotope update was found [here](https://ieeexplore.ieee.org/abstract/document/1652874), surpassing the need to use any reduction technique at all. This is the algorithm implemented both in MATLAB and C.

Another experiment has been made to find the *radius* $r(\mathcal{Z})$ of a zonotope, i.e. the maximum distance between a point in the zonotope and its center:

```math
r(\mathcal{Z}) := \max_{z\in\mathcal{Z}} ||z-c||_2.
```

An algorithm was proposed and tested in MATLAB, but it was not successful, but some more work could head in that direction if the active learning implementation using the predicted zonotope's volume $V(\hat{\mathcal{Z}}_{k+1})$ proved computationally expensive.

Some research has been developed on [*constrained zonotopes*](https://www.sciencedirect.com/science/article/pii/S0005109816300772), [*zonotope bundles*](https://ieeexplore.ieee.org/iel5/6149620/6159299/06160872.pdf) and other polytopic sets as more versatile substitutes for zonotopes, but they have not been considered nor used yet.

Afterwards, two *normalization* processes have been implemented both in MATLAB and C code to manage system-specific fixed point bit widths assertions. In particular, one normalization was performed on input/output samples such that they always fall in the interval $[-1, 1]$:

```math
N_y : [\underline{y}, \overline{y}] \leftrightarrow [-1, 1], \quad \tilde{y}=N_y(y) = m_y \ y + q_y
```

```math
N_u : [\underline{u}, \overline{u}] \leftrightarrow [-1, 1], \quad \tilde{u}=N_u(u) = m_u \ u + q_u
```

where $m_y, m_u, q_y, q_u$ are appropriate coefficients computed directly from the input/output constraints:

```math
m_x := \frac{\overline{\tilde{x}}-\underline{\tilde{x}}}{\overline{x}-\underline{x}}
```

```math
q_x := \frac{\underline{\tilde{x}}\ \overline{x} - \overline{\tilde{x}}\ \underline{x}}{\overline{x}-\underline{x}}
```

where $\underline{\tilde{x}}, \overline{\tilde{x}}$ are the interval extremes of the normalized interval, in this case $-1, 1$ respectively. Using these the code employs 

```math
\tilde{\varphi}_k := [\tilde{y}_{k-1} \dots \tilde{u}_{k-n_d-n_b+1}]^\top = D_m \ \varphi_k + q
```

```math
D_m := \begin{bmatrix} m_y I_{n_a} & 0\\ 0 & m_u I_{n_b} \end{bmatrix}, \quad q := \begin{bmatrix}
    q_y \mathbf{1}_{n_a} \\ q_u \mathbf{1}_{n_b}
\end{bmatrix}
```

for output predictions in the optimization problem.

To get a similar result for the zonotopes, some of their properties have been researched and used so that the normalized zonotope $\tilde{\mathcal{Z}} = N_\theta(\mathcal{Z})$ is contained in the hypercube $[-1, 1]^n$. In particular, one can define the *interval hull* of a zonotope $\mathcal{Z} = (c | G)$ as

```math
\text{IH}(\mathcal{Z}) = (c | D_G), \quad D_G := \text{diag}(d_1, \dots, d_n), \quad d_i = \sum_{j=1}^{n_g} |G_{ij}|
```

which is the smallest hyperrectangle containing $\mathcal{Z}$. Then we define

```math
\tilde{\mathcal{Z}} := D_G^{-1}(\mathcal{Z} - c) = (0 | D_G^{-1}G)
```

which is the *normalized* zonotope, contained by construction in $[-1,1]^n$.

It has been proved that the zonotope normalization policy and the update algorithm currently used in the codebase *commute*, i.e. the update of the normalized zonotope *is* the normalization of the updated non-normalized one. This means that the controller can make all computations on the normalized zonotope, update it, and still get a new normalized zonotope which is exactly the normalized version of the zonotope it would have got if the algorithm was run on the unnormalized one.

Both MATLAB and C code can be instructed to (not) use both normalization policies if desired. In case one does use them, the controller's output predictions (optimization problem, zonotope update) have been adapted to them. Everything inside the controller uses normalized quantities, the output predictions inside the controller are already computed in the normalized domain, the zonotope computations and update are all applied to and return normalized zonotopes; the optimization problem is also modified so a "normalized" version of it is solved, with the only difference with respect to the unnormalized case being in the input weight matrix $R$ which becomes instead $(\frac{m_y}{m_u})^2 R$.

The controller can also mimic (with a flag giving the possibility to turn this feature off) the behavior of ADC and DAC converters: in input, it receives some digital values for $y_k, y_k^*$, while in output it returns the denormalized, digital version of $\tilde{u}_k$. This process includes quantization errors due to the float-to-integer conversions. Based on the use of normalization and/or AD/DA conversions, the controller automatically computes appropriate coefficients $\hat{m}_y, \hat{m}_u, \hat{q}_y, \hat{q}_u$ to convert directly from digital (normalized) values to normalized (digital) ones for $y_k, y_k^*$ ($\tilde{u}_k$). This means that the controller never sees the unnormalized values for input/output samples, but only the digital ones (which are due to the AD/DA conversions) and the normalized ones (because everything inside the controller is normalized). In-between cases (no normalization, yes conversions and viceversa) are dealt with similarly.

One of the last progresses has been the definition of three different *profiles* for Vitis' `pragma` directives. These modify *how* the algorithm is implemented in hardware, not *what* it does. The three available profiles tend to minimize area and resource usage (`PRAGMA_PROFILE_AREA`), latency (`PRAGMA_PROFILE_LATENCY`), or give a trade-off between the two (`PRAGMA_PROFILE_BALANCED`). Some work still has to be done for improving these profiles since recently it has been observed Vitis ignores some of the pragma directives, making the differences between the three profiles negligible. Some fixes have already been implemented in this direction, but some more work is due.

Lastly, a *delay compensation* strategy (previous sections) has been implemented as an additional feature built on top of all operation modes already present in the design.

## Current state
For now, the no-learning SCMPC algorithm has been successfully implemented with various systems and particularly with an inverted pendulum. It works in both floating and fixed point arithmetic.

## Future work
Currently, the agreed upcoming work is to:
1. Research the state of the art about how active learning is implemented (first of all, following [this author](https://scholar.google.com/citations?user=z0_qJBYAAAAJ&hl=it&oi=ao) and [Bemporad](https://scholar.google.com/citations?user=XC3nc68AAAAJ&hl=it&oi=sra));
2. Implement *active learning* with **ARX** models and test it;
3. Assess whether it's possible to use state-space form instead of ARX for generality;
4. Improve the definition of the three *pragma profiles*, i.e. optimization `pragma` directives that differ in hardware synthesis:
   1. **Minimum-area:** profile that minimizes resource usage and area occupation;
   2. **Minimum-latency:** profile that enhances performance in terms of latency, trading resource usage;
   3. **Balanced:** trade-off between the previous two.
5. Test the final algorithm in a hardware in the loop (HIL) setup;

---

Some additional tweaks and features that would be needed are:
- [x] Change name of some variables (e.g. `yHist` $\rightarrow$ `yHistCtrl`).
- [x] Add the sampling time $T_s$ to C.
- [ ] Automatic test setups for C code;
- [ ] Automatic workflow for Vitis fixed point simulations, synthesis and implementation;
- [x] Automatic derivation of "good" hyperparameters values ($Q, R, \alpha, I, N_y, N_u, S$) for a system (by grid search if no optimal values are theoretically possible, MATLAB or C++);
- [x] Scale definition of new systems (e.g. every system could describe a `struct/namespace` with constraints, dimensions, noise, initial conditions, initial zonotope, sampling time, everything in `system_configs.h`). 
- [x] Give the possibility, for each system, to define a "true" plant and the controller's ARX model (already done with the pendulum).
- [x] Refactor: use `struct/namespace` instead of defines and macros to merge together and set same-topic hyperparameters ($Q, R, \alpha, I, N_y, N_u, S$, number of simulation steps, etc.) and the various operation modes' parameters, as well as system constraints and properties.
- [x] Compute logarithms instead of writing them by hand.
- [x] Cost function: independent and separated input/output contributions
- [ ] Separate nominal system (always constrained and tracking) and scenarios contributions, so that only scenarios cost/constraint violations can be turned on and off, with `#ifdef`s if more Vitis-friendly.
- [ ] Automatic definition of per-system `pragma`s.
- [x] Re-order experiments (i.e. attempts and some researches) in appropriate directories.
- [ ] Alignment between MATLAB and C++ (e.g., use `patternsearch` instead of `yalmip` and stop using `CORA` for zonotope representation - it's not really needed).
- [x] Use `constexpr` instead of `#define` for the system dimensions (`na`, `nb`, `nd` (formerly `nk`), `nGens`, `nTheta`, `nOpt`): they are scoped and typed, and give the same hardware. Only what the preprocessor itself must see stays a macro: `ACTIVE_SYSTEM` and the system ids, the mode flags, and the arguments of `HLS_PRAGMA(...)`.
- [x] Write the per-system initial values (`THETA_NOMINAL_UNNORM`, `GENERATORS_UNNORM`, `THETA_TRUE_INIT`, `Y_HIST_UNNORM`, `U_HIST_UNNORM`) as `constexpr double` arrays, read directly by `initControllerState` and `plant.h`. This removes the macro lists and the `*_TABLE` copies in `setup.h`, and a `static_assert` can check each array's length. Do it together with the renaming of point 1.
- [x] Derive the initial warm start (`U_PREV_UNNORM`) from the initial input history instead of giving it per system: it must be the same operating point anyway, and its length `NhorU` is a tuning parameter, not a property of the system.
- [x] Per-system tuning parameters ($N_y, N_u, S, I$ and the weight exponents `log2Q`, `log2P`, `log2R`) in their own file (e.g. `tuning.h`), separate from the system models in `system_configs.h`, with a global default. Per-system `log2R` is also what the buck converters' fixed-point build needs.
- [x] Compile-time computation of $D_G$ and keep it as a separate variable (diagnostics, denormalized zonotopes over time,...).