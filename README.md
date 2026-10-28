# SCMPC
This repository contains the work for my PhD programme, revolving around ***Scenario-based Model Predictive Control*** without (with ***passive*** and ***active***) learning. In particular, using Vitis 2026.1, this project's aim is to succesfully implement the aforementioned algorithms on FPGA, working in *fixed point* arithmetic (target: xc7z020-clg484-1, $10$ns clock).

A *controller* implemeting MPC strategies generates appropriate inputs for a system - described via an ***AutoRegressive with eXogenous inputs*** (ARX) model - to follow a given reference output trajectory $y_k^*$. The ARX equation describing the system is:

$y_k = a_1\,y_{k-1} + \dots + a_{n_a}\,y_{k-n_a} + b_1\,u_{k-n_d}+\dots+b_{n_b}\,u_{k-n_d-n_b+1} = \varphi_k^\top\,\theta$

where $\varphi_k = [y_{k-1}\;\dots\; y_{k-n_a} \; u_{k-n_d} \; \dots \; u_{k-n_d-n_b+1}]^\top$ and $\theta = [a_1 \; \dots \; a_{n_a} \; b_1 \; \dots \; b_{n_b}]^\top$. Notice $n_d$ is a *delay* between the input and output samples, in this project constrained to be $n_d \geq 1$ - meaning the most "recent" input sample that could contribute to the sample $y_k$ is $u_{k-n_d}$. $n_a, n_b$ are the number of output and input samples needed to compute the next output sample, respectively.

Due to system's parameters uncertainty, the control through nominal MPC could result in suboptimal tracking performance, because the nominal model of the system - known to the controller - could differ from the **true** - or *closest* - one. SCMPC is a form of *robust* MPC which can cope with such uncertainties.

The next sections contain brief summaries about the employed algorithms.

## No learning
### MPC
Nominal MPC uses a model for the system under control to compute $N_y$ (*prediction horizon*) predictions of the output's future behavior. Based on those, a - constrained - optimization problem is solved to optimize the sequence of future $N_u$ (*control horizon*) input samples $u_k, u_{k+1}, \dots, u_{k+N_u-1}$. Define, at instant $k$:

$\Delta u_k \coloneqq u_k - u_{k-1}$

$\Delta y_k \coloneqq y_k - y_{k-1}$

$e_{k+i|k} \coloneqq \hat{y}_{k+i} - y_k^*$

then the optimization problem solved at each time instant $k$ by MPC is:

$
\begin{aligned}
    \min_{u_{k}, \dots, u_{k+N_u-1}} & \sum_{i=1}^{N_u} R \left(\Delta u_{k+i}\right)^2 +\sum_{i=0}^{N_y-1} Q\,e_{k+1+i}^2 \\
    \text{s.t.} \; & y_{k+i+1} = \varphi_{k+i+1}^{\top} \theta, \ i \in \{0,\dots, N_y-1\} \\
    & \underline{y} \leq y_{k+i+1} \leq \overline{y}, \ i \in \{0, \dots, N_y-1\} \\
    & \underline{u} \leq u_{k+i} \leq \overline{u}, \ i \in \{1,\dots, N_u\} \\
    & \underline{\Delta y} \leq \Delta y_{k+i+1} \leq \overline{\Delta y}, \ i \in \{0,\dots, N_y-1\} \\
    & u_{k+i} = u_{k+i-1}, \ i \in \{N_u+1, \ldots, N_y-1\}
\end{aligned}
$

where $R, Q$ are appropriate weight matrices (scalars in this ARX case) - the values don't have a theoretical optimum -, $\underline{(\cdot)}, \overline{(\cdot)}$ are the lower and upper constraints on the relevant variables. Last equation in the problem means input samples are kept unchanged after the control horizon.

From optimal control sequence $u_k^*, \dots, u_{k+N_u-1}^*$, only the first input sample ($u_k^*$) is actually applied to the system. At instant $k+1$, the system will output a new sample $y_{k+1}$ which will be recorded by the controller and used to solve the previous optimization all over again.

The optimization is solved through a *mesh adaptive direct search* (MADS) algorithm, using a *progressive barrier polling* approach to manage constraints without enforcing them as hard ones.

### SCMPC
SCMPC is a form of **robust optimal control** which stems from MPC. It solves a similar optimization problem as before, but also considers different *scenarios*, i.e. realizations of the system's parameters ($\theta$) uncertainty, to impose constraints on. This helps the controller choose a control action that is more likely to produce a contraint-compliant output.

#### Zonotopes
To this end, ***zonotopes*** (a particular class of convex polytopes) are employed as sets for uncertainty representation. They are described by a *center* $c\in\mathbb{R}^{n_a+n_b}$ and a *generator matrix* $G\in\mathbb{R}^{(n_a+n_b)\times n_g}$ whose $n_g$ columns are the *generators*. By definition,

$
\mathcal{Z} = \left\{ \theta \in\mathbb{R}^{n_a + n_b} : \theta = c + G \xi, \, \xi \in [-1, 1]^{n_g} \right\}.
$

These sets make the sampling of scenarios easy and inexpensive: it's sufficient to randomly pick $n_g$ numbers in $[-1, 1]$ and use the zonotope's definition to find a point (i.e. a scenario) inside it.

---

Then, at instant $k$, after sampling $S$ scenarios $\theta^{(j)}, j = 1,\dots,S$ from $\mathcal{Z}$ and defining $\theta^{(0)} \coloneqq c \in \mathcal{Z}$, SCMPC solves the following, new version of the optimization problem:

$
\begin{aligned}
    \min_{u_{k}, \dots, u_{k+N_u-1}} & \sum_{i=1}^{N_u} R \left(\Delta u_{k+i}\right)^2 + \sum_{i=0}^{N_y-1} Q \left(e^{(0)}_{k+1+i}\right)^2 + \sum_{j=1}^S \sum_{i=0}^{N_y-1} \frac{Q}{S} \left(e^{(j)}_{k+1+i}\right)^2  \\
    \textrm{s.t.} \ & y^{(j)}_{k-i} = y_{k-i}, \ i \in \{0,\dots, n_a-1\}\\
    & y^{(j)}_{k+i+1} = \left(\varphi_{k+i+1}^{(j)}\right)^{\top} \theta^{(j)}, \ i \in \{0,\dots, N_y-1\} \\
    & \underline{y} \leq y^{(j)}_{k+i+1} \leq \overline{y}, \ i \in \{0, \dots, N_y-1\} \\
    & \underline{u} \leq u_{k+i} \leq \overline{u}, \ i \in \{1,\dots, N_u\} \\
    & \underline{\Delta y} \leq \Delta y^{(j)}_{k+i+1} \leq \overline{\Delta y}, \ i \in \{0,\dots, N_y-1\} \\
    & u_{k+i} = u_{k+i-1}, \ i \in \{N_u+1, \ldots, N_y-1\}
\end{aligned}
$

for all $j=0,\dots,S$ ($j=0$ corresponding to the nominal system) with trivial meaning for the $(\cdot)^{(j)}$ quantities. Notice that constraints are imposed on all scenarios, making the optimal control sequence more conservative, resulting in a possibly slower tracking performance.

## Passive Learning
Passive learning (PL) consists in using the system's current output sample to *passively* shrink the size of a system's parameters uncertainty. [Algorithms](https://ieeexplore.ieee.org/abstract/document/1652874) that decrease a zonotope's size (in terms of **volume**) exist and are suitable in system control and for hardware implementation.

The optimization problem is exactly the same as in the SCMPC case.

## Active learning
While in passive learning zonotopes are only used to sample scenarios that will be used for the optimization of the control input $u^*_k$, with ***active learning*** (AL) the optimization directly takes into account the volume of the future zonotope - resulting from the use of a certain control action $u_k$ at the current step. In formulas:

$J_A(u_k, \dots, u_{k+N_u-1}) \coloneqq J_P(u_k, \dots, u_{k+N_u-1}) + \alpha V(\hat{\mathcal{Z}}_{k+1})$

where $J_P$ is the cost function optimized for SCMPC and $V(\hat{\mathcal{Z}}_{k+1})$ is the volume of the zonotope $\hat{\mathcal{Z}}_{k+1}$, i.e. the zonotope which would result from the prediction of $y_{k+1}$ computed inside the cost function.

# Past, present, future
This section keeps track of the project's past progress, current state and future work.

## Work history

## Current state
For now, the no-learning SCMPC algorithm has been succesfully implemented with various systems and particularly with an inverted pendulum. It works in both floating and fixed point arithmetic.

## Future work
Currently, the agreed upcoming work is to:
1. Research the state of the art about how active learning is implemented (first of all, following [this author](https://scholar.google.com/citations?user=z0_qJBYAAAAJ&hl=it&oi=ao));
2. Implement *active learning* with **ARX** models and test it;
3. Assess whether it's possible to use state-space form instead of ARX for generality;
4. Improve the definition of the three *pragma profiles*, i.e. optimization `pragma` directives that differ in hardware synthesis:
   1. **Minimum-area:** profile that minimizes resource usage and area occupation;
   2. **Minimum-latency:** profile that enhances performance in terms of latency, trading resource usage;
   3. **Balanced:** trade-off between the previous two.
5. Test the final algorithm in a hardware in the loop (HIL) setup;

---

Some additional tweaks and features that would be needed are:
1. Change name of some variables (e.g. `yHist` $\rightarrow$ `yInit`).
2. Add the sampling time $T_s$ to C.
3. Automatic test setups for C code;
4. Automatic workflow for Vitis fixed point simulations, synthesis and implementation;
5. Automatic derivation of "good" hyperparameters values ($Q, R, \alpha, I, N_y, N_u, S$) for a system (by grid search if no optimal values are theoretically possible, MATLAB or C++);
6. Difference between "true" plant and controller's plant model.
7. Refactor: use structs instead of defines and macros to merge together and set same-topic hyperparameters ($Q, R, \alpha, I, N_y, N_u, S$ number of runs, number of simulation steps, etc.) and the various operation modes' parameters, as well as system constraints and properties.
8. Compute logarithms instead of writing them by hand.
9. Cost function: independent and separated input/output contributions, maybe even separate nominal system (always constrained and tracking) and scenarios.
10. Automatic definition of per-system `pragma`s.
11. Re-order experiments (i.e. attempts and some researches) in appropriate directories.
12. Alignment between MATLAB and C++ (e.g., use `patternsearch` instead of `yalmip` and stop using `CORA` for zonotope representation - it's not really needed).