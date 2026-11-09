/**
 * @file  hyperparams.h
 * @brief THE list of the controller's tunable hyperparameters, in one place.
 *
 * Every hyperparameter is one line of SCMPC_HYPERPARAMS below:
 *
 *     X(namespace, name, value, "description")
 *
 * The same list is expanded twice:
 *   - here, into the definitions  namespace ns { constexpr int name = value; }
 *     (the rest of the code uses them qualified: mpc::predHrzn, mads::iter, ...);
 *   - in testMain.cpp, into a file that records every value of a run
 *     (printHyperparams), so the outputs always say which values made them.
 * A new hyperparameter is therefore one new line, and it cannot be defined
 * without also being logged. (This "X-macro" idiom: the list macro takes
 * another macro, X, and applies it to every entry.)
 *
 * What derives from these values (nScen = 2^log2nScen, the weights 2^log2*,
 * householderClamp = 2^-gamma, the static_asserts) stays in setup.h.
 *
 * Rules for the list:
 *   - every value is an int (exponents and counts);
 *   - a line ends with "\" (the macro continues): comments inside the list
 *     must be block comments, a // comment would swallow the next line;
 *   - one entry per line, in this exact form: tools that change a value
 *     (the grid search) rewrite the line that matches "X(ns, name,".
 *
 * Everything here may change the RTL: the horizons, nScen and iter size the
 * arrays and loops of the hardware.
 */
#pragma once

#define SCMPC_HYPERPARAMS(X)                                                                         \
    /* MPC (README: N_y, N_u, Q, P, R) */                                                            \
    X(mpc,  predHrzn,     15, "prediction horizon N_y [steps]")                                      \
    X(mpc,  ctrlHrzn,      3, "control horizon N_u <= N_y [steps]: u held constant after it")        \
    X(mpc,  log2Q,         4, "stage output weight Q = 2^log2Q")                                     \
    X(mpc,  log2P,         4, "terminal output weight P = 2^log2P")                                  \
    X(mpc,  log2R,         2, "input-increment weight R = 2^log2R (normalized units with NRMLZ)")    \
    /* Scenarios (README: S) */                                                                      \
    X(scen, log2nScen,     4, "number of scenarios S = 2^log2nScen, nominal model excluded")         \
    /* MADS (Ravera et al.: I, gamma, -tau) */                                                       \
    X(mads, iter,         10, "MADS iterations I per controller call")                               \
    X(mads, frameExpStep,  1, "frame exponent step -tau > 0: +step on success, -step on failure")    \
    X(mads, gamma,         1, "Householder scale 2^gamma; entries clamped to 2^-gamma (gamma >= 0)") \
    X(mads, frameExpMin, -20, "minimum frame exponent")                                              \
    X(mads, d0,           -8, "initial frame exponent: initial frame size 2^d0")

#define SCMPC_HP_DEFINE(ns, name, value, description) namespace ns { constexpr int name = value; }
SCMPC_HYPERPARAMS(SCMPC_HP_DEFINE)
#undef SCMPC_HP_DEFINE
