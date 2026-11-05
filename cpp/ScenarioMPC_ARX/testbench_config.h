/* ======================================================================
   EXPERIMENT SETTINGS - edit these
   ====================================================================== */
#pragma once
namespace testbench_settings
{
    /** Number of closed-loop simulations. 1 = the classic single run, written
     *  to output.txt (plotOutputCpp.m); more = one file per run plus a summary
     *  (plotMultipleOutputsCpp.m). */
    constexpr int nRuns = 10;

    /**
     * Number of simulations steps in a single run. 
     */
    constexpr int nSim = 500;

    /** false: true plant = thetaTrueInit (system_configs.h), the same every run.
     *  true: a new true plant per run, drawn by plantSampleTrue(). */
    constexpr bool randomTruePlant = true;

    /** Seed of the test-bench random generator, which draws the true plants
     *  and the measurement noise. It is separate from the controller's own
     *  generator (pseudoRand.cpp): the controller consumes random numbers
     *  differently with and without scenarios, so sharing one generator would
     *  give the two configurations DIFFERENT true plants and noise, and the
     *  comparison would not be fair. With this one, the same seed gives the
     *  same plants and noise to every controller configuration. */
    constexpr unsigned int testbenchSeed = 1u;

    /** Controller latency dt emulated by the test bench, as a fraction of the
     *  sampling period (0 <= dt/Ts < 1). Without DELAY_COMPENSATION (setup.h)
     *  the input computed at instant k takes effect at k*Ts + dt: the plant
     *  keeps the previous input for the first dt of the interval, as a real
     *  DAC would. 0 is the ideal, instantaneous controller (the original test
     *  bench). With DELAY_COMPENSATION every input is applied at the sampling
     *  instant after the one it was computed at, so dt plays no role as long
     *  as it is below Ts. The inverted pendulum's nonlinear plant applies the
     *  switch inside its Euler sub-steps; ARX plants, which only know one input
     *  per interval, get the interval's average, (dt/Ts)*u_old + (1-dt/Ts)*u_new. */
    constexpr double computeDelayFraction = 0.0;
} // namespace testbench_settings

static_assert(testbench_settings::computeDelayFraction >= 0.0 && testbench_settings::computeDelayFraction < 1.0,
    "testbench_settings::computeDelayFraction must be in [0, 1): a latency of a full period or more misses the next sample.");