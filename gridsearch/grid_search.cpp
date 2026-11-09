/**
 * @file  grid_search.cpp
 * @brief Grid search over the hyperparameters of cpp/ScenarioMPC_ARX.
 *
 * For every combination of the values listed in a grid file, the tool
 *   1. copies the controller sources (cpp/ScenarioMPC_ARX, systems/ included)
 *      into a build directory of its own;
 *   2. edits the copy: the hyperparameters in hyperparams.h, the system and
 *      mode flags in setup.h, nRuns and the seed in testbench_config.h;
 *   3. builds it with g++ and runs the test bench (nRuns closed-loop runs);
 *   4. reads the summary file the test bench writes and appends one row to
 *      a CSV file: the hyperparameter values, the closed-loop metrics, and a
 *      latency estimate.
 * The repository files are never modified. Combinations run in parallel
 * (--jobs). Plot the CSV with gridsearch/plotGridSearch.m.
 *
 * Build (C++17, from the repository root):
 *   g++ -std=c++17 -O2 -pthread -o gridsearch/grid_search gridsearch/grid_search.cpp
 * Run:
 *   gridsearch/grid_search gridsearch/example_grid.txt --jobs 4
 *
 * Grid file: one setting per line, '#' starts a comment.
 *   system  SYSTEM_INVERTED_PENDULUM     ACTIVE_SYSTEM (setup.h)
 *   flag    FIXED off                     a mode flag of setup.h: on / off
 *   runs    10                            testbench_settings::nRuns (>= 2)
 *   seed    1                             testbench_settings::testbenchSeed
 *   param   mpc::predHrzn 10 15 20        a hyperparameter (hyperparams.h)
 *                                         and the values to try
 *   ap_include /path/to/ap_types/include  needed only with FIXED on
 * Settings that are not given keep the value in the sources. The grid is
 * the Cartesian product of the param lines.
 *
 * Every edit must match exactly one line of its file, otherwise the tool
 * stops before building anything: a typo cannot silently test the default.
 */

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

#ifdef _WIN32
static const char *const SIM_COMMAND = "sim.exe";
#else
static const char *const SIM_COMMAND = "./sim";
#endif

/* ======================================================================
   Small helpers
   ====================================================================== */

static std::string readFile(const fs::path &p)
{
    std::ifstream in(p, std::ios::binary);
    if (!in) throw std::runtime_error("cannot read " + p.string());
    std::ostringstream s;
    s << in.rdbuf();
    return s.str();
}

static void writeFile(const fs::path &p, const std::string &text)
{
    std::ofstream out(p, std::ios::binary);
    if (!out) throw std::runtime_error("cannot write " + p.string());
    out << text;
}

static std::vector<std::string> splitWords(const std::string &line)
{
    std::istringstream in(line);
    std::vector<std::string> w;
    std::string s;
    while (in >> s) w.push_back(s);
    return w;
}

/* In the single line of text that re matches (std::regex_search), replace
 * the match with replacement(match). Throws unless exactly one line
 * matches. Line by line, so that ^ and $ need no multiline mode; the
 * replacement is built in code, not with a regex format string. */
static std::string replaceOnce(const std::string &text, const std::regex &re,
                               const std::function<std::string(const std::smatch &)> &replacement,
                               const std::string &what)
{
    std::istringstream in(text);
    std::string line, result;
    int count = 0;
    while (std::getline(in, line))
    {
        std::smatch m;
        if (std::regex_search(line, m, re))
        {
            count++;
            line = m.prefix().str() + replacement(m) + m.suffix().str();
        }
        result += line + "\n";
    }
    if (count != 1)
        throw std::runtime_error(what + ": expected exactly one matching line, found " + std::to_string(count));
    return result;
}

/* Run a shell command in dir, output to log; returns the exit status. */
static int runIn(const fs::path &dir, const std::string &cmd, const fs::path &log)
{
#ifdef _WIN32
    const std::string full = "cd /d \"" + dir.string() + "\" && " + cmd + " > \"" + log.string() + "\" 2>&1";
#else
    const std::string full = "cd \"" + dir.string() + "\" && " + cmd + " > \"" + log.string() + "\" 2>&1";
#endif
    return std::system(full.c_str());
}

/* ======================================================================
   The grid file
   ====================================================================== */

struct Param
{
    std::string ns, name;          /* "mpc", "predHrzn" */
    std::vector<int> values;
    std::string full() const { return ns + "::" + name; }
};

struct Grid
{
    std::string system;                          /* empty: keep the sources' ACTIVE_SYSTEM */
    std::vector<std::pair<std::string, bool>> flags;
    int runs = 10;
    int seed = -1;                               /* -1: keep the sources' seed */
    std::string apInclude;
    std::vector<Param> params;
};

static Grid readGrid(const fs::path &file)
{
    Grid g;
    std::istringstream in(readFile(file));
    std::string line;
    int lineNo = 0;
    while (std::getline(in, line))
    {
        lineNo++;
        if (const auto hash = line.find('#'); hash != std::string::npos) line.erase(hash);
        const auto w = splitWords(line);
        if (w.empty()) continue;
        const std::string where = file.string() + ":" + std::to_string(lineNo) + ": ";
        auto need = [&](size_t n) { if (w.size() < n) throw std::runtime_error(where + "too few words"); };

        if (w[0] == "system")          { need(2); g.system = w[1]; }
        else if (w[0] == "flag")
        {
            need(3);
            if (w[2] != "on" && w[2] != "off") throw std::runtime_error(where + "flag value must be on or off");
            g.flags.push_back({w[1], w[2] == "on"});
        }
        else if (w[0] == "runs")       { need(2); g.runs = std::stoi(w[1]); }
        else if (w[0] == "seed")       { need(2); g.seed = std::stoi(w[1]); }
        else if (w[0] == "ap_include") { need(2); g.apInclude = w[1]; }
        else if (w[0] == "param")
        {
            need(3);
            const auto sep = w[1].find("::");
            if (sep == std::string::npos) throw std::runtime_error(where + "param name must be namespace::name");
            Param p{w[1].substr(0, sep), w[1].substr(sep + 2), {}};
            for (size_t i = 2; i < w.size(); i++) p.values.push_back(std::stoi(w[i]));
            g.params.push_back(p);
        }
        else throw std::runtime_error(where + "unknown setting '" + w[0] + "'");
    }
    if (g.runs < 2) throw std::runtime_error("runs must be >= 2: the test bench writes its summary file only then");
    if (g.params.empty()) throw std::runtime_error("no param lines: nothing to search");
    return g;
}

/* One grid point: the value index of every param (odometer order: the last
 * param changes fastest). */
static std::vector<std::vector<int>> combinations(const Grid &g)
{
    std::vector<std::vector<int>> all{{}};
    for (const auto &p : g.params)
    {
        std::vector<std::vector<int>> next;
        for (const auto &c : all)
            for (int v : p.values) { auto c2 = c; c2.push_back(v); next.push_back(c2); }
        all = next;
    }
    return all;
}

/* ======================================================================
   Editing the copied sources
   ====================================================================== */

/* Every X(ns, name, value, ...) entry of hyperparams.h: the values the
 * build actually uses, swept or not (needed for the latency estimate). */
static std::map<std::string, int> readHyperparams(const std::string &text)
{
    std::map<std::string, int> hp;
    const std::regex re(R"(X\(\s*(\w+)\s*,\s*(\w+)\s*,\s*(-?\d+)\s*,)");
    for (auto it = std::sregex_iterator(text.begin(), text.end(), re); it != std::sregex_iterator(); ++it)
        hp[(*it)[1].str() + "::" + (*it)[2].str()] = std::stoi((*it)[3].str());
    return hp;
}

static void editSources(const fs::path &src, const Grid &g, const std::vector<int> &values)
{
    /* hyperparams.h: X(ns, name, <value>, "...") */
    std::string hp = readFile(src / "hyperparams.h");
    for (size_t i = 0; i < g.params.size(); i++)
    {
        const auto &p = g.params[i];
        const std::regex re("(X\\(\\s*" + p.ns + "\\s*,\\s*" + p.name + "\\s*,\\s*)-?\\d+");
        const std::string v = std::to_string(values[i]);
        hp = replaceOnce(hp, re, [&](const std::smatch &m) { return m[1].str() + v; }, "hyperparams.h, " + p.full());
    }
    writeFile(src / "hyperparams.h", hp);

    /* setup.h: ACTIVE_SYSTEM and the mode flags. A flag line is
     * "#define FLAG ..." or "// #define FLAG ..." (any spacing). */
    std::string setup = readFile(src / "setup.h");
    if (!g.system.empty())
        setup = replaceOnce(setup, std::regex(R"(^#define ACTIVE_SYSTEM\s+\w+)"),
                            [&](const std::smatch &) { return "#define ACTIVE_SYSTEM   " + g.system; }, "setup.h, ACTIVE_SYSTEM");
    for (const auto &f : g.flags)
        setup = replaceOnce(setup, std::regex(R"(^[ \t]*(//[ \t]*)?#define[ \t]+)" + f.first + R"(\b.*$)"),
                            [&](const std::smatch &) { return (f.second ? "#define " : "// #define ") + f.first; },
                            "setup.h, flag " + f.first);
    writeFile(src / "setup.h", setup);

    /* testbench_config.h: nRuns, testbenchSeed */
    std::string tb = readFile(src / "testbench_config.h");
    tb = replaceOnce(tb, std::regex(R"((constexpr int nRuns\s*=\s*)\d+)"),
                     [&](const std::smatch &m) { return m[1].str() + std::to_string(g.runs); }, "testbench_config.h, nRuns");
    if (g.seed >= 0)
        tb = replaceOnce(tb, std::regex(R"((constexpr unsigned int testbenchSeed\s*=\s*)\d+u?)"),
                         [&](const std::smatch &m) { return m[1].str() + std::to_string(g.seed) + "u"; },
                         "testbench_config.h, testbenchSeed");
    writeFile(src / "testbench_config.h", tb);
}

/* Copy the controller sources: the top-level .cpp/.h files and the headers
 * of the subdirectories (systems/), as the plain g++ build uses them;
 * passiveLearning/ is not part of the build. */
static void copySources(const fs::path &from, const fs::path &to)
{
    fs::create_directories(to);
    for (const auto &e : fs::recursive_directory_iterator(from))
    {
        if (!e.is_regular_file()) continue;
        const fs::path rel = fs::relative(e.path(), from);
        const auto ext = e.path().extension();
        const bool topLevel = rel.parent_path().empty();
        if (!topLevel && (*rel.begin() == "passiveLearning" || ext != ".h")) continue;
        if (topLevel && ext != ".cpp" && ext != ".h") continue;
        fs::create_directories((to / rel).parent_path());
        fs::copy_file(e.path(), to / rel, fs::copy_options::overwrite_existing);
    }
}

/* ======================================================================
   Results
   ====================================================================== */

/*
 * Latency estimate of one controller() call, PRAGMA_PROFILE_BALANCED, in
 * clock cycles (10 ns each). Calibrated on the csynth report of 2026-11-09
 * (pendulum, Ny = 15, Nu = 3, S = 16, I = 10: 34208 cycles, matched
 * exactly by the formula below); for other values it extrapolates the
 * loop structure, it is not a synthesis result:
 *   costFunctionArx  Ny*(S + 1 + 16) + 7    model loop II=1 over S+1 models,
 *                                            16 = pipeline depth + loop overhead
 *   MADS iteration   2*Nu*(eval + 4) + 55   2*Nu poll points + poll matrix
 *   MADSARX          (I + 1)*iteration + 1  iteration -1 is the start point
 *   controller       + 11*S + 1             generateScenarios
 *                    + S + 6                computeArxCoeffs (S+1 rows, II=1)
 *                    + 7                    conversions, warm start
 */
static long estimateCyclesBalanced(const std::map<std::string, int> &hp)
{
    const long ny = hp.at("mpc::predHrzn"), nu = hp.at("mpc::ctrlHrzn");
    const long s = 1L << hp.at("scen::log2nScen"), it = hp.at("mads::iter");
    const long eval = ny * (s + 1 + 16) + 7;
    const long iteration = 2 * nu * (eval + 4) + 55;
    const long mads = (it + 1) * iteration + 1;
    return mads + (11 * s + 1) + (s + 6) + 7;
}

struct Metrics
{
    double meanRmse = 0, maxRmse = 0, maxViolY = 0, maxViolDy = 0, ts = 0;
    int runs = 0, runsViolY = 0, runsViolDy = 0;
};

/* Column index by header name: the summary's columns depend on the plant
 * (the pendulum adds l, m), so they are found by name. */
static Metrics readSummary(const fs::path &summary)
{
    std::istringstream in(readFile(summary));
    std::string line;
    if (!std::getline(in, line)) throw std::runtime_error("empty summary " + summary.string());
    const auto header = splitWords(line);
    auto col = [&](const std::string &name) {
        for (size_t i = 0; i < header.size(); i++) if (header[i] == name) return i;
        throw std::runtime_error("summary has no column " + name);
    };
    const size_t cRmse = col("rmse"), cNy = col("nViolY"), cNdy = col("nViolDy"),
                 cMy = col("maxViolY"), cMdy = col("maxViolDy");
    Metrics m;
    while (std::getline(in, line))
    {
        const auto w = splitWords(line);
        if (w.size() < header.size()) continue;
        const double rmse = std::stod(w[cRmse]);
        m.runs++;
        m.meanRmse += rmse;
        m.maxRmse = std::max(m.maxRmse, rmse);
        if (std::stoi(w[cNy]) > 0)  m.runsViolY++;
        if (std::stoi(w[cNdy]) > 0) m.runsViolDy++;
        m.maxViolY  = std::max(m.maxViolY,  std::stod(w[cMy]));
        m.maxViolDy = std::max(m.maxViolDy, std::stod(w[cMdy]));
    }
    if (m.runs == 0) throw std::runtime_error("no runs in " + summary.string());
    m.meanRmse /= m.runs;
    return m;
}

/* Ts: the last column of a run file's first data row (testMain.cpp logs it). */
static double readTs(const fs::path &runFile)
{
    std::istringstream in(readFile(runFile));
    std::string header, row;
    std::getline(in, header);
    std::getline(in, row);
    const auto w = splitWords(row);
    return w.empty() ? 0.0 : std::stod(w.back());
}

/* The first file below dir whose name ends with suffix. */
static fs::path findFile(const fs::path &dir, const std::string &suffix)
{
    for (const auto &e : fs::recursive_directory_iterator(dir))
    {
        const std::string n = e.path().filename().string();
        if (e.is_regular_file() && n.size() >= suffix.size() &&
            n.compare(n.size() - suffix.size(), suffix.size(), suffix) == 0)
            return e.path();
    }
    return {};
}

/* First error line of a build log, for the CSV (e.g. a static_assert that
 * rejects the combination: ctrlHrzn > predHrzn - nd, ...). */
static std::string firstError(const fs::path &log)
{
    std::istringstream in(readFile(log));
    std::string line;
    while (std::getline(in, line))
        if (line.find("error") != std::string::npos)
        {
                        const auto pos = line.find("error");
            return line.substr(pos, 200);
        }
    return "unknown";
}

/* ======================================================================
   main
   ====================================================================== */

int main(int argc, char **argv)
{
    try
    {
        if (argc < 2)
        {
            std::cerr << "usage: grid_search <grid file> [--jobs N] [--out results.csv] [--keep] [--cxx g++]\n";
            return 2;
        }
        const fs::path gridFile = argv[1];
        int jobs = 1;
        bool keep = false;
        std::string cxx = "g++";
        fs::path out;
        for (int i = 2; i < argc; i++)
        {
            const std::string a = argv[i];
            if (a == "--jobs" && i + 1 < argc)      jobs = std::max(1, std::atoi(argv[++i]));
            else if (a == "--out" && i + 1 < argc)  out = argv[++i];
            else if (a == "--cxx" && i + 1 < argc)  cxx = argv[++i];
            else if (a == "--keep")                 keep = true;
            else throw std::runtime_error("unknown option " + a);
        }

        /* The repository: the first parent of the current directory that
         * holds cpp/ScenarioMPC_ARX/setup.h. */
        fs::path repo = fs::current_path();
        while (!fs::exists(repo / "cpp" / "ScenarioMPC_ARX" / "setup.h"))
        {
            if (repo == repo.root_path()) throw std::runtime_error("run inside the scmpc repository");
            repo = repo.parent_path();
        }
        const fs::path srcDir = repo / "cpp" / "ScenarioMPC_ARX";

        const Grid g = readGrid(gridFile);
        const auto points = combinations(g);
        const std::string name = gridFile.stem().string();
        const fs::path buildRoot = repo / "gridsearch" / "build" / name;
        if (out.empty()) out = repo / "gridsearch" / "results" / (name + ".csv");
        fs::create_directories(out.parent_path());
        fs::remove_all(buildRoot);
        fs::create_directories(buildRoot);

        bool fixedOn = false;
        for (const auto &f : g.flags) if (f.first == "FIXED") fixedOn = f.second;
        std::string includes = "-I .";
        if (fixedOn)
        {
            if (g.apInclude.empty()) throw std::runtime_error("FIXED on: give ap_include in the grid file");
            /* setup.h includes hls_math.h in FIXED builds; nothing from it is used */
            writeFile(buildRoot / "hls_math.h", "#pragma once\n#include <cmath>\n");
            includes += " -I \"" + buildRoot.string() + "\" -I \"" + g.apInclude + "\"";
        }

        /* Check every edit once, on a scratch copy, before starting. */
        {
            const fs::path check = buildRoot / "check";
            copySources(srcDir, check);
            editSources(check, g, points[0]);
            fs::remove_all(check);
        }

        /* CSV header */
        std::ofstream csv(out);
        if (!csv) throw std::runtime_error("cannot write " + out.string());
        csv << "point";
        for (const auto &p : g.params) csv << "," << p.full();
        csv << ",status,runs,meanRmse,maxRmse,runsViolY,runsViolDy,maxViolY,maxViolDy,"
               "estCyclesBalanced,estLatencyUs,Ts,latencyFitsTs,message\n";
        csv.flush();

        std::cout << points.size() << " grid points, " << g.runs << " runs each, " << jobs
                  << " parallel jobs. Results: " << out.string() << "\n";

        std::mutex mtx;            /* guards csv and std::cout */
        std::atomic<size_t> next{0};
        std::atomic<int> done{0}, failed{0};

        auto worker = [&]() {
            for (size_t k = next++; k < points.size(); k = next++)
            {
                const fs::path dir = buildRoot / ("point_" + std::to_string(k));
                std::ostringstream row;
                std::string status = "ok", message;
                Metrics m;
                long cycles = 0;
                try
                {
                    copySources(srcDir, dir);
                    editSources(dir, g, points[k]);
                    cycles = estimateCyclesBalanced(readHyperparams(readFile(dir / "hyperparams.h")));

                    /* the .cpp files by name: the Windows shell does not expand *.cpp */
                    std::string sources;
                    for (const auto &e : fs::directory_iterator(dir))
                        if (e.path().extension() == ".cpp") sources += " " + e.path().filename().string();
                    if (runIn(dir, cxx + " -std=c++14 -O2 " + includes + " -o sim" + sources, dir / "build.log") != 0)
                    { status = "build_failed"; message = firstError(dir / "build.log"); }
                    else if (runIn(dir, SIM_COMMAND, dir / "run.log") != 0)
                    { status = "run_failed"; message = "see run.log"; }
                    else
                    {
                        const fs::path summary = findFile(dir, "_summary.txt");
                        if (summary.empty()) throw std::runtime_error("no summary file");
                        m = readSummary(summary);
                        const fs::path run0 = findFile(dir, "_000.txt");
                        if (!run0.empty()) m.ts = readTs(run0);
                    }
                }
                catch (const std::exception &e) { status = "error"; message = e.what(); }

                const double latencyUs = cycles * 10e-3;   /* 10 ns clock */
                row << k;
                for (int v : points[k]) row << "," << v;
                for (char &c : message) if (c == ',' || c == '"' || c == '\n') c = ';';   /* one CSV field */
                row << "," << status << "," << m.runs << "," << m.meanRmse << "," << m.maxRmse << ","
                    << m.runsViolY << "," << m.runsViolDy << "," << m.maxViolY << "," << m.maxViolDy << ","
                    << cycles << "," << latencyUs << "," << m.ts << ","
                    << (m.ts > 0 ? (latencyUs * 1e-6 < m.ts ? 1 : 0) : -1) << "," << message << "\n";

                {
                    std::lock_guard<std::mutex> lock(mtx);
                    csv << row.str();
                    csv.flush();
                    const int d = ++done;
                    if (status != "ok") failed++;
                    std::cout << "[" << d << "/" << points.size() << "] point " << k << ":";
                    for (size_t i = 0; i < g.params.size(); i++) std::cout << " " << g.params[i].name << "=" << points[k][i];
                    if (status == "ok") std::cout << "  meanRmse " << m.meanRmse << ", runs with violations: y " << m.runsViolY
                                                  << ", dy " << m.runsViolDy << " (of " << m.runs << ")\n";
                    else std::cout << "  " << status << ": " << message << "\n";
                }
                if (!keep && status == "ok") fs::remove_all(dir);
            }
        };
        std::vector<std::thread> pool;
        for (int j = 0; j < jobs; j++) pool.emplace_back(worker);
        for (auto &t : pool) t.join();

        std::cout << "Done: " << done << " points, " << failed << " failed.";
        if (failed > 0) std::cout << " Their build directories (build.log, run.log) are in " << buildRoot.string() << ".";
        std::cout << "\n";
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "grid_search: " << e.what() << "\n";
        return 1;
    }
}
