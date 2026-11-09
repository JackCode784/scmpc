/**
 * @file  scmpc_test.cpp
 * @brief Test harness for cpp/ScenarioMPC_ARX: builds the controller in a
 *        chosen configuration, runs it, and checks the result.
 *
 * Host tool only (never synthesized), so it uses C++17 (<filesystem>).
 * Build once, from the repository root:
 *
 *   g++ -std=c++17 -O2 -o tests/scmpc_test tests/harness/scmpc_test.cpp
 *
 * Commands (run from anywhere inside the repository):
 *
 *   scmpc_test regress <old> <new> [--quick] [--ignore-header]
 *       Byte-identity check. <old>/<new> are git revisions (main, HEAD,
 *       a commit hash) or WORKTREE (the files on disk, uncommitted edits
 *       included). Every configuration of the matrix is built and run
 *       from both trees; every output file must be identical.
 *       --ignore-header compares output files without their first line
 *       (for commits that only rename the output columns).
 *
 *   scmpc_test unit [<rev>]
 *       Builds tests/unit/unit_tests.cpp against the controller sources
 *       (default WORKTREE) in several configurations and runs it.
 *
 *   scmpc_test closedloop [<rev>] [--runs N] [--record]
 *       Closed-loop acceptance: tracking RMSE, constraint violations, and
 *       how far fixed point is from floating point, compared with the
 *       limits in tests/closed_loop_limits.txt. --record overwrites that
 *       file with the current values plus a margin.
 *
 * Common options:
 *   --cxx <compiler>    C++ compiler (default: g++, or $CXX)
 *   --ap <dir>          Xilinx ap_types include dir (default: $SCMPC_AP_INCLUDE);
 *                       needed by every FIXED configuration
 *   --keep              keep the build directories (tests/build/...)
 *
 * How a configuration is applied: the controller is configured by editing
 * headers (CLAUDE.md), so the harness copies the sources into
 * tests/build/<name>/ and rewrites the relevant lines of setup.h and
 * testbench_config.h there. The repository files are never modified.
 * Every rewrite must match exactly one line, otherwise the run stops: a
 * silently ignored setting would make a comparison meaningless.
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

/* ======================================================================
   Small utilities
   ====================================================================== */
static std::string readFile(const fs::path &p)
{
    std::ifstream f(p, std::ios::binary);
    if (!f) throw std::runtime_error("cannot read " + p.string());
    std::ostringstream s;
    s << f.rdbuf();
    return s.str();
}

static void writeFile(const fs::path &p, const std::string &text)
{
    std::ofstream f(p, std::ios::binary);
    if (!f) throw std::runtime_error("cannot write " + p.string());
    f << text;
}

static std::string quote(const fs::path &p) { return "\"" + p.string() + "\""; }

/* Run a shell command, output redirected to a log file. Returns the exit code. */
static int runLogged(const std::string &cmd, const fs::path &log)
{
    const std::string full = cmd + " > " + quote(log) + " 2>&1";
#ifdef _WIN32
    /* cmd.exe strips one level of outer quotes */
    const int rc = std::system(("\"" + full + "\"").c_str());
#else
    const int rc = std::system(full.c_str());
#endif
    return rc;
}

static std::string trim(const std::string &s)
{
    const auto b = s.find_first_not_of(" \t\r\n");
    const auto e = s.find_last_not_of(" \t\r\n");
    return (b == std::string::npos) ? "" : s.substr(b, e - b + 1);
}

/* ======================================================================
   Settings shared by all commands
   ====================================================================== */
struct Options
{
    fs::path repo;              /* repository root                      */
    std::string cxx = "g++";
    fs::path apInclude;         /* ap_types include dir (FIXED builds)  */
    bool keep = false;
    bool quick = false;
    bool ignoreHeader = false;
    bool record = false;
    int  runs = 10;
};

static const char *SRC_SUBDIR = "cpp/ScenarioMPC_ARX";

/* ======================================================================
   A configuration of the controller (what CLAUDE.md calls the matrix)
   ====================================================================== */
struct Config
{
    std::string name;                 /* short tag, used for directories */
    std::string system = "SYSTEM_INVERTED_PENDULUM";
    std::string profile = "BALANCED"; /* AREA, BALANCED, LATENCY         */
    std::string ctrlMode = "CTRL_MODE_SCMPC";
    bool fixed = false, conv = true, nrmlz = true;
    bool scenCost = true, scenConstr = true, dc = false;
    int nRuns = 3;
};

static Config cfg(std::string name, std::string system, bool fixed, bool conv, bool nrmlz)
{
    Config c;
    c.name = name; c.system = system; c.fixed = fixed; c.conv = conv; c.nrmlz = nrmlz;
    return c;
}

/* The byte-identity matrix: systems, float/fixed, profiles, scenarios,
 * delay compensation, passive learning. The buck converters' fixed-point
 * build fails a static_assert until log2R is per system (CLAUDE.md), so
 * they appear in floating point only for now. */
static std::vector<Config> regressionMatrix(bool quick)
{
    const std::string P = "SYSTEM_INVERTED_PENDULUM", B = "SYSTEM_BUCK_LOSS";
    std::vector<Config> m;
    m.push_back(cfg("pend_float",      P, false, true,  true));
    m.push_back(cfg("pend_fixed",      P, true,  true,  true));
    { Config c = cfg("pend_fixed_dc",  P, true,  true,  true); c.dc = true; m.push_back(c); }
    if (quick) return m;
    m.push_back(cfg("pend_float_plain", P, false, false, false));
    { Config c = cfg("pend_float_dc",  P, false, true, true); c.dc = true; m.push_back(c); }
    { Config c = cfg("pend_fixed_area",    P, true, true, true); c.profile = "AREA";    m.push_back(c); }
    { Config c = cfg("pend_fixed_latency", P, true, true, true); c.profile = "LATENCY"; m.push_back(c); }
    { Config c = cfg("pend_fixed_noscen",  P, true, true, true); c.scenCost = c.scenConstr = false; m.push_back(c); }
    { Config c = cfg("pend_fixed_pl",  P, true,  true, true); c.ctrlMode = "CTRL_MODE_PL"; m.push_back(c); }
    { Config c = cfg("pend_float_pl",  P, false, true, true); c.ctrlMode = "CTRL_MODE_PL"; m.push_back(c); }
    m.push_back(cfg("buck_float",      B, false, true, true));
    { Config c = cfg("buck_float_pl",  B, false, true, true); c.ctrlMode = "CTRL_MODE_PL"; m.push_back(c); }
    return m;
}

/* ======================================================================
   Header rewriting
   ====================================================================== */

/* Replace the single line matching `re` by `replacement` (ECMAScript
 * regex, multiline anchors emulated line by line). Throws unless exactly
 * one line matches. */
static std::string replaceOneLine(const std::string &text, const std::regex &re,
                                  const std::string &replacement, const std::string &what)
{
    std::istringstream in(text);
    std::ostringstream out;
    std::string line;
    int hits = 0;
    while (std::getline(in, line))
    {
        if (std::regex_search(line, re)) { out << replacement << "\n"; hits++; }
        else out << line << "\n";
    }
    if (hits != 1)
        throw std::runtime_error("setting " + what + ": expected exactly 1 matching line, found " + std::to_string(hits));
    return out.str();
}

/* A mode flag: a line "#define NAME ..." that may be commented out with //. */
static std::string setFlag(const std::string &text, const std::string &name, bool on)
{
    const std::regex re("^\\s*(//\\s*)?#define\\s+" + name + "(\\s|$)");
    return replaceOneLine(text, re, (on ? "#define " : "// #define ") + name, name);
}

static void applyConfig(const fs::path &src, const Config &c)
{
    std::string s = readFile(src / "setup.h");
    s = setFlag(s, "FIXED", c.fixed);
    s = setFlag(s, "CONVERSIONS_MODE", c.conv);
    s = setFlag(s, "NRMLZ", c.nrmlz);
    s = setFlag(s, "USE_SCENS_COST", c.scenCost);
    s = setFlag(s, "USE_SCENS_CONSTR", c.scenConstr);
    s = setFlag(s, "DELAY_COMPENSATION", c.dc);
    s = replaceOneLine(s, std::regex("^#define\\s+ACTIVE_SYSTEM\\s+\\w+"),
                       "#define ACTIVE_SYSTEM " + c.system, "ACTIVE_SYSTEM");
    s = replaceOneLine(s, std::regex("^#define\\s+CTRL_MODE\\s+CTRL_MODE_\\w+"),
                       "#define CTRL_MODE " + c.ctrlMode, "CTRL_MODE");
    s = replaceOneLine(s, std::regex("^#define\\s+PRAGMA_PROFILE\\s+PRAGMA_PROFILE_\\w+"),
                       "#define PRAGMA_PROFILE PRAGMA_PROFILE_" + c.profile, "PRAGMA_PROFILE");
    writeFile(src / "setup.h", s);

    /* Number of runs: testbench_config.h today, plant.h's N_RUNS in older revisions */
    if (fs::exists(src / "testbench_config.h") &&
        std::regex_search(readFile(src / "testbench_config.h"), std::regex("constexpr\\s+int\\s+nRuns\\s*=")))
    {
        std::string t = readFile(src / "testbench_config.h");
        t = replaceOneLine(t, std::regex("constexpr\\s+int\\s+nRuns\\s*=\\s*\\d+\\s*;"),
                           "    constexpr int nRuns = " + std::to_string(c.nRuns) + ";", "nRuns");
        writeFile(src / "testbench_config.h", t);
    }
    else
    {
        std::string t = readFile(src / "plant.h");
        t = replaceOneLine(t, std::regex("^#define\\s+N_RUNS\\s+\\d+"),
                           "#define N_RUNS " + std::to_string(c.nRuns), "N_RUNS");
        writeFile(src / "plant.h", t);
    }
}

/* ======================================================================
   Exporting a source tree, building, running
   ====================================================================== */

/* Copy the controller sources of <rev> (or WORKTREE) into dest: only the
 * top-level .cpp/.h files, as the Vitis project and the plain g++ build
 * use them (passiveLearning/ is not part of the build). */
static void exportSources(const Options &o, const std::string &rev, const fs::path &dest)
{
    fs::remove_all(dest);
    fs::create_directories(dest);
    fs::path from;
    fs::path tmp;
    if (rev == "WORKTREE")
        from = o.repo / SRC_SUBDIR;
    else
    {
        tmp = dest.parent_path() / (dest.filename().string() + "_export");
        fs::remove_all(tmp);
        fs::create_directories(tmp);
        const fs::path tar = tmp / "src.tar";
        const std::string cmd = "git -C " + quote(o.repo) + " archive --format=tar -o " + quote(tar) + " " + rev + " " + SRC_SUBDIR;
        if (runLogged(cmd, tmp / "git.log") != 0)
            throw std::runtime_error("git archive " + rev + " failed: " + readFile(tmp / "git.log"));
        if (runLogged("tar -xf " + quote(tar) + " -C " + quote(tmp), tmp / "tar.log") != 0)
            throw std::runtime_error("tar failed: " + readFile(tmp / "tar.log"));
        from = tmp / SRC_SUBDIR;
    }
    for (const auto &e : fs::directory_iterator(from))
    {
        const auto ext = e.path().extension();
        if (e.is_regular_file() && (ext == ".cpp" || ext == ".h"))
            fs::copy_file(e.path(), dest / e.path().filename());
    }
    /* Headers in subdirectories (e.g. systems/), at the same relative path:
     * they are included by relative path. passiveLearning/ is not part of the build. */
    for (const auto &e : fs::recursive_directory_iterator(from))
    {
        const fs::path rel = fs::relative(e.path(), from);
        if (!e.is_regular_file() || rel.parent_path().empty()) continue;
        if (*rel.begin() == "passiveLearning" || e.path().extension() != ".h") continue;
        fs::create_directories((dest / rel).parent_path());
        fs::copy_file(e.path(), dest / rel);
    }
    if (!tmp.empty()) fs::remove_all(tmp);
}

/* Stub hls_math.h: setup.h includes it in FIXED builds, nothing from it is used. */
static fs::path stubDir(const Options &o)
{
    const fs::path d = o.repo / "tests" / "build" / "stub";
    fs::create_directories(d);
    if (!fs::exists(d / "hls_math.h"))
        writeFile(d / "hls_math.h", "#pragma once\n#include <cmath>\n");
    return d;
}

static std::string exeName(const std::string &base)
{
#ifdef _WIN32
    return base + ".exe";
#else
    return base;
#endif
}

/* Compile every .cpp in src (minus `exclude`) plus `extra` into src/<exe>. */
static bool build(const Options &o, const fs::path &src, const Config &c, const std::string &exe,
                  const std::string &exclude = "", const std::vector<fs::path> &extra = {})
{
    if (c.fixed && o.apInclude.empty())
        throw std::runtime_error("configuration " + c.name + " is FIXED: pass --ap <ap_types include dir> or set SCMPC_AP_INCLUDE");
    std::string cmd = o.cxx + " -std=c++14 -O2 -I " + quote(stubDir(o));
    if (!o.apInclude.empty()) cmd += " -I " + quote(o.apInclude);
    cmd += " -I " + quote(src) + " -o " + quote(src / exeName(exe));
    for (const auto &e : fs::directory_iterator(src))
        if (e.path().extension() == ".cpp" && e.path().filename() != exclude)
            cmd += " " + quote(e.path());
    for (const auto &p : extra) cmd += " " + quote(p);
    return runLogged(cmd, src / "build.log") == 0;
}

/* Run src/<exe> with src as working directory (the test bench writes its
 * output files relative to it). */
static int runExe(const fs::path &src, const std::string &exe, const std::string &logName = "run.log")
{
#ifdef _WIN32
    const std::string cmd = "cd /d " + quote(src) + " && " + exeName(exe);
#else
    const std::string cmd = "cd " + quote(src) + " && ./" + exeName(exe);
#endif
    return runLogged(cmd, src / logName);
}

/* Output files of a test-bench run: every .txt below src. */
static std::map<std::string, fs::path> outputFiles(const fs::path &src)
{
    std::map<std::string, fs::path> files;
    for (const auto &e : fs::recursive_directory_iterator(src))
        if (e.is_regular_file() && e.path().extension() == ".txt")
            files[fs::relative(e.path(), src).generic_string()] = e.path();
    return files;
}

static std::string lastLines(const fs::path &p, int n)
{
    std::istringstream in(readFile(p));
    std::vector<std::string> lines;
    std::string l;
    while (std::getline(in, l)) lines.push_back(l);
    std::string out;
    for (size_t i = (lines.size() > (size_t)n ? lines.size() - n : 0); i < lines.size(); i++)
        out += "      " + lines[i] + "\n";
    return out;
}

/* ======================================================================
   regress
   ====================================================================== */
/* The first line if it is a header (contains a letter), so that
 * --ignore-header never skips data, e.g. golden_u.txt's first sample. */
static std::string dropHeader(const std::string &s)
{
    const auto p = s.find('\n');
    const std::string first = s.substr(0, p);
    bool letter = false;
    for (char c : first) if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z' && c != 'e')) letter = true;
    if (!letter || p == std::string::npos) return s;
    return s.substr(p + 1);
}

/* Line number (1-based) of the first difference, 0 if none. */
static int firstDiffLine(const std::string &a, const std::string &b)
{
    std::istringstream ia(a), ib(b);
    std::string la, lb;
    for (int n = 1;; n++)
    {
        const bool ga = (bool)std::getline(ia, la), gb = (bool)std::getline(ib, lb);
        if (!ga && !gb) return 0;
        if (ga != gb || la != lb) return n;
    }
}

static int cmdRegress(const Options &o, const std::string &oldRev, const std::string &newRev)
{
    int failures = 0;
    const fs::path root = o.repo / "tests" / "build" / "regress";
    for (const Config &c : regressionMatrix(o.quick))
    {
        std::map<std::string, std::map<std::string, fs::path>> out;
        bool ok = true;
        for (const auto &side : {std::make_pair(std::string("old"), oldRev), std::make_pair(std::string("new"), newRev)})
        {
            const fs::path dir = root / c.name / side.first;
            exportSources(o, side.second, dir);
            applyConfig(dir, c);
            if (!build(o, dir, c, "sim", ""))
            {
                std::cout << "  " << c.name << ": BUILD FAILED (" << side.first << " = " << side.second << ")\n"
                          << lastLines(dir / "build.log", 6);
                ok = false;
                break;
            }
            runExe(dir, "sim");
            out[side.first] = outputFiles(dir);
        }
        if (!ok) { failures++; continue; }

        std::vector<std::string> problems;
        if (out["old"].empty()) problems.push_back("no output files");
        for (const auto &f : out["old"])
        {
            auto it = out["new"].find(f.first);
            if (it == out["new"].end()) { problems.push_back("missing in new: " + f.first); continue; }
            std::string a = readFile(f.second), b = readFile(it->second);
            if (a == b) continue;
            int offset = 0;
            if (o.ignoreHeader)
            {
                const std::string a2 = dropHeader(a), b2 = dropHeader(b);
                if (a2 == b2) continue;
                if (a2.size() != a.size() && b2.size() != b.size()) { a = a2; b = b2; offset = 1; }
            }
            problems.push_back(f.first + " differs from line " + std::to_string(firstDiffLine(a, b) + offset));
        }
        for (const auto &f : out["new"])
            if (!out["old"].count(f.first)) problems.push_back("only in new: " + f.first);

        if (problems.empty())
            std::cout << "  " << c.name << ": identical (" << out["old"].size() << " files)\n";
        else
        {
            failures++;
            std::cout << "  " << c.name << ": DIFFERS\n";
            for (size_t i = 0; i < problems.size() && i < 5; i++) std::cout << "      " << problems[i] << "\n";
        }
        if (!o.keep) fs::remove_all(root / c.name);
    }
    std::cout << (failures ? "REGRESS: " + std::to_string(failures) + " configuration(s) differ or failed\n"
                           : "REGRESS: all identical\n");
    return failures ? 1 : 0;
}

/* ======================================================================
   unit
   ====================================================================== */
static std::vector<Config> unitMatrix()
{
    const std::string P = "SYSTEM_INVERTED_PENDULUM", B = "SYSTEM_BUCK_LOSS";
    std::vector<Config> m;
    m.push_back(cfg("pend_float",        P, false, true,  true));
    m.push_back(cfg("pend_float_plain",  P, false, false, false));
    m.push_back(cfg("pend_fixed",        P, true,  true,  true));
    { Config c = cfg("pend_fixed_dc",    P, true,  true,  true); c.dc = true; m.push_back(c); }
    m.push_back(cfg("buck_float",        B, false, true,  true));
    /* passive learning, no ADC/DAC: the closed-loop containment test needs
     * the controller to see exactly the plant's (noisy) output */
    { Config c = cfg("buck_float_pl",    B, false, false, true); c.ctrlMode = "CTRL_MODE_PL"; m.push_back(c); }
    return m;
}

static int cmdUnit(const Options &o, const std::string &rev)
{
    int failures = 0;
    const fs::path root = o.repo / "tests" / "build" / "unit";
    const fs::path testSrc = o.repo / "tests" / "unit" / "unit_tests.cpp";
    for (const Config &c : unitMatrix())
    {
        if (c.fixed && o.apInclude.empty())
        {
            std::cout << "  " << c.name << ": SKIPPED (FIXED: no --ap / SCMPC_AP_INCLUDE)\n";
            continue;
        }
        const fs::path dir = root / c.name;
        exportSources(o, rev, dir);
        applyConfig(dir, c);
        if (!build(o, dir, c, "unit_tests", "testMain.cpp", {testSrc}))
        {
            failures++;
            std::cout << "  " << c.name << ": BUILD FAILED\n" << lastLines(dir / "build.log", 8);
            continue;
        }
        const int rc = runExe(dir, "unit_tests");
        std::cout << "  " << c.name << ":\n" << lastLines(dir / "run.log", 1000);
        if (rc != 0) failures++;
        if (!o.keep && rc == 0) fs::remove_all(dir);
    }
    std::cout << (failures ? "UNIT: " + std::to_string(failures) + " configuration(s) failed\n" : "UNIT: all passed\n");
    return failures ? 1 : 0;
}

/* ======================================================================
   closedloop
   ====================================================================== */

/* One output file as columns, by header name. */
struct Table { std::map<std::string, std::vector<double>> col; size_t rows = 0; };

static Table readTable(const fs::path &p)
{
    Table t;
    std::istringstream in(readFile(p));
    std::string header, line;
    std::getline(in, header);
    std::vector<std::string> names;
    { std::istringstream h(header); std::string n; while (h >> n) names.push_back(n); }
    while (std::getline(in, line))
    {
        std::istringstream l(line);
        double v;
        for (size_t i = 0; i < names.size() && (l >> v); i++) t.col[names[i]].push_back(v);
        t.rows++;
    }
    return t;
}

/* A column under its current name or an older one (the columns were renamed). */
static const std::vector<double> &column(const Table &t, std::initializer_list<const char *> names)
{
    for (const char *n : names)
    {
        auto it = t.col.find(n);
        if (it != t.col.end()) return it->second;
    }
    throw std::runtime_error(std::string("output file has no column ") + *names.begin());
}

static double rms(const std::vector<double> &a, const std::vector<double> &b)
{
    const size_t n = std::min(a.size(), b.size());
    double s = 0;
    for (size_t i = 0; i < n; i++) s += (a[i] - b[i]) * (a[i] - b[i]);
    return n ? std::sqrt(s / n) : 0.0;
}

/* "N runs: a with y outside [yMin,yMax], b with |dy| > deltaY." from run.log */
static void violations(const fs::path &log, int &y, int &dy)
{
    std::smatch m;
    const std::string s = readFile(log);
    if (!std::regex_search(s, m, std::regex("runs: (\\d+) with y outside \\[yMin,yMax\\], (\\d+) with \\|dy\\|")))
        throw std::runtime_error("no violation summary in " + log.string());
    y = std::stoi(m[1]); dy = std::stoi(m[2]);
}

static int cmdClosedLoop(const Options &o, const std::string &rev)
{
    const fs::path root = o.repo / "tests" / "build" / "closedloop";
    Config fl = cfg("pend_float", "SYSTEM_INVERTED_PENDULUM", false, true, true);
    Config fx = cfg("pend_fixed", "SYSTEM_INVERTED_PENDULUM", true,  true, true);
    fl.nRuns = fx.nRuns = o.runs;
    std::map<std::string, std::map<std::string, fs::path>> out;
    for (const Config *c : {&fl, &fx})
    {
        const fs::path dir = root / c->name;
        exportSources(o, rev, dir);
        applyConfig(dir, *c);
        if (!build(o, dir, *c, "sim"))
        {
            std::cout << "  " << c->name << ": BUILD FAILED\n" << lastLines(dir / "build.log", 8);
            return 1;
        }
        runExe(dir, "sim");
        out[c->name] = outputFiles(dir);
    }

    /* Per-run trajectories: runs_<tag>_NNN.txt, same tag in both builds */
    std::map<std::string, double> metric;
    double rmseFl = 0, rmseFx = 0, dyFxFl = 0, duFxFl = 0;
    int n = 0;
    const std::regex runFile("runs_.*_(\\d{3})\\.txt$");
    for (const auto &f : out["pend_float"])
    {
        std::smatch m;
        const std::string base = fs::path(f.first).filename().string();
        if (!std::regex_search(base, m, runFile)) continue;
        fs::path other;
        for (const auto &g : out["pend_fixed"]) if (fs::path(g.first).filename() == base) other = g.second;
        if (other.empty()) continue;
        const Table a = readTable(f.second), b = readTable(other);
        const auto &ya = column(a, {"yMeasLog", "ySim"}), &yb = column(b, {"yMeasLog", "ySim"});
        const auto &ua = column(a, {"uAppliedLog", "uSim"}), &ub = column(b, {"uAppliedLog", "uSim"});
        rmseFl += rms(ya, column(a, {"yref"}));
        rmseFx += rms(yb, column(b, {"yref"}));
        dyFxFl += rms(ya, yb);
        duFxFl += rms(ua, ub);
        n++;
    }
    if (n == 0) { std::cout << "CLOSEDLOOP: no run files found\n"; return 1; }
    metric["rmse_float"] = rmseFl / n;
    metric["rmse_fixed"] = rmseFx / n;
    metric["rms_y_fixed_minus_float"] = dyFxFl / n;
    metric["rms_u_fixed_minus_float"] = duFxFl / n;
    int vy, vdy;
    violations(root / "pend_float" / "run.log", vy, vdy);
    metric["runs_viol_y_float"] = vy;  metric["runs_viol_dy_float"] = vdy;
    violations(root / "pend_fixed" / "run.log", vy, vdy);
    metric["runs_viol_y_fixed"] = vy;  metric["runs_viol_dy_fixed"] = vdy;

    const fs::path limitsFile = o.repo / "tests" / "closed_loop_limits.txt";
    if (o.record)
    {
        std::ostringstream s;
        s << "# Closed-loop acceptance limits (scmpc_test closedloop --record).\n"
             "# Pendulum, BALANCED, " << o.runs << " runs, float and fixed (NRMLZ + CONVERSIONS_MODE).\n"
             "# Each value is the recorded one plus a margin: 25% for continuous metrics,\n"
             "# one run for violation counts. Edit by hand to tighten or relax.\n"
             "runs " << o.runs << "\n";
        for (const auto &m : metric)
        {
            const bool count = m.first.rfind("runs_", 0) == 0;
            char buf[64];
            snprintf(buf, sizeof buf, count ? "%.0f" : "%.6g", count ? m.second + 1 : m.second * 1.25);
            s << m.first << " " << buf << "\n";
        }
        writeFile(limitsFile, s.str());
        std::cout << "CLOSEDLOOP: limits recorded in " << limitsFile.string() << "\n";
    }

    std::map<std::string, double> limit;
    if (fs::exists(limitsFile))
    {
        std::istringstream in(readFile(limitsFile));
        std::string line;
        while (std::getline(in, line))
        {
            line = trim(line);
            if (line.empty() || line[0] == '#') continue;
            std::istringstream l(line);
            std::string k; double v;
            if (l >> k >> v) limit[k] = v;
        }
        if (limit.count("runs") && (int)limit["runs"] != o.runs)
            std::cout << "  note: limits were recorded with " << limit["runs"] << " runs, this check used " << o.runs << "\n";
    }

    int failures = 0;
    for (const auto &m : metric)
    {
        const bool has = limit.count(m.first);
        const bool bad = has && m.second > limit[m.first];
        if (bad) failures++;
        char buf[160];
        snprintf(buf, sizeof buf, "  %-26s %12.6g   limit %s%s\n", m.first.c_str(), m.second,
                 has ? std::to_string(limit[m.first]).c_str() : "(none)", bad ? "   EXCEEDED" : "");
        std::cout << buf;
    }
    if (!o.keep) fs::remove_all(root);
    std::cout << (failures ? "CLOSEDLOOP: " + std::to_string(failures) + " limit(s) exceeded\n" : "CLOSEDLOOP: within limits\n");
    return failures ? 1 : 0;
}

/* ======================================================================
   main
   ====================================================================== */
static void usage()
{
    std::cout <<
        "usage: scmpc_test regress <old> <new> [--quick] [--ignore-header]\n"
        "       scmpc_test unit [<rev>]\n"
        "       scmpc_test closedloop [<rev>] [--runs N] [--record]\n"
        "common: --cxx <compiler>  --ap <ap_types include dir>  --keep\n"
        "<rev>: a git revision, or WORKTREE (default) for the files on disk\n";
}

int main(int argc, char **argv)
{
    try
    {
        Options o;
        if (const char *e = std::getenv("CXX")) o.cxx = e;
        if (const char *e = std::getenv("SCMPC_AP_INCLUDE")) o.apInclude = e;
        std::vector<std::string> pos;
        for (int i = 1; i < argc; i++)
        {
            const std::string a = argv[i];
            auto next = [&]() -> std::string {
                if (i + 1 >= argc) throw std::runtime_error(a + " needs a value");
                return argv[++i];
            };
            if      (a == "--cxx") o.cxx = next();
            else if (a == "--ap") o.apInclude = next();
            else if (a == "--keep") o.keep = true;
            else if (a == "--quick") o.quick = true;
            else if (a == "--ignore-header") o.ignoreHeader = true;
            else if (a == "--record") o.record = true;
            else if (a == "--runs") o.runs = std::stoi(next());
            else if (a == "-h" || a == "--help") { usage(); return 0; }
            else pos.push_back(a);
        }
        if (pos.empty()) { usage(); return 2; }
        if (!o.apInclude.empty()) o.apInclude = fs::absolute(o.apInclude);

        /* repository root: the closest parent containing cpp/ScenarioMPC_ARX */
        for (fs::path p = fs::current_path();; p = p.parent_path())
        {
            if (fs::exists(p / SRC_SUBDIR / "setup.h")) { o.repo = p; break; }
            if (p == p.parent_path()) throw std::runtime_error("run scmpc_test from inside the repository");
        }

        const std::string cmd = pos[0];
        if (cmd == "regress" && pos.size() == 3) return cmdRegress(o, pos[1], pos[2]);
        if (cmd == "unit") return cmdUnit(o, pos.size() > 1 ? pos[1] : "WORKTREE");
        if (cmd == "closedloop") return cmdClosedLoop(o, pos.size() > 1 ? pos[1] : "WORKTREE");
        usage();
        return 2;
    }
    catch (const std::exception &e)
    {
        std::cerr << "scmpc_test: " << e.what() << "\n";
        return 2;
    }
}
