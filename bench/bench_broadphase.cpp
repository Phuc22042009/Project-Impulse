#include <iostream>
#include <vector>
#include <chrono>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <random>
#include <iomanip>
#include <fstream>
#include <cassert>
#include <string>

#include "physics/BroadPhase.h"
#include "physics/RigidBody2D.h"
#include "physics/Shape.h"

static const uint32_t FIXED_SEED = 42;
static const float R_MIN = 8.0f;
static const float R_MAX = 24.0f;
static const float AREA_PER_OBJECT = 16000.0f;
static const float FIXED_DT = 1.0f / 60.0f;

static const int WARMUP_FRAMES = 100;
static const int MEASURED_FRAMES = 600;
static const int REPEAT_RUNS = 3;

volatile uint64_t g_escaped_sink = 0;

void BruteForceBroadPhase(const std::vector<RigidBody2D> &bodies,
                          std::vector<AABBEntry> &aabbBuffer,
                          std::vector<std::pair<int, int>> &outPairs)
{
    outPairs.clear();
    const int n = static_cast<int>(bodies.size());
    if (aabbBuffer.size() != bodies.size())
    {
        aabbBuffer.resize(n);
    }

    for (int i = 0; i < n; ++i)
    {
        aabbBuffer[i] = GetAABBEntry(bodies[i], i);
    }

    for (int i = 0; i < n; ++i)
    {
        for (int j = i + 1; j < n; ++j)
        {
            if (aabbBuffer[i].maxX < aabbBuffer[j].minX || aabbBuffer[j].maxX < aabbBuffer[i].minX)
                continue;
            if (aabbBuffer[i].maxY < aabbBuffer[j].minY || aabbBuffer[j].maxY < aabbBuffer[i].minY)
                continue;

            if (bodies[i].invMass == 0 && bodies[j].invMass == 0)
                continue;
            if (!bodies[i].isAwake && !bodies[j].isAwake)
                continue;

            outPairs.push_back({i, j});
        }
    }
}

void BroadPhaseStdSortEveryFrame(const std::vector<RigidBody2D> &bodies,
                                 std::vector<AABBEntry> &entries,
                                 std::vector<std::pair<int, int>> &outPairs)
{
    outPairs.clear();
    const int n = static_cast<int>(bodies.size());
    if (entries.size() != bodies.size())
    {
        entries.resize(n);
    }

    for (int i = 0; i < n; ++i)
    {
        entries[i] = GetAABBEntry(bodies[i], i);
    }

    std::sort(entries.begin(), entries.end(), [](const AABBEntry &a, const AABBEntry &b) {
        return a.minX < b.minX;
    });

    for (int i = 0; i < n; ++i)
    {
        for (int j = i + 1; j < n; ++j)
        {
            if (entries[j].minX > entries[i].maxX)
                break;

            if (entries[i].maxY < entries[j].minY || entries[i].minY > entries[j].maxY)
                continue;

            int indexA = entries[i].index;
            int indexB = entries[j].index;

            if (bodies[indexA].invMass == 0 && bodies[indexB].invMass == 0)
                continue;
            if (!bodies[indexA].isAwake && !bodies[indexB].isAwake)
                continue;

            outPairs.push_back({indexA, indexB});
        }
    }
}

struct SimObject
{
    Vec2 velocity;
    float angularVelocity = 0.0f;
    float radius = 10.0f;
};

struct BenchmarkScene
{
    std::vector<RigidBody2D> bodies;
    std::vector<SimObject> simData;
    float worldWidth = 1000.0f;
    float worldHeight = 1000.0f;

    void Integrate(float dt)
    {
        const int n = static_cast<int>(bodies.size());
        for (int i = 0; i < n; ++i)
        {
            bodies[i].position.x += simData[i].velocity.x * dt;
            bodies[i].position.y += simData[i].velocity.y * dt;
            bodies[i].angle += simData[i].angularVelocity * dt;

            float r = simData[i].radius;
            if (bodies[i].position.x - r < 0.0f)
            {
                bodies[i].position.x = r;
                simData[i].velocity.x = std::abs(simData[i].velocity.x);
            }
            else if (bodies[i].position.x + r > worldWidth)
            {
                bodies[i].position.x = worldWidth - r;
                simData[i].velocity.x = -std::abs(simData[i].velocity.x);
            }

            if (bodies[i].position.y - r < 0.0f)
            {
                bodies[i].position.y = r;
                simData[i].velocity.y = std::abs(simData[i].velocity.y);
            }
            else if (bodies[i].position.y + r > worldHeight)
            {
                bodies[i].position.y = worldHeight - r;
                simData[i].velocity.y = -std::abs(simData[i].velocity.y);
            }
        }
    }
};

BenchmarkScene CreateConstantDensityScene(int n, uint32_t seed)
{
    BenchmarkScene scene;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> radDist(R_MIN, R_MAX);
    std::uniform_real_distribution<float> speedDist(40.0f, 120.0f);
    std::uniform_real_distribution<float> angleDist(0.0f, 6.2831853f);
    std::uniform_real_distribution<float> angVelDist(-2.0f, 2.0f);

    float totalArea = n * AREA_PER_OBJECT;
    scene.worldWidth = std::sqrt(totalArea);
    scene.worldHeight = scene.worldWidth;

    std::uniform_real_distribution<float> posXDist(R_MAX, scene.worldWidth - R_MAX);
    std::uniform_real_distribution<float> posYDist(R_MAX, scene.worldHeight - R_MAX);

    scene.bodies.reserve(n);
    scene.simData.reserve(n);

    for (int i = 0; i < n; ++i)
    {
        float r = radDist(rng);
        float x = posXDist(rng);
        float y = posYDist(rng);
        float spd = speedDist(rng);
        float dir = angleDist(rng);

        std::unique_ptr<Shape> shape;
        if (i % 2 == 0)
        {
            shape = std::make_unique<Box>(r * 2.0f, r * 2.0f);
        }
        else
        {
            shape = std::make_unique<Circle>(r);
        }

        RigidBody2D body(1.0f, std::move(shape));
        body.position = Vec2(x, y);
        body.angle = angleDist(rng);
        body.velocity = Vec2(spd * std::cos(dir), spd * std::sin(dir));
        body.isAwake = true;

        SimObject sim;
        sim.velocity = body.velocity;
        sim.angularVelocity = angVelDist(rng);
        sim.radius = r;

        scene.bodies.push_back(std::move(body));
        scene.simData.push_back(sim);
    }

    return scene;
}

BenchmarkScene CreateWorstCaseColumnScene(int n, uint32_t seed)
{
    BenchmarkScene scene;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> radDist(R_MIN, R_MAX);
    std::uniform_real_distribution<float> speedDist(40.0f, 120.0f);
    std::uniform_real_distribution<float> angleDist(0.0f, 6.2831853f);
    std::uniform_real_distribution<float> angVelDist(-2.0f, 2.0f);

    scene.worldWidth = 60.0f;
    scene.worldHeight = n * 40.0f;

    std::uniform_real_distribution<float> posXDist(10.0f, 50.0f);
    std::uniform_real_distribution<float> posYDist(R_MAX, scene.worldHeight - R_MAX);

    scene.bodies.reserve(n);
    scene.simData.reserve(n);

    for (int i = 0; i < n; ++i)
    {
        float r = radDist(rng);
        float x = posXDist(rng);
        float y = posYDist(rng);
        float spd = speedDist(rng);
        float dir = angleDist(rng);

        std::unique_ptr<Shape> shape = std::make_unique<Box>(r * 2.0f, r * 2.0f);

        RigidBody2D body(1.0f, std::move(shape));
        body.position = Vec2(x, y);
        body.angle = angleDist(rng);
        body.velocity = Vec2(spd * std::cos(dir), spd * std::sin(dir));
        body.isAwake = true;

        SimObject sim;
        sim.velocity = body.velocity;
        sim.angularVelocity = angVelDist(rng);
        sim.radius = r;

        scene.bodies.push_back(std::move(body));
        scene.simData.push_back(sim);
    }

    return scene;
}

struct RunStats
{
    double meanMs = 0.0;
    double medianMs = 0.0;
    double p95Ms = 0.0;
    double avgPairs = 0.0;
};

RunStats ComputeStats(std::vector<double> &timesMs, const std::vector<uint64_t> &pairsCount)
{
    RunStats s;
    if (timesMs.empty()) return s;

    double sum = std::accumulate(timesMs.begin(), timesMs.end(), 0.0);
    s.meanMs = sum / timesMs.size();

    std::sort(timesMs.begin(), timesMs.end());
    size_t mid = timesMs.size() / 2;
    s.medianMs = (timesMs.size() % 2 == 0) ? (timesMs[mid - 1] + timesMs[mid]) * 0.5 : timesMs[mid];

    size_t p95Idx = static_cast<size_t>(timesMs.size() * 0.95);
    if (p95Idx >= timesMs.size()) p95Idx = timesMs.size() - 1;
    s.p95Ms = timesMs[p95Idx];

    double totalPairs = std::accumulate(pairsCount.begin(), pairsCount.end(), 0.0);
    s.avgPairs = totalPairs / pairsCount.size();

    return s;
}

void VerifyCorrectness(int n, const std::string &scenarioName)
{
    std::cout << "[STEP 2] Verifying correctness for " << scenarioName << " (N = " << n << ")... ";
    BenchmarkScene scene = (scenarioName == "WorstCase_Column") 
        ? CreateWorstCaseColumnScene(n, FIXED_SEED) 
        : CreateConstantDensityScene(n, FIXED_SEED);

    std::vector<AABBEntry> sapEntries;
    std::vector<AABBEntry> bruteAabbs;
    std::vector<std::pair<int, int>> sapPairs;
    std::vector<std::pair<int, int>> brutePairs;

    for (int frame = 0; frame < 25; ++frame)
    {
        scene.Integrate(FIXED_DT);

        BroadPhase(scene.bodies, sapEntries, sapPairs);
        BruteForceBroadPhase(scene.bodies, bruteAabbs, brutePairs);

        for (auto &p : sapPairs)
        {
            if (p.first > p.second) std::swap(p.first, p.second);
        }
        for (auto &p : brutePairs)
        {
            if (p.first > p.second) std::swap(p.first, p.second);
        }

        std::sort(sapPairs.begin(), sapPairs.end());
        std::sort(brutePairs.begin(), brutePairs.end());

        if (sapPairs != brutePairs)
        {
            std::cerr << "\n[CORRECTNESS ERROR] Mismatch at frame " << frame << " (N = " << n << ")!\n";
            std::cerr << "  SAP pairs count: " << sapPairs.size() << "\n";
            std::cerr << "  BruteForce pairs count: " << brutePairs.size() << "\n";
            
            size_t maxLen = std::max(sapPairs.size(), brutePairs.size());
            for (size_t k = 0; k < maxLen; ++k)
            {
                std::string s1 = (k < sapPairs.size()) ? ("(" + std::to_string(sapPairs[k].first) + "," + std::to_string(sapPairs[k].second) + ")") : "NONE";
                std::string s2 = (k < brutePairs.size()) ? ("(" + std::to_string(brutePairs[k].first) + "," + std::to_string(brutePairs[k].second) + ")") : "NONE";
                if (s1 != s2)
                {
                    std::cerr << "  First mismatch at index " << k << ": SAP=" << s1 << " vs Brute=" << s2 << "\n";
                    break;
                }
            }
            std::exit(1);
        }
    }
    std::cout << "PASSED (all 25 frames matched exactly).\n";
}

enum class AlgoType
{
    BruteForce,
    SAP_TemporalCoherence,
    SAP_StdSortEveryFrame
};

std::string AlgoName(AlgoType t)
{
    switch (t)
    {
        case AlgoType::BruteForce: return "BruteForce";
        case AlgoType::SAP_TemporalCoherence: return "SweepAndPrune_InsertionSort";
        case AlgoType::SAP_StdSortEveryFrame: return "SweepAndPrune_StdSort";
    }
    return "Unknown";
}

RunStats RunSingleBenchmark(AlgoType algo, int n, int runIndex, bool isWorstCase)
{
    BenchmarkScene scene = isWorstCase 
        ? CreateWorstCaseColumnScene(n, FIXED_SEED + runIndex * 100) 
        : CreateConstantDensityScene(n, FIXED_SEED + runIndex * 100);

    std::vector<AABBEntry> persistentEntries;
    std::vector<AABBEntry> persistentAabbBuffer;
    std::vector<std::pair<int, int>> pairs;

    for (int f = 0; f < WARMUP_FRAMES; ++f)
    {
        scene.Integrate(FIXED_DT);
        if (algo == AlgoType::BruteForce)
            BruteForceBroadPhase(scene.bodies, persistentAabbBuffer, pairs);
        else if (algo == AlgoType::SAP_TemporalCoherence)
            BroadPhase(scene.bodies, persistentEntries, pairs);
        else
            BroadPhaseStdSortEveryFrame(scene.bodies, persistentEntries, pairs);

        g_escaped_sink += pairs.size();
    }

    std::vector<double> timesMs;
    timesMs.reserve(MEASURED_FRAMES);
    std::vector<uint64_t> pairCounts;
    pairCounts.reserve(MEASURED_FRAMES);

    for (int f = 0; f < MEASURED_FRAMES; ++f)
    {
        scene.Integrate(FIXED_DT);

        auto tStart = std::chrono::steady_clock::now();
        if (algo == AlgoType::BruteForce)
            BruteForceBroadPhase(scene.bodies, persistentAabbBuffer, pairs);
        else if (algo == AlgoType::SAP_TemporalCoherence)
            BroadPhase(scene.bodies, persistentEntries, pairs);
        else
            BroadPhaseStdSortEveryFrame(scene.bodies, persistentEntries, pairs);
        auto tEnd = std::chrono::steady_clock::now();

        double elapsedMs = std::chrono::duration<double, std::milli>(tEnd - tStart).count();
        timesMs.push_back(elapsedMs);
        pairCounts.push_back(pairs.size());

        g_escaped_sink += pairs.size();
    }

    return ComputeStats(timesMs, pairCounts);
}

int main()
{
    std::cout << "================================================================\n";
    std::cout << "  2D Physics Engine: Broad-Phase Collision Detection Benchmark\n";
    std::cout << "================================================================\n";
    std::cout << "Compiler : GCC " << __VERSION__ << "\n";
    std::cout << "Flags    : -O3 -DNDEBUG (Release)\n";
    std::cout << "CPU      : Intel(R) Xeon(R) CPU E5-2680 v3 @ 2.50GHz\n";
    std::cout << "OS       : Windows 10 Pro (x64)\n";
    std::cout << "Config   : " << MEASURED_FRAMES << " measured frames (" << WARMUP_FRAMES << " warmup) x " << REPEAT_RUNS << " runs per configuration\n";
    std::cout << "================================================================\n\n";

    const std::vector<int> N_SIZES = {100, 500, 1000, 2000, 5000};
    for (int n : N_SIZES)
    {
        VerifyCorrectness(n, "Uniform_ConstantDensity");
    }
    VerifyCorrectness(500, "WorstCase_Column");
    std::cout << "\n[STATUS] All correctness checks passed! Commencing performance benchmarks...\n\n";

    std::ofstream csv("results/benchmark_results.csv");
    csv << "Scenario,Algorithm,N,Run,Mean_ms,Median_ms,P95_ms,AvgPairs\n";

    struct ResultRow
    {
        std::string scenario;
        std::string algo;
        int n;
        double medianOfMedians;
        double meanMean;
        double meanP95;
        double avgPairs;
    };
    std::vector<ResultRow> summaryTable;

    auto RunSuite = [&](const std::string &scenarioName, bool isWorstCase, const std::vector<int> &sizes, const std::vector<AlgoType> &algos)
    {
        std::cout << ">>> Running Scenario: " << scenarioName << " <<<\n";
        for (int n : sizes)
        {
            std::cout << "--------------------------------------------------------\n";
            std::cout << " N = " << n << "\n";
            std::cout << "--------------------------------------------------------\n";
            for (AlgoType algo : algos)
            {
                std::vector<double> medians;
                std::vector<double> means;
                std::vector<double> p95s;
                double sumPairs = 0.0;

                for (int r = 0; r < REPEAT_RUNS; ++r)
                {
                    RunStats stats = RunSingleBenchmark(algo, n, r, isWorstCase);
                    medians.push_back(stats.medianMs);
                    means.push_back(stats.meanMs);
                    p95s.push_back(stats.p95Ms);
                    sumPairs += stats.avgPairs;

                    csv << scenarioName << "," << AlgoName(algo) << "," << n << "," << (r + 1) << ","
                        << std::fixed << std::setprecision(5)
                        << stats.meanMs << "," << stats.medianMs << "," << stats.p95Ms << ","
                        << stats.avgPairs << "\n";
                    csv.flush();
                }

                std::sort(medians.begin(), medians.end());
                double medianMed = medians[medians.size() / 2];
                double meanMean = std::accumulate(means.begin(), means.end(), 0.0) / REPEAT_RUNS;
                double meanP95 = std::accumulate(p95s.begin(), p95s.end(), 0.0) / REPEAT_RUNS;
                double avgPairs = sumPairs / REPEAT_RUNS;

                summaryTable.push_back({scenarioName, AlgoName(algo), n, medianMed, meanMean, meanP95, avgPairs});

                std::cout << std::left << std::setw(30) << AlgoName(algo)
                          << " | Median: " << std::fixed << std::setprecision(4) << medianMed << " ms"
                          << " | Mean: " << meanMean << " ms"
                          << " | p95: " << meanP95 << " ms"
                          << " | Pairs: " << std::setprecision(1) << avgPairs << "\n";
            }
        }
        std::cout << "\n";
    };

    RunSuite("Uniform_ConstantDensity", false, N_SIZES, 
             {AlgoType::BruteForce, AlgoType::SAP_TemporalCoherence, AlgoType::SAP_StdSortEveryFrame});

    RunSuite("WorstCase_Column", true, {100, 500, 1000, 2000}, 
             {AlgoType::BruteForce, AlgoType::SAP_TemporalCoherence});

    csv.close();

    std::cout << "========================================================================================\n";
    std::cout << "  BENCHMARK SUMMARY (Uniform Constant Density)\n";
    std::cout << "========================================================================================\n";
    std::cout << "  N   | BruteForce (ms) | SAP InsertionSort (ms) | SAP StdSort (ms) | Speedup (x) | Pairs\n";
    std::cout << "------+-----------------+------------------------+------------------+-------------+-------\n";

    for (int n : N_SIZES)
    {
        double bf = 0.0, sapInsert = 0.0, sapSort = 0.0, pairs = 0.0;
        for (const auto &row : summaryTable)
        {
            if (row.scenario == "Uniform_ConstantDensity" && row.n == n)
            {
                if (row.algo == "BruteForce") bf = row.medianOfMedians;
                else if (row.algo == "SweepAndPrune_InsertionSort") { sapInsert = row.medianOfMedians; pairs = row.avgPairs; }
                else if (row.algo == "SweepAndPrune_StdSort") sapSort = row.medianOfMedians;
            }
        }
        double speedup = (sapInsert > 1e-6) ? (bf / sapInsert) : 0.0;
        std::cout << std::right << std::setw(5) << n << " | "
                  << std::setw(15) << std::fixed << std::setprecision(4) << bf << " | "
                  << std::setw(22) << sapInsert << " | "
                  << std::setw(16) << sapSort << " | "
                  << std::setw(10) << std::setprecision(2) << speedup << "x | "
                  << std::setw(6) << std::setprecision(0) << pairs << "\n";
    }

    std::cout << "\n[SINK] Escaped pair accumulator value: " << g_escaped_sink << "\n";
    std::cout << "[DONE] Raw data written to results/benchmark_results.csv\n";
    return 0;
}
