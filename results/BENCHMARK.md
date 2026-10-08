# Broad-Phase Collision Detection Benchmark Report
**Project:** 2D Rigid Body Physics Engine (Raylib / C++)  
**Author:** High School Independent Research / Technical Spike Project  
**Benchmark Target:** 1D Sweep-and-Prune (SAP) with Temporal Coherence vs. Brute-Force $O(N^2)$ Baseline  
**Environment:** Headless Benchmark Executable (`bench/bench_broadphase.cpp`), built in **Release mode (-O3, -DNDEBUG)**

---

## 1. System Specifications & Reproducibility Environment

| Component | Specification |
| :--- | :--- |
| **Operating System** | Windows 10 Pro (x64, Build 19045) |
| **CPU Model** | Intel(R) Xeon(R) CPU E5-2680 v3 @ 2.50GHz (Dual Socket) |
| **Compiler** | MinGW-w64 GCC 14.2.0 (UCRT64) |
| **Compilation Flags** | `-O3 -DNDEBUG -std=c++17 -Wall -Wextra` |
| **Measurement Tool** | `std::chrono::steady_clock` (sub-microsecond precision) |
| **Protocol** | 100 warm-up frames (discarded) + 600 measured frames per run, 3 repeated runs per $(N, \text{algorithm})$ |
| **Dead-Code Elimination** | Pair counts accumulated into a volatile escaped sink `g_escaped_sink` |
| **Python Plot Status** | Python is not installed on this machine environment; raw CSV is preserved at `results/benchmark_results.csv` |

---

## 2. Benchmark Results: Uniform Constant-Density Arena

In this benchmark, **spatial density is held constant** across all $N$ by scaling world area proportionally ($\text{Area} = N \times 16,000\text{ px}^2$). Objects have randomized dimensions ($R \in [8, 24]\text{ px}$), initial velocities ($40 \sim 120\text{ px/s}$), and bounce dynamically off arena boundaries.

All figures below are **exact, empirical measurements** (median across the 3 independent runs of 600 frames each).

| $N$ (Bodies) | Brute-Force Baseline (ms) | Sweep-and-Prune (InsertionSort) (ms) | Sweep-and-Prune (std::sort every frame) (ms) | **Speedup vs. Brute-Force** | Average Candidate Pairs / Frame |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **100** | 0.0369 ms | **0.0068 ms** | 0.0085 ms | **5.43x** | 18.7 |
| **500** | 0.6472 ms | **0.0410 ms** | 0.0562 ms | **15.79x** | 90.7 |
| **1,000** | 2.1936 ms | **0.0865 ms** | 0.1203 ms | **25.36x** | 173.8 |
| **2,000** | 8.2718 ms | **0.2372 ms** | 0.3282 ms | **34.87x** | 345.9 |
| **5,000** | 48.0765 ms | **0.7177 ms** | 0.8820 ms | **66.98x** | 869.7 |

*Note: Raw run metrics (mean, median, p95 per run) are logged in `results/benchmark_results.csv`.*

---

## 3. Extra Scenario: The Degenerate "Worst-Case" Column

When all objects are constrained inside a narrow vertical column ($X \in [10, 50]\text{ px}$), their bounding boxes continuously overlap along the sorted axis ($X$). The inner loop cannot break early (`entries[j].minX > entries[i].maxX` rarely triggers).

| $N$ (Bodies) | Brute-Force Baseline (ms) | Sweep-and-Prune (ms) | Speedup vs. Brute-Force | Average Candidate Pairs / Frame |
| :---: | :---: | :---: | :---: | :---: |
| **100** | **0.0220 ms** | 0.0274 ms | **0.80x** *(SAP is slower!)* | 99.9 |
| **500** | 0.6082 ms | 0.5212 ms | **1.17x** | 509.6 |
| **1,000** | 2.3853 ms | 1.9763 ms | **1.21x** | 1,008.8 |
| **2,000** | 9.3015 ms | 7.7296 ms | **1.20x** | 2,003.3 |

**Finding:** In the degenerate 1D column case, Sweep-and-Prune’s speedup collapses from **$35\times$ down to only $1.2\times$**. At $N=100$, SAP is actually **20% slower** than brute force due to sorting overhead without the benefit of early pruning.

---

## 4. Key Performance Insights

### A. Quadratic Scaling ($O(N^2)$) of Brute Force
- Doubling $N$ from $1,000 \to 2,000$ increases Brute-Force time from $2.19\text{ ms} \to 8.27\text{ ms}$ (**$3.77\times$**, adhering strictly to the theoretical quadratic $4\times$ multiplier).
- At $N = 5,000$, Brute Force requires **$48.08\text{ ms}$** per frame alone, limiting the engine to under **21 FPS** (completely unacceptable for interactive real-time 60 FPS simulations).

### B. Near-Linear Scaling of Sweep-and-Prune
- Sweep-and-Prune at $N = 5,000$ takes only **$0.72\text{ ms}$**, which leaves over $95\%$ of the $16.6\text{ ms}$ frame budget for physics constraint solving and rendering.
- Speedup scales continuously with $N$: **$5.4\times$ at $N=100 \implies 67.0\times$ at $N=5,000$**.

### C. The Quantitative Value of Temporal Coherence
- Because moving rigid bodies retain almost the same relative spatial ordering between consecutive frames ($\Delta t = 1/60\text{ s}$), the persistent array is nearly sorted.
- `InsertionSort` on the persistent array consistently beats `std::sort` by **$20\% \sim 38\%$** across all test sizes (e.g., at $N=1,000$: $0.086\text{ ms}$ vs. $0.120\text{ ms}$).

---

## 5. How to Read These Results (High-School Student Guide)

1. **Why Brute-Force grows like $N^2$:** Brute force tests every object against every other object ($\frac{N(N-1)}{2}$ comparisons). When you double the number of objects ($2N$), the number of pairs quadruples ($4\times$).
2. **Why Sweep-and-Prune scales almost linearly:** By sorting objects along the $X$-axis, we only compare objects whose $X$-intervals overlap. In a constant-density world, an object only has a few neighbors nearby on $X$, so the inner loop breaks early after checking a small constant number of objects.
3. **Why the Speedup grows as $N$ increases:** Dividing $O(N^2)$ by $O(N)$ yields an algorithmic speedup of $O(N)$. The larger the simulation, the more dramatic the advantage of Sweep-and-Prune.
4. **What the Worst-Case scenario teaches us:** Algorithms are sensitive to geometric distribution. When objects align along the sorted axis, Sweep-and-Prune degrades to $O(N^2)$. In commercial engines (like Box2D or PhysX), multi-axis SAP, Bounding Volume Hierarchies (Dynamic AABB Trees), or Spatial Hashes are chosen to handle non-uniform distributions robustly.

---

## 6. Assumptions & Methodological Sanity Checks

1. **Correctness Invariant:** In Step 2, an assertion compared the normalized, sorted pair lists between Brute Force and Sweep-and-Prune across 25 dynamic frames for all $N$. Both algorithms matched **100% of candidate pairs with zero discrepancies**.
2. **Density Invariance:** Scaling arena area with $N$ ensures that candidate pairs per body remain constant ($\approx 0.17$ pairs/body). If area were fixed, density would surge as $N^2$, invalidating the broad-phase pruning test.
3. **Fair Memory Baseline:** The brute-force implementation reused a pre-allocated vector for AABB computation to avoid heap allocation skew.
4. **Thermal Throttling Guard:** 100 warm-up frames were discarded to stabilize CPU branch predictors and instruction caches before timing started.
