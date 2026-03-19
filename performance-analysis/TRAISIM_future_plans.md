---
marp: true
theme: default
paginate: true
style: |
  :root {
    --color-bg: #FAFAFA;
    --color-fg: #1a1a2e;
    --color-accent: #01696F;
    --color-accent-light: #e0f2f1;
    --color-muted: #6b7280;
    font-family: 'Segoe UI', 'Calibri', 'Arial', sans-serif;
  }
  section {
    background: var(--color-bg);
    color: var(--color-fg);
    font-size: 22px;
    padding: 40px 60px;
  }
  section.lead {
    display: flex;
    flex-direction: column;
    justify-content: center;
    align-items: center;
    text-align: center;
    background: linear-gradient(135deg, #0d3b3f 0%, #01696F 60%, #1a8a8a 100%);
    color: #ffffff;
  }
  section.lead h1 {
    font-size: 2.2em;
    font-weight: 700;
    margin-bottom: 0.2em;
    border: none;
    color: #ffffff;
  }
  section.lead h2 {
    font-size: 1.1em;
    font-weight: 400;
    color: #b2dfdb;
    margin-top: 0;
  }
  section.lead p {
    color: #cce8e8;
    font-size: 0.85em;
  }
  section.section-divider {
    display: flex;
    flex-direction: column;
    justify-content: center;
    background: #01696F;
    color: #ffffff;
  }
  section.section-divider h1 {
    font-size: 2em;
    border: none;
    margin-bottom: 0.2em;
    color: #ffffff;
  }
  section.section-divider p {
    color: #b2dfdb;
    font-size: 1em;
  }
  h1 {
    color: var(--color-accent);
    font-size: 1.5em;
    border-bottom: 2px solid var(--color-accent);
    padding-bottom: 8px;
    margin-bottom: 16px;
  }
  h2 {
    color: #0d3b3f;
    font-size: 1.15em;
    margin-bottom: 8px;
  }
  strong {
    color: var(--color-accent);
  }
  table {
    font-size: 0.82em;
    width: 100%;
    border-collapse: collapse;
    margin: 12px 0;
  }
  th {
    background: var(--color-accent);
    color: #fff;
    padding: 6px 12px;
    text-align: left;
  }
  td {
    padding: 5px 12px;
    border-bottom: 1px solid #e0e0e0;
  }
  tr:nth-child(even) td {
    background: #f5f5f5;
  }
  code {
    background: #e8f5e9;
    padding: 1px 5px;
    border-radius: 3px;
    font-size: 0.9em;
    color: #0d3b3f;
  }
  blockquote {
    border-left: 4px solid var(--color-accent);
    padding-left: 16px;
    color: var(--color-muted);
    font-style: italic;
    margin: 12px 0;
  }
  ul { margin: 8px 0; }
  li { margin-bottom: 4px; }
  footer {
    color: var(--color-muted);
    font-size: 0.65em;
  }
---

<!-- _class: lead -->

# TRAISIM — Future Plans
## Performance Optimization Roadmap for Dynawo

Petros Aristidou — SPS-Lab, Cyprus University of Technology
TRAISIM Project (CRESYM) | TwinEU (Horizon Europe, Grant 101136119)

---

# Context: Real-Time Operator Training

**TRAISIM** develops a real-time operator training platform using **Dynawo**, the open-source power system simulator by RTE.

- **Partners:** CUT (SPS-Lab), RTE, CRESYM
- **Funded by:** CRESYM, part of the TwinEU Horizon Europe project
- **Goal:** Simulate large-scale power systems within **SCADA refresh rate** (2–4 s per step)
- **Test system:** French transmission network with **6,000+ buses**

The key challenge: real-time simulation demands that every computation step completes within a hard wall-clock deadline. Larger models with more detail push beyond this limit.

---

# Profiling Results: The Challenge

Benchmarking on the French system across three model detail levels:

| Metric | Model 1 | Model 2 | Model 3 |
|--------|---------|---------|---------|
| Variables | ~80,000 | ~210,000 | ~320,000 |
| Real-time feasible | Yes | No | No |
| System size vs. Model 1 | 1x | ~2.6x | ~4x |

**Key findings from profiling (PSCC 2026 paper):**

- **KLU Analyze** (symbolic factorization) consumes **32.5%** of solver time in Model 3
- Solver **parameter tuning** alone achieved a **2x speedup** for topology events
- Minor automata events (tap changers, OELs) trigger unnecessary full refactorizations

---

# Root Cause: Unnecessary Symbolic Factorizations

The solver currently forces a full symbolic factorization (`klu_analyze` + `klu_factor`) on every mode change event — regardless of whether the Jacobian structure actually changed.

**Impact on large systems:**
- ~**30%** of event-period computation time is wasted on redundant factorizations
- The `BTF` (Block Triangular Form) phase in KLU is the dominant cost in symbolic analysis
- Each symbolic factorization scales with matrix size — costs grow with model detail

> **Developer insight (Gautier Bureau, RTE):** Event severity classification can eliminate the majority of unnecessary factorizations. A prototype already exists for the IDA solver.

---

<!-- _class: section-divider -->

# Optimization Roadmap
Four phases, from quick wins to research-level improvements

---

# Phase 0 — Quick Wins (1–2 weeks)

**Target:** 15–25% speedup with minimal code changes

| Item | Description | Expected Gain |
|------|-------------|---------------|
| A11 | Event severity classification | 15–30% of event time |
| A1 | Adaptive factorization control | 5–8% |
| A2 | Matrix structure change tolerance | 3–5% |
| A3 | KLU numerical-only refactorization | 5–7% |

**A11 is the highest priority.** Gautier Bureau already prototyped this for IDA — porting to SolverSIM is well-scoped. The other items provide layered factorization control: A1 adds decision logic, A2 extends skip criteria, A3 ensures the fast `klu_refactor` path is used.

**Validation:** Nordic test system + large-scale French system benchmark.

---

# Phase 1 — Medium Effort (2–4 weeks)

**Target:** Additional 10–20% speedup through build and algorithmic improvements

| Item | Description | Expected Gain |
|------|-------------|---------------|
| P8 | Profile-Guided Optimization (PGO) | 5–10% |
| P9 | Link-Time Optimization (LTO) | 3–5% |
| — | Remaining `std::map` audit | Variable |
| A5 | Partial Jacobian updates | 10–20% |

- **P8/P9:** Build-system-only changes — zero code risk, improves all code paths
- **A5:** Highest potential payoff but complex; developer feedback flags cross-model coupling difficulties. Use a conservative Newton-failure fallback approach.

**Go/No-Go:** Cumulative speedup ≥ 20% before advancing.

---

# Phase 2 — Advanced (1–3 months)

**Target:** Additional 15–30% speedup via parallelization and solver adaptivity

| Item | Description | Expected Gain |
|------|-------------|---------------|
| P2 | OpenMP Jacobian evaluation | 8–12% |
| P3 | OpenMP SubModel evaluation | 3–5% |
| A6 | Adaptive time step control | 3–10% |
| A7 | Improved Newton convergence | 2–5% |
| A8 | Krylov preconditioner strategies | 20–40% (large systems) |

**Known risk:** RTE encountered KLU lock contention during prior OpenMP attempts. Mitigation: run a feasibility study on P2 first, measure actual contention before committing.

**Go/No-Go:** OpenMP ≥ 1.5x on 4 cores; Krylov crossover point determined.

---

# Phase 3 — Research (3–6 months)

**Target:** 2–5x speedup for the largest simulations, requiring architectural changes

| Item | Description | Expected Gain |
|------|-------------|---------------|
| P10 | GPU acceleration (cuSOLVER) | 20–50% (very large) |
| A9 | Schur complement decomposition | 15–30% |
| A10 | Waveform relaxation (multi-rate) | 10–25% |

- **P10:** Prototype GPU sparse solve, measure data transfer overhead, determine crossover system size
- **A9:** Partition network vs. device models, exploit block structure
- **A10:** Separate fast (power electronics) and slow (electromechanical) dynamics

Each item is prototyped and evaluated before production commitment.

---

# Cumulative Impact

```
Phase 0 (1–2 weeks)    ████████████████░░░░░░░░  15–25% speedup
Phase 1 (2–4 weeks)    ████████████████████████  25–45% cumulative
Phase 2 (1–3 months)   ████████████████████████  40–75% cumulative
Phase 3 (3–6 months)   ████████████████████████  60–125%+ cumulative
```

**Phase 0 alone may be sufficient** to bring Model 2 into real-time feasibility.

Phases 1–3 target Model 3 feasibility and provide headroom for even larger systems.

> All estimates are validated against profiling data from the French 6,000+ bus system.

---

# Completed Work

| Milestone | Status |
|-----------|--------|
| Performance analysis framework (profiling, benchmarking, bottleneck detection) | Done |
| Build guide for Ubuntu 24.04 | Done |
| Solver parameter tuning (2x speedup on topology events) | Done |
| Profiling of French system — 3 model detail levels | Done |
| P1: Flat vector derivatives (upstream by G. Bureau) | Done |
| Detailed optimization roadmap with 20 items | Done |
| Developer feedback integration (TRAISIM discussion) | Done |

All tools and documentation are available in the `performance-analysis/` directory of the [SPS-L/dynawo](https://github.com/SPS-L/dynawo/tree/performance-analysis-framework/performance-analysis) fork.

---

# Next Steps

1. **Implement Phase 0** — starting with A11 (event severity classification)
2. **Benchmark on the French system** — validate gains on Model 2 and Model 3
3. **Engage with RTE/Dynawo upstream** — contribute improvements back to the main repository
4. **Proceed through phases based on Go/No-Go criteria** — data-driven decisions at each gate

**Timeline alignment:** Phase 0–1 align with TRAISIM v2 extension (Jan 2026 – Jun 2027, Task #4: New Solver Implementation). Phases 2–3 are longer-term research directions.

---

<!-- _class: lead -->

# Thank You

Petros Aristidou — petros.aristidou@cut.ac.cy
SPS-Lab, Cyprus University of Technology

TRAISIM Project | CRESYM | TwinEU (Horizon Europe, Grant 101136119)
