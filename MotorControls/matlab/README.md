# MotorControls MATLAB & Simulink Workspace

Dedicated MATLAB/Simulink workspace for `XZ-Manipulator-Controls` motion-control modeling, system identification, closed-loop simulation, and thesis figure generation.

## Directory Layout

```text
MotorControls/matlab/
├── models/       # Simulink plant, trajectory, and closed-loop models (.slx)
├── scripts/      # Plain .m entry scripts (runnable via `matlab -batch "..."`)
├── functions/    # Reusable .m helper functions (parsers, kinematics, metrics)
├── diagrams/     # Exported control-loop and system block diagrams (.svg/.pdf/.png)
└── figures/      # Publication/thesis plots generated from bench records (.png/.svg/.pdf)
```

## Conventions & AI-Agent Workflow

1. **Read-Only Bench Inputs:** Do not duplicate bench captures here. Read directly from `docs/testing/records/<run_dir>/` (`summary.json`, `trials.csv`, `events.jsonl`, `manifest.json`).
2. **Plain `.m` Sources of Truth:** Prefer plain `.m` scripts and functions over `.mlx` Live Scripts or binary `.fig` files so changes stay diffable and editable by CLI agents.
3. **Headless Execution:** Scripts under `scripts/` should accept input/output paths (or default to repo-relative paths) and run non-interactively:
   ```powershell
   matlab -batch "addpath(genpath('MotorControls/matlab')); <script_name>"
   ```
4. **Build Artifacts Ignored:** Simulink caches and scratch files (`slprj/`, `*.slxc`, `*.asv`, `*.autosave`, `*.mex*`, `*.mat`) are git-ignored at the repository root. Commit only source `.m`/`.slx` files and curated exported diagrams/figures.

## Scope

- **Plant & Closed-Loop Modeling (`models/`):** X-axis (GT2 belt, 80 µsteps/mm), Z-axis (T8x8 lead screw, 400 µsteps/mm), and $\theta$-axis (direct NEMA 17, 3200 µsteps/rev) models with AS5600 12-bit quantization/INL, 2 ms polling delay, and supervisory step-loss recovery + endpoint trim loops.
- **Characterization Analysis (`scripts/`, `functions/`):** Load `motion-characterize` outputs (`summary.json`, `trials.csv`, `events.jsonl`) for speed/accel envelope plotting, FFT resonance checks on `residual_usteps`, and settle-time verification.
- **Thesis Graphics (`diagrams/`, `figures/`):** Deterministic regeneration of publication-grade plots and Simulink block diagrams.
