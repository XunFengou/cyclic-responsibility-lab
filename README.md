# Cyclic Responsibility Research Lab: canonical website

k-th sectorial partition: cyclic algebra, distributed coverage, and robot dynamics.

Live website: https://cyclic-responsibility-lab-c171.aheadmeter.chatgpt.site

Source repository: https://github.com/XunFengou/cyclic-responsibility-lab

All future simulation modifications belong here. The accepted version-3 output
folder is preserved as an archive. The published website stays attached to the
identity in `.openai/hosting.json`; reuse it for future updates.

## Development

Core tests and the static build have no external package dependency:

```text
node tests/run-tests.cjs
node tests/viewer-harness.cjs
node tests/obstacle-tests.cjs
node tests/comparison-viewer-harness.cjs
node scripts/build.mjs
node tests/validate-build.cjs
node scripts/serve.mjs
```

The preview prints its localhost URL. A preview connection from another process
may be restricted by the desktop environment; that is not a scientific-model
failure. Live browser QA must be reported separately from numerical tests.

Standard Node installations can use the equivalent `npm test`, `npm run build`,
and `npm run preview` scripts. This computer's bundled runtime exposes `node`;
the direct commands above do not require npm on PATH.

Edit `src/index.html`, `src/viewer.js`, `src/engine.js`, `src/dynamics.js`, and
`docs/`. Do not treat generated `dist/` as the canonical source. The website
interface, model explanations, and paper figure labels remain in English.

## Scientific scope

The new `src/Structure_Comparison.html` separates arithmetic mode scans, nested
fixed-site geometry, same-snapshot packet replays, failure-location exposure, and
a certified fractional capacity-constrained reference. Its assumptions and
proofs are in `docs/Structural_Comparisons.md`. It includes phase alignment and
counterchecks in which the cyclic template coincides with Voronoi, frozen cached
Voronoi has no steady traffic, or a common safety snapshot removes any extra
responsibility communication saving. Do not describe the responsibility-only
byte ratios as universal full-system savings.

Reproduce with `node scripts/run-structure-benchmarks.cjs`, then MATLAB
`plot_structure_comparison`. Validate with `node tests/structure-tests.cjs`,
`node tests/structure-viewer-harness.cjs`, and
`node tests/validate-structure-benchmarks.cjs`. The physical comparison accepts
an initial cyclic phase offset without changing the robot initial positions.

The matched multi-obstacle comparison is `src/Obstacle_Comparison.html`, with
`src/comparison.js`, `src/comparison-viewer.js`, and `docs/Obstacle_Comparison.md`.
It compares cyclic sector responsibility against actual nearest-k Voronoi
dominant regions on identical demand quadrature and initial states. Both use
single-integrator motion, the same velocity safety projection, exact swept disk
collision checks, and routed full snapshots. The page supports custom obstacle
unions, failures, complementary orders, packet budgets, common geometric cost,
load imbalance, actual inputs, and visibility diagnostics. Safety-induced traps
are retained and reported. Run `node scripts/run-obstacle-benchmarks.cjs` and
MATLAB `plot_obstacle_comparison` to reproduce the new comparison figures.

Six plant choices implement actual state equations: single integrator, double
integrator, orders 3/4/5, unicycle, differential drive, and an exact-model fully
actuated planar Euler-Lagrange mechanism. The same cyclic responsibility and
packet-based boundary update are used across the plants.

Input units, position/velocity state, containment limits, nonholonomic no-slip
constraints, and the conditional cost-descent theorems are explicit. A moving
partition's physical cost is never replaced with a monotone envelope. Native
input energy is not claimed comparable across different actuator types.

The communication-augmented clock is accounting only for inertial plants. The
plant equations evolve in motion time. No unproved delayed-plant, obstacle,
private-sensing, or pairwise collision-avoidance guarantee is claimed.

## Reproduce the recorded research figures

```text
node scripts/run-research.cjs
```

This writes 15 full recordings under `research/`, which is ignored by Git.
In MATLAB R2023b:

```matlab
addpath('D:/Codex/2026-10-26/referenced-chatgpt-conversation-this-is-an/site/scripts');
plot_research;
```

New public PNGs and summary CSVs are under `assets/result/`; editable FIG files
remain under `research/`. Rebuild the website after generating accepted figures.

## Joint development and publishing

The repository is **XunFengou/cyclic-responsibility-lab**. The owner explicitly
approved public source publication on 2026-11-02. Website visitor access and
source visibility are separate settings; public source does not grant deployment
rights. The requested collaborator is **sheepcy-cxy**. An invitation or write
permission must be confirmed separately and is not implied by this source import.

Use feature branches and pull requests. The owner reviews changes before website
publication. The hosted source history is retained locally in the publishing
checkout. Accepted GitHub changes must be synchronized there before using the
Sites publishing workflow. Do not force-push either repository to make their
histories match. GitHub CI validates source but does not deploy the website.

The source includes `.github/workflows/validate.yml`, a PR template, and the owner
as the default Code Owner. Requiring Code Owner review still needs an actual
repository rule; this import does not claim to configure branch protection.
No publishing secrets belong in source.

Public website access and private source-development rights are separate. The
hosting access policy can switch between public and owner-only. Routine source
updates preserve the existing audience; changing it requires an owner request.

## Current output locations

- `test-results/dynamics_checks.json`: scientific validation groups.
- `test-results/viewer_harness.json`: non-browser event-handler/DOM-contract tests.
- `test-results/build_checks.json`: static resource and English-interface audit.
- `assets/result/`: public paper figures and summaries.
- `research/`: full model recordings and editable figures, retained locally.
- `dist/`: generated deployment output.

Deployment credentials are short-lived and supplied only through the supported
publishing helper's stdin. They are never stored in source or browser code.
