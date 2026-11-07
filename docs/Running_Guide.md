# Cyclic Responsibility Lab: operating and reproduction guide

## Start here

The website interface, scientific labels and this guide are English. All computation in the simulation pages is browser-local. Sharing a settings link recreates the starting experiment; it does not synchronize live states between visitors. Download JSON to preserve a complete recorded run.

| Navigation | Purpose |
| --- | --- |
| Overview | Main N,k experiment: mass balance, actual inputs, geometry and Fourier response |
| Failures | Matched policies, announced removal and exact pre/post event replay |
| Dynamics | Six integrated plant models with native control units |
| Comparisons | Same-environment Voronoi, capacity and communication studies |
| ROS 2 | Enterprise robot research, offline telemetry and mission adapter draft |
| Guide | This operating guide and downloadable Markdown |
| Theory | Permanent rightmost entrance to definitions, proofs and future extensions |

## First experiment

- On Overview choose N=24, k=6 and Seeded unequal masses. Random feasible positions are the default; position and mass seeds are independent.
- Press Run experiment. Press it again to pause. One step advances 0.2 motion seconds. Steps per display frame changes display throughput, not the model time step.
- Read responsibility-load error and basic-mass/blind energies together. Small load error with nonzero blind energy means the service operator cannot observe part of the partition imbalance.
- Click a robot or choose its persistent physical ID. Gold shows its k-sector responsibility, the square is its unconstrained centroid, and the arrow is its applied velocity. A centroid may lie in a hole.
- Hover a time plot to read the nearest actual recorded samples. Curves connect sampled values; there is no smoothing or decreasing envelope. Applied-input samples are time-stamped at the end of the interval over which the command was held.
- Change a setting. The status reports pending settings; Run, One step or Apply / reset starts a new trial. Display-only controls do not reset the experiment.
- New random start changes only the position seed. Use the same mass seed and paired position seed when comparing controllers.
- Download JSON, CSV or the performance PNG. Copy settings link shares initialization only. Keep the map, density, seeds, policies, quadrature, safety settings, duration and code version with every reported experiment.

## Three quick algebra tests

- Blind-mode probe sets N24,k6 and a pure nonconstant blind harmonic with relative mass RMS 0.25. The load error stays near numerical zero while blind energy stays 0.03125 under service-only control. This is not successful equal-atom balancing.
- Repeat with complementary control adds order 7 to boundary updates; physical service remains order 6. The blind harmonic becomes observable to the controller and decays.
- Coprime sets N25,k6, whose exact blind dimension is zero. Slow finite-time decay is still possible; gcd=1 is not a uniform rate theorem.

If a blind harmonic is requested at coprime N,k, the UI explains why it is unavailable. If k=N, all nonconstant harmonics are blind, so an Observable harmonic probe is unavailable. The physical canvas supports 3<=N<=80 for tractable packet/geometry computation. For N=1000,k=21 use the large-N arithmetic study linked above the setup panel or the algebra explorer.

## Metric dictionary

Demand mass is normalized to 1, m_i is the basic atom mass, ell_i is the k-window load and d=gcd(N,k). Ideal basic mass is 1/N; ideal responsibility mass is k/N. Population-dependent targets are recomputed after removal.

| Display | Definition | Interpretation |
| --- | --- | --- |
| Load error | RMS(ell-k/N)/(k/N) | Equal responsibility workloads when zero |
| Mass error | RMS(m-1/N)/(1/N) | Equal underlying atoms when zero |
| Blind error | RMS(orbitMean(m,d)-1/N)/(1/N) | Service-invisible part of the atom imbalance |
| E_mass | massError^2/2 | Total coordination energy; dimensionless |
| E_blind | blindError^2/2 | Projected blind coordination energy; invariant for one service order |
| E_service | loadError^2/2 | Available in exports; zero does not imply equal atoms when d>1 |
| E_K | Mean relative order-load error squared / 2 | Fixed control-family energy; available in exports |
| Applied velocity RMS | sqrt(mean_i norm(u_i)^2) | Actual safe, scaled robot command in domain units/s |
| Boundary input RMS | RMS(dx_i/dt) | Cumulative-demand ray input in mass fraction/s, not robot input |
| H | Assigned squared-distance integral / (2k) | Euclidean geometric service cost, squared domain units |
| Centroid lower bound | Sum(second moment - first moment squared / load)/(2k) | Unconstrained bound for the displayed partition, not necessarily feasible |
| Clearance | Minimum wall, obstacle and pair clearance | Domain units after subtracting robot radii; negative means infeasible |
| Fourier gain G_j | Squared magnitude of R_k(j)/k | Structural response, independent of the initial state |

Log plots floor values at 10^-15 for display only. Exported values retain their numerical values. These energies are not mechanical energy. H is not replaced with a monotone envelope. Changing rays, changing assignments or removing robots can increase H; constrained robots need not reach unconstrained centroids. A completed duration is not a convergence certificate.

Read [gcd-sensitive costs and proofs](cost-theory.html) for the classification, multi-order theorem and counterexample showing H does not determine gcd.

## Failure workflow

Open Failures. Default N50->49, service k7, equal initial masses and three holes isolate the effect of a single announced removal. Single and complementary policies start with identical robot positions and atoms. Optionally add persistent switching. Unequal or harmonic profiles let the policies diverge before failure; use equal masses for a clean fault-only comparison.

Choose the failed physical ID and removal time before starting. Before removal and After removal compute to the event if needed and select exact instantaneous snapshots. Survivor positions and persistent IDs are unchanged by the event. The red cross marks the removed position only once that robot is absent. The robot is removed, not left parked as an obstacle. The left boundary of its atom is deleted, merging two adjacent positive atoms. k must remain no greater than the surviving population.

The timeline scrubs recorded states; Resume continues computation from the latest state, not from a rewound state. Event snapshots have no new applied command: CSV fields for command-dependent quantities are empty there. Normal held-input records before/after the event remain intact. Population, gcd and blind dimension are shown in the event banner. Exact equal-mass single-removal residual formulas do not apply to arbitrary unequal states or multiple failures.

The new dashboard only models one announced removal. Other frozen-responsibility or failed-obstacle semantics need a different model. The structural comparison page contains separate failed-location and multi-failure arithmetic studies; do not relabel them as physical multi-failure validation.

## Dynamics workflow

Dynamics opens a fixed-partition, no-failure experiment to isolate actual plant behavior. Choose single/double/higher-order integrator, unicycle, differential drive or the two-coordinate Euler-Lagrange mechanism. Read its equation and native-input labels before comparing curves. Higher derivatives start at zero. Euler-Lagrange holding force can remain nonzero at rest.

Cartesian higher-order and nonholonomic examples do not inherit arbitrary-obstacle safety from the new homepage. The older annular single-integrator chart has its own containment result; it is not an arbitrary multi-obstacle collision-avoidance proof. Native acceleration, jerk, wheel-rate and force inputs cannot be compared as a single physical-energy unit. The page reports model-normalized effort with that limitation. See [models and guarantees](theory.html).

## Fair comparison workflow

Comparisons separates physical motion from arithmetic and packet benchmarks. The physical comparison uses the same initial physical sites, demand quadrature, obstacle map, speed and safety model for cyclic and full order-k Voronoi dominant regions. Nearest-k minimizes assigned distance cost at fixed positions, so its lower H is expected. Evaluate load error, information requirements, packet cost and repair alongside H; do not manufacture a weighted score to make one method win.

The structural lab contains a balanced fractional-flow reference, cyclic phase control, a symmetric coincidence test, nested population sweeps, routed protocol replays and failure/capacity studies. Its phase search and capacity solver are offline references. Protocol round counts are accounting, not a proven delayed whole-body robot simulation. Preserved PNGs are explained individually in the archived figure guide; they are not new live simulations.

## ROS 2 and enterprise robots

The ROS 2 recording page is an offline integration bench, with no connected robot and no command publisher. Its prominent TIAGo Pro link opens a separate, now implemented whole-body MuJoCo simulator. Select TIAGo Pro, TIAGo++ or G1 and download a mission adapter draft. It deliberately requires the exact model revision, vendor interface and topic mapping. Selecting a platform does not instantiate its full dynamics.

Use the view-angle slider to see why two responsible robots can still give singular bearing information. Then load the explicitly synthetic contract example or import telemetry JSON. Choose physical ID, joint and recorded frame; efforts retain N or N m units. Invalid files are rejected while the previous valid replay remains. The contract limits files to 5 MB and 1000 frames. Declared hardware provenance is not independently authenticated by the viewer.

Read [enterprise robot dynamics](robot-theory.html) and [ROS 2 contract](ros2-contract.html). The separate TIAGo Pro lab now runs a declared planar-supported whole-body model and acceleration guard. Vendor hardware connection, wheel/roller simulation and hardware tests remain future work.

## Reproduce from source

Use Node.js >=20 (CI uses 22). There are no production package dependencies. From the canonical site directory run:

```text
node tests/run-tests.cjs
node tests/obstacle-tests.cjs
node tests/structure-tests.cjs
node tests/lab-tests.cjs
node tests/viewer-harness.cjs
node tests/comparison-viewer-harness.cjs
node tests/structure-viewer-harness.cjs
node tests/lab-viewer-harness.cjs
node tests/ros2-viewer-harness.cjs
node tests/validate-benchmarks.cjs
node tests/validate-structure-benchmarks.cjs
node scripts/build.mjs
node tests/validate-build.cjs
node scripts/serve.mjs
```

Open the local URL printed by the server. A static HTTP server is the supported reproduction path; do not infer a failed file:// script policy is a simulation failure. dist is generated; edit src and docs. Keep the existing hosting identity. A GitHub source commit and a Site deployment are separate operations. Source access does not grant website-publishing rights.

Test reports distinguish numerical tests, synthetic DOM/event harnesses, static resources and live HTTP verification. Synthetic DOM tests are not real browser QA. Previous native browser safety checks prevented verified browser interaction; this release must retain that verification limit unless a permitted browser check later succeeds. Check your target browser and screen size before collecting publication screenshots.

## Theory and future surfaces

Theory is intentionally the last navigation item. Its catalog links operators/proofs, metric choices, dynamics, comparisons, whole-body robots and a future surface adapter. Add new documents to the central theory catalog to extend it without crowding the homepage. Several planar holes are not a curved-surface or positive-genus implementation. Intrinsic distance, metric volume, chart handling and feasible motion must be implemented and verified before enabling a Riemannian surface simulator.

## Troubleshooting

- Invalid N,k or impossible profile: correct the parameters and press Apply / reset; the previous trial is retained.
- Run says blocked: inspect carrier/budget and the event log. Do not interpret a stopped run as convergence.
- Load error is tiny but masses differ: inspect blind energy and Fourier roots; try the complementary controller.
- Inputs approach zero but centroid excess remains: obstacles or other robots can constrain motion. Clearance and residual must be reported together.
- H rises: compare boundary and motion contributions in the specialist pages. Only fixed-partition motion descent has the stated monotonicity guarantee.
- Cannot find a page: use the seven-item top navigation. The older filenames remain available for existing links, but the homepage is now the core experiment.

## TIAGo Pro whole-body simulation

Open the TIAGo Pro lab from Dynamics or ROS 2. Wait for the pinned 9 MB MuJoCo runtime and model to load; initialization failure is shown explicitly, with no point-plant fallback. Choose a preset, N=3..8, service k, map, seed and duration. Apply / reset starts an identical paired initialization. Run toggles pause; One coverage epoch advances 0.1 s through twenty actual EL integration steps. Display speed changes epochs per screen frame only.

The map shows normalized coordinates (10 m per unit); performance uses m, s, N and N m. Select a physical ID to inspect its actual articulated link transforms, left/right shoulder torque and lift force. Scrub recorded states with the timeline; Resume computes from the latest state. Removal snapshots retain survivors and have no new native-input samples. Export JSON for full joint states and CSV for common coverage metrics.

Planned load curves coincide because the same virtual-boundary law is used. Compare physical cost, actual velocity, native forces, tracking residual and execution fractions separately. K=1 cannot complete the stated simultaneous two-robot view task. K=2 only allocates enough distinct robots; view geometry and dynamics still limit actual completion. The fixed-ray and payload presets isolate tracking and load effects. The upper-body selector can hold posture or run representative inspection scans.

The model is the PAL-published articulated inertia tree with planar support and declared ideal actuators; it is not factory-identified dynamics or a wheel-roller simulator. A 0.04 m additional acceleration-guard buffer is explicit. Contacts are compliant and may penetrate slightly; report that value and saturation instead of assuming collision-free convergence. Read [the exact model, controller and allocation interpretation](tiago-theory.html).

New setup commands are node scripts/install-physics.mjs before physics tests/build, then node tests/tiago-tests.mjs, node tests/tiago-viewer-harness.mjs and node tests/validate-tiago-benchmarks.cjs. Run node scripts/run-tiago-benchmarks.mjs to regenerate the six recorded cases and the local full metrics archive. Hardware ROS 2 remains offline; the new feature is an implemented articulated physics simulator.

## Execution-aware allocation

Open Execution_Allocation.html from Overview or Comparisons. It opens a recorded SAME-TRAJECTORY REPLAY first. Left is capacity-admitted cyclic ownership; choose repair, nearest-k or feasible geometric ownership on the right. Every method uses the same physical trajectory. Scrub the timeline and select a physical ID to connect its path color, region ownership and optical heading. Read the coverage curve against the pose ceiling: allocation cannot exceed visibility supported by the current robot poses. Export JSON or metrics CSV to retain this experiment separately from finite jobs.

Choose Closed-loop task mission to see the four actual EL fleets. These are recorded 45 s missions initially. Run recomputes them with MuJoCo in your browser. Set N=3..8, k=1..3, seed, duration, carrier and budget, then Apply / reset. One epoch advances0.1 s; display speed only changes epochs drawn per animation frame. A changed setting never silently alters the currently displayed recording. Rapidly switching tabs discards stale recording responses. All four physics fleets can be computationally expensive; use One epoch and a short duration while exploring.

For the default case keep N6,k2,ten jobs,250000 byte-hops per one-second allocation epoch, heterogeneity and the resource reservation enabled. Colored job pies show committed robot owners, green crosses completed jobs and gray pending jobs. The outer sector bands show the CYCLIC REFERENCE rather than claiming every repaired task still lies in that sector. ID2 is booked during12..27 s. Enabling announced removal removes ID3 at20 s, preserves survivor state and reindexes roles. Use at least21 s for this case. Try N5,k2 (gcd1), N6,k3 (gcd3), or N7,k2 followed by removal to N6 (gcd1->2). N/k changes also change capacity and coalition size; physical completion differences cannot be attributed to gcd alone.

The confirmed completion curve requires six consecutive delivered0.1 s reports from all k owners, valid range/FOV/3D LOS and at least one noncollinear observer pair, followed by routed completion announcements. Current service is measured only on unfinished jobs and can be zero after success. Reference workload error follows R_k; admitted weighted-job RMS measures actual allocation imbalance and is n/a when no active work remains. Full-map H uses the same nearest-k distance cost for all fleets. Chassis speed and arm torques use m/s and N m; native actuator work is not electrical energy. Byte-hop curves include actual topology, snapshots, bids, capacity tokens, prepare/ACK/commit, observations and notifications.

To inspect limits, reduce budget to18000, select the range carrier and make its range very small, or choose k1 for this inherently two-view job. A missing commit retains a prior valid lease or leads to inertial braking after expiry. Missing evidence resets dwell, rather than certifying a task. These runs can remain incomplete. The event log and JSON expose the reason. The byte budget is for each one-second allocation window including its observation reports; it is not a measured Wi-Fi throughput guarantee. Range is displayed in meters.

The 12.44% preserved metric, same-state repaired25.13%, and finite-job cumulative completion have different meanings. Read execution-theory.html before making a performance claim. The execution-aware geometric baseline is deliberately strong; cyclic repair is not guaranteed to beat it. All methods share model, scene, initial sites, targets, capacity and detector; centroid baselines and viewing-pose repair differ in their control stack. The replay isolates allocation changes alone.

For N<10, persistent physical IDs have distinct colors in paths, robot markers and basic regions. Low-order overlap is shown with colored bands instead of pretending each point has only one owner. Labels survive role reindexing. This applies to the core dashboard, small-order comparison views and new allocation maps; larger fleets retain uncluttered aggregate colors.

## Dual-arm transport

Open Bimanual_Transport.html from Dynamics or ROS2. Wait for actual MuJoCo and setup-only arm IK. Keep2 kg,1 m,both hands and press Run transport. Follow lift, translate, lower, support verification, release and settle. The robot skeleton and free tray are read from actual physical state. Initial fixture preparation is not autonomous grasp acquisition.

Inspect actual hand forces near3 s: each hand carries about9.81 N for this2 kg tray. The force curve comes from the solver; the m*g/2 line is just a reference. Placement error, actual support contact, speed and tilt gate release after13 s. A successful15 s result leaves the released object on the delivery fixture. Native shoulder torque, grasp-anchor error and contact penetration remain visible. Scrub the timeline, export full physics JSON or CSV, and reset before changing payload or distance.

Try3 kg and1.5 m, then left-hand-only and no-grasp counterchecks. Failure remains visible; the tray is not snapped to its goal. No-grasp includes a deep transient contact violation and is intentionally rejected. Failure of this one-arm fixture controller is not a general impossibility result. Both arms belong to ONE robot. Their coupled manipulation is not k2 independent-robot coverage.

The fleet reservation and the tray experiment are separate: the allocator books ID2 for15 s; it does not simulate that tray inside the fleet map. Read bimanual-theory.html for ideal grasps, planar support, controller and model limitations. Hardware remains disconnected. There is no finger slip, wheel roller, tipping or pickup perception model.

## Reproduce the execution experiments

From the site directory use the following commands. All recorded comparison data are retained compressed in assets/result; build expands them for static hosting. Full packet/state research outputs stay in the ignored research/execution directory.

```text
node scripts/install-physics.mjs
npm test
npm run test:viewer
node scripts/run-execution-benchmarks.mjs
node tests/validate-execution-benchmarks.cjs
npm run build
node tests/validate-build.cjs
npm run preview
```

The experiment script runs36 actual articulated fleet trials,301 same-state visibility audits and5 physical transport cases. MATLAB scripts/plot_execution_allocation.m creates static scientific figures from the recorded compact data. Numerical validation, synthetic DOM/events, static build checks and anonymous HTTP are distinct. Prior native browser safety restrictions remain; this update does not claim verified browser layout or hardware operation.


## Integrated inspect carry confirm

Open Integrated_Mission.html from Overview, Comparisons, Dynamics or ROS2. It loads the recorded N6/k2/seed2718 shared-world mission. All robot copies and the free tray belong to ONE MuJoCo model and clock. A real pre-inspection certificate authorizes a resource grant, actual transport changes capacity, and actual support/release plus fresh external views confirm placement. The isolated fixture and earlier scheduled-resource page remain separate archived experiments.

First use the Recorded scenario selector and Load recorded comparison. Try the three position seeds, N5/k2, N6/k3, N7-to-N6 observer removal, right-hand grasp loss, a tight shared budget, disconnection and insufficient external observers. Read the four-policy table: a completed time horizon does not mean a successful task chain. The grasp fault occurs six seconds AFTER each policy's actual carry grant; check whether it triggered before interpreting that case.

The primary comparison uses one prospective-viewpoint executor for fixed cyclic owners, nearest available-k owners, cyclic repair and geometric repair. It shares exact initial full states, targets, capacities, robot/object dynamics, carrier controller, workcell and evidence tests. This comparison isolates ownership-policy differences more directly than the older centroid stacks. Nearest available-k is a resource-constrained task baseline, not a global optimal constrained Voronoi solver.

To compute a new comparison, set N=3..8, k=1..3, seed and5..60 s, then Apply / reset. Expand fault/communication/payload settings to change the carrier graph, range in meters, admitted byte-hop budget,1..3 kg tray and0.5..1.5 m travel. Observer removal requires k<=N-1 and at least13 s; ID3 is removed at12 s. Carrier ID1 remains physically present. Run toggles pause; One step advances all four actual fleets by0.1 s; display speed changes steps per animation frame only. Changes retain the displayed result until you apply them. Invalid settings show an error without silently changing an old run.

Select Detailed policy to inspect its actual articulated carrier, tray, hand forces, resources and event log. Every path and robot color follows its persistent physical ID. Gold is the tray; dashed circles are a shared reserved workcell. Colored sector bands are the CYCLIC REFERENCE, while job pies are actual coalitions. The waiting placement task is dependency-blocked, not an invisible completed job. Event-enriched recordings are sampled at least every0.5 s; the cursor chooses the latest recorded state at or before a common physical time, and each map labels its actual state timestamp. Live runs have0.1 s task sampling.

The carrier begins at a pre-grasp fixture. Before grant it holds base yaw, torso and arms; only a reachable head-only camera slot is available. During actual transport/settling that slot becomes zero, even while an old lease still names it. A successful support/release notification restores the stationary camera slot. Hands are already occupied by the initial fixture, and autonomous pickup/retract/re-grasp transfer are outside this mission. A hand failure quarantines the payload and keeps resources occupied instead of fabricating recovery.

End-to-end workflow success needs pre-inspection, a routed grant, actual carry/release/support and a fresh external confirmation. Background completion is reported separately. Carrier busy duration is integrated from real execution intervals, not a fixed release timer. H uses the same complete map for every method. Speed is measured in m/s, arm torque in N m and hand force in N; actuator work is not electrical energy. The communication ledger includes actual allocation, target, lifecycle, physical-state and observation packets. Mandatory topology discovery is charged before data admission, so the setting is not a strict full-radio throughput cap.

The reference N counts active complete robots, including an occupied carrier. Occupation changes availability without changing N or gcd(N,k). Actual announced removal changes N and reconstructs the reference. For N7->6,k2, gcd changes1->2; during a carry at N6 the available cameras can be5 while the reference still has N6. The reference mass error can be tiny while a task is blocked by missing external views, a failed grasp or communication. Read integrated-theory.html before interpreting the arithmetic.

Scrub the timeline to inspect grants, resource changes, grasp release and fault events. JSON exports contain joint/object state, task versions and packet/event histories. CSV exports keep background completion, end-to-end success, busy duration, N/k/gcd, H, bytes and native hand forces in separate columns. A physical placement disturbance invalidates confirmation evidence; a failed or missing certificate does not turn green when the horizon ends.

Reproduce the new mission from the site directory:

```text
node scripts/install-physics.mjs
node tests/integrated-tests.mjs
node tests/integrated-viewer-harness.mjs
node scripts/run-integrated-benchmarks.mjs
node tests/validate-integrated-benchmarks.cjs
node scripts/prepare-integrated-figures.cjs
```

Then run MATLAB scripts/plot_integrated_missions.m, build with node scripts/build.mjs and validate with node tests/validate-build.cjs. The runner saves40 full trial recordings under ignored research/integrated, plus compressed public comparison files under assets/result. All accepted previous figures remain. These are numerical experiments with conditional protocol proofs; browser rendering and hardware ROS2 remain unverified under the prior safety restrictions.
