# TIAGo Pro whole-body dynamics and actual allocation

The original baseline below and its12.44% planar two-view result are preserved. The new [execution-aware allocation lab](Execution_Allocation.html) adds capability bids, routed leases and actual evidence; the [dual-arm transport lab](Bimanual_Transport.html) adds a free physical object. Their metrics and assumptions are defined separately in [execution theory](execution-theory.html) and [transport theory](bimanual-theory.html).

## What is implemented

The TIAGo comparison page runs the official MuJoCo 3.8.1 WebAssembly engine with a pinned PAL Robotics articulated model. It integrates forces into velocities and configurations. It does not replace a point icon with a robot drawing while retaining single-integrator motion. Every active physical robot contributes 20 independent configuration and velocity coordinates, 20 bounded ideal actuators, a complete articulated inertia tree and primitive contact geometry. Six robots therefore have 120 dynamic coordinates.

The source is [PAL's TIAGo Pro MJCF at the pinned revision](https://github.com/pal-robotics/pal_mjlab/blob/952abb83a590d0e771cfd8e570517ba054182196/src/pal_mjlab/robots/pal_tiago_pro/xmls/tiago_pro.xml). The [model preparation script](https://github.com/XunFengou/cyclic-responsibility-lab/blob/main/scripts/prepare-tiago.py) and downloadable provenance record enumerate every modification. All 34 published explicit inertial entries are preserved exactly. Mesh visuals are removed; primitive collision shapes remain. The on-screen skeleton comes from actual MuJoCo body transforms.

This is a real articulated EL simulation of a declared vendor-derived model, not an identified digital twin of a purchased robot. The published inertial entries sum to 40.9428932 kg. That number describes THIS simplified simulation asset, not the factory robot's measured mass. Keeping an official model's provenance does not make its inertias calibrated hardware measurements.

## Why this robot can be one multi-agent unit

One agent is one independently actuated mobile manipulator, with its own physical ID, base, torso, head, arms and local state. Joints are internal coordinates of that agent; two arms are not two independently survivable robots. A fleet consists of several such complete robots. In the simulation each copy has namespaced bodies, joints and actuators, and an independent configuration/velocity vector; contact couples robots physically, while responsibility messages couple their allocation layer.

The [TIAGo Pro product description](https://pal-robotics.com/robot/tiago-pro/) documents its mobile base, dual arms, sensing and ROS 2 software. The [manufacturer's robot repository](https://github.com/pal-robotics/tiago_pro_robot) and [head model](https://github.com/pal-robotics/tiago_pro_head_robot/tree/a10e252dc55968c6ac91d2a84eb4626a4811bc78) make a model-based integration possible. These are reasons it is a feasible experimental agent platform. They are not evidence that a multi-robot field deployment has already been validated here. Each real robot would still require a supported controller, localization, calibrated sensing, clock/TF consistency and fleet coordination.

TIAGo Pro has an omnidirectional base. A differential-drive no-sideways-velocity constraint is inappropriate for this selected platform. The current reduction permits planar x/y/yaw motion and models its required chassis generalized force; Mecanum wheel/roller contacts, slip, suspension and wheel-motor loops are not modeled. A claim about actual wheel torque or rough-ground stability would require a different plant. Model size is deliberately limited to 3..8 robots in the browser; the N=1000 arithmetic lab is not a 1000-TIAGo experiment.

## Configuration and EL equations

For robot a, use q_a=(x,y,psi,z_t,pan,tilt,theta_L,theta_R), where x,y are chassis position in m, psi is chassis yaw, z_t is torso height, pan/tilt are head joints and theta_L/theta_R each contain seven arm angles. There are 3+1+2+7+7=20 coordinates. Chassis vertical translation, roll and pitch are constrained by planar support and are eliminated. Wheel and gripper-finger bodies retain their published inertia but are fixed relative to their parent in this reduced model.

```text
q_dot = v.
M(q) v_dot + b(q,v) = tau_act + tau_passive + J_contact(q)^T lambda.
```

M is the full articulated mass matrix, b includes Coriolis/centrifugal and gravity terms, tau_passive contains modeled joint damping and lambda contains unilateral/friction/contact-limit reactions evaluated by MuJoCo. The inertia matrix is configuration dependent and has nonzero base/arm coupling. It is not twenty unrelated diagonal scalar plants. The browser uses MuJoCo's mass-matrix product and bias terms at every 0.005 s step.

The published asset fixes the torso and head. The preparation script restores the torso slide in [0,0.35] m and head pan/tilt with the pinned head axes and limits. The head effort limits are 5.197 and 2.77 N m. Published arm force limits and joint ranges remain; extra rotor inertia and damping are explicit experimental parameters. The model uses ideal bounded motors: x/y chassis force is capped componentwise at 250 N, yaw moment at 100 N m, torso at 2000 N and arm joints at their declared ranges. These are simulation interface assumptions, not a statement that an installed TIAGo exposes these raw motor commands.

The payload experiment attaches a declared 2 kg rigid payload at the left tool of even-numbered physical robots, with a small declared inertia. It changes those robots' mass matrices and gravitational effort without changing the demand map or service multiplicity.

The joint allocation/plant state is a product of the positive mass simplex, N copies of (Q,TQ), a role-to-physical-ID permutation and the communication graph. R_k acts on the mass/role coordinate, not on the full physical inertia matrix. Contact geometry, heterogeneous payload and unequal postures need not have circulant symmetry. A gcd classification of responsibility blindness is therefore not a classification of every mode of the entire robot plant.

## Same allocation and same environment

Both plants use a disk of radius R=10 m, the same circular obstacle union, normalized hotspot demand, quadrature, seed, initial physical chassis sites, cyclic role order, service k and virtual-boundary updates. Robot positions are initially feasible, seeded and separated. Their corresponding initial base positions are exactly equal; EL velocities start at zero. All model settings and input limits are recorded in JSON.

The ideal reference obeys p_dot=u and applies bounded, safety-filtered Euclidean-centroid velocity requests, with speed limit 0.45 m/s. Its chassis footprint proxy is radius 0.55 m and desired proxy clearance is 0.08 m. It has no joint torques or inertia. Both share the same range, horizontal FOV and LOS diagnostic; the ideal reference can set its requested gaze instantaneously.

The EL nominal chassis command tracks the same Euclidean centroid velocity request through an acceleration loop. Its posture tasks move the torso and arms within stated ranges, while the head and base track an inspection gaze direction. A local histogram of assigned demand selects the best of 36 horizontal gaze headings under 7 m range, 120-degree FOV and LOS. The same heading selection rule is used for the ideal sensor, evaluated at its own current position. This is a local inspection heuristic, not an optimal joint information planner. Upper-body scan is optional; holding posture provides an ablation.

For guarded chassis acceleration, let h=||p-o||^2-r_safe^2 around an obstacle and use h_ddot+2*omega*h_dot+omega^2*h>=0 with omega=2. Similar inequalities enforce the disk interior and pairwise footprint separation. They are affine in requested accelerations. The code cycles halfspace projections and acceleration-ball projections, with residual monitoring, and uses an additional 0.04 m inertial guard buffer. Chassis acceleration requests have norm at most 2 m/s squared. This extra braking margin is an explicit EL-controller choice, not an identical SI feedback law.

```text
a_des = guarded chassis acceleration plus bounded joint task accelerations.
tau_requested = M(q) a_des + b(q,v) - tau_passive.
tau_command = componentwise actuator-limited tau_requested.
```

There is no subtraction of contact reaction to cancel collisions. Joint motion and contact remain part of the integrated plant. Saturation can invalidate a requested acceleration barrier. Primitive contacts can penetrate slightly because the model uses compliant numerical contact. The recorded experiments report footprint margin, contact count, sampled penetration and saturation, without claiming an unconditional collision-free tracking theorem. No endpoint clipping, position teleportation or velocity reset is used.

## Why the planned load curves coincide

Basic masses m are positive with sum 1; ell=R_k m is the logical responsibility load, with desired k/N. The virtual rays are independent of robot polar angle and joint position. Both plants receive the same positive cumulative-demand boundary update:

```text
m_next = (I-beta*average_(h in K) Q_h) m,
Q_h = 2I-P^h-P^(-h),     beta=0.1.
```

Thus their planned load errors coincide exactly, by construction and by the decoupling theorem. This isolates the plant effect and prevents falsely attributing a different gcd law to a more complicated robot. For a single order the blind dimension remains gcd(N,k)-1; adding an adjacent order removes the common nonconstant kernel. Geometry, sensing and actuator limits can still prevent physical execution.

The numerical benchmark assumes complete, reliable, known-map updates and centralized access to the physical simulation state for collision handling. The packet counter exported with the shared partition reference is not a measurement of ROS 2 traffic for whole-body state streaming. No asynchronous radio/whole-body stability or hardware communication saving is inferred.

An announced removal deletes one robot and one virtual boundary. The scene is recompiled without that physical robot; every surviving generalized position and velocity is copied exactly by persistent ID and joint name. Solver warm-start/contact memory is not a physical robot state and is reinitialized. The robot is removed, not parked as an obstacle. Current full-body fault case is N7->6,k2 at 10 s, which changes d=1 to d=2; order 3 repairs the new common blind mode. Autonomous failure detection is not implemented.

## Actual allocation, execution and K-th meaning

Let task sample y_l have demand weight w_l. Logical assignment a_al is binary, with sum_a a_al=k over DISTINCT physical robots. Cyclic responsibility restricts this assignment to a fixed window-incidence template. Its planned load is ell_a=sum_l w_l*a_al.

Define e_al as a_al times the indicators for sensing range, horizontal FOV and obstacle LOS. A location is k-visible only when all k assigned robots pass those tests. For the cooperative inspection mission, require a simultaneous noncollinear pair as well: at least two eligible assigned bearings must satisfy |sin(alpha_a-alpha_b)|>=sin(25 degrees). This excludes near-parallel and near-antiparallel bearing geometry. The weighted fraction of completed samples is the second execution metric.

In the ideal independent bearing-noise model, a single view gives a rank-one information contribution. Two nonparallel independent views give rank two. Therefore K=1 cannot complete this specified simultaneous two-robot localization/inspection task, even when its distance cost is lower. A moving single robot could acquire sequential views under another target/time model; that is a different task assumption. Multiple cameras on one robot are not counted as independent robot redundancy.

K=2 is a necessary allocation multiplicity for this task, not a guarantee of visibility or rank everywhere. Joint limits, gaze slew, payload, collision constraints and occlusion affect execution. The plotted finite-horizon task fractions remain far below 1, so this simple centroid-plus-gaze controller does not solve full sensor-feasible task allocation. The gap is a measured research result, not hidden behind equal planned loads.

For heterogeneous agents, equal ell is not equal completion time or equal energy. A practical extension should optimize assignment under per-robot task duration, reachability, information and capacity constraints. One possible model minimizes total predicted service cost with sum_a a_al=k, eligible-robot restrictions and individual capacity bounds; the current cyclic template is a structured restricted assignment family. The present implementation demonstrates the plant and allocation/execution gap. It does not claim an optimal capacity-aware allocator or full manipulation task completion. Arm scans are representative inspection posture tasks, not a simulated grasp of a real object.

## Metrics and units

| Quantity | Meaning and fair interpretation |
| --- | --- |
| Planned load RMS | RMS(ell-k/N)/(k/N); same partition reference for both plants |
| E_mass, E_blind | Dimensionless mass and blind-projection energies; not mechanical energy |
| H | Assigned squared-distance integral / (2k), in m squared, using actual chassis positions |
| Chassis speed RMS | Common physical output in m/s; SI input versus EL velocity state |
| Base force RMS | Actual ideal x/y motor force, N; there is no SI force counterpart |
| Arm torque RMS | Actual arm motor torque, N m, separate from torso force in N |
| Absolute actuator work | Integral sum |tau_j*v_j| dt, J for the ideal actuators; not electrical consumption |
| k-visible demand | Fraction passing all k assigned range/FOV/LOS tests |
| Two-view task fraction | k-visible fraction additionally containing a noncollinear pair |
| Footprint margin | Sampled proxy clearance in m; not a swept full-arm no-collision certificate |
| Contact distance | Actual primitive-contact signed distance at recorded times; negative is penetration |

H and planned load do not measure the same goal. Native velocity and force units must not be merged into an invented common effort-energy plot. Physical H is shown without a monotone envelope. Ongoing inspection motion and gravity compensation explain persistent joint effort; finite-horizon completion does not prove a stationary limit.

## Recorded experiment results

Seven matched 30 s cases use the same seed: frozen rays, complementary moving rays, single-order moving rays, 2 kg payload, N7->6 removal, k=1, and a convex-disk hold-posture control case. Full metrics are recorded every 0.1 s when the script runs. The published compact JSON samples every 0.5 s and retains removal instants; its summaries use the full recording. No smoothing is applied. Online JSON exports retain all live frames and joint states.

For N6,k2 with moving complementary control, both planned mass errors reach numerical precision. At 30 s H is approximately 9.473 m squared for SI and 9.608 for the EL fleet. The time-averaged simultaneous two-view task fraction is about 16.9% for the ideal reference and 12.4% for the EL fleet, under this particular map and heuristic. The latter retains nonzero transient/contact/posture effects; global centroid convergence is not claimed.

Single-order control balances its service loads while retaining relative basic-mass error about 0.01947. The 2 kg tool-load experiment leaves the same partition law but increases total absolute ideal-actuator work from about 2427 J to 2599 J across the fleet. These are finite-run numbers from a declared model, not hardware power measurements. Recorded primitive contact penetration across the six cases reaches about 1.2 mm; it is reported rather than relabeled as collision-free operation.

The convex-disk hold-posture case supplies a useful countercheck: the EL fleet reaches approximately 5 micrometers RMS centroid residual at 30 s, has no recorded primitive contacts, and its lift still requires about 178 N holding force. Thus the obstacle/ongoing-scan residual is not a universal failure of the EL tracking law.

The k=1 run has a lower final H, about 4.44 m squared, but zero simultaneous two-robot task fraction by the mission definition. This is the defensible application argument for higher-order responsibility. It does not establish that larger k is always preferable or that k=2 beats a task-aware geometric allocator.

## Reproduction and evidence

```text
node scripts/install-physics.mjs
node tests/tiago-tests.mjs
node tests/tiago-viewer-harness.mjs
node scripts/run-tiago-benchmarks.mjs
node tests/validate-tiago-benchmarks.cjs
node scripts/build.mjs
node tests/validate-build.cjs
```

The runtime installer verifies the pinned npm archive's SHA-512 before extracting only known runtime filenames. Static deployment serves the engine locally; it does not depend on an external CDN at run time. Python model regeneration needs only stdlib. An optional independent audit uses Python MuJoCo 3.8.1 and NumPy; it checks the preserved inertias, coordinate count and positive-definite coupled mass matrix.

Actual WASM physics is tested headlessly in Node. The synthetic DOM harness additionally exercises the page controls with that same engine. These do not constitute real-browser rendering or hardware QA. Previous browser safety restrictions remain in force.

See [third-party models and license notices](third-party.html).

Future hardware integration should replace ideal chassis-force and joint-actuator assumptions with the purchased robot's supported interfaces and validated tracking layer. For wheel-slip, tipping or contact-rich manipulation, extend and identify the plant before transferring conclusions. The present contribution is a reproducible whole-body dynamics baseline with honest allocation and execution metrics.
