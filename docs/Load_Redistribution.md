# Contact-only transport for coverage with movable demand

The experiment starts with a narrow source patch centered at the pickup support. Its angular width is 0.05 rad, about 2.86 degrees. The patch contains 14.5% of the total workload as stationary demand and another 14.5% assigned to a movable parcel. The remaining 71% is the fixed background. Parcel mass in kilograms and workload share are independent parameters.

## The demand field and measurements

Let p(t) be the integrated planar center of the free parcel. The experiment uses the field `rho(t) = 0.71 rho_background + 0.145 K_source + 0.145 K_p(t)`. Each component integrates to one before weighting. The two compact patches distribute their weights uniformly over their angular support and use the patch radius for radial moments. Angular-bin weights integrate the patch overlaps exactly. The background and moment quadrature are shared across policies.

The moving patch follows the parcel during pickup, transport, an emergency set-down or a drop. The credit for a certified relocation stays at zero until the parcel is released on the selected delivery pad and satisfies the support and dwell checks. This distinguishes a change in measured demand location from completed work.

The same fixed-angle partition, with a boundary at pi, evaluates the initial and relocated fields. Its order-k load imbalance is the relative RMS around the mean assigned load `k/N`. A second comparison evaluates nearest-k load imbalance at the same frozen initial robot positions. Both separate the field change from movement of the robots or changes in the angular boundaries. The constant no-transfer reference holds the parcel at the source on that frozen geometry.

The physical angle of a cyclic boundary is the inverse angular mass cumulative distribution at the current boundary coordinate. High mass density can force adjacent boundaries close together even when responsibility loads are equal. We show the smallest current angle and, separately, the best smallest angle among 72 sampled phases of equal-mass sectors. The phase scan is a finite diagnostic. It can reveal whether distributing the demand makes balanced sectors less pinched.

Every policy chooses its final delivery pad from the same finite menu. The shared score combines fixed-angle imbalance (weight 0.6), frozen-pose nearest-k imbalance (0.4), sampled equal-mass angular pinch (0.12) and travel distance (0.006 per meter). The same corridor, robot dynamics, seeded poses, bid capacity, force controller, communication budgets and outcome certificates apply to fixed cyclic, cyclic repair, nearest-k Voronoi and geometric repair. Policies differ in assignment and carrier bids. This comparison isolates those policies while giving each the same transport capability.

## Physical acquisition and friction

The scene has a free parcel, four driven parallel jaw pads per robot and zero equality constraints. Open jaws and folded arms start separated from the parcel. The acquisition screen checks sampled clearance and joint margins along the proposed segments. The executing robot backs away, raises its torso, extends while clear, advances with its tools elevated, and lowers toward the handles. A measured two-tool alignment gate allows bounded corrective references before closing. Velocity, acceleration, force and torque commands are limited. No executing pose or velocity is reset.

The original light-payload failure began during hover before closure. With the clearance maneuver, measured alignment, bounded drive and revised force loop, the same seeded 1 kg case selects robot 4 and completes the delivery. This is an experiment on the planar-supported vendor-derived model, with experimental gripper geometry and simulated state feedback.

The jaw controller searches at low force until contact, then regulates each pad's measured normal load with a bounded damped force command. Its nominal per-pad target is `min(22 N, 2.35 m g / (4 mu))`. Four opposing contacts and a measured force and dwell gate authorize a trial lift. The screening inequality `mu sum(F_normal) > 1.22 m g` is a scalar capacity test. Actual support-free transport depends on the whole contact wrench, actuator authority, alignment and slip. MuJoCo's elliptic contact model includes tangential and torsional friction. Explicit pad–handle pairs specify the coefficients and contact softness. The reported slip is the maximum tangential relative velocity at an active pad–handle contact. A deliberately reduced jaw-force trial tests contact loss and retains the failed outcome.

## Finite supports and powered fault recovery

The four pads are separated physical platforms. Source and destination support normal pickup and delivery. A degraded powered chassis can move slowly within a 0.52 m envelope while its arms and jaws remain available. Each candidate refuge is checked for residual travel, base corridor clearance and a replacement robot's opposite-side joint reach. The viable shortest move is selected. The original carrier puts down the actual parcel, waits for upward contact load and a stable centered pose, and opens the jaws. Stable support and an open-grip dwell precede release certification and the replacement auction. The failed carrier withdraws from the workcell and remains a physical body. The replacement grips the same parcel and follows the same release and external confirmation rules. A force-loss counterexample leaves the transfer incomplete if contact cannot be maintained.

The support margin measures the parcel center's distance to the edge of the supporting top. The tray ends and handles can overhang while the center is supported. The certificate checks a centered margin, upward contact load above 0.84 mg, speed, tilt, height and post-opening dwell. Numerical collision penetration and soft joint-limit excess are reported explicitly. The supports, residual chassis authority and mapped routes are model assumptions for the recovery experiment.

## N, k and gcd

For N active cyclic roles, the order-k reference maps basis masses `m` to `R_k m` with `R_k = I + P + ... + P^(k-1)`. Each column counts k assignments. The step-k graph has `d = gcd(N,k)` orbits of length `N/d`. The invisible zero-sum directions have dimension `d-1`. The moving parcel changes the input mass vector and the inverse angular mass map. The arithmetic stays attached to the reference operator. Consecutive complementary orders remove common blind directions, with the existing mass-coordinate controller retaining that reference design.

N6/k2 has two orbits and one blind direction. N5/k2 has one orbit and no blind direction. N6/k3 has three orbits and two blind directions. Increasing k also requires more independent observers. Physical availability, a powered fault, actual grasp feasibility and routed admission are evaluated separately. In a role-retirement trial N changes because the active role set changes, while the physically present parked body stays in the scene.

## What the recordings show

The six matched scenarios contain 24 trials and 15 completed deliveries with fresh external confirmation. Fixed cyclic reaches a friction grasp in the recorded cases and waits for fresh view authorization through the horizon. The reduced-force cases retain zero certified relocation after grip loss. The early and late powered fault trials produce six certified handovers across the three completing policies. The early fault selects refuge A. The later fault selects refuge B. Each recorded planner lists the distance and admission verdict for both refuges.

In the default N6/k2 light trial, robot 4 confirms at 71.0 s under cyclic repair, 72.5 s under geometric repair and 77.1 s under Voronoi. In N6/k3, geometric repair confirms at 71.1 s, Voronoi at 73.8 s and cyclic repair at 84.8 s. The comparison therefore depends on the task and team configuration.

For the default light trial, the concentrated source share falls from 29% to 14.5% and fixed-angle load RMS falls from 42.00% to 29.71%. The best sampled equal-mass minimum angle grows from 10.24 to 26.19 degrees. The actual moving reference reaches a transient minimum of 3.68 degrees and ends at about 16.72 degrees. The difference separates the field's potential for better sector geometry from the trajectory of the existing boundary controller.

## Reproduction and sources

Run `node scripts/run-redistribution-benchmarks.mjs all` and then `node scripts/run-redistribution-benchmarks.mjs --aggregate` from the site checkout. The comparison page exposes compact recorded states, exact event times and live controls. The native renderer takes the full-state recording from the research directory. It draws the measured qpos and object position at their recorded times. MP4 provenance includes the source hash and decoded frame count.

[MuJoCo contact computation and elliptic friction cones](https://mujoco.readthedocs.io/en/stable/computation.html)

[MuJoCo XML contact-pair parameters](https://mujoco.readthedocs.io/en/latest/XMLreference.html#contact-pair)
