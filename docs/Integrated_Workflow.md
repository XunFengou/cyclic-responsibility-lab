# A shared-world inspect, carry and confirm mission

## What changed

The new mission couples the fleet and a free tray in ONE MuJoCo model/data pair and one physical clock. The earlier allocation lab's scheduled15-second reservation remains an archived experiment; it is not used to drive this mission. Here a routed inspection certificate authorizes transport, actual physical transport removes a robot's inspection capacity, and actual release/support plus a routed notice enable a new external-view confirmation task. The object state is never copied from a separately running animation or simulator.

The fixture starts pre-grasped, as in the previous transport lab. This integrates causal inspection, transport and confirmation; it does not implement autonomous approach, grasp acquisition or re-grasp transfer. One object workflow runs alongside six weighted background inspection jobs by default. A single complete TIAGo Pro, ID1, is the declared transport carrier. The experiment compares inspection responsibility policies, not which robot should acquire the object.

## Typed tasks, resources and state

Let I be pre-inspection, T transport and P placement confirmation. The task DAG has edges I->T->P. Background jobs B_j are independent of this chain. Both I and P require k DISTINCT external physical robots; ID1 is excluded from their coalitions. For k>=3, all k owners must see the target and at least one pair must have bearing separation satisfying abs(sin(delta))>=sin25 degrees. This does not require every pair to be separated. k=1 cannot satisfy the declared two-view requirement.

Each job has a unique ID, target, weight, sampled dwell, soft-priority due time, eligibility, progress and versioned evidence. P has a different ID and zero initial progress, and is excluded from BOTH planning and observation until its predecessors pass. Dependency-blocked jobs and eligible-but-unassigned jobs are different states. Background weights sum to0.7, and I/P each have0.15 for the admitted-work ledger. The primary reported outcomes are end-to-end workflow success and background completion separately; a mixture of these weights is not a substitute for successful object delivery.

Each robot has a resource vector for base motion, head/camera processing, left arm and right arm. The staged carrier holds its base x/y/yaw, torso and both arm postures; only its head may inspect one reachable task. Its arms already hold the pre-grasp fixture. A transport grant reserves the whole-body execution channel and sets its camera capacity to ZERO. The carrier remains occupied throughout physical transport, settling and any delayed placement notice. After actual placement and an acknowledged notice, its stationary head-only slot returns. Autonomous arm retreat and departure from the docking workcell are not implemented.

Other robots have the same heterogeneous capabilities as the execution-aware lab: head/torso limits, payload-induced inertia, speed/force limits and finite processing slots. Their prospective bids require a navigable viewing pose and bounded head/torso references. The carrier's bids are special: its base/torso/arms cannot move, and head PAN as well as tilt reachability is checked relative to fixed chassis yaw. It cannot sell an imaginary freely moving viewpoint.

The joint state is a hybrid union over active ID sets and N:

```text
state in union_N [ positive mass simplex Delta_N
                   x fleet configuration/velocity TQ_N
                   x free object pose/twist TSE(3)
                   x typed task automaton and evidence
                   x resource states, leases, versions and carrier graph ].
```

Quaternion storage uses seven object position coordinates but only six object velocity coordinates/physical freedoms. Thus nq=20N+7, nv=20N+6 and nu=20N. At N6, that is127 stored position coordinates,126 velocities and120 bounded actuators. Two ideal fixed hand welds supply twelve soft equality rows when both are active. The object still has a FREE joint and no direct actuator.

## Shared articulated and object dynamics

All robots retain the pinned PAL-derived20-coordinate planar-supported model. The same actual MuJoCo3.8.1 mass matrix, bias, passive terms, grasp reactions and contacts evolve the entire world:

```text
M(q) v_dot + b(q,v) = S^T tau + tau_passive
                       + J_grasp(q)^T lambda_grasp
                       + J_contact(q)^T lambda_contact.
```

The carrier uses the existing bounded full-mass controller, a smooth0.18 m torso lift,1 m translation, lowering/seating, and known-payload gravity feedforward split over active hands. The free-object reference acceleration is zero in the mass product; object motion comes from actual hand forces and constraints. Inspection robots use the SAME prospective-viewpoint executor for every policy, with native head/torso control and acceleration/force limits. Physical integration is0.005 s, and task evidence is sampled every0.1 s.

The carrier's15-second nominal reference begins ONLY when the resource commit reaches it. Release after13 s additionally requires actual delivery-support contact, horizontal error<0.04 m, height error<0.01 m, tilt<0.08 rad and speed<0.08 m/s. A placement certificate requires actual release, speed<0.03 m/s and those placement/support conditions, at least0.5 s of sampled stable support, at least15 s since the grant, and no contact penetration below-0.01 m. Failure to release by20 s quarantines the resources; reaching a timer never fabricates placement.

Observers plan outside a common reserved swept workcell, represented as three overlapping keepout disks around the known carrier path. They use the same keepout and obstacle navigation under every policy. Their acceleration projection is solved for observers with that reserved geometry. The carrier servo is not inserted afterward into a joint barrier solution that assumed it could freely change acceleration. This avoids that particular invalid proof transfer. Actual contacts, footpoint margins and residuals remain measured; the implementation does NOT establish collision-free whole-body convergence. Keepout disks are planning reservations, not opaque physical obstacle cylinders.

## Causal routed protocol

The existing reliable synchronous auction is extended with target-bearing request packets and lifecycle transactions. For this primary comparison every policy queries private prospective-viewpoint bids, including fixed cyclic and nearest-k. Camera FOV, range7 m,120/60 degree horizontal/vertical FOV, exact obstacle-cylinder3D LOS, capacity and actual FK observation tests are common. The known map and task catalogue are shared; current moving-object target coordinates/time are carried in actual routed requests. Observation workers evaluate the actual object as a perfect simulated sensor target; image recognition, calibration uncertainty and robot/self occlusion are not modeled.

Each one-second window shares ONE byte-hop/round ledger for topology, snapshots, target requests, private bids, capacity tokens, allocation prepare/ACK/commit, observations, physical object reports, resource grants and placement/fault notices. Mandatory topology discovery is charged before data-packet budget admission, so the setting is an admitted DATA-transaction budget, not a hard total-radio bound. Reported cumulative traffic includes that overhead; there is no free lifecycle side channel or DDS measurement claim.

Pre-inspection needs six consecutive fresh delivered0.1 s samples from all k assigned external robots, valid optical geometry and a fresh carrier object-state report. Missing evidence resets dwell. Completion announcements reach all robots before I is complete. Its certificate must be at most3 s old and the tray must remain stably on pickup support when transport is granted; otherwise I is re-enabled for a fresh check.

Lifecycle messages have a monotone workflow version and use prepare, acknowledgements and commit. The full three-batch cost is checked BEFORE any lifecycle batch is sent. A denied transaction stays pending and cannot start a physical carry clock or partially release resources. After a successful grant the carrier starts its actual motion, loses camera capacity and forces a new allocation epoch. An old inspection lease can still name it briefly but cannot authorize processing while its capacity is zero.

Actual release and stable support create a physical placement certificate. The carrier broadcasts that through the same reliable transaction protocol before P becomes eligible or its camera slot returns. During P, fresh object-state reports carry actual pose, twist, release and support information to the job coordinator. Those physical predicates must remain valid while new external view evidence accumulates; a missing report or physical disturbance resets confirmation. Pre-inspection dwell cannot be reused. End-to-end success requires all three certificates in order.

Reliable synchronous atomic batches and perfect epoch knowledge are assumptions. These are not a Byzantine, crash-tolerant or asynchronous distributed-commit theorem. A private-map learning system, autonomous failure detector and real ROS middleware need additional protocols.

## Conditional safety results

**Proposition1: precedence and certificate safety.** In the stated reliable sampled model, every successful workflow has a routed pre-inspection certificate, a resource grant preceding physical transport, a valid physical release/support certificate, and a later independent placement-confirmation certificate.

**Proof.** Initially P is disabled and the carrier start time is null. Only the complete resource-grant branch can assign a start time, and it requires I completed, a fresh certificate and current pickup stability. Only the actual integration branch can release hand constraints, and its predicates include actual contact/pose/speed. The support certificate additionally requires release, sampled stability and a complete lifecycle commit. Only that branch enables P and initializes its own progress. A completed P requires six new routed samples with current physical predicates and a routed task-completion announcement. The success branch checks those physical predicates again. Induction over discrete transitions yields I,grant,T-support,P in order. No missing message or timer alone supplies any of them.

**Proposition2: processing and occupancy safety.** During every physically integrated carry/settling interval the carrier processes zero inspection jobs. A successful allocation admits no robot beyond its advertised processing slots. A denied resource transaction cannot create a partial occupation transition.

**Proof.** The grant sets the carrier's advertised availability and capacity to false/zero before the NEXT integration interval and requests immediate reallocation. The observation detector uses actual current capacity and truncates local processing to that value, independently of stale leases. The carrier remains zero-capacity until the placement notification commits; quarantine retains zero capacity. The allocation capacity token is decremented only for distinct owners with positive remaining slots and every owner ACKs the prepared plan. Lifecycle admission checks the complete batch sequence before reliable delivery. Therefore resource and processing invariants hold under the assumptions. Frame metadata describes the state AFTER endpoint transitions, while processed observations belong to the interval ending there; carrierBusyDuringInterval records that distinction explicitly.

**Proposition3: reconstruction continuity.** Announced noncarrier removal preserves every survivor configuration/velocity, tray position/quaternion/twist, hand activation and physical time at reconstruction.

**Proof.** The rebuild records those states by persistent physical ID and by the object's free-joint indices. It reconstructs the shared fleet/object scene and writes each saved state into the matching new indices exactly once, without changing values. It also copies generalized acceleration warm starts, while solver constraint rows are rebuilt. Direct before/after comparisons test the stated equality. Solver-row reconstruction can change later numerical transients, so this is an instantaneous state-continuity result, not identical future solver trajectories.

These propositions are safety statements, not universal liveness or optimization results. Persistent communication, enough external observers, feasible grasps/viewpoints, sufficient resources and a complete fair scheduler would be needed for stronger completion guarantees. The implemented finite-candidate greedy scheduler does not establish those hypotheses. Task failure remains a valid output.

## N, k and gcd in a real task loop

Let P_N shift cyclic roles, R_k=I+P_N+...+P_N^(k-1), and ell=R_k m be the REFERENCE mass load. For d=gcd(N,k), adding k has d orbits/cosets, each of length q=N/d. The quotient is Z/dZ; zero-sum blindness has dimension d-1 and Fourier frequencies q,2q,...,(d-1)q. Write k=ds with gcd(q,s)=1. The full proof remains in [cyclic theory](algebra-theory.html) and [execution theory](execution-theory.html).

All policies use the same positive complementary boundary law. Consecutive control orders k,k+1, or k,k-1 at the upper endpoint, have gcd1 jointly with N and eliminate the common zero-sum reference blind space. This equalizes the logical reference independently of robot tasks. It neither creates an extra free arm nor supplies a missing external camera view.

N counts all active physical role holders, INCLUDING an occupied carrier. A transport reservation changes capability and effective incidence A(t), not N. It generally destroys circulant symmetry. Replacing N by the available-camera count inside the old gcd theorem would analyze a different, compressed-role operator that this experiment does not implement.

For example N6,k2 has d2 and one service blind direction. During transport it still has N6,d2, although only five robots have inspection capacity and ID1 cannot join I/P anyway. Removing observer ID3 from N7 gives N6 and d1->2; the native model drops from146 to126 velocity coordinates while the tray and survivor states stay continuous. Complementary control keeps the reference mass observable. Execution can still worsen because one observer was lost. A smaller d does not guarantee a faster workflow when geometry or resource capacity worsens.

At the larger algebraic scale, N1000,k21 is coprime. One actual removal gives N999,d3 and two blind frequencies333,666. Occupying one of the1000 robots for manipulation does NOT produce N999: the reference remains N1000 while an availability mask changes the task hypergraph. This distinction connects the large-N algebra explorer to the small physical fleet without claiming a1000-robot articulated simulation.

## Fair policy comparison

The primary policies differ in which robot bidders they consider. Fixed cyclic retains the prescribed consecutive owners and leaves an infeasible job pending. Nearest available-k selects the closest currently available robots with remaining slots, queries their feasible views, and leaves the job pending if that candidate set cannot provide the required coalition. This is a resource-constrained nearest-k task baseline, not a global optimal constrained Voronoi controller. Cyclic repair queries a cyclic neighborhood and expands when needed; geometric repair queries the full active fleet. All use the SAME prospective-viewpoint executor, carrier trajectory/controller, task detector, capacities, workcell, DAG and routed lifecycle protocol.

Within each scenario, initial full robot/object states, targets, seeds and budgets are paired exactly. H is the same full-domain nearest-k mean squared-distance objective for all fleets. It does not remove unserved jobs or exclude an occupied carrier to improve a denominator. Chassis speed and native torque are reported separately; actuator work is not electrical energy. The archived centroid-based comparison is still available but is not mixed into this policy-only result.

The40 recorded trials cover three paired N6/k2 initial-position seeds, N5/k2, N6/k3, announced N7->6 removal, a carry-relative right-hand failure, tight shared budget, disconnection and insufficient external quorum. Fault activation is reported explicitly: a policy that never starts carry has not experienced a carry-relative hand fault. Common horizon completion is separate from workflow success.

For the default seed2718, cyclic repair obtains the pre-check/grant at3.4 s, a placement notice at18.5 s and external confirmation at19.3 s. Fixed cyclic confirms at30.0 s, nearest-k at24.7 s and geometric repair at19.9 s. These finite examples do not prove a general cyclic advantage. Inspect all seeds and fault cases, communication, H and background progress; geometric repair can win other tradeoffs. The recorded benchmarks and figures are the authoritative numerical results.

## Reproduction and scope

Open [the integrated lab](Integrated_Mission.html) and [the operating guide](Guide.html#integrated-inspect-carry-confirm). Source is integrated-mission.mjs plus mission-plant.mjs; the former owns the task/resource automaton, and the latter owns the one shared physical world. Numerical tests inject a real external object force to check that unstable placement cannot be confirmed. Source changes to either layer require relevant physics/protocol checks, not just screenshots.

The [official MuJoCo constraint model](https://mujoco.readthedocs.io/en/stable/computation/index.html) supplies the articulated/free-object/contact solver. [MIT ACL's CBBA/CCBBA overview](https://acl.mit.edu/projects/consensus-based-bundle-algorithm) provides background on heterogeneous allocation and coupled tasks; its approximation theorems are not imported into this coalition protocol. Manufacturer provenance and the simplified planar-supported model remain documented in [TIAGo model scope](tiago-theory.html).

No factory-identified dynamics, wheel/roller slip, tipping, autonomous pickup, finger friction/slip, re-grasp transfer, robot/self occlusion or hardware ROS2 control is claimed. Numerical checks, synthetic UI events, static resources and anonymous live HTTP are distinct. Previous native browser safety restrictions remain; these other checks are not actual browser or hardware validation.
