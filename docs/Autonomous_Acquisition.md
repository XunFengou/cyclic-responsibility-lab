# Autonomous carrier acquisition and supported fault handover

This experiment extends the preserved fixed-carrier mission. Every executing robot starts at a seeded feasible random base position, with folded arms, open experimental jaws and inactive object constraints. A private scratch inverse-kinematics solve is a bid computation, never an executing state reset. The tray is a free dynamic body from the first step. All robots and the object share one MuJoCo model and physical clock.

## State, task and capability definitions

- N_phys is the number of physical robot bodies. Initially N = N_phys is also the number of active cyclic allocation roles, ordered by persistent physical ID.
- k is the number of independent physical robot views required by an inspection job. Two arms of one robot never count as two views.
- Each robot has q_i in R^24: planar base x/y/yaw, torso, head, fourteen arm coordinates and four experimental jaw slides. Its generalized velocity is v_i. The free object has a 7-coordinate pose and a 6-coordinate twist. Thus nq = 24 N_phys + 7 and nv = 24 N_phys + 6.
- a_i is availability, c_i is inspection capacity, p_i^max is the declared task payload limit, and r_i is the remaining simulated resource reserve. A carrier reservation immediately sets its inspection capacity to zero. Payload limits are experiment parameters, not manufacturer ratings.
- Inspection jobs have a world target, required dwell, independent owners and completion evidence. Pre-inspection and placement confirmation use different task IDs and reset evidence after a takeover.
- Manipulation has exclusive ownership: at most one carrier token, and only that robot's two hand constraints may attach to the object. Jobs reserve robot slots, not independent arm-agents.

The plant obeys M(q) v_dot + b(q,v) = B u + passive(q,v) + J(q)^T f. M and b come from the vendor-derived articulated model; u has bounded native forces and torques. Generalized acceleration tracking uses articulated computed torque, gravity compensation and measured-state feedback. Chassis approach is speed limited; arms follow smooth joint trajectories through raise, hover and lower phases. Experimental jaw slides use the same coupled mass matrix. Saturation, actual joint violations and contact penetration remain recorded; no endpoint or velocity reset repairs a failed run.

This is a planar-supported vendor-derived model with ideal force actuators and a replacement experimental tool. It is not an identified factory robot, a wheel-slip/tipping model or validation of the manufacturer's gripper. The public vendor inertia model is retained; the additional actuated jaws have explicitly declared inertias.

## Carrier auction and actual acquisition

Each available robot privately evaluates both approach faces of the object. A bid requires a navigable base path, payload capacity, resource reserve, enough torso travel, a six-dimensional two-arm IK solution with joint margins, and a static gravity-torque ratio below 0.8. The static torque test is a screening condition, not a proof of collision-free dynamic reachability. The physical controller and recorded constraints decide actual feasibility.

Candidate scores use the same received bid fields. Fixed cyclic prioritizes cyclic role distance then estimated travel time. Nearest-k ranks feasible manipulation candidates by Euclidean approach distance. Cyclic repair ranks estimated travel time plus resource backlog and cyclic role distance. Geometric repair omits the cyclic term. All share the same feasibility filter and physical executor. Background inspection uses the corresponding policy's owner rule with the same prospective-viewpoint executor. The selection score is the intended policy difference; the physical capabilities and success detectors are identical.

Carrier requests, private joint-space bids, prepare/ack/commit grants and object evidence travel through the same routed communication ledger and admission budget as inspection allocation. A synchronous logical ring is a reliable comparison carrier; the range/obstacle-LOS graph can disconnect. Topology discovery is charged before data admission and is not capped by that budget. This is not measured ROS/DDS throughput, asynchronous consensus or a hard real-time network guarantee.

The selected carrier physically approaches. At arrival it solves IK from the actual base pose, raises its torso, moves to a hover posture, lowers the tool and closes the four jaws. Grasp certification requires four actual pad-handle contacts: two opposing contacts on each of two distinct handles, each with normal force above 1 N. Normals must oppose with dot product below -0.8. Object speed must be below 0.06 m/s and maximum arm-joint speed below 0.08 rad/s throughout 0.2 s of sampled dwell.

Only a delivered lifecycle grant can then activate an ideal post-contact closure. The relative poses and anchors are computed from the current hand/object transforms using the pinned MuJoCo 3.8.1 equality layout. q and v are unchanged at activation; the equality residual is logged. This is a contact-certified ideal latch, not friction-only force closure, slip simulation or perception-based grasp search. The handles and bench are known.

External pre-inspection may accumulate while the carrier approaches, but lifting needs a fresh completed certificate, at most 5 s old, with k distinct external robot IDs. A stale pre-check is cleared and reacquired. Lifting, carrying, lowering and unloading are actual motor actions. Grasp acquisition alone never grants task success.

## Recoverable mobility fault and takeover

The recoverable model disables the carrier's mission role but retains powered braking, torso, arms and jaws. It is not complete power loss or structural collapse. A known continuous support bench lies under the transport route. At the configured carry-relative fault age, travel is braked and the torso lowers the still-held object. Load feedforward is gradually removed so the bench can assume the weight.

Before release, contact with the bench and measured upward support load greater than 0.85 mg must coexist with object speed below 0.04 m/s, tilt below 0.10 rad and height error below 0.015 m. These predicates must persist for a consecutive sampled dwell. After actual release, another 0.5 s stable dwell with all object grasp constraints inactive is required. The second check prevents robot-induced contact load alone from masquerading as successful set-down.

Only a routed safe-setdown certificate allows arm retraction, parking and a fresh auction. The old robot remains in the same model as a powered parked body and is ineligible for new tasks. A different eligible robot approaches the same measured object pose, acquires new contact evidence, obtains fresh external inspection and carries to the original goal. No object or survivor pose/twist reset is used. If there is no eligible replacement, reachable support, delivered grant or valid contact, recovery remains incomplete.

The structural countercase removes both grasp constraints during carry and quarantines the carrier. It does not award a safe placement or fabricate a replacement grasp. Total power-loss recovery would require independent passive support, a catcher or another physical rescue mechanism; it is outside the certified model.

## Exact arithmetic: what N, k and gcd mean here

Let G_N = Z/NZ, with basis-partition mass m_i and cyclic shift (P m)_i = m_(i-1). The group-algebra responsibility operator is R_k = I + P + ... + P^(k-1). Its output is the nominal cyclic workload, not an execution certificate. Define d = gcd(N,k), q = N/d and s = k/d; gcd(q,s) = 1.

Theorem 1. The jump graph i -> i+k has d connected cyclic orbits, the cosets of the subgroup generated by k. Each orbit has q vertices. Its quotient is G_N / <k> isomorphic to Z/dZ. If R_k m is constant then m is constant on each orbit. Conversely every such m produces a constant workload, equal to s times the sum of its d orbit values.

Proof. Taking adjacent differences of R_k m gives m_i - m_(i-k). Equality of workloads forces this to vanish. The subgroup <k> consists exactly of multiples of d modulo N, by Bezout's identity; it has N/d elements. A length-k consecutive window contains k/d = s representatives of every residue modulo d. This proves the converse and classifies the constant-workload preimage.

Theorem 2. For Fourier character chi_l(i) = exp(2 pi sqrt(-1) l i/N), R_k has eigenvalue sum_(h=0)^(k-1) exp(-2 pi sqrt(-1) l h/N). It equals k at l=0 and vanishes precisely at l = q,2q,...,(d-1)q. Over the real state space dim ker R_k = d-1. These invisible zero-sum variations are functions on the quotient group with zero mean.

Proof. P chi_l = exp(-2 pi sqrt(-1) l/N) chi_l. The geometric-series numerator vanishes if N divides kl, while its denominator is nonzero for l != 0. Since gcd(q,s)=1, this is equivalent to q dividing l. Real and imaginary conjugate characters supply the same total real dimension d-1.

Theorem 3. For positive responsibility weights and orders K = {k_1,...,k_r}, the common zero-sum blind space has dimension gcd(N,k_1,...,k_r)-1. In particular orders k and k+1 eliminate every arithmetic blind mode for every N.

Proof. A common blind frequency must satisfy N | k_j l for every j and l != 0. Bezout reduces these simultaneous divisibilities to N | g l, where g = gcd(N,k_1,...,k_r). The nonzero solutions are N/g,...,(g-1)N/g. Positive weights cannot cancel the squared norms of different outputs. Eliminating blindness establishes observability/coercivity on the finite zero-sum space, not a scale-independent convergence rate.

For fixed N the Fourier eigenvalues above describe each k. For fixed k the number d repeats with N mod k, but orbit length, geometry, matrix size and spectral spacing still change with N. The triple (d,q,s) separates orbit count, orbit length and normalized jump. Neither periodic gcd nor complementary orders imply a uniform spectral gap as N grows.

At N=1000,k=21: d=1,q=1000,s=21; there is one orbit and no blind Fourier frequency. At N=999,k=21: d=3,q=333,s=7, with blind frequencies 333 and 666. At N=987,k=21: d=21,q=47,s=1 and 20 blind frequencies. Adding order 22 removes the common blind space in all three cases. The small physical fleet uses N6,k2 (one blind direction), N5,k2 (none) and N6,k3 (two) so the same algebra is visible without rendering 1000 articulated bodies.

## Communication Laplacians and actual eligibility

The undirected jump graph has Laplacian L_jump = 2I-P^k-P^(-k), counting both jump directions. Its kernel is exactly the d-dimensional orbit-constant space. Thus ker R_k is the zero-mean part of ker L_jump. Its Fourier eigenvalues are 4 sin²(pi k l/N). This is an exact correspondence between orbit connectivity and responsibility blindness.

The actual carrier graph is a different graph. A general undirected connected communication Laplacian L_comm has kernel span{1}; it shares the Fourier basis with R_k only when it is circulant. For the quadratic load-disagreement operator sum_j R_(k_j)^T L_comm R_(k_j), its kernel on 1-perp has dimension gcd(N,k_1,...,k_r)-1 when L_comm is connected. To prove this, its quadratic form is the sum of nonnegative edge-disagreement energies. Zero energy forces every R_(k_j) x to be constant; since x has zero sum, those constants are zero. Theorem 3 applies. A disconnected graph adds blind directions and a time-varying graph needs explicit temporal-connectivity and persistence assumptions.

Actual task incidence also depends on FOV, obstacle LOS, head/torso limits, resource occupancy, distinct-view angles and measured dwell. It is generally noncirculant. Its feasible owner sets form a time-varying task hypergraph; a payload job additionally consumes the coupled two-arm resource of one robot. Fourier arithmetic classifies the cyclic reference, while capability masks, hyperedge feasibility and the task precedence graph determine completion. gcd alone cannot predict a grasp, a takeover or the success ranking.

A busy or mobility-failed carrier changes availability without changing N. In the announced role-retirement scenario N_active changes from 6 to 5, hence d changes from 2 to 1; the retired physical robot is parked, so N_phys=6 and nq/nv remain unchanged. Reindexing the active roles changes the reference operator without removing a physical body. These are two different failure models.

## Conditional safety propositions

Proposition 1, protocol exclusion. Assuming reliable delivery within an admitted synchronous transaction, there is at most one carrier token and no inspection processing by an occupied carrier during the same physical interval. Proof: the carrier is selected only from a committed exclusive grant; a nonempty grasp token forbids another latch. Reservation sets inspection capacity to zero before the next physical interval, and the common allocator/observer enforces those capacities. A denied transaction makes no lifecycle transition. This does not establish asynchronous fault-tolerant consensus.

Proposition 2, evidence-preserving transition. Assuming truthful simulated measurements and the explicit detector tolerances, every certified grasp has prior physical contact/preload dwell; every takeover has prior supported release and stable post-release dwell; every final success has a new external confirmation after placement. Proof: inspect the guarded state transitions. No other branch writes these certificates; the old pre-check is cleared at handover. State continuity follows because re-bidding and activation do not assign q or v, and the physical model is retained across the fault.

Proposition 3, conditional liveness. If an eligible carrier and external k-view owner set remain reachable, their physical trajectories attain the guarded tolerances, enough communication admission is available and measurements persist for the required dwell, the finite workflow advances. This is conditional on physical attainability and budget. Neither arithmetic complementarity nor IK at one pose proves these premises. Payload, disconnected, external-quorum and structural counterexamples are retained to make this distinction testable.

The guarded assertions are sampled numerical properties at 0.1 s workflow intervals with 0.005 s physics steps. They are not continuous-time collision avoidance or whole-body convergence theorems. Physical cost need not decrease monotonically. The existing sector-boundary reference is updated with complementary orders independently of actual manipulation success; its load error must not be reported as achieved robot service.

## Fair results and metrics

Each scenario pairs fixed cyclic, nearest-k Voronoi, cyclic repair and geometric repair with identical initial full qpos, object, demand, dynamics, capability limits, executor, support geometry, fault trigger relative to transport, communication budget, stopping horizon and detectors. Policies may choose different carriers, so their carry-relative fault times differ by design. Report actual trigger time and carrier history; never compare an untriggered failure as a successful rescue.

Report end-to-end confirmation time, background demand completion, occupied robot-seconds, support load, acquisition/set-down certificates, object placement error, routed byte-hops, actual torque RMS and absolute actuator work. H is the same nearest-k full-map squared-distance objective evaluated on the active role pool. Assigned-work imbalance and R_k reference imbalance have different denominators; zero active work is n/a. A policy can have lower H and fail manipulation. No preferred method is assumed to win every seed.

## Reproduction and recording

Install the pinned physics runtime, run the autonomous numerical and viewer checks, then run `node scripts/run-autonomous-benchmarks.mjs`. Full qpos recordings remain in the ignored research/autonomous directory on D:. Public compact paired recordings and summaries use the autonomous_ prefix, preserving all earlier archives.

`scripts/render-autonomous-video.py` renders exact logged qpos using native MuJoCo 3.8.1 into 1280x720 MP4 at 10 fps. It uses zero-order hold between samples, no pose interpolation or new physics rollout. Provenance records source/model/video hashes, decoded frame count and duration, plus the rendering scope. The two videos show actual acquisition and a supported takeover. Numerical tests, synthetic viewer tests, static resource checks and anonymous live HTTP checks remain distinct from native browser and hardware QA.

## Primary references

- [MuJoCo 3.8.1 model and equality representation](https://github.com/google-deepmind/mujoco/blob/3.8.1/include/mujoco/mjmodel.h).
- [MuJoCo 3.8.1 constraint construction](https://github.com/google-deepmind/mujoco/blob/3.8.1/src/engine/engine_core_constraint.c).
- [PAL Robotics TIAGo robot description](https://github.com/pal-robotics/tiago_robot).
- [Preserved vendor/runtime provenance](third-party.html) and [earlier integrated task definitions](integrated-theory.html).
Leader-coordinated private bidding is used here: each robot computes its own reachability bid, and a routed coordinator grants exclusive resources. It is not a fully decentralized or asynchronous consensus implementation. The same coordination model is used by all four policies.

The recorded N6/k2 default gives confirmation at 54.3 s for cyclic repair, 55.0 s for nearest-k and 54.8 s for geometric repair; fixed cyclic does not confirm by 75 s. At N6/k3, nearest-k confirms at 53.8 s, geometric repair at 55.0 s and cyclic repair at 58.9 s. These paired examples explicitly reject a universal ranking. gcd classifies a reference blind space; it does not order physical completion times. The light-payload case expands the eligible carrier pool, while the 3 kg case has no eligible carrier under the declared experiment limits.

The expanded 1 kg pool selects robot4 in this seed. A static IK/payload screen is insufficient: its arm trajectory disturbs the object before contact certification. A measured displacement guard stops acquisition at25.5s, opens the jaws and quarantines its resources. The run remains incomplete and awards no grasp or placement certificate. This is an execution-failure counterexample, not a successful lightweight transport demonstration. The guard is sampled protection against continued acquisition; it does not prove global collision avoidance or safe object support after every disturbance.
Retirement merges adjacent basis sectors and relabels the surviving cyclic roles. It is not a quotient homomorphism from Z/6Z to Z/5Z: such a surjective group map does not exist because5 does not divide6. Quotient classification applies separately inside each fixed-N group; the hybrid transition between different N uses a state-transfer map with physical-state continuity, not a presumed group inclusion.
The light-payload countercase also exceeds the model's soft joint limits (maximum0.215 in native joint units) and reaches23.7mm recorded contact penetration. It is explicitly rejected as a physical-feasibility result. The resource cards show these diagnostics. The18 successful workflows have at most0.001321 recorded native joint-limit excess and7.341mm contact penetration in this numerical model; this tolerance is not a proof of exact nonpenetration. Across48 paired trials,18 finish and30 remain incomplete. All four policies perform a certified physical takeover in the recovery scenario; final confirmation succeeds for three. Geometric repair confirms at100.2s, cyclic repair114.8s and nearest-k119.1s; fixed cyclic remains unconfirmed by200s. No method is claimed to dominate.

For the numerical success audit, the joint-limit tolerance is0.01m for slide coordinates and0.01rad for hinge coordinates. The cumulative maximum native excess is a diagnostic across those declared coordinate units, not a physical distance norm. Neither soft-limit tolerance nor the contact threshold is a hardware safety certificate.
