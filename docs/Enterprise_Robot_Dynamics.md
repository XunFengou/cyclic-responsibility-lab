# Enterprise whole-body robots for K-th responsibility

## Recommended first platform

Use a fleet of PAL TIAGo Pro mobile manipulators for cooperative bearing / camera observation, with Unitree G1 as a more demanding contact-dynamics extension. The scientific reason for k>1 is independent viewpoints, occlusion resilience and task information. A complex robot by itself does not create a mathematical need for k-th coverage.

The [official TIAGo Pro specification](https://pal-robotics.com/robot/tiago-pro/) describes two 7-DoF arms, actuator torque sensing, a lifting torso and a pan-tilt head, with ROS 2 software. The [manufacturer's public model repository](https://github.com/pal-robotics/tiago_pro_robot) contains description and bringup packages. The related [TIAGo++ source](https://github.com/pal-robotics/tiago_dual_robot) is another dual-arm integration target. These give actual model structures rather than an invented two-coordinate mass matrix. The product's total DoF label is not automatically the number of independent configuration or velocity coordinates in every URDF variant.

Torque sensing is not evidence that an unrestricted torque-command interface is available on a particular robot. Pin the purchased configuration, controller interfaces, model revision and vendor-supported software distribution. The [PAL competition software page](https://rgmc_mmt.pal-robotics.com/software.html) describes joint trajectory position interfaces and warns of software changes after March 2026; its older distribution note must not be copied as a present-day hardware guarantee. Position and effort command interfaces require different tracking layers. The [official ros2_control trajectory-controller documentation](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html) explains their distinction.

## Common constrained dynamics

Use a configuration manifold Q, configuration q in Q and generalized velocity nu in T_q Q, expressed in chosen coordinates. They need not have the same dimension. For robot a:

```text
q_dot = G(q) nu.
M(q) nu_dot + h(q,nu) = S(q)^T tau + J_c(q)^T lambda + A(q)^T eta.
A(q) nu = 0.
J_c(q) nu_dot + Jdot_c(q,nu) nu = 0   (fixed sticking contact mode).
```

M is the positive-definite inertia matrix on independent generalized velocities; h includes Coriolis, gravity and modeled dissipation. tau is actuator effort; S selects or maps actuated directions; lambda is contact wrench and eta is the reaction enforcing rolling constraints. J_c is the contact Jacobian and A represents nonholonomic rolling constraints. This schematic equation requires a mode-specific consistent coordinate choice; do not redundantly include the same wheel-ground constraint in both A and J_c.

For a wheeled mobile manipulator, Q contains a planar base pose in SE(2), torso, head, arm and wheel coordinates as chosen by the model. Wheel-level rolling constraints depend on the base. In particular, TIAGo Pro is omnidirectional, so a differential-drive no-sideways constraint must not be imposed on its reduced chassis motion. The full inertia couples moving arms, torso and base; moving only a point centroid omits those effects. Limits on effort, joint position, joint velocity, self-collision and inter-robot clearance enter feasibility.

For a floating-base humanoid, Q contains SE(3) times the joint configuration. A quaternion base has 7 configuration entries but 6 velocity entries. Contact modes, friction cones, support stability and impact transitions are essential. The [official Unitree ROS 2 repository](https://github.com/unitreerobotics/unitree_ros2) includes G1-specific interfaces and examples; the Go2 message example is not a G1 specification. The [official Unitree MuJoCo project](https://github.com/unitreerobotics/unitree_mujoco) supplies simulation support. Hardware variants and modes must be matched explicitly.

The [Menagerie G1 model](https://github.com/google-deepmind/mujoco_menagerie/blob/main/unitree_g1/g1.xml) has a free base and 29 actuated scalar joints. For that specific model, nq=7+29=36 and nv=6+29=35, by direct coordinate counting. Its shown MJCF uses position actuators. It is not a raw torque-actuator benchmark without a documented change to actuator mapping. This count and actuation choice do not describe every G1 hardware variant.

## Coverage layer to whole-body tasks

The responsibility layer supplies W_a, desired sensing point c_a, task weight ell_a and optional observation-quality objective. It does not directly supply joint torques. A sensor transform T_ws=h_sensor(q) and Jacobian J_sensor(q) connect a camera or tool pose to nu. Whole-body motion should track a feasible sensor/base task while retaining contact, rolling, joint and collision constraints.

A prospective inverse-dynamics quadratic program chooses (nu_dot,tau,lambda,eta) to minimize weighted task-acceleration error and effort, subject to the dynamics, contact equations, friction and limits. Slack may be allowed on coverage tasks, but not silently on hard contact/safety constraints. [Pinocchio](https://github.com/stack-of-tasks/pinocchio) provides model-based rigid-body computations; [TSID](https://github.com/stack-of-tasks/tsid) provides task-space inverse-dynamics components. Their availability does not prove feasibility of every moving coverage target. The first integration should use the robot's supported base/joint interface and a tested tracking controller rather than assume whole-body torque access.

The existing browser Euler-Lagrange plant is explicitly a fully actuated two-coordinate teaching mechanism. Selecting TIAGo or G1 on the ROS 2 page only prepares an adapter draft. It does not integrate those model equations. The separate [TIAGo Pro lab](TIAGo_Comparison.html) now runs a pinned PAL-derived 20-DoF planar-supported articulated model in MuJoCo. Read its [exact assumptions](tiago-theory.html). No enterprise hardware was run.

## Why independent robots and view diversity matter

Consider a target y in R^2, robot observation position p_a, bearing b_a=atan2(y_2-p_(a,2),y_1-p_(a,1)), and independent angular noise of variance sigma^2. Write dx=y_1-p_(a,1), dy=y_2-p_(a,2), r^2=dx^2+dy^2. Differentiation gives gradient_y b_a=(-dy,dx)/r^2. The Fisher information is

```text
I(y) = sum_(a observing y) (1/(sigma^2*r_a^4))
       [[dy_a^2, -dx_a*dy_a],[-dx_a*dy_a, dx_a^2]].
```

Every contribution has rank 1. Two collinear bearings leave an unobservable direction; two nonparallel views give rank 2. At equal range r and relative bearing angle alpha, the two eigenvalues are (1 +/- |cos(alpha)|)/(sigma^2*r^2). Hence the minimum eigenvalue vanishes at alpha=0 and pi, and is maximal at pi/2. The ROS 2 page reproduces this exact ideal model. Count k refers to independent physical robots, not the number of joints or cameras on one robot. Correlated camera noise, uncertain robot pose, occlusion and dynamic targets require a richer model.

This motivates a layered objective: retain equal responsibility and explicit fault-repair structure, then optimize sensor-pose information inside feasible whole-body tasks. Constraining lambda_min(I)>=epsilon is a future task design, not a consequence of gcd=1. Multi-view information provides a testable reason for higher-order service without claiming superiority under every distance objective.

## Proposed experimental progression

- Reproduce the browser partition and metric tests with paired seeds and stated limitations.
- Pin one vendor model revision. Validate forward kinematics, inertia, actuator mapping and contact conventions offline.
- In full-body simulation, compare a base-only observer with head/torso/arm sensor reconfiguration under the same responsibility task. Record task tracking, joint/effort limits, collisions, contact feasibility and information eigenvalues.
- Compare k=1,2,3 independent robots under occlusion and a known removal. Keep sensing noise and failed physical IDs paired across methods; distinguish information recovery from equal-mass recovery.
- Import read-only ROS 2 recordings through the neutral telemetry contract. Preserve timestamps, frames, model checksum and provenance.
- Hardware control remains a separate integration step with the platform's supported controller, operator process and robot-specific validation. No browser actuation interface is enabled here.

Source review date: 2026-11-06. External branches and products can change; pin exact revisions for experiments.
