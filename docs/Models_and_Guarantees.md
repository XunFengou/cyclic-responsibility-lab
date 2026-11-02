# Robot dynamics, performance, and precise guarantees

The website now separates the arithmetic responsibility layer, positive virtual-boundary layer, physical robot plant, and communication accounting. Changing the robot model does not change the cyclic responsibility operator. Every model uses the same locally assembled service moments and the same centroid target.

## 1. Shared coverage problem

There are N positive basic atoms S_i, with normalized masses m_i, first moments b_i in R^2, and second moments h_i. The service order is k. Role i owns the union of k consecutive atoms ending at i. Its load, centroid, and physical service cost are:

```text
ell_i = sum_{a=0}^{k-1} m_{i-a}
c_i   = sum_{a=0}^{k-1} b_{i-a} / ell_i
J     = (1/2) sum_i integral_{W_i} ||z - p_sigma(i)||^2 rho(z) dz
J/k   = cost normalized by the k-fold service multiplicity
```

The instantaneous unconstrained centroid lower bound is J_star/k = (1/(2k)) sum_i [h(W_i) - ||b(W_i)||^2/ell_i]. Exactly,

```text
(J - J_star)/k = (1/(2k)) sum_i ell_i ||p_sigma(i) - c_i||^2.
```

This lower bound need not be attainable inside an annulus: a raw Euclidean centroid can lie in the hole. The simulator distinguishes raw centroid excess from feasible-target tracking error.

For control orders K with nonnegative weights summing to one, the virtual cumulative-mass coordinates x_i satisfy:

```text
m_i = x_{i+1} - x_i,  x_{i+N} = x_i + 1
x_i^+ = x_i + beta sum_{r in K} gamma_r [ell_{i+r-1}^{(r)} - ell_{i-1}^{(r)}]
m^+ = [I - beta sum_{r in K} gamma_r Q_r] m
Q_r = 2I - P^r - P^(-r),  0 < beta < 1/2.
```

Thus masses stay positive. The common blind dimension is gcd(N,K)-1. Fixed complementary orders, or a persistently excited complementary switching family, balance all basic atoms. A robot-model change leaves this theorem unchanged if every snapshot continues to be delivered. A finite-range carrier that disconnects blocks the baseline; communication feasibility is not inherited from the arithmetic theorem.

See the algebra and topology derivations in the preserved Theory_and_Topology.md document. Virtual rays remain independent of robot headings and polar angles.

## 2. Single integrator

In the convex disk the actual plant and controller are p_dot = u and u = a(c-p), with a>0. For a fixed partition:

```text
J_dot = -a sum_i ell_i ||p_i-c_i||^2 <= 0.
```

The convex disk is invariant. Convergence of the partition and its fixed phase gives convergent centroids, and the stable first-order filter gives position convergence.

In the star annulus only this baseline uses the feasible polar chart: alpha is a lifted angle, u_rad is a normalized radial fraction, and r = r_minus(alpha)+delta+u_rad[r_plus(alpha)-r_minus(alpha)-2delta]. The two chart coordinates attract a radially clipped feasible target. The physical velocity input is the chart Jacobian times the chart-coordinate velocity. This preserves the band by construction. It is not unmodified Euclidean centroid descent, so physical J is not asserted to decrease. A convergent target angle lift is required for position convergence.

## 3. Second-order and higher-order integrators

The physical Cartesian plant is p^(r)=u, with state (p,p_dot,...,p^(r-1)). The double-integrator option uses r=2. The higher-order option lets the user choose r=3,4,5. The actual input is:

```text
u = a^r (c-p) - sum_{l=1}^{r-1} binomial(r,l) a^(r-l) p^(l).
```

The closed-loop polynomial is (s+a)^r. This is an acceleration, jerk, snap, or fifth-derivative input as appropriate, not a renamed velocity controller. Higher derivatives start at zero and are not reset during role transfer.

For a fixed target and zero initial derivatives, the exact position solution is:

```text
p(t)-c = [p(0)-c] exp(-a t) sum_{l=0}^{r-1} (a t)^l/l!.
```

The bracketed scalar is positive and decreases. Therefore the frozen-partition coverage cost decreases for these initial conditions. For r=2, the storage V = (a^2/2)||p-c||^2 + (1/2)||p_dot||^2 has V_dot = -2a||p_dot||^2 for a fixed target, even with nonzero initial velocity. Physical coverage cost alone does not have that general monotonicity guarantee.

For arbitrary bounded targets inside a convex set, zero initial higher derivatives give a positive convolution kernel a^r t^(r-1) exp(-a t)/(r-1)!. Its integral plus the initial-condition weight is one. Consequently the position remains in the convex hull of the initial position and target history. This proves disk containment for these specific filters without input saturation. A nonconvex annulus has no such guarantee: a valid Cartesian trajectory can cross its hole. The website reports the actual domain margin rather than clipping the state.

For convergent centroids, Hurwitz stability gives p->c_star and all higher derivatives ->0. This proof applies to the motion-time model; it does not prove delayed-plant stability.

## 4. Unicycle and nonholonomic differential drive

A unicycle is already a nonholonomic system. Its state is (p_x,p_y,psi), with heading h=(cos psi,sin psi), sideways direction h_perp=(-sin psi,cos psi), and constraint h_perp^T p_dot=0. Define e=c-p, e_parallel=e^T h, e_perp=e^T h_perp, and phi=atan2(e_perp,e_parallel).

```text
p_dot = v h,  psi_dot = omega
v     = sat(a e_parallel, v_max)
omega = sat(b phi, omega_max),  b>a, omega_max>a/2.
```

Reverse speed is allowed; no terminal heading is prescribed. At the exact goal the ideal controller uses zero commands. The implementation adds an explicit numerical position deadband of 10^-8 domain units.

For a fixed target, J_dot = -sum_i ell_i e_parallel,i sat(a e_parallel,i) <= 0. Away from the goal, the polar equations are r_dot = -v cos(phi) and phi_dot = (v/r) sin(phi)-omega. In the unsaturated angular region, the sign of phi_dot is toward zero because |sin(phi)cos(phi)|<=|phi| and b>a. In the saturated region the disturbing term is at most a/2, whereas omega_max>a/2. Thus heading error tends to zero and radius tends to zero. The branch at phi=pi is assigned a deterministic turn direction; no desired-heading stabilization claim is made.

For converging moving centroids with integrable motion and vanishing velocity, the same position conclusion follows from the perturbed cost inequality and the impossibility of maintaining e_parallel=0 with a nonzero transverse error while the heading keeps turning. The sampled implementation uses held centroids with summable jumps; the numerical deadband changes the conclusion to practical convergence to that tolerance. Arbitrary moving targets and arbitrary asynchronous delays are outside this theorem.

The differential-drive option has actual wheel-rate inputs w_L,w_R:

```text
w_L = (v - L omega/2)/r_wheel
w_R = (v + L omega/2)/r_wheel
v_actual = r_wheel (w_L+w_R)/2
omega_actual = r_wheel (w_R-w_L)/L.
```

A common positive scaling enforces both wheel limits. It preserves the body-motion direction and the sign of cost descent. It also preserves the no-slip constraint. The normalized effort chart uses wheel-speed references, while the native wheel charts remain in rad/s.

These models do not automatically preserve a convex disk or a nonconvex annulus. The test suite includes an outward-moving nonholonomic example to confirm that the simulator reports an exit rather than secretly projecting the robot back inside. A domain-barrier or navigation layer would require a separate controller and feasibility proof.

## 5. Euler-Lagrange planar robot

This option is a fully actuated two-coordinate planar mechanism with q=p. It is not a claim to simulate every industrial manipulator. Let m>0, 0<=epsilon<0.8, d>=0, g0>=0:

```text
M(q) = m diag(1+epsilon cos(q_y), 1+epsilon sin(q_x))
A = -m epsilon sin(q_y),  B = m epsilon cos(q_x)
C(q,v)v = [A v_x v_y - (B/2) v_y^2,
            B v_x v_y - (A/2) v_x^2]
g(q) = [0, g0 sin(q_y)],  D=d I
M q_ddot + C(q,q_dot)q_dot + g(q) + D q_dot = tau.
```

M is uniformly positive definite. The Christoffel construction satisfies that M_dot-2C is skew-symmetric. With virtual acceleration a_virtual=a^2(c-q)-2a q_dot, use exact-model computed torque:

```text
tau = M a_virtual + C(q,q_dot)q_dot + g(q) + D q_dot.
```

The simulator computes tau and then solves the plant equation; it does not simply draw a second-order trajectory with a force label. Exact cancellation yields the same critical second-order position equation, hence the same stated convergence and convex-disk guarantees. This requires known M,C,g,D, full actuation, and no torque saturation or model mismatch. Underactuated and uncertain manipulators require new analysis.

At rest tau_star=g(q_star) can be nonzero. The squared-force integral can then keep growing while mechanical work tau^T q_dot tends to zero. Therefore the plotted squared-input integral is explicitly called normalized input effort, not physical energy.

## 6. What each performance plot means

- Basic-mass error is RMS(m_i-1/N)/(1/N).
- Service-load error is RMS(ell_i-k/N)/(k/N). A zero value can coexist with a nonzero basic-mass error when gcd(N,k)>1.
- The service-blind error is the norm of the orbit-mean projection relative to the uniform mass.
- The two selected-robot plots show actual native input channels and their units. The heading arrow displays the nonholonomic orientation.
- The population input plot shows the RMS and peak of reference-scaled native input norms. These are model-dependent quantities, not equal-unit energy comparisons across models.
- The boundary-input plot shows RMS(x_dot) in normalized mass fraction per motion second. It is independent of the plant's native actuator coordinates.
- The cost plot shows the recorded physical J/k and its instantaneous centroid lower bound. It never replaces the data with a decreasing envelope.
- The cost-contribution plot uses the old partition to measure the robot-motion change, then attributes the remaining change to updated boundaries. For steps without membership or assignment events, these contributions add exactly to the total cost change. Failures and role transfers create separate exogenous jumps.

The mass-balancing boundary energy, physical service cost, and kinetic storage are different functions. A proof that one decreases is not a proof that the others do. Use the fixed-partition preset to isolate a cost-descent theorem, and the failure preset to study the complete coupled experiment honestly.

## 7. Numerical and communication conventions

Virtual boundaries use the same positive update as version 3, with beta=0.2 unless the partition is frozen. Robot plants use RK4 substeps of at most 0.01 motion seconds by default. State derivatives are defined with respect to motion time.

The second clock adds an assumed communication-round duration. For inertial robots it is accounting only: the code does not claim that a physical plant can pause at nonzero velocity during message exchange. A continuously evolving plant with network delays, stale centroids, and safety constraints is a separate research extension.

The reliable message-passing protocol and shared known demand map remain explicit assumptions. General unknown/private sensing, collision avoidance, automatic failure detection, and parked failed obstacles are not added by selecting a different robot model.

## Sources and scope

- [Tedrake, Underactuated Robotics: multi-body dynamics](https://underactuated.mit.edu/multibody.html) gives the Lagrangian and manipulator-equation background.
- [Lynch and Park, Modern Robotics: wheeled mobile robots](https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-1-wheeled-mobile-robots/) distinguishes nonholonomic velocity constraints and wheel-speed kinematics.
- The particular cyclic boundary law, integrator filters, planar inertia example, cost decomposition, and tests above are explicitly specified here. They are not attributed as a blanket theorem from those general robotics sources.
