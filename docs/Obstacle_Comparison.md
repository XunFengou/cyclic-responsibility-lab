# Matched multi-obstacle coverage experiments

This page defines the comparison implemented in Obstacle_Comparison.html. The new module uses safe single-integrator motion. The original six-model simulator is preserved and has different workspace guarantees.

## 1. Common domain, robots, and sensing objective

Let D be the closed unit disk in R^2. Let O be the union of M closed circular obstacles, with 0 <= M <= 20. Every obstacle lies inside the disk of radius 0.90. Obstacles may overlap. Let Omega = D minus O. A known, static demand density rho is strictly positive and bounded on Omega. Normalize its integral to 1. Obstacles remove demand; they also obstruct robot motion and, when selected, radio line of sight.

There are N physical robots, with persistent IDs, positions p_i in Omega, disk radius r_b, and single-integrator dynamics p_dot_i = u_i. The same initial positions, random seed, N, service order k, density, time step, radius, clearance, gain, velocity limits, announced failures, and communication policy are used in both runs. The matched physical viewer supports 3 <= N <= 80. Large-N arithmetic experiments remain in the algebra explorer; the physical all-gather implementation makes no scalability claim.

For a size-k responsibility set S(q), use the normalized Euclidean objective

```
H(P,S) = (1/(2k)) integral_Omega sum_{i in S(q)} ||q-p_i||^2 rho(q) dq.
ell_i = integral_{q: i in S(q)} rho(q) dq.
c_i   = (1/ell_i) integral_{q: i in S(q)} q rho(q) dq, when ell_i > 0.
sum_i ell_i = k; the equal-load target is ell_i = k/N.
```

Distance is Euclidean and sensing in this objective can pass through obstacles. This is a specific sensing model. It is not geodesic coverage or visibility-constrained assignment. A separate diagnostic records the demand fraction with at least k robots in obstacle-free line of sight and within a common sensing radius. A high Euclidean score does not imply high visible k-coverage.

## 2. Cyclic sectors on a domain with holes

Let theta be the polar angle. Its marginal demand w(theta) is the integral of rho(r,theta) r over all free radial intervals on that ray. A ray may have several such intervals. Because all obstacles lie inside radius 0.90, every ray has a common free outer band. Strictly positive rho therefore gives w(theta) > 0. Define the lifted cumulative mass coordinate F by integrating w and normalizing F(theta + 2*pi) = F(theta) + 1. F has a strictly increasing inverse.

Choose lifted boundary states x_0 < ... < x_(N-1) < x_0 + 1. The physical ray is theta_i = F^(-1)(x_i). Basic atom A_i is the free space between consecutive rays, with m_i = x_(i+1)-x_i, using x_N=x_0+1. Atoms partition demand. An atom can be disconnected by obstacles; nonempty mass does not mean connected geometry.

Role i serves W_i = A_i union A_(i-1) union ... union A_(i-k+1), with indices modulo N. Every free-space point has k responsible roles, and ell = R_k m, where R_k = I + P + ... + P^(k-1) and (Pm)_i=m_(i-1). Physical robot i initially owns role i; an announced removal preserves survivor order.

## 3. Arithmetic classification and complementary boundary control

**Theorem 1 (geometry-independent cyclic classification).** Let d=gcd(N,k), q=N/d, s=k/d. Equal service loads imply m_i=m_(i-k). The jump map i -> i+k has d orbits, each of size q, corresponding to residue classes modulo d. The quotient Z_N / <k> is Z_d. The kernel of R_k has dimension d-1 and consists of orbit-constant vectors of zero total sum. With Fourier vector f_j(i)=exp(2*pi*sqrt(-1)*j*i/N), the blind indices are j=q,2q,...,(d-1)q.

Proof: subtract neighboring window sums to get m_i-m_(i-k)=0. Repeated addition by k visits exactly the residue class modulo d, since gcd(q,s)=1. An orbit-constant vector has d free values. Every length-k window contains k/d copies of each residue, so the zero-load condition is their zero sum, leaving d-1 dimensions. Fourier diagonalization gives eigenvalue sum_(h=0)^(k-1) exp(-2*pi*sqrt(-1)*j*h/N). The geometric sum vanishes exactly when N divides jk and N does not divide j. These are precisely the stated indices. Obstacles affect F and the physical moments, while these mass identities remain unchanged.

For an auxiliary order h define ell^(h)=R_h m and Q_h=2I-P^h-P^(-h). The implemented boundary law, with uniform weights on the active set K, is

```
x_dot_i = boundaryGain / |K| * sum_{h in K} (ell^(h)_(i+h-1) - ell^(h)_(i-1)).
m_dot   = -boundaryGain / |K| * sum_{h in K} Q_h m.
```

The service order remains k; auxiliary orders change the boundary law only.

**Theorem 2 (nonempty atoms and mass repair).** With time step dt, beta=dt*boundaryGain in (0,1/2), the update has

```
m_new_i = (1-2*beta)m_i + beta/|K| * sum_{h in K}(m_(i+h)+m_(i-h)).
```

Strictly positive masses remain positive and sum to 1. The common nonconstant blind dimension is gcd(N,h for h in K)-1. All masses converge to 1/N exactly when this gcd is 1. Consecutive orders k and k+1 always meet this condition when admissible. If k=N, the implementation uses k-1 instead. For fixed K, each Fourier mode contracts by 1-beta*lambda_j, where lambda_j is the average of 4*sin^2(pi*j*h/N). The largest nonconstant factor in absolute value determines the rate; it depends on N and the selected orders.

Proof: the formula is a positive convex average and preserves the total by cyclic reindexing. Fourier diagonalization simultaneously diagonalizes all Q_h. A common zero eigenvalue requires N to divide jh for every h, so there are exactly gcd(N,K) common zero modes including the constant one. Every other factor lies strictly between -1 and 1. This proves the classification and convergence. Persistent switching between orders has the same conclusion if every required order appears with bounded gaps and beta is bounded away from 0 and 1/2: products of the scalar Fourier factors then contract in each bounded window. Arbitrarily long inactive intervals give no uniform rate.

No uniform-in-N convergence rate is inferred from nonabelian expander results. The simulation does not alter mass traces to make them look convergent.

## 4. Order-k Voronoi baseline

For a k-element subset I of robot IDs define

```
V_I(P) = {q in Omega: max_{i in I} ||q-p_i|| <= min_{j not in I} ||q-p_j||}.
W_i(P) = union_{I containing i} V_I(P).
```

Ties use increasing physical ID. The controller uses the centroid of W_i, the full dominant region, rather than an ordinary first-order cell or the centroid of one k-subset cell. Clipping by several obstacles can make a cell disconnected. Not all of the N-choose-k possible subsets occur. The viewer colors full owner subsets and highlights all cells containing a selected robot. Sampled subset counts do not count connected components and can miss tiny cells.

**Theorem 3 (fair common geometric score).** At fixed positions P, choosing the k nearest robots minimizes H over all size-k assignments. In particular H(P,Voronoi) <= H(P,cyclic). This is a statement at the same positions, not a comparison between two different closed-loop final configurations.

Proof: for each q, the sum of the k smallest squared distances is no greater than the sum for any other k-element subset. Integrate against positive rho. The same inequality holds for convex mixtures of subset assignments used at split quadrature bins.

The viewer therefore plots both assigned H and the common k-nearest H evaluated separately at each trajectory's positions. Standard Voronoi descent has no equal-load constraint. Applying a circular Fourier blind-space classification to its evolving geometric hypergraph would be incorrect.

This reproduces the k-nearest subset and higher-order centroid mechanisms in [Jiang, Sun, Anderson, and Lageman](https://arxiv.org/html/1703.02424v2). The dominant-region definition also appears in [Self-Triggered Distributed K-Order Coverage Control](https://people-ece.vse.gmu.edu/~cnowzari/papers/jp15.pdf). It is not a reproduction of that paper's self-triggered protocol or its numerical figures.

## 5. Common safe single-integrator controller

Both methods use u_nom_i = gain*(c_i-p_i). If ell_i=0, u_nom_i=0. Centroids are not projected out of obstacles. A centroid inside an obstacle remains visible in the map.

For buffer epsilon, define expanded-obstacle radius R_o=r_o+r_b+epsilon, outer admissible radius R_D=1-r_b-epsilon, and pair separation D_b=2*r_b+epsilon. Use alpha>0 and the constraints

```
2(p_i-o) dot u_i >= -alpha*(||p_i-o||^2-R_o^2),  for every obstacle o;
-2*p_i dot u_i  >= -alpha*(R_D^2-||p_i||^2),     for the outer wall;
2(p_i-p_j) dot u_i >= -(alpha/2)*(||p_i-p_j||^2-D_b^2), for each other robot j;
|u_ix|, |u_iy| <= speed/sqrt(2).
```

Each robot's velocity is the Euclidean projection of u_nom_i onto these half planes. The exact two-dimensional projection enumerates the nominal point, face projections, and face intersections. Redundant distant constraints are omitted only when the speed bound proves they cannot bind. Zero is feasible at every safe state.

**Proposition 4 (continuous-time safety and motion descent).** Under continuous recomputation from complete exact states, these constraints preserve the safe set. Adding the two reciprocal pair constraints gives the barrier inequality for pair separation. For fixed responsibility, H_dot <= -sum_i ell_i*||u_i||^2/(k*gain) <= 0.

Proof: every obstacle, wall, and pair safety function satisfies h_dot >= -alpha*h. Hence h(t)>=exp(-alpha*t)h(0)>=0. Euclidean projection onto a closed convex set containing zero gives u_nom_i dot u_i >= ||u_i||^2. Since grad_(p_i) H = (ell_i/k)(p_i-c_i), the descent inequality follows. This is a specific barrier-style construction inspired by [Ames et al.](https://arxiv.org/abs/1609.06408), not a claim to implement a general-purpose CBF optimizer.

**Proposition 5 (the implemented discrete certificate).** Commands are held constant for dt. A common scalar step reduction is applied until every swept robot-obstacle segment, every swept relative robot-robot segment, and every outer-wall endpoint satisfies its exact disk clearance, and the fixed-assignment quadratic cost does not increase. A zero command is used if no candidate passes. Thus every accepted straight motion segment is safe. This changes commanded velocity; it never clips an endpoint or resets a plant state.

Proof: distance from a point to a line segment gives the exact closest approach to a circular obstacle. Relative linear motion gives the exact pair test. The outer admissible disk is convex. The fixed-assignment cost is evaluated directly before acceptance. Voronoi reassignment at the new positions cannot increase that cost by Theorem 3. Therefore safe robot motion with frozen sectors, or with Voronoi reassignment, has nonincreasing recorded H between removal events. Moving sector boundaries are a separate operation and can increase H; its actual cost contribution is exported separately.

Safety does not prove centroid convergence. For example, a centroid at the center of an obstacle and a robot on its expanded boundary can give zero safe velocity despite nonzero centroid error. Pair separation can also prevent several robots sharing a centroid. The page reports centroid residual and applied velocity so such traps remain observable. No global optimality, deadlock-free navigation, arbitrary-obstacle higher-order dynamics guarantee, or monotone cost for simultaneous boundary adaptation is claimed.

## 6. Failures and communication

An announced failure removes the same physical ID at the same epoch in both runs, with no survivor position changes. The sector method deletes that robot's ray: the preceding surviving atom absorbs the deleted atom. Positive adjacent masses merge, total mass remains 1, and the new cyclic operator uses the survivor count. Voronoi recomputes nearest-k subsets among survivors. At least k robots must remain. This is robot removal; a parked broken robot would require a new obstacle and a new demand-map model.

For N=50,k=7, initial gcd is 1. After one removal N=49 and d=7: seven orbits, each of length 7, six blind Fourier modes at indices 7,14,21,28,35,42. Starting from equal basic masses, the projected relative basic-mass residue is sqrt(6)/50, approximately 0.04898979, independent of the failed position's phase. Auxiliary order 8 eliminates the common blind space. The physical obstacle map changes the locations of the rays and centroids without changing this calculation.

For N=1000,k=21, gcd=1. At N=999 the gcd is 3 and the blind dimension is 2; at N=998 it returns to 1; at N=1001 it is 7 and the blind dimension is 6. Order 22 complements order 21 for each population. Physical convergence rate and communication cost still depend on population and geometry. These large arithmetic examples can be inspected without pretending the all-gather physical viewer supports 1000 robots in real time.

The comparison intentionally uses the same complete all-gather snapshot protocol on both methods. Position, physical ID, and cyclic boundary mass coordinate are routed hop by hop. Reliable logical ring edges are ideal links. Alternatively, radio edges exist exactly when endpoints are within range and their segment misses every obstacle. Link-state discovery and directed edge queues use the existing Network engine. Each epoch's communication-round budget includes discovery and snapshot delivery. The entire snapshot is charged before a budget overrun is detected; the update is then rejected. This is an admission test, not a scheduler that truncates packets at the budget.

A disconnected carrier or rejected epoch freezes that run. The viewer does not silently extrapolate missing packets. Each method can encounter a different moving radio graph because its positions differ. The two runs can therefore stop at different times, which are shown explicitly. Communication rounds are accounting; the single-integrator motion clock does not include physical transmission delay. Safe projection needs the complete state snapshot, so limited-range local-only safety and unknown private maps are not claimed.

The numerical simulator executes the replicated computation in one JavaScript process after validating packet delivery. This is a distributed reference protocol with a shared known map, not a deployment to physical robots or an optimized neighbor-only implementation. Equal all-gather permits an objective comparison of geometry without inventing a communication advantage.

## 7. Quadrature, reproducibility, and reporting

The common polar quadrature has angular bins and equal-area radial bins. Samples inside obstacle unions are removed, demand weights normalized once, and the same samples used by both methods. The angular marginal and first/second moments are piecewise constant within each angular bin. A cyclic boundary crossing a bin splits its moment by angular fraction. This makes the mass coordinate and cyclic algebra exact for the discretized angular model. Voronoi selects the exact k nearest sample sites with deterministic ties. Maps are quadrature approximations, not analytic Voronoi polygon constructions. Obstacles are drawn at their exact radii.

Use resolution refinement and multiple seeds before publication. Every export includes settings, quadrature size, raw paired histories, trajectories, packet traces, failure events, actual motion and boundary cost changes, and assumptions. The reported input integral is sum_i integral ||u_i||^2 dt in common velocity units; it is not physical energy.

The recorded benchmark suite separates obstacle count, service order, failure policy, communication failure, and grid refinement. Tests compare fixed configurations for the nearest-subset inequality, check exact swept safety, validate the mass update and Fourier residue, and test an explicit stationary trap. Synthetic DOM tests and static resource checks are reported separately from live browser verification.

For the recorded three-obstacle run (N=18,k=3,t=24), the coarse quadrature gives assigned H approximately 0.04413 for cyclic sectors and 0.04132 for Voronoi. Relative responsibility-load error is approximately 1.6e-11 and 0.263 respectively. Doubling angular and radial resolution changes these assigned costs by approximately 0.075% and 0.15%, while Voronoi load imbalance changes to approximately 0.280. Thus cost is less grid-sensitive in this check; the finite-grid imbalance value is not a certified continuous-domain result. Random initializations show different sampled Voronoi subset counts and constrained equilibria. These observations support a cost/load tradeoff, not universal superiority of either controller.
