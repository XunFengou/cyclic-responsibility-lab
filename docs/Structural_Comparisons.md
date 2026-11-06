# Fair structural comparisons: where cyclic responsibility helps

The defensible claim is a predictable responsibility template and a simple algebraic repair mechanism, with a conditional communication benefit. It is not universal superiority in distance cost, load balancing, fault locality, or total system traffic.

The physical comparison, arithmetic experiments, fixed-position assignment references, and protocol replays are different experiments. Their clocks and assumptions must remain visible.

The physical experiments inherit the known static unit-disk domain with three circular obstacle holes, positive normalized demand, and safe single-integrator plants from the [base problem definition](https://cyclic-responsibility-lab-c171.aheadmeter.chatgpt.site/comparison-theory.html). Distance cost is Euclidean through obstacles; visible/range-limited coverage is a separate diagnostic. The protocol tests do not replace the base safety controller with an unproved local one.

## 1. Four objectives and a stronger geometric baseline

Use the same normalized demand measure mu, total mass 1, and service order k. For robot positions P, define assigned coverage cost H = integral sum of the k assigned squared distances / (2k). Let ell_i be responsibility mass. Report relative RMS load error, maximum normalized load N*max(ell_i)/k, actual control effort, path length, physical clearance, and communication separately. No arbitrary weighted score is used to manufacture a winner.

Nearest-k Voronoi minimizes H among size-k assignments at fixed P. This follows pointwise by selecting the k smallest distances. Comparing different final configurations is a separate optimization question. A fixed cyclic template can incur a distance penalty in exchange for a restricted responsibility family and explicit dynamics of its masses.

Balanced geometric methods already exist. For example, [Pavone, Arsie, Frazzoli and Bullo](https://arxiv.org/abs/0903.5267) study distributed equitable power-diagram partitions. That work concerns a different geometric partitioning problem; it nevertheless rules out describing load balancing as unique to cyclic sectors. [Higher-order coverage control](https://arxiv.org/html/1703.02424v2) uses the union of all nearest-k cells containing a robot as its dominant region. The new tests use that full region, not an ordinary first-order cell.

For a strong same-position benchmark, the website solves the fractional capacity-constrained quadrature problem:

```
minimize sum_(q,i) f_qi * ||q-p_i||^2 / (2k)
subject to sum_i f_qi = k*w_q;
           0 <= f_qi <= w_q;
           sum_q f_qi = k/N for every robot i.
```

Here q is a quadrature point and w_q its positive demand mass. Each source point supplies k*w_q; its edge to robot i has capacity w_q; each robot has sink capacity k/N. Fractions f_qi/w_q lie in the convex hull of size-k membership vectors. A cell may split between more than k robots, but its total membership is k and no robot receives more than 1. This discrete reference is not a binary sampled Voronoi diagram or a distributed capacity-aware controller.

Successive shortest augmenting paths with reduced-cost potentials solve the flow problem. The code then checks absence of a negative residual cycle. Feasibility and this residual condition certify optimality: any cheaper feasible flow would differ by a circulation, which decomposes into residual cycles and would require a negative-cost cycle. Both capacities and weights are real-valued; numerical tolerances are explicit in the code.

Equal-mass cyclic atoms provide a feasible fractional assignment on exactly the same quadrature. Therefore

```
H_nearest(P) <= H_capacity(P) <= H_equal-cyclic(P).
```

The balanced reference may have a lower cost than balanced cyclic sectors. Its centralized optimization and initialization information are not claimed free. This experiment measures the geometric price of the cyclic restriction, rather than comparing different load constraints.

## 2. Control the global cyclic phase

Let F(theta) be normalized cumulative angular demand, and use equal basic masses with boundaries x_i = phi + i/N. The phase phi rotates all rays in mass coordinates while preserving their order and every basic mass. An arbitrary phase can create an avoidable cost penalty or unnecessary robot travel.

The code reports phase 0 and an offline best phase. In the piecewise-constant angular quadrature, cumulative first and second moments are piecewise linear in F. H(phi) is consequently piecewise linear. Its breakpoints occur when some boundary phi+i/N meets an angular-bin mass coordinate. Enumerating phi = F_bin - i/N modulo 1, including 0, finds a global minimum for this discretized phase problem. Thus

```
H_nearest <= H_capacity <= H_best-phase-cyclic <= H_phase-0-cyclic.
```

This removes an orientation bias in the fixed-position comparison. It does not optimize the physical robot positions. The offline search is bounded to the small reference experiment and is not presented as a free distributed online controller.

The physical comparison now accepts an initial phase offset in mass turns. The recorded phase ablation uses the same robots, density, obstacles, gain, speed limit, safe single-integrator filter, and full-snapshot protocol in both runs. Only the sector phase changes. Compare travel and velocity effort as well as final H: a much better initial phase need not improve the final constrained cost appreciably.

## 3. A geometric case in which the methods coincide

Place robot i at radius r>0 and angle 2*pi*i/N, with uniform angular demand. For a query point away from the origin, distances rank according to angular proximity because

```
||q-p_i||^2 = ||q||^2 + r^2 - 2*r*||q||*cos(theta_q-2*pi*i/N).
```

The nearest k robots are consecutive in cyclic order. The owner set {i,...,i+k-1} has its angular interval centered at 2*pi*i/N+(k-1)*pi/N. Cyclic atom i has that same interval when phi=(k-2)/(2N). The assignments coincide except on measure-zero ties. Their dominant moments, loads, and assigned costs agree. The recorded quadrature aligns its angular bins with these rays and verifies the equality numerically.

This connects the two methods naturally: a cyclic interval hypergraph is the nearest-k template on a rotationally symmetric configuration. Away from that symmetry, Voronoi adapts its owner subsets to geometry, while cyclic responsibility retains its chosen template. A ring-like robot arrangement alone does not guarantee coincidence under nonuniform equal-mass rays or irregular radii.

## 4. Joint N,k classification and spectral scale

For masses m in R^N, define (Pm)_i=m_(i-1), R_k=I+P+...+P^(k-1), and ell=R_k m. Equal neighboring service loads imply m_i=m_(i-k). With d=gcd(N,k), q=N/d, s=k/d, the jump i -> i+k has d orbits of length q, corresponding to Z_N/<k> isomorphic to Z_d. The nonconstant blind space has dimension d-1. Its Fourier indices are q,2q,...,(d-1)q.

Boundary control at order h gives Q_h=2I-P^h-P^(-h). Its Fourier eigenvalue is 4*sin^2(pi*j*h/N). For active orders K, the implemented positive Euler mass update with beta in (0,1/2) has mode factors

```
lambda_j(K) = average_(h in K) 4*sin^2(pi*j*h/N);
m_new = (I-beta*average Q_h) m;
factor_j = 1-beta*lambda_j.
```

Its common blind dimension is gcd(N,h for h in K)-1. With no nonconstant common kernel, the worst contraction factor is max_(j!=0) |factor_j|. This depends on N and the jump lengths. The number of updates reported in the table shrinks any initial nonconstant mass error by a factor 10^-6; it is not a physical settling time.

For fixed k, the blind dimension is periodic in N modulo k. The orbit length, Fourier indices, physical ray positions, and spectral rate are not specified by N modulo k alone. Calling the whole system periodic in N would lose these scale effects.

For N=1000,k=21 there is no blind mode. At N=999 there are two, at N=998 none, and at N=1001 there are six. For N=970,k=25 there are four, with d=5,q=194,s=5. Adjacent orders remove every common nonconstant blind mode. Auxiliary order 1, or boundary order 1 alone, also repairs all populations; complementarity is not the only possible strategy. Compare spectral rates and routed packet costs before choosing the auxiliary law.

At fixed k,h and increasing N, the j=1 eigenvalue is at most a constant times (k^2+h^2)/N^2. Consequently these fixed generators do not provide a scale-independent spectral gap. A favorable numerical gap for N=1000 is not an expander theorem.

Persistent switching works when each required order occurs repeatedly with bounded gaps and positive update weights bounded away from the endpoints. Circulant operators commute, so scalar Fourier contractions multiply. The test alternates order k and an adjacent order in 15-epoch blocks. Neither simultaneous mixing nor switching is claimed uniformly faster in every mode.

## 5. Fixed geometry while N and k change

Arithmetic sweeps alone cannot establish geometric performance. The recorded nested-population experiment creates one master set of 35 physical positions. A fixed seeded permutation selects populations 18,...,35; surviving positions never move. The same obstacle map, density and quadrature are retained. Roles follow angular order. For each population, evaluate both assignments and the cyclic gcd classification, without running a new optimizer.

A second sweep holds the same 30 sites fixed and varies k=1,...,12. For the normalized nearest-k objective, increasing k cannot decrease the average of the k smallest squared distances. Adding sites at fixed k cannot increase the k-nearest cost. Both are independently checked. These are geometric rank properties, not gcd laws. The observed Voronoi subset count or load error can change at any geometric configuration; a fixed state-independent cyclic quotient is not imposed on it.

Both sweeps report phase-0 and offline best-phase cyclic costs. This distinguishes the cost of a restricted cyclic owner family from an arbitrary initial orientation. Load constraints are still different from ordinary unweighted Voronoi; the capacity reference is used to compare the same load target.

The cyclic owner family has N subsets for k<N and one for k=N. One must not infer that planar Voronoi actually realizes N choose k cells: planar order-k diagrams have much stronger geometric restrictions. The [higher-order Voronoi edge-labeling study](https://arxiv.org/abs/2109.13002) is an entry point to that combinatorial structure. Obstacle clipping can split regions without introducing a new nearest-k owner set.

## 6. Fair communication experiments

### The existing full-snapshot baseline

The physical comparison deliberately routes the same [ID, x-position, y-position, boundary-coordinate] snapshot in both runs. If that snapshot is already delivered and the static demand map is known, either assignment can be computed locally. Additional responsibility communication is zero for both. Claiming a total communication advantage for sectors in that model would be incorrect.

### Native responsibility-layer replay

The new replay isolates responsibility computation from robot motion and safety. Every protocol uses exactly the same timestamps, positions, mass-coordinate boundaries, density quadrature and carrier. Both sector-generated and Voronoi-generated trajectories are replayed; no protocol receives a favorable private trajectory. The global simulator seed is for reproducibility, not an extra distributed measurement from which a controller may reconstruct every private state.

The cyclic protocol sends the next boundary to compute its local atom. It circulates moment tokens for service order k and any extra scalar masses needed for auxiliary orders. Previous-load and routed future-load messages give the same boundary law as Q_h. Every received service moment and resulting mass update is checked against the exact global operator. For fixed small k on a logical ring, the number of moment transmissions is linear in N and k. For large k it grows, and a cut carrier ring can make logically neighboring messages traverse a long path. No topology-independent O(N) total-cost theorem is claimed.

The Voronoi protocol is not restricted to a fresh global snapshot. Each robot keeps timestamped peer positions. If a peer was last seen at p_hat_j at time t_j and the common speed bound is v_max, its current position lies in a ball of radius b_j=v_max*(t-t_j). For a query q, its distance lies between max(0,||q-p_hat_j||-b_j) and ||q-p_hat_j||+b_j. Robot i knows its own exact position. It certifies membership if at least k peers are definitely closer, or fewer than k peers can possibly be closer. Otherwise it requests exact positions from uncertain competitors. Request and response messages are routed and charged separately. After refreshing the union of uncertain competitors, every membership is resolved and checked against a fresh nearest-k calculation.

This certificate follows from the triangle inequality and the advertised speed bound. A certified decision remains valid after an uncertainty interval shrinks, so the one-pass union of uncertain competitors suffices. Deterministic physical-ID ties are handled with exact states. The approach uses the same uncertainty-region idea as [Self-Triggered Distributed K-Order Coverage Control](https://people-ece.vse.gmu.edu/~cnowzari/papers/jp15.pdf), but is a new finite-quadrature replay protocol rather than a reproduction of that paper's continuous self-triggered controller.

The tests also run a fresh full-snapshot Voronoi baseline. The reported cyclic / best-recorded-Voronoi ratio uses the cheaper of the complete cached and scheduled runs. This favors Voronoi without pretending an online oracle mixes their packets for free. A cached query design can cost more than a scheduled snapshot when many peers require refresh.

All protocols use the same actual Network implementation: one packet per directed edge per round, explicit shortest carrier routes, 24-byte headers, initial link-state discovery, and bounded trace logs. Payloads are doubles: a moment is 32 bytes, a boundary or scalar load 8, a query 16, a position/time reply 24, and a full record 32. Packet hops, payload byte-hops, total byte-hops, rounds, startup cost and later cost are all recorded.

Logical rings are ideal links. A path removes the same closing edge. Radio edges require the same range and obstacle-free line of sight. The current radio reference performs link-state flooding every epoch; its cost is included and can dominate both methods. Wireless interference, propagation delay, asynchronous plants and retransmission are not modeled.

### Budget and counterchecks

For a total round budget B, count the complete prefix of replay epochs whose cumulative delivery cost does not exceed B. Do not select scattered cheap epochs. This is an admission audit of a prerecorded trace, not an actuated budget-limited robot experiment or a stability theorem for stale data.

Known frozen positions give b_j=0. After its initial cache, Voronoi needs no refresh; the fresh cyclic protocol continues sending boundary/moment updates. This counterexample prevents a universal communication claim. Event-based caching of cyclic statistics could improve that implementation too, but is not silently substituted into the measurements.

The replay does not implement a local-only obstacle and pairwise collision-avoidance controller. Adding a shared full safety snapshot and then charging these responsibility packets again would double-count available information. Full-stack savings require an additional distributed safety design or a justified separation of information streams.

## 7. Fault position, quotient mode and reconstruction

An announced failure removes a robot, not a parked obstacle. Survivors retain their physical states. The cyclic ray is removed and adjacent atom masses merge. This preserves positivity and total demand. The new R_k and Q_h use the survivor count. Reliable failure announcement is a common assumption for both methods; autonomous detection and consensus on membership are not proved here. A replay that changes membership includes the same routed announcement charge for both protocols.

For one failed robot at fixed surviving positions, ordinary nearest-k reassignment changes a point's owner set only if the failed robot used to belong to it; it then inserts the previous (k+1)-st candidate. It does not need to change every point's owner set. The cyclic delete-ray rule also replaces one membership only where the failed role was responsible. The exact mass-coordinate interval calculation and nearest-k quadrature both check this fact.

If normalized turnover is integral of the number of new memberships divided by k, its value for either rule is the old failed robot's responsibility load divided by k. Equal cyclic loads therefore give turnover 1/N for every failed ID. The maximum load of any size-k assignment is at least k/N, so equal loads attain the smallest possible worst-case single-failure exposure. A capacity-balanced geometric assignment attains that same bound; the benefit is not exclusive.

With equal pre-fault basic masses and one removal N0 -> N0-1, the cyclic blind projection has relative RMS norm sqrt(d-1)/N0, where d=gcd(N0-1,k). Changing the failed ID cyclically shifts the mass perturbation, so its Fourier magnitudes and blind norm are unchanged. Physical post-fault cost can still vary with location. For N0=50,k=7 this gives sqrt(6)/50. Adjacent complementary orders remove that blind space.

With several removals, d determines dimension but not amplitude. Different deletion patterns can add or cancel Fourier components. The equal-population test N0=50 -> 48,k=6 compares adjacent failures 13+14, separated failures 13+25 and 13+28, and the wraparound adjacent pair 1+50. All have five blind dimensions; their projection norms differ. Do not infer fault impact from gcd alone.

## 8. What would justify the paper's contribution?

A credible contribution combines the exact quotient/Fourier classification with a constructive positive boundary update, certified auxiliary-order selection, and explicit failure reconstruction on real free-space sectors. Communication benefits must be stated for a specified protocol and information model. A cost/load tradeoff should include a capacity-aware geometric reference and phase alignment. Rate claims must include N, generator length and carrier cost.

The current evidence supports this program: a compact responsibility representation, a predictable arithmetic failure mode, and a simple repair that avoids recomputing a geometric owner family. It also exposes counterexamples: symmetric coincidence, geometric balancing with lower cost, cached static Voronoi with no steady traffic, and identical total communication under shared safety snapshots.

The website includes raw recordings, three new figures, reproducible settings, algebra/packet/flow tests and synthetic DOM tests. Actual browser QA is reported separately; a safety stop is never bypassed or converted into a claim of successful live interaction.

## 9. Recorded observations and their limits

For the 18-robot, k=3 ring-carrier replay, cyclic responsibility uses 81,648 byte-hops versus 660,040 for certified cached Voronoi and 755,568 for scheduled full snapshots over the same nine position frames. The cyclic / better-recorded-Voronoi ratio is approximately 0.124. On the known-frozen 31-frame replay, the cached Voronoi total is 102,384 and cyclic fresh-moment total 230,544; Voronoi wins that conditional comparison. On a geometric radio carrier, repeated common discovery reduces the apparent cyclic advantage substantially. The path example also shows that fewer byte-hops do not imply fewer rounds.

At one fixed near-ring 18-robot, k=3 configuration, the 444-point reference gives nearest-k cost 0.04824, capacity-balanced optimum 0.05466, and best-phase equal-mass cyclic cost 0.05547. The latter is approximately 1.48% above the capacity optimum for this discretization. A random configuration has a much larger gap. This is evidence of a geometry-dependent tradeoff, not a continuous-domain accuracy certificate or a universal approximation ratio.

The separate 24-second physical phase ablation uses N=18,k=3 and a finer 120-by-12 quadrature. Phase initialization reduces cyclic travel from approximately 8.226 to 2.804 and its velocity-squared input integral from 1.319 to 0.268. Final assigned H remains approximately 0.04413; Voronoi remains near 0.04132. Initial positions and the Voronoi trajectory are identical across this ablation. Thus phase selection improves initialization without proving a final-cost advantage.
