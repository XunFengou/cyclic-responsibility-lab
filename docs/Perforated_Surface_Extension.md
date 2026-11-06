# Future extension to perforated Riemannian surfaces

## Scope and geometric state

This is an extension contract and research plan. The current physical simulator uses a planar disk with circular holes; it does not implement intrinsic curved-surface coverage.

Let (M,g) be a compact orientable Riemannian surface, possibly with boundary, and mu=rho*dvol_g a normalized demand measure. The genus counts handles; boundary components count boundary loops. Several planar obstacle holes do not by themselves create positive genus. Robot positions lie in a feasible subset of M; orientations and joint states live in a separate robot configuration manifold.

In a chart y=(y_1,y_2), dvol_g=sqrt(det(g(y)))*dy_1*dy_2. A mapping of planar samples must therefore include the metric Jacobian in the pulled-back density. Reusing a flat quadrature with unchanged weights generally changes the demand measure. Charts must avoid double counting overlaps and have consistent orientation/frame transforms.

## Which algebra survives?

If there are N positive atoms with the same cyclic k-window incidence, R_k and every gcd/Fourier classification are unchanged: they act on finite mass vectors, not on coordinates. Surface curvature changes moments, distances and physical feasibility, but not a stipulated cyclic incidence matrix. This is a conditional statement about a chosen partition template, not a theorem that every surface has a natural global angular order.

A global circle-valued coordinate f:M->S^1 must be constructed. Ordinary polar angle can have singularities and critical fibers. For a cumulative boundary coordinate to be invertible, the pushforward demand f_*mu must have positive mass on every angular interval and suitable continuity. Coarea formulas provide the relevant fiber integration, with critical sets handled explicitly. On complicated surfaces, cuts, several charts or a graph of local cyclic fibers may be necessary. Then the correct operator may be a general incidence map or graph product rather than one global circulant.

Covering spaces describe topology of the surface or charts; the finite quotient Z_N/<k> describes the responsibility jump graph. They are not interchangeable objects. A group action preserving both measure and incidence would be additional structure. A universal cover or a nonabelian fundamental group does not automatically create an expander or a size-independent convergence rate in this finite cyclic controller.

## Intrinsic cost and motion

Replace squared Euclidean distance by d_g(p,y)^2. An intrinsic centroid is a minimizer of its integral (a Frechet / Karcher mean), and it may be nonunique outside a geodesically convex region. Away from cut loci with a unique logarithm, the descent direction is an average of log_p(y) in the tangent space. Use a retraction or exponential map for time stepping. A mapped Euclidean centroid is generally not an intrinsic centroid.

Curvature, injectivity radius, obstacle boundaries and moving contacts require new feasibility and convergence hypotheses. Positive atom masses alone cannot prove robot containment or no collision. Euclidean swept-segment checks do not certify a curved-surface trajectory. Any future theorem should separate incidence convergence, existence of an intrinsic mean, feasible task tracking and the robot's constrained dynamics.

## Geometry adapter contract

The existing demand / moment construction is isolated in comparison.js Domain; planar safety remains separate. A future implementation should implement and test these operations before enabling a new domain in the UI:

| Operation | Required meaning |
| --- | --- |
| measure / integrate | Normalized density including metric volume; chart-overlap accounting |
| distanceSquared(p,y) | Intrinsic geodesic distance with declared numerical tolerance |
| logMap(p,y), retract(p,v) | Tangent vector and manifold update, with cut-locus handling |
| feasible(p), sweptFeasible(path) | Surface, obstacle and pairwise constraints along the whole path |
| coordinatePushforward | Optional circle coordinate, demand positivity and inverse behavior |
| sensorFrame(q), tangentFrame(p) | Consistent robot/surface/world frame transforms |
| geometryRevision | Mesh, metric, atlas, density and mapping checksum |

No placeholder curved-surface button currently masquerades as a working simulator. The Theory center keeps this extension visible so the later implementation has explicit contracts and proof obligations.
