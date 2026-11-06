# Dual-arm tray transport with an actual free object

## What the experiment demonstrates

One TIAGo Pro is one mobile-manipulator agent. Its left and right arms share the base and torso and are resources of the same agent. A transport task reserves both arms of one robot; a dual-view inspection task needs two independently positioned robots. These two kinds of multiplicity are not interchangeable.

The lab adds a free dynamic tray to the pinned PAL-derived20-coordinate planar-supported TIAGo model. The tray has mass2 kg by default, half-extents(0.14,0.25,0.025) m and an explicit mass-scaled inertia. A free joint contributes7 position coordinates (translation plus quaternion) and6 velocity coordinates, producing nq27,nv26,nu20. Two ideal fixed-grasp weld constraints couple actual hand/object anchors. The object is not kinematically attached, teleported, or moved by rendering keyframes.

## Initial fixture and equations

During setup only, damped position IK puts left/right tool sites near(0.62,+/-0.25,0.95) m, and the tray is placed consistently on a raised pickup support. Arm configuration, initial tray pose and weld relative orientations are then fixed as initial conditions. Fingers remain fixed and nearby tray-hand collision pairs are excluded to avoid redundant contacts at the ideal grasp. Other primitive contacts, gravity and limits remain. Grasp acquisition and frictional finger slip are outside this fixture experiment.

Let q_r,v_r be the robot state and X_o,V_o the object pose/twist. The combined EL model is

```text
M(q) v_dot + b(q,v) = S^T tau + tau_passive
                         + J_g(q)^T lambda_g + J_c(q)^T lambda_c.
phi_g(q) = 0 approximately, while a hand grasp is active.
```

M includes articulated coupling and the free object. J_g is the fixed-grasp constraint Jacobian, J_c the contact Jacobian, and MuJoCo solves compliant equality/contact reactions. Welds constrain six relative freedoms and have numerical residuals. [MuJoCo's equality documentation](https://mujoco.readthedocs.io/en/3.1.6/XMLreference.html) defines these fixed-grasp semantics; the runtime itself is pinned at3.8.1. No equality reaction is canceled to make an animation follow a scripted object trajectory.

The robot reference uses smooth quintic segments: torso lift during0..3 s, chassis translation along world y during3..9 s, lowering9..12 s, and an additional8 mm seating request during11.5..12.8 s. The default lift is0.18 m and translation1 m. Other robot reference coordinates hold their setup values. The object has no commanded trajectory.

At each0.005 s physical step, a bounded full-mass computed-acceleration controller acts on the robot coordinates:

```text
a_r = qref_ddot + Kp(qref-q_r) + Kd(vref-v_r),
tau_requested = [M(q) a]_robot + b_robot - tau_passive,robot
                + sum(active hands h) J_h,z^T * (m_object*g/n_active).
tau = actuator-range clipping(tau_requested).
```

The20 robot accelerations are used; free-object desired acceleration is zero in this mass product. Base gains use Kp36,Kd12; other coordinates Kp100,Kd20. The last term is a declared known-payload gravitational feedforward, shared across currently active hands. It is a control input, not a fabricated force measurement. Native hand load curves instead read actual equality-solver translation forces on the tray; their units are N. Rotational equality residual coordinates are not mislabeled as meter errors. Grasp-anchor mismatch is computed separately from actual object and hand transforms.

## Contact-gated release and physical success

Release is allowed after13 s only if actual tray/delivery-support contact exists, horizontal placement error<0.04 m, absolute height error<0.01 m, tilt<0.08 rad and speed<0.08 m/s. Releasing disables active hand equalities. The object must then remain on the separate support. Final success at15 s also requires speed<0.03 m/s, maximum recorded joint-range violation<0.01 in native joint units, and no recorded contact penetration deeper than0.01 m. Pose, velocity and contact errors remain in the export; there is no endpoint or velocity reset.

The default dual-arm trial succeeds with final horizontal error0.0118151 m. At3 s, actual left/right vertical forces are9.80711 and9.80714 N, close to half the19.62 N payload weight. Maximum active-grasp anchor mismatch is49.34 micrometers, maximum orientation mismatch about0.028 rad and maximum contact penetration1.93 mm. These finite residuals are consistent with a numerical compliant model; they are not exact rigidity or hardware accuracy guarantees. No actuator saturation or joint-range violation occurs in this default trial.

The3 kg and1.5 m dual-arm cases also succeed with the same controller. Disabling one or both grasps produces FAILED physical outcomes that stay visible and downloadable. The left-only case ends with horizontal error0.136 m. The no-grasp case leaves the tray behind and includes a21.58 mm transient contact penetration, violating the accepted success bound. It is a failure countercheck, not a certified safe operation. A failed left-only fixture controller does not prove a properly designed single-arm grasp could never carry this payload.

## Connection to distributed task allocation

An agent bidding for transport must reserve both arms, payload capacity, a reachable initial grasp and a feasible loaded path. The transport simulation supplies one physically evaluated resource-occupation interval and explicit success predicates. The fleet allocation lab separately reserves ID2 during[12,27) s and reassigns inspection jobs. That fleet reservation is a declared schedule; the moving tray is NOT currently co-simulated inside the fleet map. Coupling actual transport completion/failure events into the distributed task graph is the next integration step, not an already-established guarantee.

A useful next mission has inspection coalitions certify an object, one eligible dual-arm robot transport it, then another independent-view coalition confirm placement. This needs a typed task graph, precedence edges and exclusive-resource constraints alongside the cyclic reference. Head-camera feasibility and two-hand object closure require different certificates. The reference N,k,gcd class can remain unchanged while these capability hyperedges change over time.

The model retains planar base support, ideal chassis/generalized-force actuation, simplified collision geometry and known payload feedforward. It does not model wheel rollers, tipping, autonomous pickup, fingers/slip, unknown load estimation or deployment on a real TIAGo. The runtime/model provenance is documented in [the whole-body baseline](tiago-theory.html) and [third-party sources](third-party.html). ROS2 remains offline.

Open the [transport lab](Bimanual_Transport.html), [execution-aware allocation](Execution_Allocation.html) and [operating guide](Guide.html#dual-arm-transport). The maps are state-derived schematics rather than CAD renderings. Numerical checks and synthetic DOM/event tests do not establish real browser rendering or hardware validation.
