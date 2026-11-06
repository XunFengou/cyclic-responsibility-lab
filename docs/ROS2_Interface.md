# ROS 2 offline adapter and telemetry contract

## Implemented now

The separate [TIAGo Pro dynamics lab](TIAGo_Comparison.html) now runs actual articulated MuJoCo physics; its model is planar-supported and uses declared ideal actuators. ROS 2 hardware remains disconnected. The ROS 2 recording page validates and replays neutral JSON, downloads a synthetic example and produces an offline mission adapter draft. It never opens a ROS bridge, publishes commands or claims a connected robot. The platform selector is an integration target. The synthetic three-joint rig is not a TIAGo or G1 simulation.

## Telemetry v1

Use schema kth-robot-telemetry-v1. Required top-level fields are provenance, platform, model.uri, model.revision, frameId, units and frames. Provenance is synthetic-demo, simulation or hardware-recording. The browser validates structure, not authenticity of a provenance claim. Provide a pinned model revision/checksum and retain the source recording separately.

```json
{
  "schema": "kth-robot-telemetry-v1",
  "provenance": "hardware-recording",
  "platform": "Exact robot and variant",
  "model": {"uri": "package://pinned_model/robot.urdf", "revision": "commit-and-model-checksum"},
  "frameId": "map",
  "units": {"time": "s", "basePosition": "m", "orientation": "quaternion_xyzw"},
  "frames": [{"time": 0, "robots": [{
    "id": 1,
    "basePose": {"position": [0,0,0], "quaternion_xyzw": [0,0,0,1]},
    "joints": {"names": ["joint_a"], "position": [0], "velocity": [0], "effort": [0], "positionUnits": ["rad"], "effortUnits": ["N m"]},
    "contactMode": "Declared support mode"
  }]}]
}
```

SensorPose is optional, with the same position/quaternion fields. A missing sensor pose is not synthesized from the base. Revolute positions use rad and efforts N m; prismatic positions use m and efforts N. Joint velocity follows position units per second. This format stores physical joint state, not a fabricated common energy unit or an unrestricted full generalized-force vector.

Limits: file <=5 MB in the viewer; 1..1000 frames; 1..80 robots; 1..100 joints per robot. Times must be finite, nonnegative and strictly increasing. Robot IDs are unique positive integers, persistent in a segment; joint order and units cannot change. Quaternions must be normalized within 0.001. Split at a population change and declare the removal separately. Invalid imports retain the last valid recording and display an error.

## ROS adapter responsibilities

- Convert vendor messages and TF into a declared common frame. Do not assume the floating-base configuration and tangent velocity have the same length.
- Preserve acquisition timestamps, clock source and measured/commanded effort distinction. If hardware cannot supply effort, this v1 contract needs an explicit future schema extension rather than inventing zero measurements.
- Keep physical IDs separate from temporary cyclic role numbers. Record the model revision, mission map revision and contact mode with each experiment.
- Map manufacturer-supported interfaces explicitly. The downloaded coverage_adapter topic names are examples, not existing vendor topics.
- Select compatible QoS, record dropped/stale samples and reject inconsistent snapshots. DDS QoS does not by itself prove the synchronous coverage algorithm.

The [official ROS 2 QoS documentation source](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst) defines reliability, durability and related policies. The [official rosbag2 project](https://github.com/ros2/rosbag2) provides recording/replay tooling. Use the supported distribution and CLI options of the actual installation; verify topic types with that vendor stack. Save the raw bag, metadata, converter version, model checksum and neutral export together.

## Integration acceptance criteria

Before adding live control, validate model kinematics and native units, command/state interfaces, timestamps and transforms, joint/contact feasibility, and a robot-specific tracking controller. Then measure missed updates, tracking error, coverage information, clearance and recovery under the same fault scenario. The algebraic theorem requires complete complementary update windows; it does not certify the plant's whole-body tracking or radio behavior.
