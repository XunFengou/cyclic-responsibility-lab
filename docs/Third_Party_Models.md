# Third-party models and physics runtime

The TIAGo Pro source asset, its license, reference constants and head xacro files are retained under src/vendor/tiago-pro. PAL Robotics' original model is pinned to commit 952abb83a590d0e771cfd8e570517ba054182196 of pal_mjlab. The head source is pinned to a10e252dc55968c6ac91d2a84eb4626a4811bc78 of tiago_pro_head_robot. They are Apache-2.0 licensed. The generated planar20.xml is a modified derivative; provenance.json and prepare-tiago.py list the changes and verify exact preservation of explicit inertial entries.

MuJoCo is copyright its respective authors, distributed under Apache-2.0. Its full license is retained as src/vendor/MuJoCo-LICENSE and copied beside the deployed runtime. The official @mujoco/mujoco npm archive is pinned to 3.8.1 and verified against SHA-512 integrity in scripts/install-physics.mjs. Runtime binaries are reproducibly downloaded into an ignored D:-local directory and copied into the Site build; they are not stored as opaque binaries in the research source repository. The generated runtime manifest contains SHA-256 hashes of the deployed JS and WASM.

No PAL visual meshes are redistributed by this website. The original MJCF is retained for provenance; the generated simulator uses explicit inertias, primitive collision geometry and a state-derived skeleton renderer. Product compatibility, measured hardware parameters and real-robot validation are separate from these software licenses.

### Experimental acquisition tool

The autonomous acquisition extension retains the vendor-derived link inertias but disables the earlier fixed palm/fingertip collision shapes in favor of four explicitly modeled actuated parallel pads per robot. The experimental EE offset is0.25m. This replacement tool and its contact-certified ideal latch are research additions in acquisition-plant.mjs; they are not an official PAL gripper or a manufacturer-validated grasp controller. Full generated scenes and exact qpos recordings are included with the MP4 provenance.
