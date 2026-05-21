# Media Artifacts

This folder contains the final demo recording and error images referenced by the repository documentation.

## Demo

- `demo2.mp4`: Final demo video and key mission recording shown from the repository README.
- `demo.webm`: Original WebM export of the demo recording.

## Error Images

These images show errors faced during development and are not final demo moments.

- `global_joint_error.png`: Global joint-axis / transform mismatch issue observed during earlier iterations (robot body heading was stable but limb axes behaved independently).
- `joint_limit_error_1.png`: Joint-limit / pose tuning issue observed after fixing the global joint error (values not fully verified against the official G1 controller defaults).
- `joint_limit_error_2.png`: Additional joint-limit / pose tuning issue from the same phase of debugging.
- `wall_stuck_error.png`: Early wall collision/property error where the robot could overlap or climb wall geometry and become stuck before the world collision setup was corrected.
