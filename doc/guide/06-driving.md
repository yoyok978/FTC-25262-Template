# 06 - Mecanum driving and Pinpoint localization

[עברית](he/06-driving.md)

[Guide index](../../README.md#learning-path) | Sources: [StarterTeleOp](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/StarterTeleOp.java), [PedroDrive](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/drivetrain/PedroDrive.java), [Constants](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java) | Next: [autonomous](07-autonomous.md)

## Required robot configuration

The starter and Example05 need **four mecanum motors, goBILDA Pinpoint, and compatible odometry pods**. This is not a generic tank-drive or wheel-encoder template. Without Pinpoint, change the localizer implementation intentionally rather than deleting the device lookup and expecting localization to work.

| Constants field | Default | Configure or verify |
|---|---|---|
| `FRONT_LEFT_NAME`, `FRONT_RIGHT_NAME` | `frontLeft`, `frontRight` | Independent front motors |
| `BACK_LEFT_NAME`, `BACK_RIGHT_NAME` | `backLeft`, `backRight` | Independent rear motors |
| `PINPOINT_NAME` | `odo` | Pinpoint I2C device |
| `LEFT_DIRECTION`, `RIGHT_DIRECTION` | REVERSE, FORWARD | Actual mounting and motor conventions |
| `POD_TYPE` | goBILDA 4-bar pod | Your actual supported pod or intentionally configured encoder resolution |
| `FORWARD_POD_DIRECTION`, `STRAFE_POD_DIRECTION` | FORWARD, FORWARD | Verify with measured movement |
| `POD_Y_DIST` | 0 cm placeholder | Signed lateral offset of the forward-measuring pod |
| `POD_X_DIST` | 0 cm placeholder | Signed forward offset of the lateral-measuring pod |

Constants validates nonempty unique device names before hardware lookup. Correct names still do not establish correct physical corners or directions. Check mecanum wheel roller orientation and all four individual motors. If the hub configuration uses `leftFront` instead of `frontLeft`, either change that configured name or the matching field.

## Coordinate conventions and offsets

At heading zero, +X is forward and +Y is left. Positive heading is counterclockwise. Poses use a global field frame you establish; the template does not know your starting tile. Distances passed to Pedro/localization are inches, headings stored in Pose are radians, and Pinpoint offset configuration is explicitly centimeters.

The forward-measuring pod's sideways location maps through `POD_Y_DIST` to Pinpoint's `xPodOffset`. The strafe-measuring pod's forward location maps through `POD_X_DIST` to `yPodOffset`. Names identify the measured axis versus the perpendicular offset; verify signs against Pinpoint's hardware documentation and your installation. Convert inches to centimeters with 2.54 if measuring in inches, and write the resulting signed centimeters here.

Start with the robot aligned to a known forward direction. Move it forward a measured distance, left a measured distance, and rotate in place. Verify axis signs, units and approximate heading. Significant translation while rotating suggests offset/sign problems. Fix localization before adjusting path-control gains; a controller cannot compensate for incorrect position measurements.

## How the starter drives

StarterTeleOp constructs PedroDrive at pose zero, binds controls, and sets a default command. The drive command reads raw gamepad suppliers every cycle. Stick deadband eliminates small centered-stick noise. Up, left and counterclockwise stick requests are converted to Pedro's positive axes. Start with max scale 0.4, or hold LB for 0.2.

PedroDrive converts field-centric inputs by rotating translation through **negative robot heading**. For example, at a 90-degree robot heading, a +field-X request becomes robot-right. Rotation remains a relative turn input. In robot-centric mode, translation is already in robot coordinates and no heading rotation is applied. Mecanum normalizes combined wheel requests internally.

X toggles field-centric mode. D-pad UP sets current pose to zero: it changes your coordinate interpretation, not the robot's physical location. Field-centric mode needs a correct heading origin; do not reset heading arbitrarily during a match and expect the previous field frame to remain valid. The generic starter intentionally does not carry pose across OpModes; add explicit handoff later if the team needs it.

PedroDrive `periodic()` is the one follower update per scheduler cycle. Commands set requests; periodic applies the follower's mode and localizer updates. Do not call `follower.update()` a second time from the OpMode. `stop()` sets idle mode and immediately zeroes the drivetrain. The default drive command invokes this cleanup if interrupted.

## First-drive verification

1. INIT should resolve all five configured device names. Confirm Pinpoint is ready/calibrated according to its status and documentation.
2. START with centered sticks: no movement. Check STOP at once.
3. Test forward, left, then counterclockwise individually at reduced power. Fix wiring/directions before combining sticks.
4. Check localization distances and heading against physical motion.
5. Only then enable field-centric control and test at two different headings.

Exercise: copy StarterTeleOp and add a speed mode using a second bumper. Keep drive power zero at centered sticks and preserve the stop path. Avoid adding season-specific aiming to the generic drive subsystem; implement it as an optional command that requires the drivetrain.
