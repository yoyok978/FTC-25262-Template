# 07 - Pedro autonomous and PoseFactory

[עברית](he/07-autonomous.md)

[Guide index](../../README.md#learning-path) | Example: [06 Autonomous](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example06Autonomous.java) | Next: [Panels and tuning](08-panels-tuning.md)

## Prerequisites and construction

Autonomous uses the same four-motor mecanum drivetrain, Pinpoint and pods as TeleOp. It additionally needs measured localization and tuned motion/controller/braking settings. The sample is disabled because a successful build cannot establish those measurements.

Constants creates three cooperating objects: `Mecanum` converts requested drive powers into four motors, `PinpointLocalizer` supplies pose/velocity, and `Foresight` computes path corrections. `new Follower(localizer, drivetrain, algorithm)` connects them. Each initialization creates new controllers/configuration snapshots. PedroDrive wraps that follower as a scheduler-owned subsystem.

## Poses, units and curves

Use the shared factories:

```java
Pose start = Constants.FIELD_POSES.of(0, 0, 0);   // Heading supplied in degrees.
Pose end = Constants.FIELD_POSES.of(12, 0, 90);   // Stored heading is PI/2 radians.
Pose measured = Constants.RADIAN_POSES.of(x, y, headingRadians);
```

The resulting object is still Pedro's Pose; factories only define the input convention and optional transformations. Keep trigonometry, `withHeading(...)`, follower telemetry, and calculated bearings in radians. If importing `FIELD_POSES` statically, the example can write `FIELD_POSES.of(...)` directly.

`Paths.line(start, end)` creates a straight spatial path. `.linear(start, end)` interpolates heading between their stored headings; line creation alone does not establish the heading policy. `Paths.curve(...)` creates a Bezier curve: first/last poses are endpoints, middle poses are control points that bend the curve. Intermediate points generally are not locations the robot must pass through. `Paths.path(first, second)` joins segments. `.constant(pose)` supplies constant heading; `.tangent()` would follow the curve's tangent instead.

Example06 travels from (0,0,0 degrees) to (12,0,90 degrees), waits 500 ms, and follows a return curve with controls (12,12) and (0,12), ending at (0,0,0 degrees). Its initial pose matches its first path's heading. Use a physically meaningful known start before copying it to a field routine.

## Commands and progress

The example schedules a SequentialCommandGroup during initialize, but the base does not run the scheduler until START. PedroDrive's periodic method supplies follower updates once per active loop. A `FollowPathCommand` starts a path and waits for follower completion; endpoint holding/timeout are algorithm behavior, not proof of physical accuracy.

A ParallelDeadlineGroup follows the first path while another command updates the stage text. To coordinate a real mechanism, replace that independent stage command with the appropriate subsystem command, and provide/configure that hardware. Do not run two commands requiring the same subsystem in parallel. Add the drive requirement to the parent routine because FollowPathCommand does not know our wrapper. Do not independently schedule a mechanism command from a callback if its parent routine already owns that mechanism.

Use `.withTimeout(milliseconds)` if you need a time bound, but explicitly decide what happens on failure: a timed-out path can otherwise be followed by scoring as if it arrived. The sample's WaitCommand is a non-blocking pause, not a path timeout. STOP cancels the routine and zeros the drivetrain.

## Foresight fields to measure and understand

| Fields | Units / effect | Template status |
|---|---|---|
| `FORWARD_VELOCITY`, `STRAFE_VELOCITY` | Inches/s, attainable forward/lateral speed | Old library default estimates, not robot measurements |
| `FORWARD_DECELERATION`, `STRAFE_DECELERATION` | Positive inches/s squared, natural zero-power slowdown | Old library magnitudes; characterize your robot |
| `TRANSLATIONAL_P`, `TRANSLATIONAL_STATIC` | Position feedback gain and static correction power | Shared starting values for separate forward/strafe controllers |
| `HEADING_P`, `HEADING_STATIC` | Heading feedback gain and static correction power | Starting estimates; integral/derivative are zero |
| `FORWARD_LINEAR_BRAKE`, `STRAFE_LINEAR_BRAKE` | Displacement coefficients multiplying velocity; seconds | Zero initial estimates |
| `FORWARD_QUADRATIC_BRAKE`, `STRAFE_QUADRATIC_BRAKE` | Coefficients multiplying velocity times its magnitude; seconds squared/inch | Initial constant-deceleration estimates |
| `HEADING_LINEAR_BRAKE`, `HEADING_QUADRATIC_BRAKE` | Angular displacement model | Zero initial estimates; heading coefficients require characterization |
| `parametricTConstraint` | Endpoint check uses t >= 1 - 0.01 | Configured in createForesightConfig |
| `timeoutConstraint` | Endpoint timeout of 100 ms | Not an overall autonomous timeout |

Coast and brake initially use proportional velocity feedforward `1 / FORWARD_VELOCITY`; this is not an exact conversion of a previous drive PIDF. Diagonal matrices encode independent forward/lateral braking coefficients; the heading vector holds its linear/quadratic terms. Quadratic initializers use fixed literals, so changing deceleration fields does not recalculate them. Editing fields through Panels does not rebuild existing controllers: reinitialize after changing these configuration snapshots.

## Build your routine incrementally

Verify localization first, then a short straight line, its endpoint and heading, a curve, a second segment, and only then mechanism coordination. Record actual versus requested pose and completion times. A path drawn correctly on paper can still demand more acceleration/clearance than the robot can achieve.

Exercise: change the line endpoint to 18 inches while keeping the starting heading fixed. Next add a third line segment and explain its heading interpolation before deploying. Keep game-specific goal coordinates and alliance mirroring in your team routine rather than the generic constants.
