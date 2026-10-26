# Changelog

## 2026-10-20 - Remove automated testing scaffolding

- Removed automated test sources and test-only dependencies, source settings, Android stub configuration and compiler overrides from the active and archived builds.
- Removed testing lessons and instructions from the current guide and READMEs; chapter 09 now covers troubleshooting. Physical robot checks remain.
- Retained runtime constructors and behavior, and preserved earlier changelog and upgrade entries as historical records.

## 2026-10-20 - Complete template programming guide

- Completed chapters 08-10: Panels configuration and Sloth deployment, the manual tuning utility, desktop testing/troubleshooting, and copying/adapting examples through a team workflow.
- Replaced the stale empty-module TeamCode readme with the current root-application layout and guide links; pointed the tuning utility to its current guide.
- Added the missing pre-template archive index and documented its exclusion from compiled source sets. Preserved archived sources and existing working-tree changes.
- Reviewed the seven disabled examples and enabled starter against the guide; no runtime behavior or public interfaces changed in this completion pass.
- Validation: `assembleDebug testDebugUnitTest lintDebug` passed (16 tests; lint reported zero errors and 15 warnings). Warnings concern dependency-version advice, dependency native-library alignment and unused resources. No controller installation or physical robot validation was performed.

## 2026-10-20 - Adopt Pedro PoseFactory

- Added shared immutable `FIELD_POSES = PoseFactory.degrees()` and `RADIAN_POSES = PoseFactory.radians()` factories in `pedroPathing/Constants.java`.
- Replaced direct `new Pose(...)` calls in active team code and test fixtures with factory `of(x, y, heading)` calls. Previously omitted headings are explicitly zero.
- Authored autonomous field poses now express headings directly in degrees, such as `90`, `114`, and `180`. Path interpolation uses Pedro's pose-based `linear(...)` and `constant(...)` overloads.
- Runtime tuning geometry and test fixtures use the radians factory. Pedro still stores pose headings in radians; trigonometry, calculated bearings, `withHeading(...)`, and runtime turn angles remain in radians. `Pose.zero()` remains unchanged.
- Preserved coordinates, control points, path headings, and the autonomous initial heading of zero. The existing mismatch with the first path's 90-degree heading remains unresolved.
- Left calibration values, control bindings, dependencies, and the archived Pedro 2 reference suite unchanged by this refactor.
- Validation: `assembleDebug testDebugUnitTest lintDebug` passed, including all 11 existing regression tests. Existing numeric path assertions remain independent of the production factories. Confirmed no direct `new Pose(...)` calls remain in active sources/tests and the diff whitespace check passes.

## 2026-10-20 - Dependency, build, and Pedro 3 migration

This entry describes the upgrade from the repository's pre-upgrade Git HEAD to the current working tree. It is a record of implemented changes, not a claim that the robot has been calibrated or tested on hardware. No release tag or device deployment was created.

### Dependency changes

The previous root build and module build described different dependency sets. Values below distinguish declarations in the unused module script from dependencies actually selected by the root build.

| Component | Before | After | Reason or compatibility note |
|---|---|---|---|
| Dairy FTC plugin | `12.0.0-1.2.2` | `12.0.0-1.2.3` | Upgrade to the verified stable plugin release. |
| FTC SDK | Root used `sdk.TeamCode()` and the plugin's SDK 12 default | Explicit `sdk.TeamCode("12.0.0")` | Makes the existing SDK generation explicit; this is not an SDK 11-to-12 upgrade. |
| Sloth loader | `0.3.2` | `0.3.2` | Already current; applied once in the root build. |
| Sloth runtime | `0.3.2` through Dairy and the old module declaration | `0.3.2` through Dairy | Consolidates the declaration without changing the runtime version. |
| FullPanels | Root selected regular `com.bylazar:fullpanels:1.0.12`; old module separately declared the Sloth variant | `com.bylazar.sloth:fullpanels:0.3.2+1.0.13` | Selects one Sloth-compatible Panels distribution. The old module already named this version but was not included by settings. |
| Pedro hardware integration | Old module declared `com.pedropathing:ftc:3.0.1`; robot source used Pedro 2 APIs | `com.pedropathing:revhub:3.0.2` | Uses the published Pedro 3 hardware artifact. Transitive Pedro core resolves to `3.0.2`. |
| Pedro telemetry/tuning | Old module declared `com.pedropathing:telemetry:1.0.0` | `com.pedropathing:tuning:1.0.2` | Replaces the old tuning integration; these artifacts do not provide identical features. The newer tuning beta was excluded. |
| SolversLib core | Old module declared `0.3.5` | `0.3.6` | Uses the current stable release. |
| SolversLib Pedro integration | Old module declared `0.3.5` | `0.3.6` | Uses commands compatible with Pedro 3. Its older transitive revhub request resolves to the explicit `3.0.2`. |
| JUnit | No explicit project test dependency | `4.13.2` | Added for regression tests. |
| Mockito | No explicit project test dependency | `5.24.0` | Added for desktop mocks; not packaged into the robot app. |
| Android lint | AGP's bundled lint `8.13.2` | Standalone lint `9.4.1` | Removes Kotlin 2.4 metadata incompatibility diagnostics during analysis. |
| Gradle wrapper | `9.1.0` | `9.1.0` | Retained to match Dairy's supported setup; wrapper files were not changed. |
| Foojay toolchain resolver | `1.0.0` | `1.0.0` | Retained. |

Android Gradle plugin `8.13.2` remains managed by Dairy. AndroidX and vision libraries retain the SDK/Dairy dependency constraints. This upgrade does not force every transitive dependency to its independently newest version. Stable releases were preferred; lint's suggestion to use `9.5.0-alpha10` was not adopted.

### Gradle and project structure

#### Root build.gradle.kts

- Updated the Dairy plugin and made SDK `12.0.0` explicit.
- Removed the `ftc.kotlin()` call because the team sources are Java. Kotlin runtime dependencies required by libraries remain in the dependency graph.
- Replaced the regular FullPanels DSL declaration with the explicit Sloth-compatible artifact.
- Declared Pedro, SolversLib, and test dependencies in the root application build, where they are now used.
- Added explicit Google, Maven Central, and Dairy dependency repositories.
- Connected the existing team files to the root application's source sets:

  | Source set | Location |
  |---|---|
  | Main Java | `TeamCode/src/main/java` |
  | Main resources | `TeamCode/src/main/res` |
  | Main manifest | `TeamCode/src/main/AndroidManifest.xml` |
  | Unit tests | `TeamCode/src/test/java` |

- Enabled default return values for Android stubs in local unit tests. These tests do not emulate Android hardware behavior.
- Configured unit-test Java compilation to use a Java 25 compiler in a forked process after Android task configuration. Dairy's Java 8 production compilation remains in place, and its test runtime uses Java 25.

#### settings.gradle.kts

- Removed commented-out legacy plugin-management and module-inclusion blocks.
- Kept the active plugin repositories and Foojay resolver.
- Explicitly set `rootProject.name = "FTC-25262-Template"`.
- Continued using a single root application; neither legacy module is included as a Gradle subproject.

#### gradle.properties and removed build scripts

- Added `android.experimental.lint.version=9.4.1`. This upgrades lint independently of the Android Gradle plugin.
- Preserved AndroidX, disabled Jetifier, heap size, resource-class settings, build caching, and configuration caching.
- Removed `TeamCode/build.gradle`, including its duplicate Sloth plugin application, separate loader buildscript, local controller dependency, and references to missing `build.common.gradle` and `build.dependencies.gradle`.
- Removed `FtcRobotController/build.gradle`. Controller samples remain in the repository, but the running controller application is supplied by Dairy's packaged FTC SDK dependency.
- Kept the existing `local.properties` private and unchanged. Its unusual JDK-named directory was verified to contain an Android SDK, platforms, and build tools.

Build from the repository root with a Java 25 Gradle JVM. Production bytecode remains Java 8; Java 25 is a build/test requirement, not a new robot runtime requirement.

### Constants.java: configuration and API changes

Source: [Constants.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java).

#### Follower construction and public API

| Previous API | Current implementation | Effect |
|---|---|---|
| `FollowerConstants` and public `followerConstants` | `ForesightConfig` from new public `createForesightConfig()` | Makes the Pedro 3 controller and motion settings explicit. |
| `MecanumConstants` and public `driveConstants` | A fresh local `MecanumConfig` inside `createFollower()` | Uses the new revhub drivetrain interface. |
| `PinpointConstants` and public `localizerConstants` | A fresh local `PinpointConfig` and `PinpointLocalizer` | Actually attaches the previously unused localizer configuration. |
| `PathConstraints` and public `pathConstraints` | Foresight end-condition fields | Translates the endpoint threshold and timeout; the former four-argument object is not retained. |
| `FollowerBuilder(...).mecanumDrivetrain(...).build()` | `new Follower(localizer, drivetrain, algorithm)` | Explicitly wires Pinpoint, Mecanum, and Foresight together. |

`createFollower(HardwareMap)` retains its name, parameter, and `Follower` return type. Callers of the removed public configuration objects must migrate to the new fields or factory. Every factory call constructs fresh configurations and controller instances.

Imports moved from `com.pedropathing.ftc` to `com.pedropathing.revhub`, with additional controller, algorithm, matrix, and vector imports. The class remains annotated `@Configurable`.

#### Preserved fields and hardware settings

| Setting | Current value | Change and meaning |
|---|---|---|
| `ROBOT_MASS` | `8` | Preserved as an intended kilogram measurement/reference. It no longer feeds the path algorithm, because the old `.mass(ROBOT_MASS)` configuration was removed. |
| `POD_X_DIST` | `-9 * 2.52` (`-22.68`) | Preserved numerically; consumed as centimeters. The conversion factor remains unverified. |
| `POD_Y_DIST` | `8.4` | Preserved numerically; consumed as centimeters. |
| `FRONT_RIGHT_NAME` | `"frontLeft"` | New configurable field exposing the pre-existing right-front mapping. The duplicate mapping has not been corrected. |
| Other motor names | `frontLeft`, `backLeft`, `backRight` | Preserved. |
| Left motor directions | `REVERSE` | Preserved for both left motors. |
| Right motor directions | `FORWARD` | Preserved for both right motors. |
| Pinpoint hardware name | `odo` | Preserved. |
| Pod type | `goBILDA_4_BAR_POD` | Preserved. |
| Both Pinpoint encoder directions | `REVERSED` | Preserved. |

The right-front and left-front names currently address the same hardware device. Direction assignments can therefore overwrite each other. This remains an unresolved hardware configuration issue, not a fix delivered by the migration.

#### Added motion and controller fields

These values are initial settings, not measurements of this robot. Distances used by Foresight are inches and headings are radians.

| Field | Initial value | Purpose / units | Origin |
|---|---|---|---|
| `FORWARD_VELOCITY` | `81.34056` | Maximum achievable forward speed, inches/second | Carried from the old Pedro drivetrain default. |
| `STRAFE_VELOCITY` | `65.43028` | Maximum achievable lateral speed, inches/second | Carried from the old Pedro drivetrain default. |
| `FORWARD_DECELERATION` | `34.62719` | Positive natural forward deceleration magnitude, inches/second squared | Magnitude of the old negative zero-power acceleration default. |
| `STRAFE_DECELERATION` | `78.15554` | Positive natural lateral deceleration magnitude, inches/second squared | Magnitude of the old negative zero-power acceleration default. |
| `TRANSLATIONAL_P` | `0.1` | Proportional translational correction gain | Carried from the old follower default. |
| `TRANSLATIONAL_STATIC` | `0.015` | Static correction power in the error direction | Carried from the old follower default. |
| `HEADING_P` | `1.0` | Proportional heading correction gain | Carried from the old follower default. |
| `HEADING_STATIC` | `0.01` | Static heading correction power | Carried from the old follower default. |
| `FORWARD_LINEAR_BRAKE` | `0` | Forward displacement coefficient multiplying forward velocity; seconds | New zero initial estimate. |
| `STRAFE_LINEAR_BRAKE` | `0` | Lateral displacement coefficient multiplying lateral velocity; seconds | New zero initial estimate. |
| `FORWARD_QUADRATIC_BRAKE` | `1 / (2 * 34.62719)` | Forward coefficient multiplying velocity times its magnitude; seconds squared/inch | New constant-deceleration estimate based on `d = v² / (2a)`. |
| `STRAFE_QUADRATIC_BRAKE` | `1 / (2 * 78.15554)` | Lateral coefficient multiplying velocity times its magnitude; seconds squared/inch | New constant-deceleration estimate based on `d = v² / (2a)`. |
| `HEADING_LINEAR_BRAKE` | `0` | Angular displacement coefficient multiplying angular velocity; seconds | New zero initial estimate. |
| `HEADING_QUADRATIC_BRAKE` | `0` | Angular coefficient multiplying angular velocity times its magnitude; seconds squared/radian | New zero initial estimate. |

The forward and strafe quadratic initializers use fixed numeric literals. Changing `FORWARD_DECELERATION` or `STRAFE_DECELERATION` does **not** recalculate those brake fields, either during tuning or when constructing another follower. Measure and update the braking coefficients separately.

#### Controller behavior and end conditions

- Heading correction uses a proportional PID controller with zero integral and derivative terms, plus a separate static feedforward controller.
- Forward and strafe corrections each combine their own proportional PID controller with static feedforward, using the shared translational gain fields.
- Both coast and brake controllers initially use proportional velocity feedforward with coefficient `1 / FORWARD_VELOCITY`. The strafe maximum speed is configured separately; it does not provide a separate feedforward gain here.
- The new feedforward controllers are estimates, not exact translations of Pedro 2's drive PIDF, derivative filtering, or braking behavior.
- Forward/lateral brake coefficients are placed in diagonal matrices. Heading brake coefficients are placed in a two-component vector.
- The old path endpoint threshold `0.99` is represented by `parametricTConstraint = 0.01`; Pedro 3 checks `t >= 1 - constraint`. This field also participates in start-of-path correction behavior.
- The endpoint timeout remains `100` milliseconds. This is an algorithm endpoint timeout, not a timeout on the complete autonomous routine or every turn command.
- The explicit old `.maxPower(1)` setting and the remaining legacy `PathConstraints` constructor arguments have no direct retained configuration calls. They should not be assumed to map one-for-one to the new algorithm's defaults.

Configuration fields are read when the factories build the follower. Editing a static field through Panels does not rebuild an already constructed configuration or controller; reinitialize the OpMode to apply those changes.

#### Pinpoint localization and units

- `forwardPodY(POD_Y_DIST)` maps to `xPodOffset = POD_Y_DIST`.
- `strafePodX(POD_X_DIST)` maps to `yPodOffset = POD_X_DIST`.
- Offset units remain explicitly centimeters; field/localization distance units are explicitly inches.
- The pod offsets, resolution, and direction settings now reach the actual Pinpoint localizer. Previously the configuration object existed but `createFollower()` did not attach it.
- This is a functional localization change. Verify physical offsets, signs, encoder directions, and reported distances before relying on paths or goal lock.

### Robot behavior and command changes

- Migrated poses to `com.pedropathing.math.Pose` and new accessors such as `x()`, `y()`, and `heading()`. Migrated follower pose access and initialization to `pose()` and `setPose()`.
- Rebuilt autonomous paths with Pedro 3 `Path` and `Paths.line/curve/path` APIs. Existing control points, segment order, and heading interpolation were preserved.
- Added `Stats.startingPose()`: saved pose takes precedence, while no saved pose yields `Pose.zero()`. This prevents a null starting pose but does not establish the robot's physical field origin.
- Added finite `stopCommand()` factories to intake, conveyor, and shooter. `stopShot()` and `stopLoad()` now use these factories so sequential routines can advance. Continuous `defaultStopCommand()` factories remain available for idle subsystem behavior.
- Added shared `MotorCommands.run(...)` lifecycle handling. Commands avoid applying an action while their finish/fault condition is true and stop outputs when they end normally or are canceled.
- Changed motor stop methods to `stopMotor()`, directly zeroing motor power instead of asking a velocity-feedback controller for zero speed.
- Added `RobotCommandOpMode`, which cancels commands before resetting scheduler state and on OpMode end. Team command OpModes now use this base; primary TeleOps and autonomous also explicitly stop the drivetrain.
- Added `RobotCommands.load(DoubleSupplier)`. It reads conveyor power on each execution, spins the intake, checks both devices for over-current, and stops both on termination. Both primary TeleOps use this overload; Blue previously created an unused command inside a loop.
- Changed target aiming to compute the bearing when the turn command initializes, rather than when the autonomous sequence is assembled.
- Moved path 3's load trigger into the parent command group. It starts at first-segment parametric completion `>= 0.75`, or once that segment has advanced, without scheduling a separate command that competes with its parent for subsystem requirements.
- Initialized Pedro manual mode with zero powers. Field-centric translation is now rotated into robot coordinates explicitly; reverse adds a half-turn to that transformation.
- Changed goal-lock turn sign for Pedro 3's counterclockwise convention, reset its PID on toggling, and clamp the final turn request to `[-1, 1]`.
- Updated drivetrain telemetry to show the new velocity API and follower mode instead of the removed heading-error accessor.

The existing alliance goal positions, primary gamepad assignments, operator A load/unload conflict, motor names, mechanism velocity targets, shooter PIDF values, autonomous delays, and initial autonomous heading were not intentionally retuned or reassigned.

### Tuning tools and documentation

- Replaced the copied Pedro 2 tuning implementation, which depends on removed APIs, with a Pedro 3-compatible `Tuning` OpMode.
- Added selectable localization/offset inspection, forward/lateral/turn characterization, coast measurements, line tests, triangle/circle tests, heading hold, and a predictive-braking line test.
- Characterization records peak body velocities and coast distance/angle/time, and displays a constant-deceleration estimate. Manual characterization uses float mode. Test power starts at `0.3`; its peak velocity is not a full-power speed measurement.
- Added a robot-configuration checklist through the stable Pedro tuning framework.
- Preserved the original suite in [doc/pedro2-tuning.java.txt](doc/pedro2-tuning.java.txt), outside compiled sources. The replacement does not reproduce its automatic coefficient fitting or old field drawing UI. Path tests alone do not establish equivalent tuning.
- Added [the upgrade review](doc/UPGRADE_REVIEW.md) with setup instructions, issue priorities, and hardware acceptance checks. Added a team-specific introduction to the README above the inherited SDK documentation.

### Validation recorded for the upgrade

These results come from the completed implementation checks; writing this changelog does not constitute another build or hardware test.

| Check | Recorded result |
|---|---|
| `./gradlew.bat assembleDebug testDebugUnitTest lintDebug` | Successful combined build. |
| Command lifecycle regression tests | 6 passed; zero failures/errors. Covers finite versus default stops, cancellation, faults, sequence continuation, dynamic load power, and cancellation before scheduler reset. |
| Navigation regression tests | 5 passed; zero failures/errors. Covers missing/saved poses, deferred aiming, autonomous geometry, required Foresight configuration, and tuning path closure/headings. |
| Production bytecode inspection | Class-file major version `52`, corresponding to Java 8. |
| APK inspection | Contains `BlueBasketAuto`, `BluePrimaryTeleOp`, `RedPrimaryTeleOp`, `ExampleTeleOp`, and `Tuning` descriptors. This verifies packaging, not Driver Station discovery. |
| Runtime dependency inspection | Pedro core/revhub `3.0.2`, SolversLib `0.3.6`, and one Sloth-compatible Panels distribution. No old Pedro `ftc`/`telemetry` artifacts. |
| Android lint `9.4.1` | Zero errors, 15 warnings; no Kotlin metadata incompatibility diagnostics. |
| Diff whitespace check | Passed after implementation. |

Lint warnings comprise 12 repeated native-library 16 KB alignment findings in SDK vision dependencies, two unused resources (`raw/readme.md` and `xml/teamwebcamcalibrations.xml`), and one suggestion to use an alpha lint version. Gradle also reports deprecated features relevant to a future Gradle 10 migration; the Java 25 test compiler warns about Java 8 source/target settings.

Build output: `build/outputs/apk/debug/FTC-25262-Template-debug.apk`. Detailed logs, test reports, and lint reports are generated under ignored `build/` directories and are not release artifacts committed by this entry.

### Unresolved issues and required follow-up

| Priority | Issue | Required action |
|---|---|---|
| Critical | Both front drive motors map to `frontLeft`. | Confirm the robot configuration and assign a unique right-front name, likely `frontRight`. |
| High | Operator A requests both load and unload. | Choose separate intended bindings; both were deliberately retained. |
| High | Motion, controller, and braking values are estimates. | Characterize the robot and retune Pedro 3 before autonomous use. |
| High | Pod offsets and `2.52` conversion are unverified. | Measure offsets and confirm units/signs; an inches-to-centimeters conversion would use `2.54`. |
| High | Conveyor preparation has an absolute encoder threshold and no timeout. | Establish homing/relative positioning and an appropriate timeout. |
| High | Shooter motor model, velocity controller ownership, and feed readiness are unverified. | Confirm hardware, tune the chosen controller, and define readiness. SDK PIDF does not tune SolversLib's software controller. |
| High | Current-alert thresholds are not set explicitly; held commands may restart after faults clear. | Select measured thresholds and decide whether fault recovery should latch. |
| Medium | Initial autonomous heading is zero; the first path begins at 90 degrees. | Confirm physical orientation and align the intended settings. |
| Medium | Three shot windows total at least 10.8 seconds, plus paths, preparation, turns, and a 7-second park delay. Turns have no timeout. | Measure total routine time and choose timeout behavior. |
| Medium | Saved pose, axis signs, field-centric/reverse control, and speed restoration need physical verification. | Test alliance transitions, known-origin setup, and both gamepads on the robot. |
| Low | Example TeleOp uses `intakeMotor`; primary OpModes use `intake`. | Confirm the example's hardware configuration. |

The Driver Station must match the Robot Controller SDK generation. Live Sloth loading, Panels behavior, OpMode discovery, mechanisms, localization, and complete autonomous operation still require device testing. See [the upgrade review](doc/UPGRADE_REVIEW.md) for the full robot checklist.
