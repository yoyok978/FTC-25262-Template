# Dependency upgrade and robot review

Reviewed 2026-10-20. The root `build.gradle.kts` is now the only application build. It compiles `TeamCode/src/main`; `FtcRobotController` remains a sample/reference directory and the controller application comes from Dairy's SDK artifacts.

## Versions and build requirements

| Component | Version | Notes |
|---|---|---|
| Dairy FTC plugin | 12.0.0-1.2.3 | Latest published stable plugin; manages compatible Android tooling |
| FTC SDK | 12.0.0 | Latest official FIRST release; update Driver Station to match |
| Sloth loader / runtime | 0.3.2 | Latest stable |
| Sloth FullPanels | 0.3.2+1.0.13 | One Sloth-compatible distribution, no regular Panels duplicate |
| Pedro core / revhub | 3.0.2 | Replaces the obsolete `ftc` artifact and Pedro 2 APIs |
| Pedro tuning | 1.0.2 | Latest stable; 1.0.3-beta.1 deliberately excluded |
| SolversLib core / pedroPathing | 0.3.6 | Pedro 3 integration |
| Gradle | 9.1.0 | Dairy-supported wrapper |
| Android Gradle plugin | 8.13.2 | Managed by Dairy; not independently forced to AGP 9 |
| Android lint | 9.4.1 | Latest stable standalone lint; understands Kotlin 2.4 metadata |
| JUnit / Mockito (tests only) | 4.13.2 / 5.24.0 | Production retains Java 8 bytecode |

Versions were checked against Dairy and Maven Central metadata and FIRST/Pedro GitHub releases. SDK-owned AndroidX and vision dependencies remain at the versions pinned by the SDK/Dairy; this upgrade does not claim every transitive library is at its independently newest release.

Set the IDE's **Gradle JVM to Java 25**. Production compilation uses Dairy's Java 8 toolchain; tests compile and run on Java 25. Foojay resolves missing toolchains. Keep `local.properties` private and point `sdk.dir` at an installed Android SDK. On this machine the existing path, despite its JDK-like name, contains Android SDK platforms and build tools and was verified usable. No path correction was necessary.

From PowerShell, with `JAVA_HOME` pointing to your Java 25 installation:

```powershell
./gradlew.bat assembleDebug testDebugUnitTest lintDebug
./gradlew.bat dependencies --configuration debugRuntimeClasspath
```

APK: `build/outputs/apk/debug/FTC-25262-Template-debug.apk`.

## Code changes

- Explicitly connected the TeamCode sources, resources, and manifest to the root application. Removed unused module scripts that referenced missing common build files. Preserved controller samples.
- Migrated poses, paths, follower construction and manual drive to Pedro 3. Autonomous control points and heading interpolation are preserved. Attached the previously unused Pinpoint configuration, with centimeter offsets and inch field coordinates.
- Added a non-null starting pose when TeleOp runs without autonomous. A saved pose still takes precedence; this does not establish a physical field origin.
- Stop commands used in sequences are finite; subsystem default stop commands remain continuous. Motor command cancellation/fault termination releases hardware using direct `stopMotor()`, rather than requesting zero velocity through a feedback controller.
- OpModes cancel scheduled commands before scheduler resets and explicitly stop the drivetrain on exit.
- Loading now executes every loop and reads the requested conveyor power dynamically. Both operator A bindings remain unchanged as requested.
- Goal-lock output uses Pedro 3's positive counterclockwise turn convention. Heading error is wrapped and turn power clamped; test physical direction before driving normally.
- Autonomous target aiming reads localization when the turn starts. The collection trigger is owned by the parent command group instead of independently scheduling a conflicting command. It still starts at t >= 0.75 of the first segment of path 3 (or after that segment has completed).

## Problems requiring team action

| Priority | Location | Finding and action |
|---|---|---|
| Critical | `pedroPathing/Constants.java`: `FRONT_RIGHT_NAME` | Both front motors currently resolve to `frontLeft`. The later motor direction assignment can also overwrite the earlier one. The mapping is intentionally retained pending hardware confirmation. Verify the configuration and change the right-front name, likely to `frontRight`, before driving. |
| High | Blue and Red primary TeleOps: operator A | Load and unload commands require the same subsystems and are bound to the same button. They compete/cancel each other. Choose separate controls; this conflict was explicitly left unchanged. |
| High | `pedroPathing/Constants.java`: Foresight configuration | Pedro 3 replaces the old drive PIDF and braking model. Existing code only specified mass and relied on generic Pedro 2 defaults. Speed/deceleration defaults are retained as estimates; quadratic braking uses v^2/(2a), linear/heading braking starts at zero, and drive feedforward is an initial estimate. These are NOT validated robot tuning. Characterize and retune before running autonomous. Mass is retained for reference but no longer configures the algorithm. |
| High | `pedroPathing/Constants.java`: pod offsets | Measure offsets and verify signs and encoder directions. `-9 * 2.52` is suspicious if intended to convert inches to centimeters (2.54). Values have not been guessed or changed. |
| High | `Conv.prepareToShootCommand` | Uses an absolute encoder threshold with no timeout, homing, or relative target. A stalled/disconnected encoder can prevent shooting indefinitely unless a configured current alert trips. Choose a measured positioning strategy and timeout. |
| High | `Shooter` and motor command factories | Verify actual motor type, encoder ticks/revolution and velocity signs. Shooter uses `GoBILDA.BARE`; its raw SDK PIDF settings do not configure SolversLib's separate software velocity controller. Confirm which controller should own velocity and tune it. `isAtTargetVelocity` currently checks only a lower bound, not absolute error, and shooting starts feeding after a fixed 100 ms rather than readiness. |
| High | Intake/conveyor/shooter current checks | No explicit current-alert thresholds are configured in this code. Set thresholds based on actual hardware. A held button can restart a command after a transient fault clears; decide whether faults should latch. |
| Medium | `BlueBasketAuto.initialize` | Initial pose has heading zero, while path 1 starts at 90 degrees. Confirm physical starting orientation before changing either value. |
| Medium | Autonomous timing and turns | Three shots consume at least 10.8 seconds, plus preparation, paths, turns and a 7-second park delay. Time the routine against the match limit. Turn commands have no timeout and can wait indefinitely for heading tolerance. |
| Medium | TeleOp state and inputs | A saved pose persists across OpModes in the same process, including switching alliances. Verify desired reset behavior, gamepad 2 as driver/gamepad 1 as operator, axis signs, and the change from initial speed 0.75 to 1.0 after releasing slow mode. |
| Low | ExampleTeleOp | Uses hardware name `intakeMotor`, while primary OpModes use `intake`. Verify its separate example configuration. |

Paths above are relative to `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

## Tuning migration and robot checks

The old copied Pedro 2 tuning suite relies on removed APIs. It is preserved verbatim in `doc/pedro2-tuning.java.txt` for reference and is not compiled. The replacement `Tuning` OpMode supports localization/offset inspection, forward/lateral/turn velocity and coast measurements, line/translational/drive tests, triangle/circle tests, heading hold and a predictive-braking line test. Select during INIT with D-pad left/right. A starts a path/turn or drives characterization while held; release A to coast; B stops. Characterization uses float mode; tests start from the current pose. `TEST_POWER` defaults to 0.3, so measured peak speed at that setting is NOT the full-power speed constant.

Pedro 3 has no drop-in replacement for the old automatic PID/braking tuners. The new suite provides measurements and path tests; it does not automatically reproduce their coefficient fitting or the old field drawing UI. Record measurements in Constants and reinitialize to apply configuration changes. The stable Pedro tuning framework includes a registered robot-configuration checklist; numerical tuning still requires robot characterization. This is a remaining calibration/tooling limitation, not proof of equivalent autonomous behavior.

Before field use:

1. Confirm four unique drive names and directions, lift the wheels and test each motor, then verify STOP zeros all outputs.
2. Verify Pinpoint forward/strafe/rotation signs, centimeter offsets and inch telemetry against measured movement. Set a known field pose before goal lock/field-centric use.
3. Test manual, field-centric, reverse and goal-lock turns slowly; verify input signs on both alliances.
4. Resolve operator A conflict. Verify load power changes, release, fault cancellation, shooter toggle and OpMode transitions.
5. Measure velocity/deceleration/braking and tune Foresight. Confirm heading controller, path completion and endpoint holding.
6. Test the preserved autonomous geometry, callback timing, aiming, preparation threshold and total duration on the robot.
7. Keep Driver Station/Robot Controller SDK versions matched. OpMode discovery, live Sloth loading, Panels UI, and hardware behavior require device testing.

## Validation

Final command: `./gradlew.bat assembleDebug testDebugUnitTest lintDebug` -- BUILD SUCCESSFUL.

- 11 regression tests passed (6 command lifecycle, 5 navigation), with zero failures/errors.
- Debug APK built. ZIP/DEX inspection confirmed BlueBasketAuto, BluePrimaryTeleOp, RedPrimaryTeleOp, ExampleTeleOp and Tuning are packaged. This verifies packaging, not Driver Station discovery on hardware.
- Production class-file major version is 52 (Java 8).
- Runtime dependency inspection confirmed Pedro core/revhub 3.0.2 and SolversLib 0.3.6, with no old Pedro `ftc`/`telemetry` artifacts or duplicate regular Panels distribution.
- Lint 9.4.1 completed without Kotlin metadata incompatibility diagnostics and reported zero errors, 15 warnings: 12 repeated native 16 KB alignment findings in SDK vision dependencies, two unused resources (`raw/readme.md` and `xml/teamwebcamcalibrations.xml`), and a suggestion to use lint 9.5.0-alpha10. The alpha is intentionally excluded. Alignment matters for future devices requiring 16 KB pages; SDK vision libraries require upstream updates rather than guessed binary replacements.
- Gradle reports upstream deprecated features that will need review before Gradle 10. Java 25 warns about the tests' Java 8 target. Neither prevents this build.
- `git diff --check` passed. Logs and reports are under ignored `build/upgrade-inspection`, `build/test-results`, and `build/reports`.

Passing desktop validation does not validate hardware mapping, live plugin loading, or robot tuning. No device deployment was performed.

Release sources: [Dairy plugin metadata](https://repo.dairy.foundation/releases/dev/frozenmilk/teamcode/dev.frozenmilk.teamcode.gradle.plugin/maven-metadata.xml), [FTC SDK 12.0](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases/tag/v12.0), [Pedro 3.0.2](https://github.com/Pedro-Pathing/PedroPathing/releases/tag/v3.0.2), [SolversLib metadata](https://repo.dairy.foundation/releases/org/solverslib/core/maven-metadata.xml), [Dairy templates](https://github.com/Dairy-Foundation/Templates).
