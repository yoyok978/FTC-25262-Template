[עברית](README.he.md)

# Team 25262 FTC template and programming guide

Build your team's robot code with Java, the FTC SDK, SolversLib commands, Pedro 3 path following, Panels, and Sloth. This repository is both a working starting point and a sequence of small examples you can copy and adapt.

**The enabled Starter TeleOp REQUIRES four mecanum drive motors, a goBILDA Pinpoint, and compatible odometry pods.** Configure and measure your robot before driving. Motor/servo lessons need only their named devices; lessons 01 and 07 need no robot peripherals.

## Start here

1. Read [setup and first run](doc/guide/01-setup.md). Set the IDE's Gradle JVM to **Java 25**, install an Android SDK, and sync the root project.
2. Match your Robot Controller configuration to [the hardware table](#required-hardware). Edit [Constants.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java): names, directions, pod type, and signed pod offsets.
3. Let the initial Gradle sync finish. Sync again after dependency/build configuration changes or when the IDE prompts you; ordinary Java edits usually do not need it. Production uses Java 8 bytecode even though Gradle uses Java 25.
4. Connect and select the controller in Android Studio/IntelliJ, select the project's **Android App** run configuration, and click **Run ▶** to [build and install the app](doc/guide/01-setup.md#install-and-run). A separate Build action is usually unnecessary. Keep the Driver Station compatible with FTC SDK 12.0.
5. Verify directions and odometry, then choose **Starter TeleOp** on the Driver Station. Begin with robot-centric movement at low speed. Autonomous requires additional measurement and tuning.

For a first lesson without motors, copy and enable [01 Lifecycle](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example01Lifecycle.java). Every teaching example is disabled until you intentionally enable it.

## Required hardware

| Use | Robot configuration name | Device |
|---|---|---|
| Starter / drive examples / tuning | `frontLeft` | Front-left mecanum DC motor |
| Same | `frontRight` | Front-right mecanum DC motor |
| Same | `backLeft` | Back-left mecanum DC motor |
| Same | `backRight` | Back-right mecanum DC motor |
| Same | `odo` | goBILDA Pinpoint I2C device, connected to compatible odometry pods |
| Motor and command-group lessons only | `exampleMotor` | A separate DC motor |
| Servo and command-group lessons only | `exampleServo` | A positional servo |

Also needed: an FTC controller/hub, battery, Driver Station and gamepad. All names must match exactly, including case. You may change the example names in code to match your configuration. There is no wheel-encoder fallback when Pinpoint is missing.

Pod offsets start at **zero placeholders**, and motion/braking settings are **unmeasured Pedro 3 library defaults**. See [driving and localization](doc/guide/06-driving.md) and [autonomous](doc/guide/07-autonomous.md) before relying on localization or paths. Follow [Pedro 3 AutoTune](doc/guide/08-panels-tuning.md#pedro-3-autotune) on port `10158` and save its generated configurations in Constants.

## What the project provides

| Feature | Role |
|---|---|
| FTC SDK 12.0 | OpModes, gamepads, hardwareMap, motors, servos, Driver Station connection |
| Dairy plugin `12.0.0-1.2.3` | Root Android build and compatible SDK/toolchain setup |
| SolversLib `0.3.6` | Subsystems, command scheduler, button bindings, command composition |
| Pedro core/revhub `3.0.2` | Mecanum hardware integration, Pinpoint localization and Foresight path following |
| Pedro tuning `1.0.2` | AutoTune procedures for mecanum, Pinpoint, Foresight and validation |
| PoseFactory | Degree-authored field poses and radian-based runtime poses |
| Sloth `0.3.2` | Optional fast team-code deployment after a full app installation |
| Sloth FullPanels `0.3.2+1.0.13` | Browser telemetry/configuration compatible with Sloth |

Do not add regular FullPanels alongside the Sloth variant. Open the root project in the IDE: TeamCode is a source directory in this Dairy application, not a separate Gradle subproject.

## Learning path

| Chapter | What you will learn |
|---|---|
| [01 Setup](doc/guide/01-setup.md) | Tools, build, device installation, first run |
| [02 Project structure](doc/guide/02-structure.md) | Where code belongs and how Gradle packages it |
| [03 Java and FTC foundations](doc/guide/03-foundations.md) | Classes, lambdas, suppliers, lifecycle, inputs and telemetry |
| [04 Commands](doc/guide/04-commands.md) | Defaults, ownership, groups, cancellation and timeouts |
| [05 Hardware](doc/guide/05-hardware.md) | Copying motor/servo subsystems and adding mechanisms |
| [06 Driving](doc/guide/06-driving.md) | Mecanum, Pinpoint, frames, names, offsets and controls |
| [07 Autonomous](doc/guide/07-autonomous.md) | PoseFactory, paths, heading interpolation and command integration |
| [08 Panels, Sloth and tuning](doc/guide/08-panels-tuning.md) | Browser connection, live versus snapshot settings and deployment |
| [09 Troubleshooting](doc/guide/09-troubleshooting.md) | How to diagnose common setup, deployment and robot problems |
| [10 Team workflow](doc/guide/10-workflow.md) | Copying examples, adding features, reviewing and measuring changes |

## Copyable examples

| Lesson | Source | Needs |
|---|---|---|
| 01 | [Lifecycle and telemetry](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example01Lifecycle.java) | No peripherals |
| 02 | [Motor subsystem](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example02Motor.java) | `exampleMotor` |
| 03 | [Servo subsystem](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example03Servo.java) | `exampleServo` |
| 04 | [Command groups](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example04CommandGroups.java) | Both example devices |
| 05 | [Mecanum and Pinpoint](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example05Mecanum.java) | Complete drive hardware |
| 06 | [Line and curve autonomous](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example06Autonomous.java) | Complete drive hardware, tuned follower |
| 07 | [Panels configuration](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example07Panels.java) | No peripherals |

Copy a lesson into the `opmodes` package, change the package declaration, rename its file/class and displayed OpMode name, then remove `@Disabled`. Configure only the devices that lesson needs. Follow [the full copying procedure](doc/guide/10-workflow.md#copy-an-example).

## Starter controls

All controls use **gamepad 1**. Left stick translates, right stick X turns, left bumper reduces maximum power from `0.4` to `0.2`, X toggles field-centric control, and D-pad UP declares the current location/heading to be pose zero. Resetting pose does not move the robot. Start robot-centric; field-centric behavior depends on a correctly established heading. Centered sticks within the deadband request zero motion.

## History and references

The previous season-specific robot code, including its settings, is preserved in [the pre-template archive](doc/archive/pre-template/INDEX.md). It is **not compiled or registered as OpModes**. It contains known robot-specific problems and is not the current starter.

[CHANGELOG.md](CHANGELOG.md) records development history. [The upgrade review](doc/UPGRADE_REVIEW.md) is a historical snapshot, not the current configuration guide. The `FtcRobotController` tree contains upstream samples and references; Dairy supplies the controller application dependency.

Retain [LICENSE](LICENSE) and applicable third-party notices when reusing the template. Useful upstream documentation: [FTC](https://ftc-docs.firstinspires.org/), [Pedro](https://pedropathing.com/), [SolversLib](https://github.com/FTC-23511/SolversLib), [Panels](https://panels.bylazar.com/), and [Sloth](https://github.com/Dairy-Foundation/Sloth).
