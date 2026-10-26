# 08 - Panels, Sloth and tuning

[עברית](he/08-panels-tuning.md)

[Guide index](../../README.md#learning-path) | Example: [07 Panels](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example07Panels.java) | Next: [troubleshooting](09-troubleshooting.md)

## Connect the browser

Panels displays telemetry and exposes configurable fields. This project already includes the Sloth-compatible FullPanels dependency; keep that variant when using Sloth. Do not add regular FullPanels alongside it.

1. Install the full app using [chapter 01](01-setup.md#install-and-run) and leave the Robot Controller app running.
2. Connect your computer to the Robot Controller's Wi-Fi network. Find its credentials in Program and Manage.
3. Open `http://192.168.43.1:8001` for a Control Hub, or `http://192.168.49.1:8001` for a phone Robot Controller. These are the standard addresses in the [upstream connection guide](https://ftcontrol.bylazar.com/docs/panels/overview/); use the controller's actual address if your network differs.
4. Open the telemetry and configurables widgets. Copy and enable Example07, then INIT and START it from the Driver Station. It needs no motor, servo or Pinpoint.

If the page does not load, confirm the Wi-Fi connection, controller address, running app and installed dependency before changing Java code. A VPN or network routing policy can also prevent access. An ADB connection alone does not establish browser access to the robot's Wi-Fi address. Driver Station telemetry remains useful when diagnosing browser problems.

## Live fields and snapshots

Example07 uses `JoinedTelemetry` to send the same values to Driver Station and Panels. Its class has `@Configurable` and exposes the public static field `REQUESTED`. After START, change it from 0.2 to 0.3 in the configurables widget and apply the edit. The `Live field` telemetry changes because each loop reads `REQUESTED` again. `Initialization snapshot` stays at the value copied during `initialize()`. STOP, INIT and START again to take a new snapshot.

The same distinction matters elsewhere:

| Setting | When this template reads it |
|---|---|
| Example02 motor `POWER` | Every execution cycle through a supplier |
| Example03 servo positions | When the bound instant command initializes |
| Constants' follower/controller configuration | When components/controllers are constructed; restart after edits |

Editing a field does not rebuild an existing controller or path. STOP and reinitialize after configuration changes that create snapshots. Keep numeric edits finite and within measured hardware limits. Record accepted values in source: changing a runtime field is not a saved source-code edit, and its lifetime across reloads should not be used as configuration storage.

## Deploy team code with Sloth

Sloth can reload team code after a full app installation. This project's loader plugin and runtime are already configured. The [upstream Sloth instructions](https://github.com/Dairy-Foundation/Sloth#gradle-tasks) describe the deployment and remote-code removal tasks; this repository runs them on the root project.

The normal Android app **Run ▶** workflow from chapter 01 remains the simplest starting point. Sloth is optional. To create its IDE button once:

1. Open **Run > Edit Configurations**, click **+**, and choose **Gradle**.
2. Name the configuration **Deploy Sloth**, select the repository's root Gradle project, and enter `deploySloth` in the **Tasks/Run** field. This is a task name in an IDE configuration, not a terminal command. Save with **Apply/OK**.
3. With the robot stopped and the intended controller connected and authorized, select **Deploy Sloth** in the toolbar and click **Run ▶**.

Use this for supported changes within `org.firstinspires.ftc.teamcode` and its subpackages. After loading, reselect/reinitialize the OpMode and check a recognizable telemetry change. For dependencies, resources, manifests or other changes outside reloadable team code, use the Android app's **Run ▶** for a full installation.

Before returning to a full installation after Sloth uploads, create a second Gradle run configuration named **Remove Sloth Remote**, targeting the root project with `removeSlothRemote` in its **Tasks/Run** field. Run it and wait for success, then select the **Android App** configuration and click **Run ▶**. This removes remotely loaded code so an older upload cannot override the installed classes.

These deployment actions modify the connected controller. Upstream screenshots showing a separate TeamCode Gradle module do not match this root application. With multiple devices, ensure the deployment task targets the intended controller; if uncertain, leave only that controller connected. Inspect tasks in the **Gradle** tool window, and see [JetBrains' Gradle run configuration guide](https://www.jetbrains.com/help/idea/run-debug-gradle.html) for the configuration controls. No separate Build action is needed before deployment.

<a id="pedro-3-autotune"></a>
## Pedro 3 AutoTune

[Tuning.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Tuning.java) registers four procedures; it is **not a TeleOp** and has no `@Disabled` to remove. The implementations are adapted from the [official Quickstart](https://github.com/Pedro-Pathing/Quickstart/tree/9dc89787898fab6b5cd9095df38489a14bf51a95/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/procedures). See [source provenance and license](../PEDRO_QUICKSTART_SOURCE.md).

1. Install the full app, leave Robot Controller running and connect to its Wi-Fi.
2. Open `http://192.168.43.1:10158` for Control Hub, or `http://192.168.49.1:10158` for a phone controller. Substitute the actual controller address if different. **Port 10158 is AutoTune; port 8001 is Panels.**
3. Select a procedure in the browser and follow its input/confirmation prompts. When it requests a tuning OpMode, select the procedure's step on Driver Station and use INIT/START/STOP as instructed. Do not select the removed manual "Tuning" TeleOp. Some measurement steps finish themselves; others explicitly ask you to stop after a physical push or rotation.
4. Run the following sequence, saving and deploying each generated configuration before procedures that depend on it.

| Procedure | Prerequisite and result |
|---|---|
| Mecanum Tuner | Enter the four hardware names. Motors spin individually; confirm the correct wheel and its direction. Replace `Constants.drivetrainConfig` with the generated block. |
| Pinpoint Tuner | Enter `odo` (the procedure initially suggests `pinpoint`), select the actual pod type, and perform the requested pushes/rotation. Replace `Constants.localizerConfig` with its block. |
| Tests: Pose / Localization / Driving / Odometry | Verify physical directions, measured distances and heading before algorithm tuning. Pose is a passive localizer check; other tests can drive the robot. |
| Foresight Tuner | Requires the saved drivetrain and verified localizer. Follow velocity, natural-deceleration, heading, braking and translational identification steps. Replace `Constants.foresightConfig` with the complete generated block. |
| Tests: Hold / Line / Curve / Interpolation | Requires all three saved configurations. Check endpoint, heading, repeatability and STOP before using autonomous scoring routines. |

### Save results and verify

Replace the existing declaration, not just its initializer values, and do not add a second field with the same name. Generated blocks use `MecanumConfig`, `PinpointConfig`, `ForesightConfig`, `DcMotorSimple`, `GoBildaPinpointDriver`, `DistanceUnit`, `Controller`, `Matrix` and `Vector2D`; retain the needed imports. Custom Pinpoint pod output also needs `java.util.OptionalDouble`.

The shared factories `createDrivetrain`, `createLocalizer` and `createAlgorithm` feed both AutoTune and `createFollower`; there is no separate tuning robot configuration. Motor and Pinpoint setup procedures collect their own hardware inputs so they can run before algorithm calibration. Save their results first, deploy, then start a fresh dependent procedure.

Both Pinpoint offset and global-distance units are **inches**. When transferring old centimeter offsets, divide by 2.54: old `POD_Y_DIST` maps to `xPodOffset`, old `POD_X_DIST` to `yPodOffset`. Speeds are inches/second, decelerations inches/second squared and runtime headings radians. Keep the unit declarations in the generated block.

The initial Foresight configuration uses **unmeasured Pedro 3 library defaults**. The old Pedro 2 velocity/deceleration numbers and braking approximations are removed; they are not substitute measurements. Foresight output includes separate forward/strafe piecewise controllers and independent coast/brake feedforward fits. Copy the entire block and reinitialize; neither browser results nor Panels edits save Java source automatically.

Read each step's motion description and clear the full travel area before START. Upstream procedures can use substantial power, repeated movement and distances larger than the small teaching examples. Browser input is not a universal power limit. Keep Driver Station STOP available. Record battery, surface, load and actual results; robot measurements and STOP behavior remain to be verified on hardware.

If no procedures appear, confirm the correct app is installed, the address/port and network are correct, and `@Tuner` factories remain public static, zero-argument methods returning exactly `Procedure`. Clear stale Sloth remote code before a full installation as described above.

Exercise: demonstrate Example07's live/snapshot distinction, then explain why Mecanum and Pinpoint results must be saved before Foresight tuning. After tuning, record three repeatable short-path results and identify which configuration produced them.
