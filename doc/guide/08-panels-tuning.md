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
| Constants' follower/controller configuration | When a follower is constructed |
| Tuning `TEST_POWER` | Each active loop |
| Tuning `TEST_DISTANCE`, `TURN_ANGLE` | When a new test path or heading target is requested |

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

## Use the measurement utility

[Tuning.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Tuning.java) is disabled. Enable it intentionally only after configuring all four motors, Pinpoint, pods and measured offsets. It is a collection of manual tests and a configuration checklist, not a complete automatic tuning procedure. Browser telemetry does not calibrate the robot for you.

Select a mode with D-pad left/right **during INIT**. After START:

| Mode | Input and behavior |
|---|---|
| Localization / offsets | Sticks request forward, left and turn, scaled by `TEST_POWER` |
| Forward, lateral or turn velocity / coast | Hold A for the selected motion; release A to request zero power with manual float behavior and record coast motion |
| Line / translational / drive | Press A to follow a line of `TEST_DISTANCE` inches along field +X |
| Triangle | Press A to follow a closed triangle from the current pose |
| Circle / centripetal | Press A to follow four cubic curves approximating a closed circle |
| Heading hold | Press A to request current heading plus `TURN_ANGLE` radians |
| Predictive braking | Press A to run the same line geometry; compare measured stopping behavior while adjusting the braking configuration |

B stops the follower and drivetrain, ends coast recording and resets recorded maximum velocities on its press edge. STOP also stops the drivetrain. A line or curve continues after releasing A; release-to-coast applies only to the velocity/coast modes. Repeated A presses in path modes create a new test from the current pose. Path offsets use field axes, so a line is not necessarily forward relative to the robot's current heading.

Distances are inches, linear velocities inches/second, headings radians and angular velocities radians/second. `TEST_POWER` defaults to 0.3, `TEST_DISTANCE` to 24 inches and `TURN_ANGLE` to PI/2 radians. Inspect the full path and clear space before starting; the circle extends one test distance in field +X from its start. Keep test power and distances small initially.

The utility records release velocity, displacement and elapsed time. Its displayed `v^2/2d` is a rough constant-deceleration estimate from linear speed and straight-line displacement; it is not an angular deceleration estimate or a fitted braking model. Curved motion, noisy localization or short stopping distances can make it misleading. Recorded maxima accumulate until B; old coast results can remain visible after a later test. The utility uses float for manual coast measurements, while the starter uses its configured manual brake behavior.

First verify localization by measured forward, lateral and rotational movement. Then characterize velocity and coast, enter measured values in Constants, reinitialize, and evaluate short lines, heading changes and curves. Change one parameter at a time and record battery condition, surface, load, requested motion and actual endpoint. Chapter [07](07-autonomous.md#foresight-fields-to-measure-and-understand) explains the configuration fields and units.

Exercise: demonstrate Example07's live/snapshot difference, then explain which configuration changes require reinitializing your drivetrain. On a configured robot, record three repeatable short tests before changing any gain.
