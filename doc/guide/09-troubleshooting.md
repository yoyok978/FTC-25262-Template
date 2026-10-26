# 09 - Troubleshooting

[עברית](he/09-troubleshooting.md)

[Guide index](../../README.md#learning-path) | Previous: [Panels and tuning](08-panels-tuning.md) | Next: [team workflow](10-workflow.md)

## Read IDE errors

Your normal workflow is **edit → select the controller and Android app configuration → Run ▶**. Run builds and installs automatically. Sync after dependency or build configuration changes, or when the IDE prompts you.

When something fails, open the **Build**, **Run** or **Sync** tool window, expand the first relevant error and follow its file link. Fix that cause before retrying. Confirm the selected run configuration is the Android app when deploying.

## Diagnose common problems

| Symptom | Check and next action |
|---|---|
| Gradle cannot start, unsupported Java version or class version | Open Settings > Build, Execution, Deployment > Build Tools > Gradle and select Java 25 as Gradle JDK/JVM, then sync. Editor language level is a separate setting. |
| Java toolchain cannot be found | Expand the Sync/Build error to identify the requested JDK; production needs Java 8 while Gradle runs on Java 25. Add/download the missing JDK through the IDE's JDK controls and check toolchain download access before changing compiler versions. |
| Android SDK location/platform missing | Select the correct location and install requested packages in SDK Manager, then sync. Let the IDE update local SDK settings; keep machine paths out of Git. |
| TeamCode Gradle project not found | Select the root Android application in Run > Edit Configurations, or the root project for Gradle check configurations. TeamCode is a configured source directory. |
| Dependency cannot resolve | Read which artifact/repository failed; check network access and repository configuration. Offline mode only works with cached artifacts. |
| Duplicate Panels classes or incompatible configuration integration | Retain the declared Sloth FullPanels variant and inspect dependencies; do not add the regular variant alongside it. |
| OpMode absent from Driver Station | Teaching examples have `@Disabled`. Enable a renamed copy, check its annotation, click the Android app's Run ▶, and check for stale Sloth code. |
| AutoTune procedures absent | Open port `10158`, not Panels port `8001`. `Tuning` is not a selectable TeleOp; check installed code and static zero-argument `@Tuner` factories returning `Procedure`. |
| INIT fails with a hardware lookup error | Match case-sensitive device names and device types to the selected Robot Controller configuration. Drive lessons require Pinpoint as well as four motors. |
| A command never runs or repeatedly interrupts another | Keep `super.run()`, check bindings and subsystem requirements, and use one subsystem instance per physical mechanism. |
| Sequence never advances | Continuous commands need a deadline, timeout or finish condition. Use finite `stopCommand()` inside a sequence. Durations here are milliseconds. |
| A held action restarts after finishing | `whileHeld` may schedule it again while the button stays held. Choose press-edge binding or explicit fault latching when retries are inappropriate. |
| Servo command finishes before travel completes | Completion means the request was sent. Add measured timing or real feedback when later actions depend on arrival. |
| Robot moves the wrong way or pose drifts during rotation | Verify wheel layout, motor/encoder directions, pod type and signed offsets before tuning controller gains. |
| Field-centric direction is wrong | Verify heading sign/origin and coordinate frame. Resetting pose changes the field frame. |
| Panels edit has no effect | Check whether code reads the field each loop or copied it during initialization/path construction; STOP and reinitialize snapshots. |
| Deployed code appears unchanged | Confirm the selected device and successful task output; remove old Sloth remote code before a full install as described in chapter 08. |

The standalone lint version is intentional for these dependencies' Kotlin metadata. An experimental-setting warning or Gradle deprecation warning is different from a failed check. Do not upgrade the entire build stack just to silence a warning without reviewing compatibility.

## Verify on the robot

Run only the hardware checks relevant to your change. For a motor action, verify zero output at INIT, low-power direction, release, interruption and STOP. For a servo, verify measured linkage limits and remember that STOP does not restore a position. For driving, follow [chapter 06](06-driving.md#first-drive-verification); for paths, follow [chapter 07](07-autonomous.md#build-your-routine-incrementally).

Record what actually ran: code revision, hardware configuration, measurements and observed results. Mark checks that need hardware as pending until someone performs them. A successful APK build is not a completed driving test.
