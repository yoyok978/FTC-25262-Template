[עברית](readme.he.md)

# TeamCode sources

This directory contains the template's active robot code. TeamCode is a source directory in the root Dairy Android application, not a separate Gradle module. Open the root project in Android Studio/IntelliJ, select the controller and Android app configuration, and click **Run ▶** to build and install. Sync after build configuration changes or when prompted; ordinary Java edits usually need only Run.

Start with the [project README](../../../../../../../../README.md) and [programming guide](../../../../../../../../doc/guide/01-setup.md).

- `opmodes`: enabled Starter TeleOp and your team's routines.
- `examples`: seven disabled lessons to copy, rename and intentionally enable.
- `subsystems`: mechanism owners and command factories; `drivetrain/PedroDrive` owns the follower.
- `pedroPathing`: shared Pedro 3 configurations, AutoTune registration and `procedures` for mecanum/Pinpoint/Foresight/validation.
- `RobotCommandOpMode`: shared scheduler cancellation and lifecycle support.

The Starter TeleOp requires four mecanum motors, Pinpoint and compatible pods. Configure the robot and measure offsets before running it. Lessons 01 and 07 need no peripherals.

Follow [the copying procedure](../../../../../../../../doc/guide/10-workflow.md#copy-an-example) to add an OpMode. Keep one subsystem instance per physical mechanism and preserve cancellation/STOP cleanup.

Upstream FTC samples in `FtcRobotController` and old robot code in `doc/archive` are references outside the compiled source sets. Copy and adapt a sample into active TeamCode sources to compile it. Retain applicable notices when reusing code.
