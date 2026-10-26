# 10 - Team workflow and adapting the template

[עברית](he/10-workflow.md)

[Guide index](../../README.md#learning-path) | Previous: [troubleshooting](09-troubleshooting.md)

## Make the repository your team's starting point

Create a team-owned repository from this template and use the IDE's **Get from VCS / Clone Repository** button to open your team copy. For an existing project, use **File > Open** on the root directory and manage remotes through the IDE's **Git > Manage Remotes** action. Keep the existing dependency versions until you intentionally review an upgrade. Configure names, motor and encoder directions, pod type and measured offsets for your robot before driving; the provided values do not describe your installation.

Keep the generic numbered lessons as references. Put team routines in `opmodes` and mechanism owners in `subsystems`. The old season code is preserved in [the archive](../archive/pre-template/INDEX.md), outside the compiled source directories. Consult it as history; copy only behavior you understand and have adapted to the current libraries and hardware.

## Copy an example

1. Choose the smallest lesson that demonstrates the behavior you need. Lessons 01 and 07 need no peripherals; the README lists the hardware for every other lesson.
2. Copy the Java source from `examples` into the sibling `opmodes` directory. Change its package to `org.firstinspires.ftc.teamcode.opmodes`.
3. Rename the file and public class together, for example `IntakeTeleOp.java` and `IntakeTeleOp`. Use a unique displayed `@TeleOp` or `@Autonomous` name and a suitable group.
4. Replace hardware names, directions and numeric requests with your robot's measured choices. Copy/rename a subsystem when its responsibilities change; rename its constructors too. Keep one subsystem instance for each physical mechanism in an OpMode.
5. Remove `@Disabled` from your copy. Keep it on the original teaching example. Preserve `reset()`, `super.run()` and command cancellation/drive cleanup when adapting the lifecycle.
6. Update the copy's opening comment with its required hardware, controls and verification steps. Select the controller and Android app configuration and click **Run ▶** to build and install. Alternatively, use the optional Sloth configuration from chapter 08. Select the new displayed OpMode name on Driver Station. Ordinary Java edits do not need a separate Sync or Build action.
7. Verify INIT, normal operation, release/cancellation and STOP at low power before adding another feature.

An upstream FTC SDK sample can also be copied into TeamCode, but follow its SDK lifecycle rather than assuming it already uses SolversLib. The `FtcRobotController` sample tree itself is excluded from this build. Do not create a new TeamCode Gradle module to enable a sample.

## Add a feature in small steps

Start with the hardware boundary: map the device once, define units and limits, and expose meaningful subsystem operations. Add commands that declare requirements and clean up continuous output on completion or interruption. Then bind controls or compose a routine. Keep game-specific coordinates and scoring decisions in the team routine.

For an intake, begin with a copied motor lesson and measured open-loop powers. For a claw, use measured servo endpoints and make any travel delay explicit. For autonomous, first run one short path, inspect endpoint and heading, then add a second segment and mechanism coordination. A timeout limits elapsed time; decide how a routine handles failure before allowing later scoring actions to assume arrival.

Change one physical or tuning variable at a time. Record battery condition, surface, load, signed measurements, units and results in team notes; enter accepted constants in source. Panels edits alone do not preserve the team's configuration.

## Review and share changes

Before editing, inspect the IDE's **Commit / Local Changes** view so you know which work is already present. Create a feature branch with the IDE's branch selector and use **Show Diff** before selecting files to commit. Keep unrelated changes separate and do not discard another programmer's work. Commit messages and pull requests should explain the resulting behavior, the relevant checks and remaining hardware validation.

Before sharing a change:

- Select the controller and Android app configuration, then use **Run ▶** to build and install. Consult [chapter 09](09-troubleshooting.md) if something fails.
- Check required devices, displayed OpMode names, controls and documentation links.
- Review subsystem requirements, initialization output and cancellation/STOP behavior.
- Record robot checks and any pending measurements.
- Exclude local SDK paths, IDE settings, generated output and credentials; retain the repository license and applicable third-party notices.

The FTC development keystore supplied in `libs` belongs to the development setup; preserve it as documented. Do not replace it with personal production signing material. Keep dependency/build changes deliberate and document compatibility checks.

When you later change this template's defaults, update the guide, examples and README together. Add a dated changelog entry that distinguishes implemented behavior from measured robot performance. This lets the next programmer reproduce the working setup without relying on a previous chat.

Exercise: create a renamed motor lesson for a team mechanism, write its hardware/control checklist, and have a teammate trace its initialization, command requirements and stop path before running it.
