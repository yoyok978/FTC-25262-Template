# Pre-template robot archive

[Current template](../../../README.md) | [Current guide](../../guide/01-setup.md)

This directory preserves the season-specific robot code and configuration from before the generic template conversion. It is historical reference, not an alternative starter. The root build compiles only `TeamCode/src/main`; none of this archive is compiled or registered as OpModes. Original packages are retained for reference.

| Contents | Purpose |
|---|---|
| [README-original.md](README-original.md) | Previous repository readme |
| [build.gradle.kts](build.gradle.kts) | Previous root build configuration |
| [java/org/firstinspires/ftc/teamcode](java/org/firstinspires/ftc/teamcode) | Archived robot sources, including autonomous, alliance TeleOps, intake, conveyor, shooter, drivetrain and constants |

The archived hardware names, gains, motor velocities, poses and mechanism behavior belong to that robot. Known problems and migration caveats are recorded in [the historical upgrade review](../../UPGRADE_REVIEW.md). Do not copy settings without adapting and measuring them for your robot. The active numbered examples replace this code as the teaching path.

To reuse an idea, copy only the needed behavior into active TeamCode sources, adapt it to the current APIs and your hardware, and verify command ownership, initialization, cancellation and STOP. Keep this archive outside source sets and retain applicable licenses/notices.
