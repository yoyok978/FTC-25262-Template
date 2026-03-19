package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.conv.Conv;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

public class RobotCommands {
    private final Conv conv;
    private final Shooter shooter;
    private final Intake intake;

    public RobotCommands(Conv conv, Shooter shooter, Intake intake) {
        this.conv = conv;
        this.shooter = shooter;
        this.intake = intake;
    }

    public Command shoot() {
        return shoot(794);
    }

    public Command shoot(int vel) {
        // ParallelDeadlineGroup runs everything inside it until the "deadline" command (the WaitCommand) finishes.
        // This cleanly replaces the SleepActions while allowing the RunCommands to process their over-current checks.
        return new SequentialCommandGroup(
                    conv.prepareToShootCommand(100),
                    new ParallelDeadlineGroup(
                    new WaitCommand(3600), // Deadline: 0.1s initial wait + 3.5s shoot duration
                    shooter.spinUpCommand(vel),
                    new SequentialCommandGroup(
                            new WaitCommand(100), // Wait 0.1s before feeding
                            new ParallelCommandGroup(
                                    intake.spinUpCommand(),
                                    conv.loadCommand(0.08)
                            )
                    )
            )).andThen(stopShot()); // Explicitly stop everything once the deadline is reached
    }

    public Command stopShot() {
        // InstantCommands fire exactly once to zero out the hardware
        return new ParallelCommandGroup(
                intake.defaultStopCommand(),
                conv.defaultStopCommand(),
                shooter.defaultStopCommand()
        );
    }

    public Command load() {
        return load(0.08);
    }

    public Command load(double power) {
        return new ParallelCommandGroup(
                intake.spinUpCommand(),
                conv.loadCommand(power)
        );
    }

    public Command unLoad() {
        return new ParallelCommandGroup(
                intake.spinDownCommand(),
                conv.unLoadCommand()
        );
    }

    public Command unLoad(double durationSeconds) {
        return new ParallelDeadlineGroup(
                new WaitCommand((long) (durationSeconds * 1000)),
                intake.spinDownCommand(),
                conv.unLoadCommand()
        ).andThen(stopLoad());
    }

    public Command stopLoad() {
        return new ParallelCommandGroup(
                intake.defaultStopCommand(),
                conv.defaultStopCommand()
        );
    }
}