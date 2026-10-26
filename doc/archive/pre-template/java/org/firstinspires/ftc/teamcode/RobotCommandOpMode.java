package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

/** SolversLib resets discard scheduler state; cancel first so commands release hardware. */
public abstract class RobotCommandOpMode extends CommandOpMode {
    @Override
    public void reset() {
        CommandScheduler.getInstance().cancelAll();
        super.reset();
    }

    @Override
    public void end() {
        CommandScheduler.getInstance().cancelAll();
    }
}
