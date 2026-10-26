package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.Subsystem;
import java.util.function.BooleanSupplier;

/** Motor actions release outputs on normal completion, cancellation, and faults. */
public final class MotorCommands {
    private MotorCommands() { }

    public static Command run(Runnable action, Runnable stop, BooleanSupplier finished,
                              Subsystem... requirements) {
        return new FunctionalCommand(() -> { }, () -> {
            if (!finished.getAsBoolean()) action.run();
        }, interrupted -> stop.run(), finished, requirements);
    }
}
