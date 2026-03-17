package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.panels.Panels;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends CommandOpMode {

    private Intake intake;
    private GamepadEx driverOp;

    @Override
    public void initialize() {
        JoinedTelemetry joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        // 1. Initialize Gamepad and Subsystems
        driverOp = new GamepadEx(gamepad1);
        intake = new Intake(hardwareMap, "intakeMotor");

        // 2. Set the Default Command
        // Whenever you let go of the intake buttons, the scheduler will automatically
        // revert to this default command, stopping the motor.
        intake.setDefaultCommand(intake.defaultStopCommand());

        // 3. Bind Commands to Buttons
        // When A is held, it interrupts the default command and spins up.
        // When A is released, the command ends, and the default command takes over again.
        driverOp.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(intake.spinUpCommand());

        driverOp.getGamepadButton(GamepadKeys.Button.B)
                .whileHeld(intake.spinDownCommand());
                
        // Note: For complex interactions between subsystems (like Intake + Conveyor),
        // you can build sequential groups right here in the OpMode or RobotContainer.
    }
}