package org.firstinspires.ftc.teamcode.examples;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.gamepad.*;
import org.firstinspires.ftc.teamcode.RobotCommandOpMode;
import org.firstinspires.ftc.teamcode.subsystems.MotorSubsystem;

/** 02: One motor named exampleMotor. Guide: doc/guide/05-hardware.md.
 * Copy/change: motor name, direction, POWER, file/class/@TeleOp; remove @Disabled.
 * A held: forward, B held: reverse. Release: stop. Don't hold both competing buttons.
 * Check low-power direction first, then release/STOP. No drivetrain or Pinpoint needed.
 */
@Disabled
@Configurable
@TeleOp(name = "02 Motor subsystem", group = "Examples")
public class Example02Motor extends RobotCommandOpMode {
    public static double POWER = 0.2;
    private MotorSubsystem motor;
    @Override public void initialize() {
        reset();
        motor = new MotorSubsystem(hardwareMap, "exampleMotor", DcMotorSimple.Direction.FORWARD);
        GamepadEx operator = new GamepadEx(gamepad1);
        operator.getGamepadButton(GamepadKeys.Button.A).whileHeld(motor.run(() -> POWER));
        operator.getGamepadButton(GamepadKeys.Button.B).whileHeld(motor.run(() -> -POWER));
    }
    @Override public void run() {
        super.run();
        telemetry.addData("Motor velocity (ticks/s)", motor.velocity());
        telemetry.update();
    }
}
