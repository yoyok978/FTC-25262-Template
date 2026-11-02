package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.command.*;
import com.seattlesolvers.solverslib.gamepad.*;
import org.firstinspires.ftc.teamcode.RobotCommandOpMode;
import org.firstinspires.ftc.teamcode.subsystems.*;

/** 04: exampleMotor + exampleServo. Guide: doc/guide/04-commands.md.
 * Copy/change: hardware names, power, servo positions, durations (milliseconds), class/name.
 * A starts a sequence; B cancels it. Check both completion and cancellation stop the motor.
 * Different subsystems may run in parallel; commands sharing a motor may not.
 */
@Disabled
@TeleOp(name = "04 Command composition", group = "Examples")
public class Example04CommandGroups extends RobotCommandOpMode {
    @Override public void initialize() {
        reset();
        MotorSubsystem motor = new MotorSubsystem(hardwareMap, "exampleMotor", DcMotorSimple.Direction.FORWARD);
        ServoSubsystem servo = new ServoSubsystem(hardwareMap, "exampleServo");
        Command sequence = new SequentialCommandGroup(
                servo.moveTo(() -> 0.45),
                new WaitCommand(250), // Allows a chosen delay; not proof the servo arrived.
                // עברית: ההמתנה מאפשרת השהיה נבחרת ואינה הוכחה שהסרוו הגיע.
                new ParallelDeadlineGroup(new WaitCommand(750), motor.run(() -> 0.2),
                        servo.moveTo(() -> 0.55)),
                // ParallelCommandGroup waits for both independent children to finish.
                // עברית: ParallelCommandGroup ממתינה לסיום שני הילדים העצמאיים.
                new ParallelCommandGroup(motor.run(() -> -0.1).withTimeout(300),
                        servo.moveTo(() -> 0.45)),
                motor.stopCommand()); // Finite stop: the sequence can now finish.
        // עברית: העצירה סופית, ולכן הרצף יכול להסתיים כעת.
        GamepadEx operator = new GamepadEx(gamepad1);
        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(sequence);
        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(sequence::cancel);
    }
}
