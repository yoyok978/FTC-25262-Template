package org.firstinspires.ftc.teamcode.examples;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.seattlesolvers.solverslib.gamepad.*;
import org.firstinspires.ftc.teamcode.RobotCommandOpMode;
import org.firstinspires.ftc.teamcode.subsystems.ServoSubsystem;

/** 03: Positional servo named exampleServo. Guide: doc/guide/05-hardware.md.
 * Copy/change: hardware name, two measured positions, file/class/@TeleOp; remove @Disabled.
 * A requests POSITION_A; B requests POSITION_B. Constructor does not move the servo.
 * Check travel without a binding linkage. STOP does not return a servo to a safe position.
 * עברית: 03 — סרוו מיקום בשם exampleServo. מדריך: doc/guide/he/05-hardware.md.
 * להעתקה: שנו שם חומרה, שני מיקומים מדודים ושמות קובץ, מחלקה ו־@TeleOp; הסירו @Disabled.
 * A מבקש POSITION_A;‏ B מבקש POSITION_B. הבנאי אינו מזיז את הסרוו.
 * בדקו תנועה ללא חיבור מכני שנתקע. STOP אינו מחזיר סרוו למיקום בטוח.
 */
@Disabled
@Configurable
@TeleOp(name = "03 Servo subsystem", group = "Examples")
public class Example03Servo extends RobotCommandOpMode {
    public static double POSITION_A = 0.45;
    public static double POSITION_B = 0.55;
    private ServoSubsystem servo;
    @Override public void initialize() {
        reset();
        servo = new ServoSubsystem(hardwareMap, "exampleServo");
        GamepadEx operator = new GamepadEx(gamepad1);
        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(servo.moveTo(() -> POSITION_A));
        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(servo.moveTo(() -> POSITION_B));
    }
    @Override public void run() {
        super.run();
        telemetry.addData("Last requested servo position (not measured angle)", servo.position());
        telemetry.update();
    }
}
