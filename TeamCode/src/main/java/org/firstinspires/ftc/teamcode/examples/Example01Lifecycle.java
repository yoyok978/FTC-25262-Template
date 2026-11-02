package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.RobotCommandOpMode;

/** 01: Hardware-free lifecycle. Guide: doc/guide/03-foundations.md.
 * Copy: rename file/class/@TeleOp, remove @Disabled. No hardware configuration required.
 * Gamepad 1 A is shown in telemetry. Check: INIT status, START loop count, STOP cleanup.
 */
@Disabled
@TeleOp(name = "01 Lifecycle (no hardware)", group = "Examples")
public class Example01Lifecycle extends RobotCommandOpMode {
    private int loops;
    @Override public void initialize() { reset(); loops = 0; }
    @Override public void initialize_loop() {
        telemetry.addData("Lifecycle", "INIT: waiting for START");
        telemetry.update();
    }
    @Override public void preRun() { telemetry.addLine("START received"); }
    @Override public void run() {
        super.run(); // Keep this when you add commands/subsystems later.
        telemetry.addData("Loops since START", ++loops);
        telemetry.addData("Gamepad 1 A", gamepad1.a);
        telemetry.update();
    }
    @Override public void end() { super.end(); }
}
