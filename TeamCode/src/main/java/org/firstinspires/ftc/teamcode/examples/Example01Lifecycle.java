package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.RobotCommandOpMode;

/** 01: Hardware-free lifecycle. Guide: doc/guide/03-foundations.md.
 * Copy: rename file/class/@TeleOp, remove @Disabled. No hardware configuration required.
 * Gamepad 1 A is shown in telemetry. Check: INIT status, START loop count, STOP cleanup.
 * עברית: 01 — מחזור חיים ללא חומרה. מדריך: doc/guide/he/03-foundations.md.
 * להעתקה: שנו שמות קובץ, מחלקה ו־@TeleOp והסירו @Disabled. אין צורך בתצורת חומרה.
 * כפתור A בבקר 1 מוצג בטלמטריה. בדקו מצב INIT, מונה לולאות ב־START וניקוי ב־STOP.
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
        // עברית: שמרו קריאה זו כשתוסיפו בהמשך פקודות ותתי־מערכות.
        telemetry.addData("Loops since START", ++loops);
        telemetry.addData("Gamepad 1 A", gamepad1.a);
        telemetry.update();
    }
    @Override public void end() { super.end(); }
}
