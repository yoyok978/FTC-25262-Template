package org.firstinspires.ftc.teamcode.examples;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.RobotCommandOpMode;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.PedroDrive;

/** 05: Four mecanum motors + Pinpoint + pods REQUIRED. Guide: doc/guide/06-driving.md.
 * Copy/change: Constants hardware settings and measured offsets; class/@TeleOp; remove @Disabled.
 * Gamepad 1 sticks drive; LB reduces speed. Robot-centric only, no saved pose from another OpMode.
 * Check centered-stick zero motion, then forward, left and CCW. See StarterTeleOp for frame/reset bindings.
 * עברית: 05 — נדרשים ארבעה מנועי mecanum,‏ Pinpoint וגלגלי אודומטריה.
 * מדריך: doc/guide/he/06-driving.md. התאימו הגדרות חומרה והיסטים מדודים ב־Constants.
 * שנו מחלקה ו־@TeleOp והסירו @Disabled. הסטיקים בבקר 1 נוהגים; LB מאט.
 * הנהיגה יחסית לרובוט בלבד, ללא תנוחה שמורה מ־OpMode אחר.
 * בדקו אפס תנועה במרכז הסטיקים, ואז קדימה, שמאלה ונגד כיוון השעון.
 * לקישורי שינוי מערכת צירים ואיפוס ראו StarterTeleOp.
 */
@Disabled
@TeleOp(name = "05 Mecanum and Pinpoint", group = "Examples")
public class Example05Mecanum extends RobotCommandOpMode {
    private PedroDrive drive;
    @Override public void initialize() {
        reset();
        drive = new PedroDrive(hardwareMap, Pose.zero());
        drive.setDefaultCommand(drive.drive(() -> axis(-gamepad1.left_stick_y),
                () -> axis(-gamepad1.left_stick_x), () -> axis(-gamepad1.right_stick_x),
                () -> false, () -> gamepad1.left_bumper ? 0.15 : 0.3));
    }
    private static double axis(double v) { return Math.abs(v) < 0.05 ? 0 : v; }
    @Override public void run() { super.run(); drive.telemetryDebug(telemetry); telemetry.update(); }
    @Override public void end() {
        try { super.end(); } finally { if (drive != null) drive.stop(); }
    }
}
