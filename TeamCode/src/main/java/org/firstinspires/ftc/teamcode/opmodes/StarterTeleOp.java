package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.RobotCommandOpMode;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.PedroDrive;

/** MAIN STARTER: four mecanum motors, Pinpoint and pods REQUIRED. See README.md.
 * Copy: change file/class name and @TeleOp name together. Guide: doc/guide/06-driving.md.
 * Gamepad 1: left stick translates, right X turns, LB slows, X toggles field-centric,
 * D-pad UP establishes the CURRENT physical location/heading as pose zero.
 * Check: START with centered sticks gives zero powers; test robot-centric before field-centric.
 * עברית: תוכנית ההתחלה הראשית דורשת ארבעה מנועי mecanum,‏ Pinpoint וגלגלי אודומטריה. ראו README.he.md.
 * להעתקה: שנו יחד שמות קובץ, מחלקה ו־@TeleOp. מדריך: doc/guide/he/06-driving.md.
 * בקר 1: סטיק שמאלי מזיז, X ימני מסובב, LB מאט ו־X מחליף נהיגה ביחס למגרש.
 * D-pad UP מגדיר את המקום והזווית הפיזיים הנוכחיים כתנוחת אפס.
 * בדקו ש־START עם סטיקים ממורכזים נותן אפס הספק; בדקו יחס לרובוט לפני יחס למגרש.
 */
@TeleOp(name = "Starter TeleOp", group = "Team")
public class StarterTeleOp extends RobotCommandOpMode {
    private PedroDrive drive;
    private JoinedTelemetry joined;
    private boolean fieldCentric;
    @Override public void initialize() {
        reset();
        fieldCentric = false;
        joined = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        drive = new PedroDrive(hardwareMap, Pose.zero());
        GamepadEx driver = new GamepadEx(gamepad1);
        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> fieldCentric = !fieldCentric);
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> drive.resetOdometry(Pose.zero()), drive));
        // Raw gamepad axes: up is negative Y, right is positive X. Pedro uses forward/left/CCW.
        // עברית: בקלט הגולמי למעלה הוא Y שלילי וימינה X חיובי. Pedro משתמש בקדימה, שמאלה ונגד כיוון השעון.
        drive.setDefaultCommand(drive.drive(
                () -> deadband(-gamepad1.left_stick_y), () -> deadband(-gamepad1.left_stick_x),
                () -> deadband(-gamepad1.right_stick_x), () -> fieldCentric,
                () -> gamepad1.left_bumper ? 0.2 : 0.4));
    }
    private static double deadband(double value) { return Math.abs(value) < 0.05 ? 0 : value; }
    @Override public void initialize_loop() {
        joined.addData("Required hardware", "4 mecanum motors + Pinpoint + odometry pods");
        joined.addData("Next", "START enables joystick driving. Verify configuration first.");
        joined.update();
    }
    @Override public void run() {
        super.run(); // Runs subsystem periodic(), buttons and commands once each loop.
        // עברית: מריץ periodic() של תתי־מערכות, כפתורים ופקודות פעם אחת בכל לולאה.
        joined.addData("Control frame", fieldCentric ? "Field" : "Robot");
        drive.telemetryDebug(joined);
        joined.update();
    }
    @Override public void end() {
        try { super.end(); } finally { if (drive != null) drive.stop(); }
    }
}
