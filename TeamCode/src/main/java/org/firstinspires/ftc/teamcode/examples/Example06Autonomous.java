package org.firstinspires.ftc.teamcode.examples;

import static com.pedropathing.api.Paths.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FIELD_POSES;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.seattlesolvers.solverslib.command.*;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import org.firstinspires.ftc.teamcode.RobotCommandOpMode;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.PedroDrive;

/** 06: Four mecanum motors + Pinpoint + pods and a TUNED follower REQUIRED.
 * Guide: doc/guide/07-autonomous.md. Copy/change: poses, starting orientation, Constants, class/name.
 * Remove @Disabled only after localization/drive tests. START runs one line and a return curve.
 * Check endpoint/heading telemetry and STOP before increasing distance. No mechanism required.
 * עברית: 06 — נדרשים ארבעה מנועי mecanum,‏ Pinpoint, גלגלי אודומטריה ועוקב מכוונן.
 * מדריך: doc/guide/he/07-autonomous.md. להעתקה שנו תנוחות, כיוון התחלה, Constants ושמות מחלקה ותצוגה.
 * הסירו @Disabled רק אחרי בדיקות מיקום והנעה. START מריץ קו אחד ועקומת חזרה.
 * בדקו טלמטריית נקודת קצה וזווית ו־STOP לפני הגדלת המרחק. אין צורך במנגנון נוסף.
 */
@Disabled
@Autonomous(name = "06 Line and curve", group = "Examples")
public class Example06Autonomous extends RobotCommandOpMode {
    private PedroDrive drive;
    private String stage;
    public static Path[] examplePaths() {
        Pose start = FIELD_POSES.of(0, 0, 0);
        Pose end = FIELD_POSES.of(12, 0, 90);
        return new Path[] {
                line(start, end).linear(start, end),
                curve(end, FIELD_POSES.of(12, 12, 0), FIELD_POSES.of(0, 12, 0), start).linear(end, start)
        };
    }
    @Override public void initialize() {
        reset();
        stage = "Waiting for START";
        drive = new PedroDrive(hardwareMap, FIELD_POSES.of(0, 0, 0));
        Path[] paths = examplePaths();
        // Subsystem periodic() is the only follower.update() site; don't add a second update here.
        // עברית: periodic() של תת־המערכת הוא המקום היחיד לעדכון העוקב; אל תוסיפו כאן עדכון שני.
        SequentialCommandGroup routine = new SequentialCommandGroup(
                new ParallelDeadlineGroup(new FollowPathCommand(drive.follower(), paths[0]),
                        new RunCommand(() -> stage = "Following line")),
                new WaitCommand(500),
                new InstantCommand(() -> stage = "Following return curve"),
                new FollowPathCommand(drive.follower(), paths[1]),
                new InstantCommand(() -> { stage = "Complete"; drive.stop(); }, drive));
        // FollowPathCommand doesn't declare our wrapper subsystem. The parent owns it explicitly.
        // עברית: FollowPathCommand אינה מצהירה על תת־מערכת המעטפת שלנו; ההורה מחזיק בה במפורש.
        routine.addRequirements(drive);
        schedule(routine); // Scheduler does not run until START in RobotCommandOpMode.
        // עברית: ב־RobotCommandOpMode המתזמן אינו רץ עד START.
    }
    @Override public void run() {
        super.run();
        telemetry.addData("Stage", stage);
        drive.telemetryDebug(telemetry);
        telemetry.update();
    }
    @Override public void end() {
        try { super.end(); } finally { if (drive != null) drive.stop(); }
    }
}
