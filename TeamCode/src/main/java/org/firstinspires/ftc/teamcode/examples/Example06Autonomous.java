package org.firstinspires.ftc.teamcode.examples;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
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
 */
@Disabled
@Autonomous(name = "06 Line and curve", group = "Examples")
public class Example06Autonomous extends RobotCommandOpMode {
    public static final PoseFactory FIELD_POSES = PoseFactory.degrees();
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
        SequentialCommandGroup routine = new SequentialCommandGroup(
                new ParallelDeadlineGroup(new FollowPathCommand(drive.follower(), paths[0]),
                        new RunCommand(() -> stage = "Following line")),
                new WaitCommand(500),
                new InstantCommand(() -> stage = "Following return curve"),
                new FollowPathCommand(drive.follower(), paths[1]),
                new InstantCommand(() -> { stage = "Complete"; drive.stop(); }, drive));
        // FollowPathCommand doesn't declare our wrapper subsystem. The parent owns it explicitly.
        routine.addRequirements(drive);
        schedule(routine); // Scheduler does not run until START in RobotCommandOpMode.
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
