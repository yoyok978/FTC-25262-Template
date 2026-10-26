package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FIELD_POSES;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;

import com.pedropathing.follower.Follower;

import static com.pedropathing.api.Paths.*;


import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.seattlesolvers.solverslib.command.*;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;
import org.firstinspires.ftc.teamcode.Helpers;
import org.firstinspires.ftc.teamcode.RobotCommandOpMode;
import org.firstinspires.ftc.teamcode.Stats;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.RobotCommands;
import org.firstinspires.ftc.teamcode.subsystems.conv.Conv;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

@Configurable
@Autonomous(name = "Blue Basket Auto")
public class BlueBasketAuto extends RobotCommandOpMode {

    // 1. Define Constants for Tuning (Managed by Panels)

    private JoinedTelemetry joinedTelemetry; // Panels Telemetry instance
    private Follower follower;
    private Paths paths; // Paths defined in the Paths class


    @Override
    public void initialize() {
        super.reset();
        joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        // --- HARDWARE INITIALIZATION ---
        Conv conv = new Conv(hardwareMap, "conv");

        Intake intake = new Intake(hardwareMap, "intake");
        Shooter shooter = new Shooter(hardwareMap, "shooterRight", "shooterLeft");

        RobotCommands robotCommands = new RobotCommands(conv, shooter, intake);

        // --- PEDRO PATHING INITIALIZATION ---
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower, robotCommands);
        // Define Poses

        follower.setPose(FIELD_POSES.of(63.000, 9.000, 0));

        joinedTelemetry.addData("Status", "Initialized");
        joinedTelemetry.update();

        // --- COMMAND SEQUENCING ---
        Command score1 = new SequentialCommandGroup(
                new FollowPathCommand(follower, paths.path1),
                robotCommands.shoot(950)
        );

        Command collect1 = new ParallelDeadlineGroup(
                new FollowPathCommand(follower, paths.path2),
                robotCommands.load(0.08)
        );
        Command score2 = new SequentialCommandGroup(
                new TurnToCommand(follower, Math.toRadians(114)),
                robotCommands.shoot(950)
        );

        Command collect2 = new SequentialCommandGroup(
                // Loading is handled inside the path
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, paths.path3),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> follower.pathIndex() > 0 || follower.parametricCompletion() >= 0.75),
                                robotCommands.load())),
                robotCommands.stopLoad()
        );

        Command score3 = new SequentialCommandGroup(
                Helpers.TurnToPosCommand(follower, Helpers.GOAL_POSE_BLUE),
                robotCommands.shoot(950)
        );


        Command park = new SequentialCommandGroup(
                new WaitCommand(7000),
                new FollowPathCommand(follower, paths.path4)
        );

        Command fullAutoRoutine = new SequentialCommandGroup(
                score1,
                collect1,
                score2,
                collect2,
                score3,
                park
        );

        // Schedule the routine to start when play is pressed
        schedule(fullAutoRoutine);
    }

    @Override
    public void run() {
        // Update Pedro Pathing localization and drive motors every loop
        follower.update();
        // Must call super.run() so the CommandScheduler executes scheduled commands
        super.run();
        Stats.robotPose = follower.pose();
        joinedTelemetry.addData("X", follower.pose().x());
        joinedTelemetry.addData("Y", follower.pose().y());
        joinedTelemetry.addData("Heading", follower.pose().heading());
        joinedTelemetry.update();

    }

    @Override
    public void end() {
        super.end();
        if (follower != null) {
            follower.stop();
            follower.drivetrain.stop();
        }
    }

    public static class Paths {
        public final Path path1;
        public final Path path2;
        public final Path path3;
        public final Path path4;

        public Paths(Follower follower, RobotCommands robotCommands) {
            Pose pathStart = FIELD_POSES.of(63, 9, 90);
            Pose shootingPose = FIELD_POSES.of(61, 16, 114);
            Pose returningPose = FIELD_POSES.of(61, 16, 180);
            Pose pickupPose = FIELD_POSES.of(43, 60, 180);
            Pose parkPose = FIELD_POSES.of(38, 34, 90);

            path1 = line(pathStart, shootingPose).linear(pathStart, shootingPose);
            path2 = curve(returningPose, FIELD_POSES.of(37, 46, 0), FIELD_POSES.of(0, 38, 0),
                    FIELD_POSES.of(0, 33, 0), FIELD_POSES.of(0, 31, 0), FIELD_POSES.of(49, 29, 0), returningPose)
                    .constant(returningPose);
            path3 = path(
                    curve(shootingPose, FIELD_POSES.of(56, 62, 0), pickupPose)
                            .linear(shootingPose, pickupPose),
                    curve(pickupPose, FIELD_POSES.of(1, 59, 0), FIELD_POSES.of(4, 56, 0), returningPose)
                            .constant(returningPose));
            path4 = line(shootingPose, parkPose).linear(shootingPose, parkPose);
        }
    }
}
