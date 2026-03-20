package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;

import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.seattlesolvers.solverslib.command.*;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;
import org.firstinspires.ftc.teamcode.Helpers;
import org.firstinspires.ftc.teamcode.Stats;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.RobotCommands;
import org.firstinspires.ftc.teamcode.subsystems.conv.Conv;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

@Configurable
@Autonomous(name = "Blue Basket Auto")
public class BlueBasketAuto extends CommandOpMode {

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

        follower.setStartingPose(new Pose(63.000, 9.000));

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
                new FollowPathCommand(follower, paths.path3),
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
        Stats.robotPose = follower.getPose();
        joinedTelemetry.addData("X", follower.getPose().getX());
        joinedTelemetry.addData("Y", follower.getPose().getY());
        joinedTelemetry.addData("Heading", follower.getPose().getHeading());
        joinedTelemetry.update();

    }

    public static class Paths {
        public PathChain path1;
        public PathChain path2;
        public PathChain path3;
        public PathChain path4;


        public Paths(Follower follower, RobotCommands robotCommands) {
            path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(63.000, 9.000),
                                    new Pose(61.000, 16.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(114))
                    .build();

            path2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(61.000, 16.000),
                                    new Pose(37.000, 46.000),
                                    new Pose(0.000, 38.000),
                                    new Pose(0.000, 33.000),
                                    new Pose(0.000, 31.000),
                                    new Pose(49.000, 29.000),
                                    new Pose(61.000, 16.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();


            path3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(61.000, 16.000),
                                    new Pose(56.000, 62.000),
                                    new Pose(43.000, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))
                    .addParametricCallback(0.75, () -> CommandScheduler.getInstance().schedule(robotCommands.load()))
                    .addPath(
                            new BezierCurve(
                                    new Pose(43.000, 60.000),
                                    new Pose(1.000, 59.000),
                                    new Pose(4.000, 56.000),
                                    new Pose(61.000, 16.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();


            path4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(61.000, 16.000),
                                    new Pose(38.000, 34.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(90))
                    .build();
        }

    }
}