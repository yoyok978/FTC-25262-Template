package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

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
    public static double UNLOAD_TIME_SHORT = 0.1;
    public static double UNLOAD_TIME_LONG = 0.3;

    private Follower follower;

    @Override
    public void initialize() {
        super.reset();
        // --- HARDWARE INITIALIZATION ---
        Conv conv = new Conv(hardwareMap, "conv");

        Intake intake = new Intake(hardwareMap, "intake");
        Shooter shooter = new Shooter(hardwareMap, "shooterRight", "shooterLeft");

        RobotCommands robotCommands = new RobotCommands(conv, shooter, intake);

        // --- PEDRO PATHING INITIALIZATION ---
        follower = Constants.createFollower(hardwareMap);

        // Define Poses
        double goalHeading = Math.atan2(34 - 72, 24 - 72) + Math.PI;
        Pose initialPose = new Pose(-56, -56, Math.toRadians(45));
        Pose scoringPosition = new Pose(-24, -34, goalHeading);
        Pose firstCollectionPosition = new Pose(-26, -40, Math.PI * 1.5 - 0.13);
        Pose secondCollectionPosition = new Pose(-26, -60, Math.PI * 1.5 - 0.13);
        Pose intermediateScoringPosition = new Pose(-26, -44, Math.PI * 1.5);
        Pose parkingPosition = new Pose(32, -50, 0);

        follower.setStartingPose(initialPose);

        // --- PATH BUILDING ---
        PathChain driveToScoringPosition = follower.pathBuilder()
                .addPath(new BezierLine(initialPose, scoringPosition))
                .setLinearHeadingInterpolation(initialPose.getHeading(), scoringPosition.getHeading())
                .build();

        PathChain driveToFirstCollection = follower.pathBuilder()
                .addPath(new BezierLine(scoringPosition, firstCollectionPosition))
                .setLinearHeadingInterpolation(scoringPosition.getHeading(), firstCollectionPosition.getHeading())
                .build();

        PathChain driveToSecondCollection = follower.pathBuilder()
                .addPath(new BezierLine(firstCollectionPosition, secondCollectionPosition))
                .setConstantHeadingInterpolation(firstCollectionPosition.getHeading())
                .build();

        PathChain driveToScoringPositionFromCollection = follower.pathBuilder()
                .addPath(new BezierLine(secondCollectionPosition, intermediateScoringPosition))
                .setLinearHeadingInterpolation(secondCollectionPosition.getHeading(), intermediateScoringPosition.getHeading())
                .addPath(new BezierLine(intermediateScoringPosition, scoringPosition))
                .setLinearHeadingInterpolation(intermediateScoringPosition.getHeading(), scoringPosition.getHeading())
                .build();

        PathChain driveToParkingPosition = follower.pathBuilder()
                .addPath(new BezierLine(scoringPosition, parkingPosition))
                .setLinearHeadingInterpolation(scoringPosition.getHeading(), parkingPosition.getHeading())
                .build();

        // --- COMMAND SEQUENCING ---
        Command score1 = new SequentialCommandGroup(
                new FollowPathCommand(follower, driveToScoringPosition),
                robotCommands.shoot(),
                robotCommands.stopShot()
        );

        Command collect1 = new ParallelCommandGroup(
                robotCommands.stopShot(),
                robotCommands.unLoadShooter(0.2),
                new FollowPathCommand(follower, driveToFirstCollection),
                robotCommands.load(0.08)
        );

        Command moveToScore = new ParallelCommandGroup(
                new FollowPathCommand(follower, driveToScoringPositionFromCollection),
                robotCommands.unLoad(0.2),
                robotCommands.unLoadShooter(1.0)
        );

        Command score2 = new SequentialCommandGroup(
                new FollowPathCommand(follower, driveToSecondCollection),
                robotCommands.stopLoad(),
                moveToScore,
                new InstantCommand(conv::stop, conv),
                new ParallelCommandGroup(
                        robotCommands.unLoadShooter(0.2),
                        robotCommands.unLoad(1.0)
                ),
                robotCommands.shoot(),
                robotCommands.stopShot()
        );

        Command park = new SequentialCommandGroup(
                new WaitCommand(7000),
                new FollowPathCommand(follower, driveToParkingPosition)
        );

        Command fullAutoRoutine = new SequentialCommandGroup(
                score1,
                collect1,
                score2,
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

    }
}