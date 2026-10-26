package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.drivetrain.DrivePowers;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import com.seattlesolvers.solverslib.controller.PIDController;
import org.firstinspires.ftc.teamcode.Helpers;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Configurable
public class PedroDrive extends SubsystemBase {
    public static double GOAL_LOCK_P = 1.0;
    public static double GOAL_LOCK_I = 0.0;
    public static double GOAL_LOCK_D = 1.0;

    private final Follower follower;
    private final PIDController goalLockPid;

    private boolean goalLockActive = false;
    private boolean isReversed = false;


    private double speedMult = 0.75;


    public PedroDrive(HardwareMap hardwareMap, Pose startingPose) {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startingPose);
        follower.manual(DrivePowers.zero());

        goalLockPid = new PIDController(GOAL_LOCK_P, GOAL_LOCK_I, GOAL_LOCK_D);
    }

    /**
     * Converts field inputs to robot coordinates for Pedro 3 manual drive.
     * Handles Field Centric and Goal Lock internally to keep commands clean.
     */
    public void driveTeleOp(double forward, double strafe, double turn, boolean fieldCentric, boolean isBlue) {
        Pose currentPose = follower.pose();
        double heading = currentPose.heading();


        forward *= speedMult;
        strafe *= speedMult;
        turn *= speedMult;


        // 1. Goal Lock (Overrides manual turn input)
        if (goalLockActive) {
            double targetAngle = Helpers.getAngleToBasket(currentPose, isBlue);
            // Calculate shortest path error
            double error = targetAngle - heading;
            while (error > Math.PI) error -= 2 * Math.PI;
            while (error < -Math.PI) error += 2 * Math.PI;

            turn = goalLockPid.calculate(0, error); // Apply PID to turn vector
        }

        // 3. Send vectors to Pedro Pathing
        double angle = (fieldCentric ? -heading : 0) + (isReversed ? Math.PI : 0);
        double robotForward = forward * Math.cos(angle) - strafe * Math.sin(angle);
        double robotStrafe = forward * Math.sin(angle) + strafe * Math.cos(angle);
        follower.manual(robotForward, robotStrafe, Math.max(-1, Math.min(1, turn)));
    }

    public void toggleGoalLock() {
        goalLockActive = !goalLockActive;
        goalLockPid.reset();
    }
    public void toggleReverse() {
        isReversed = !isReversed;
    }

    public void setSpeedMult(double mult) {
        speedMult = mult;
    }

    public void resetOdometry(Pose resetPose) {
        follower.setPose(resetPose);
    }

    public Pose getPose() {
        return follower.pose();
    }

    public void stop() {
        follower.stop();
        follower.drivetrain.stop();
    }

    /**
     * @param forward A function which returns the forward input
     * @param strafe A function which returns the strafe input
     * @param turn A function which returns the turn input
     * @param fieldCentric A function which returns if the robot is FieldCentric
     * @param isBlue A function which returns what color alliance the robot is
     * @return A command which drives the robot
     */
    public Command drive(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn, BooleanSupplier fieldCentric, BooleanSupplier isBlue){

        return new RunCommand(() ->
        driveTeleOp(
                forward.getAsDouble(),
                strafe.getAsDouble(),
                turn.getAsDouble(),
                fieldCentric.getAsBoolean(),
                isBlue.getAsBoolean()
        ),this);

    }


    @Override
    public void periodic() {
        // This runs automatically every loop cycle through the Command Scheduler!
        follower.update();
    }

    /**
     * Movement telemetry
     * @param telemetry Robot Telemetry
     */
    public void telemetryDebug(JoinedTelemetry telemetry) {
        telemetry.addData("Robot Pose", follower.pose().toString());
        telemetry.addData("Robot Velocity", follower.velocity().toString());
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Reverse Mode", isReversed ? "ACTIVE" : "DISABLED");
        telemetry.addData("Goal Lock", goalLockActive ? "ACTIVE" : "DISABLED");

    }

}
