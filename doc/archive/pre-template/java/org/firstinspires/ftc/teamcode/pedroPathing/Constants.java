package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {
    // Authored field headings use degrees; Pedro stores the resulting headings in radians.
    public static final PoseFactory FIELD_POSES = PoseFactory.degrees();
    // Runtime calculations and tuning angles already use radians.
    public static final PoseFactory RADIAN_POSES = PoseFactory.radians();

    // Retained measurements; verify on the robot. Pedro 3 no longer uses mass directly.
    public static double ROBOT_MASS = 8;
    public static double POD_X_DIST = -9 * 2.54;
    public static double POD_Y_DIST = 8.4;
    // Retained pending verification against the robot configuration. See doc/UPGRADE_REVIEW.md.
    public static String FRONT_RIGHT_NAME = "frontLeft";

    // Starting estimates from Pedro 2 defaults, NOT measurements of this robot.
    public static double FORWARD_VELOCITY = 81.34056;
    public static double STRAFE_VELOCITY = 65.43028;
    public static double FORWARD_DECELERATION = 34.62719;
    public static double STRAFE_DECELERATION = 78.15554;
    public static double TRANSLATIONAL_P = 0.1;
    public static double TRANSLATIONAL_STATIC = 0.015;
    public static double HEADING_P = 1.0;
    public static double HEADING_STATIC = 0.01;
    // New braking coefficients require measurement. Quadratic estimates use v^2 / (2a).
    public static double FORWARD_LINEAR_BRAKE = 0;
    public static double STRAFE_LINEAR_BRAKE = 0;
    public static double FORWARD_QUADRATIC_BRAKE = 1 / (2 * 34.62719);
    public static double STRAFE_QUADRATIC_BRAKE = 1 / (2 * 78.15554);
    public static double HEADING_LINEAR_BRAKE = 0;
    public static double HEADING_QUADRATIC_BRAKE = 0;

    public static ForesightConfig createForesightConfig() {
        return new ForesightConfig(c -> {
            c.headingFeedback.set(Controller.pid(HEADING_P, 0, 0));
            c.headingStaticFF.set(Controller.staticFeedforward(HEADING_STATIC));
            c.forwardTranslational.set(Controller.sum(Controller.pid(TRANSLATIONAL_P, 0, 0),
                    Controller.staticFeedforward(TRANSLATIONAL_STATIC)));
            c.strafeTranslational.set(Controller.sum(Controller.pid(TRANSLATIONAL_P, 0, 0),
                    Controller.staticFeedforward(TRANSLATIONAL_STATIC)));
            // New feedforward interfaces have no exact equivalent to Pedro 2 drive PIDF.
            c.coast.set(Controller.proportionalFeedforward(1 / FORWARD_VELOCITY));
            c.brake.set(Controller.proportionalFeedforward(1 / FORWARD_VELOCITY));
            c.maxAchievableForwardVelocity.set(FORWARD_VELOCITY);
            c.maxAchievableStrafeVelocity.set(STRAFE_VELOCITY);
            c.naturalForwardDeceleration.set(FORWARD_DECELERATION);
            c.naturalStrafeDeceleration.set(STRAFE_DECELERATION);
            c.linearBrakeCoefficients.set(Matrix.diag(FORWARD_LINEAR_BRAKE, STRAFE_LINEAR_BRAKE));
            c.quadraticBrakeCoefficients.set(Matrix.diag(FORWARD_QUADRATIC_BRAKE, STRAFE_QUADRATIC_BRAKE));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(HEADING_LINEAR_BRAKE, HEADING_QUADRATIC_BRAKE));
            c.parametricTConstraint.set(0.01); // Former end t = 0.99.
            c.timeoutConstraint.set(100.0);
        });
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        MecanumConfig drive = new MecanumConfig(c -> {
            c.frontLeftName.set("frontLeft");
            c.backLeftName.set("backLeft");
            c.frontRightName.set(FRONT_RIGHT_NAME);
            c.backRightName.set("backRight");
            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        });
        PinpointConfig pinpoint = new PinpointConfig(c -> {
            c.name.set("odo");
            c.xPodOffset.set(POD_Y_DIST);
            c.yPodOffset.set(POD_X_DIST);
            c.offsetUnits.set(DistanceUnit.CM);
            c.globalDistanceUnit.set(DistanceUnit.INCH); // Existing field coordinates are inches.
            c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        });
        return new Follower(new PinpointLocalizer(hardwareMap, pinpoint),
                new Mecanum(hardwareMap, drive), new Foresight(createForesightConfig()));
    }
}
