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
    // עברית: זוויות מגרש שנכתבות בקוד הן במעלות; Pedro שומר את הזוויות המתקבלות ברדיאנים.
    public static final PoseFactory FIELD_POSES = PoseFactory.degrees();
    // Runtime calculations and tuning angles already use radians.
    // עברית: חישובי זמן ריצה וזוויות כוונון כבר משתמשים ברדיאנים.
    public static final PoseFactory RADIAN_POSES = PoseFactory.radians();

    // REQUIRED: four independently configured motors and a goBILDA Pinpoint with odometry pods.
    // עברית: נדרשים ארבעה מנועים המוגדרים בנפרד ו־goBILDA Pinpoint עם גלגלי אודומטריה.
    // These names must match the active Robot Controller hardware configuration exactly.
    // עברית: שמות אלה חייבים להתאים בדיוק לתצורת החומרה הפעילה של Robot Controller.
    public static String FRONT_LEFT_NAME = "frontLeft";
    public static String FRONT_RIGHT_NAME = "frontRight";
    public static String BACK_LEFT_NAME = "backLeft";
    public static String BACK_RIGHT_NAME = "backRight";
    public static String PINPOINT_NAME = "odo";
    public static DcMotorSimple.Direction LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection FORWARD_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection STRAFE_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    // PLACEHOLDERS in centimeters. Measure signed pod offsets; see doc/guide/06-driving.md.
    // עברית: ערכים זמניים בסנטימטרים. מדדו היסטי אודומטריה עם סימן; ראו doc/guide/he/06-driving.md.
    public static double POD_X_DIST = 0;
    public static double POD_Y_DIST = 0;

    // Starting estimates from Pedro 2 defaults, NOT measurements of this robot.
    // עברית: אומדני התחלה מברירות המחדל של Pedro 2, ולא מדידות של רובוט זה.
    public static double FORWARD_VELOCITY = 81.34056;
    public static double STRAFE_VELOCITY = 65.43028;
    public static double FORWARD_DECELERATION = 34.62719;
    public static double STRAFE_DECELERATION = 78.15554;
    public static double TRANSLATIONAL_P = 0.1;
    public static double TRANSLATIONAL_STATIC = 0.015;
    public static double HEADING_P = 1.0;
    public static double HEADING_STATIC = 0.01;
    // New braking coefficients require measurement. Quadratic estimates use v^2 / (2a).
    // עברית: מקדמי בלימה חדשים דורשים מדידה. אומדנים ריבועיים משתמשים ב־v^2 / (2a).
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
            // עברית: לממשקי הבקרה המקדימה החדשים אין מקבילה מדויקת ל־PIDF ההנעה של Pedro 2.
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
            // עברית: בעבר תנאי הקצה היה t = 0.99.
            c.timeoutConstraint.set(100.0);
        });
    }

    /** Validate before looking up hardware so duplicate drive names fail with a useful message.
     * עברית: אימות לפני איתור חומרה מאפשר לדווח באופן מועיל על שמות הנעה כפולים.
     */
    public static void validateHardwareNames() {
        java.util.Set<String> names = new java.util.HashSet<>();
        for (String name : new String[] {FRONT_LEFT_NAME, FRONT_RIGHT_NAME, BACK_LEFT_NAME,
                BACK_RIGHT_NAME, PINPOINT_NAME}) {
            if (name == null || name.trim().isEmpty() || !names.add(name)) {
                throw new IllegalArgumentException("Drive and Pinpoint names must be nonempty and unique: " + name);
            }
        }
    }

    /** Configuration is read once per construction; reinitialize after editing it in Panels.
     * עברית: התצורה נקראת פעם אחת בכל בנייה; אתחלו מחדש לאחר עריכתה ב־Panels.
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        validateHardwareNames();
        MecanumConfig drive = new MecanumConfig(c -> {
            c.frontLeftName.set(FRONT_LEFT_NAME);
            c.backLeftName.set(BACK_LEFT_NAME);
            c.frontRightName.set(FRONT_RIGHT_NAME);
            c.backRightName.set(BACK_RIGHT_NAME);
            c.frontLeftDirection.set(LEFT_DIRECTION);
            c.backLeftDirection.set(LEFT_DIRECTION);
            c.frontRightDirection.set(RIGHT_DIRECTION);
            c.backRightDirection.set(RIGHT_DIRECTION);
        });
        PinpointConfig pinpoint = new PinpointConfig(c -> {
            c.name.set(PINPOINT_NAME);
            c.xPodOffset.set(POD_Y_DIST);
            c.yPodOffset.set(POD_X_DIST);
            c.offsetUnits.set(DistanceUnit.CM);
            c.globalDistanceUnit.set(DistanceUnit.INCH); // Existing field coordinates are inches.
            // עברית: קואורדינטות המגרש הקיימות הן באינצ׳ים.
            c.podType.set(POD_TYPE);
            c.xPodDirection.set(FORWARD_POD_DIRECTION);
            c.yPodDirection.set(STRAFE_POD_DIRECTION);
        });
        return new Follower(new PinpointLocalizer(hardwareMap, pinpoint),
                new Mecanum(hardwareMap, drive), new Foresight(createForesightConfig()));
    }
}
