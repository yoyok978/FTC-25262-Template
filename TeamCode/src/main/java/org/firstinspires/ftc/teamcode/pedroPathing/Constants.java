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

/** Shared Pedro 3 configurations. Replace each block with its measured AutoTune output.
 * הגדרות Pedro 3 משותפות. החליפו כל בלוק בפלט המדוד המתאים של AutoTune.
 */
@Configurable
public class Constants {
    // Field-authoring uses degrees; runtime calculations and stored Pose headings use radians.
    // כתיבת תנוחות מגרש במעלות; חישובי זמן ריצה והזוויות השמורות ב־Pose ברדיאנים.
    public static final PoseFactory FIELD_POSES = PoseFactory.degrees();
    public static final PoseFactory RADIAN_POSES = PoseFactory.radians();

    // Replace with Mecanum Tuner output. Names are preserved; verify each motor direction.
    // החליפו בפלט Mecanum Tuner. השמות נשמרו; אמתו בנפרד את כיוון כל מנוע.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // Replace with Pinpoint Tuner output. BOTH offset and global distance units are inches.
    // Zero offsets are unmeasured placeholders. Divide previous centimeter values by 2.54.
    // החליפו בפלט Pinpoint Tuner. יחידות ההיסטים והמרחק הגלובלי הן אינצ׳ים.
    // היסטי אפס הם ערכים זמניים שלא נמדדו. ערכים קודמים בסנטימטרים יש לחלק ב־2.54.
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odo");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(0.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // UNMEASURED Pedro 3 library defaults, not a tuned robot configuration.
    // Replace the entire declaration with Foresight Tuner's generated foresightConfig block.
    // Controller, Matrix and Vector2D imports support that generated block.
    // ברירות מחדל של Pedro 3 שלא נמדדו; זו אינה תצורה מכווננת לרובוט.
    // החליפו את ההצהרה כולה בבלוק foresightConfig שנוצר ב־Foresight Tuner.
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> { });

    /** Reject empty or duplicate configured names before hardware lookup.
     * דחיית שמות ריקים או כפולים לפני איתור חומרה.
     */
    public static void validateHardwareNames() {
        java.util.Set<String> names = new java.util.HashSet<>();
        for (String name : new String[] {drivetrainConfig.frontLeftName.get(),
                drivetrainConfig.frontRightName.get(), drivetrainConfig.backLeftName.get(),
                drivetrainConfig.backRightName.get(), localizerConfig.name.get()}) {
            if (name == null || name.trim().isEmpty() || !names.add(name)) {
                throw new IllegalArgumentException("Drive and Pinpoint names must be nonempty and unique: " + name);
            }
        }
    }

    /** New components per run; procedures and normal driving share the same configuration.
     * רכיבים חדשים לכל הרצה; הכוונון והנהיגה משתמשים באותן הגדרות.
     */
    public static Mecanum createDrivetrain(HardwareMap hardwareMap) {
        validateHardwareNames();
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static PinpointLocalizer createLocalizer(HardwareMap hardwareMap) {
        validateHardwareNames();
        return new PinpointLocalizer(hardwareMap, localizerConfig);
    }

    public static Foresight createAlgorithm() {
        return new Foresight(foresightConfig);
    }

    /** Reinitialize after configuration edits; never reuse a running follower for tuning.
     * אתחלו מחדש אחרי עריכת תצורה; אין להשתמש בעוקב פעיל מחדש לצורך כוונון.
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(createLocalizer(hardwareMap), createDrivetrain(hardwareMap), createAlgorithm());
    }
}
