package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.Tests;

/** Pedro 3 AutoTune factories, discovered automatically; this class is not a TeleOp.
 * Open {@code http://controller-address:10158} and follow browser/Driver Station prompts.
 * Save generated configurations in Constants.java; see doc/guide/08-panels-tuning.md.
 * מפעלי AutoTune של Pedro 3 מתגלים אוטומטית; המחלקה אינה TeleOp.
 * פתחו את כתובת הבקר בפורט 10158 ופעלו לפי ההנחיות בדפדפן וב־Driver Station.
 * שמרו הגדרות ב־Constants.java; ראו doc/guide/he/08-panels-tuning.md.
 */
public final class Tuning {
    private Tuning() { }

    @Tuner(name = "Mecanum Tuner")
    public static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner(name = "Pinpoint Tuner")
    public static Procedure pinpoint() {
        return new PinpointTuner();
    }

    @Tuner(name = "Foresight Tuner")
    public static Procedure foresight() {
        return new ForesightTuner(Constants::createLocalizer, Constants::createDrivetrain);
    }

    @Tuner(name = "Tests")
    public static Procedure tests() {
        return new Tests(Constants::createDrivetrain, Constants::createLocalizer, Constants::createAlgorithm);
    }
}
