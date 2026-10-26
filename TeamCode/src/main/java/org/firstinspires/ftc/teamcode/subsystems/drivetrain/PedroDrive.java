package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.MotorCommands;

/** Four mecanum motors + Pinpoint + compatible pods REQUIRED.
 * Guide: doc/guide/06-driving.md. Localizer updates happen only in periodic().
 * עברית: נדרשים ארבעה מנועי mecanum,‏ Pinpoint וגלגלי אודומטריה תואמים.
 * מדריך: doc/guide/he/06-driving.md. עדכוני מעריך המיקום מתבצעים רק ב־periodic().
 */
public class PedroDrive extends SubsystemBase {
    private final Follower follower;
    public PedroDrive(HardwareMap map, Pose start) { this(Constants.createFollower(map), start); }

    /** Accepts an existing follower and establishes its starting pose.
     * עברית: מקבל עוקב קיים וקובע את תנוחת ההתחלה שלו.
     */
    public PedroDrive(Follower follower, Pose start) {
        this.follower = follower;
        follower.setPose(start);
        follower.manual(DrivePowers.zero());
    }

    public static DrivePowers toRobotPowers(double forward, double left, double ccw,
                                            double heading, boolean fieldCentric, double speed) {
        double scale = Math.max(0, Math.min(1, speed));
        double angle = fieldCentric ? -heading : 0;
        return new DrivePowers((forward * Math.cos(angle) - left * Math.sin(angle)) * scale,
                (forward * Math.sin(angle) + left * Math.cos(angle)) * scale,
                Math.max(-1, Math.min(1, ccw)) * scale);
    }

    public Command drive(DoubleSupplier forward, DoubleSupplier left, DoubleSupplier ccw,
                         BooleanSupplier fieldCentric, DoubleSupplier speed) {
        return MotorCommands.run(() -> follower.manual(toRobotPowers(
                forward.getAsDouble(), left.getAsDouble(), ccw.getAsDouble(),
                follower.pose().heading(), fieldCentric.getAsBoolean(), speed.getAsDouble())),
                this::stop, () -> false, this);
    }
    public Pose getPose() { return follower.pose(); }
    public void resetOdometry(Pose pose) { follower.setPose(pose); }
    public Follower follower() { return follower; }
    @Override public void periodic() { follower.update(); }
    public void stop() {
        follower.stop();
        follower.drivetrain.stop(); // Zero immediately; don't wait for another update.
        // עברית: מאפס מיד, ללא המתנה לעדכון נוסף.
    }
    public void telemetryDebug(Telemetry telemetry) {
        telemetry.addData("Pose (in, rad)", follower.pose());
        telemetry.addData("Velocity", follower.velocity());
        telemetry.addData("Follower mode", follower.mode());
    }
}
