package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.*;
import java.util.function.DoubleSupplier;

/** Copy for an intake or conveyor. Guide: doc/guide/05-hardware.md.
 * Uses SDK open-loop power, not velocity control. Supply your actual hardware name/direction.
 * עברית: העתיקו למנגנון איסוף או למסוע. מדריך: doc/guide/he/05-hardware.md.
 * משתמש בהספק חוג פתוח של SDK ולא בבקרת מהירות. ספקו שם חומרה וכיוון אמיתיים.
 */
public class MotorSubsystem extends SubsystemBase {
    private final DcMotorEx motor;

    public MotorSubsystem(HardwareMap map, String name, DcMotorSimple.Direction direction) {
        this(map.get(DcMotorEx.class, name), direction);
    }

    /** Accepts an existing motor without looking it up in HardwareMap.
     * עברית: מקבל מנוע קיים ללא איתור ב־HardwareMap.
     */
    public MotorSubsystem(DcMotorEx motor, DcMotorSimple.Direction direction) {
        this.motor = motor;
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stop();
        // SubsystemBase registers automatically. The default command must never finish.
        // עברית: SubsystemBase נרשמת אוטומטית. פקודת ברירת המחדל חייבת לא להסתיים לעולם.
        setDefaultCommand(defaultStopCommand());
    }

    public void setPower(double power) {
        if (!Double.isFinite(power)) throw new IllegalArgumentException("Motor power must be finite");
        motor.setPower(Math.max(-1, Math.min(1, power)));
    }

    public void stop() { motor.setPower(0); }
    public double velocity() { return motor.getVelocity(); } // Encoder ticks per second.
    // עברית: המהירות בטיקים של מקודד לשנייה.

    public Command run(DoubleSupplier power) {
        // The supplier is evaluated each loop, so dashboard/trigger changes are live.
        // עברית: הספק נקרא בכל לולאה, ולכן שינויי לוח בקרה או הדק נקראים בזמן אמת.
        return MotorCommands.run(() -> setPower(power.getAsDouble()), this::stop, () -> false, this);
    }

    public Command stopCommand() { return new InstantCommand(this::stop, this); }
    public Command defaultStopCommand() { return new RunCommand(this::stop, this); }
}
