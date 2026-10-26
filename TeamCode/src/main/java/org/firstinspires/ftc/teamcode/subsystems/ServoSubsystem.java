package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.*;
import java.util.function.DoubleSupplier;

/** Copy for a positional-servo mechanism; not a continuous-rotation servo.
 * Guide: doc/guide/05-hardware.md. Construction intentionally does not command a position.
 * עברית: העתיקו למנגנון סרוו מיקום, ולא לסרוו סיבוב רציף.
 * מדריך: doc/guide/he/05-hardware.md. הבנייה אינה שולחת בקשת מיקום במכוון.
 */
public class ServoSubsystem extends SubsystemBase {
    private final Servo servo;
    public ServoSubsystem(HardwareMap map, String name) { this(map.get(Servo.class, name)); }
    public ServoSubsystem(Servo servo) { this.servo = servo; }

    public void setPosition(double position) {
        if (!Double.isFinite(position) || position < 0 || position > 1) {
            throw new IllegalArgumentException("Servo position must be finite and between 0 and 1");
        }
        servo.setPosition(position);
    }

    public double position() { return servo.getPosition(); }

    public Command moveTo(DoubleSupplier position) {
        // Reads once at initialization; completion means request sent, not physical arrival.
        // עברית: נקרא פעם אחת באתחול; סיום מעיד על שליחת בקשה, לא על הגעה פיזית.
        return new InstantCommand(() -> setPosition(position.getAsDouble()), this);
    }
    // There is no universal servo 'stop' position. An OpMode ending does not reset the servo.
    // עברית: אין מיקום עצירה אוניברסלי לסרוו. סיום OpMode אינו מאפס את הסרוו.
}
