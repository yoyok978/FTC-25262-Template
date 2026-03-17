package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Configurable
public class Shooter extends SubsystemBase {

    public static double DEFAULT_SPINUP_VELOCITY = 794.0;
    public static double VELOCITY_TOLERANCE = 2.0;

    // Motor configuration flags
    public static boolean rightIsReversed = false; // Original was FORWARD
    public static boolean leftIsReversed = true;   // Original was REVERSE

    private final MotorEx rightMotor;
    private final MotorEx leftMotor;

    public Shooter(final HardwareMap hMap, final String rightName, final String leftName) {
        // Instantiate using SolversLib MotorEx wrappers (Placeholder RPM depending on your actual shooter motors)
        rightMotor = new MotorEx(hMap, rightName, Motor.GoBILDA.BARE);
        leftMotor = new MotorEx(hMap, leftName, Motor.GoBILDA.BARE);

        rightMotor.setRunMode(Motor.RunMode.VelocityControl); // Corresponds to RUN_USING_ENCODER
        leftMotor.setRunMode(Motor.RunMode.VelocityControl);

        rightMotor.setInverted(rightIsReversed);
        leftMotor.setInverted(leftIsReversed);

        // Grab underlying SDK hardware for custom PIDF and raw TPS velocity control
        DcMotorEx rawRight = rightMotor.motorEx;
        DcMotorEx rawLeft = leftMotor.motorEx;

        // Apply custom PIDF coefficients from the original RoadRunner code
        PIDFCoefficients coefficients = new PIDFCoefficients(50.0, 0.0, 5.0, 16.0);
        rawRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);
        rawLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);

        register();
        setDefaultCommand(defaultStopCommand());
    }

    // ==========================================
    // ATOMIC ACTIONS (Raw hardware control)
    // ==========================================

    public void setVelocity(double velTPS) {
        rightMotor.setVelocity(velTPS);
        leftMotor.setVelocity(velTPS);
    }

    public void setPower(double power) {
        rightMotor.set(power);
        leftMotor.set(power);
    }

    public void stop() {
        rightMotor.set(0);
        leftMotor.set(0);
    }

    public boolean isOverCurrent() {
        return rightMotor.isOverCurrent() || leftMotor.isOverCurrent();
    }

    public boolean isAtTargetVelocity(double targetVelocity) {
        double velLeft = rightMotor.getVelocity();
        double velRight = leftMotor.getVelocity();

        // Checks if both motors have reached the target velocity (accounting for tolerance)
        return velLeft >= (targetVelocity - VELOCITY_TOLERANCE) &&
                velRight >= (targetVelocity - VELOCITY_TOLERANCE);
    }

    // ==========================================
    // COMMAND FACTORIES
    // ==========================================

    /**
     * Spins the shooter up to a specified velocity in TPS.
     * Interrupts automatically and stops if an over-current spike is detected.
     */
    public Command spinUpCommand(double velTPS) {
        return new RunCommand(() -> setVelocity(velTPS), this)
                .interruptOn(this::isOverCurrent);
    }

    /**
     * Spins the shooter up to the default configuration velocity.
     */
    public Command spinUpCommand() {
        return spinUpCommand(DEFAULT_SPINUP_VELOCITY);
    }

    /**
     * Unloads the shooter with a specific power fraction.
     */
    public Command unloadShooterCommand(double power) {
        return new RunCommand(() -> setPower(power), this)
                .interruptOn(this::isOverCurrent);
    }

    /**
     * Safety command to keep the shooter stopped when not in active use.
     */
    public Command defaultStopCommand() {
        return new RunCommand(this::stop, this);
    }
}