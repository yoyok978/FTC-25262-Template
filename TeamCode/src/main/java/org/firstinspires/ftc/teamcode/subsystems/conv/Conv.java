package org.firstinspires.ftc.teamcode.subsystems.conv;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Configurable
public class Conv extends SubsystemBase {
    // Set to true to match RoadRunner's original setting of DcMotorEx.Direction.REVERSE
    public static boolean isReversed = true;

    private final MotorEx convMotor;

    public Conv(final HardwareMap hMap, final String name) {
        convMotor = new MotorEx(hMap, name, Motor.GoBILDA.RPM_312);

        // The RR code used RUN_USING_ENCODER, which corresponds to VelocityControl in SolversLib
        convMotor.setRunMode(Motor.RunMode.VelocityControl);
        convMotor.setInverted(isReversed);

        register();
        setDefaultCommand(defaultStopCommand());


    }

    // ==========================================
    // ATOMIC ACTIONS (Raw hardware control)
    // ==========================================

    public void setPower(double power) {
        convMotor.set(power);
    }

    public void stop() {
        convMotor.set(0);
    }

    public double getVelocity() {
        return convMotor.getCorrectedVelocity();
    }

    public double getCurrentPosition() {
        return convMotor.getCurrentPosition();
    }

    public boolean isOverCurrent() {
        return convMotor.isOverCurrent();
    }

    // ==========================================
    // COMMAND FACTORIES
    // ==========================================

    public Command loadCommand(double power) {
        return new RunCommand(() -> setPower(power), this)
                .interruptOn(this::isOverCurrent);
    }

    public Command unLoadCommand() {
        return new RunCommand(() -> setPower(-1.0), this)
                .interruptOn(this::isOverCurrent);
    }

    public Command prepareToShootCommand(int targetPosition) {
        return new RunCommand(() -> setPower(-0.3), this)
                .interruptOn(() -> getCurrentPosition() < (targetPosition - 20) || isOverCurrent());
    }

    public Command defaultStopCommand() {
        return new RunCommand(this::stop, this);
    }
}