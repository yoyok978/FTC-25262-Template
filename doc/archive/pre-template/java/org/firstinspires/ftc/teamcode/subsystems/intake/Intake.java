package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.MotorCommands;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Configurable
public class Intake extends SubsystemBase {
    public static boolean isReversed = false;

    private final MotorEx intakeMotor;


    public Intake(final HardwareMap hMap, final String name){
        intakeMotor = new MotorEx(hMap, name, Motor.GoBILDA.RPM_312);
        intakeMotor.setRunMode(Motor.RunMode.VelocityControl);
        intakeMotor.setInverted(isReversed);

        register();
        setDefaultCommand(defaultStopCommand());
    }

    public void spinDown() {
        intakeMotor.set(-0.2);
    }

    public void spinDown(double power) {
        intakeMotor.set(-power);
    }
    public void spinUp() {
        intakeMotor.set(1);
    }

    public void spinUp(double power) {
        intakeMotor.set(power);
    }

    public void stop(){
        intakeMotor.stopMotor();
    }

    public double getVelocity() {
        return intakeMotor.getCorrectedVelocity();
    }

    public boolean isOverCurrent() {
        return intakeMotor.isOverCurrent();
    }
    
    // ==========================================
    // COMMAND FACTORIES
    // ==========================================

    // Motor commands run until canceled or a fault is detected.
    public Command spinUpCommand() {
        return MotorCommands.run(this::spinUp, this::stop, this::isOverCurrent, this);
    }

    public Command spinDownCommand() {
        return MotorCommands.run(this::spinDown, this::stop, this::isOverCurrent, this);
    }

    // RunCommands repeatedly run every loop cycle until interrupted.
    // This is perfect for a default command that actively ensures the motor stays off.
    public Command stopCommand() {
        return new InstantCommand(this::stop, this);
    }

    public Command defaultStopCommand() {
        return new RunCommand(this::stop, this);
    }
}
