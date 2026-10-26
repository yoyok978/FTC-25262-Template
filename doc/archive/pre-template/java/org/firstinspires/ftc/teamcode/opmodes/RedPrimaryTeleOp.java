package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FIELD_POSES;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotCommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.Stats;
import org.firstinspires.ftc.teamcode.subsystems.RobotCommands;
import org.firstinspires.ftc.teamcode.subsystems.conv.Conv;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.PedroDrive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;

@Configurable
@TeleOp(name = "Unified Red TeleOp", group = "TeleOp")
public class RedPrimaryTeleOp extends RobotCommandOpMode {

    public static boolean IS_BLUE = false;
    public static boolean FIELD_CENTRIC = false; // Toggle this from Dashboard to switch modes
    public static double SHOOTER_SPEED = 794.0;

    private PedroDrive driveSubsystem;

    // State toggles
    private JoinedTelemetry joinedTelemetry;

    @Override
    public void initialize() {
        reset();
        joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        // --- 1. INITIALIZE HARDWARE & SUBSYSTEMS ---
        Pose startingPose = Stats.startingPose();

        driveSubsystem = new PedroDrive(hardwareMap, startingPose);
        Conv conv = new Conv(hardwareMap, "conv");
        Intake intake = new Intake(hardwareMap, "intake");
        Shooter shooter = new Shooter(hardwareMap, "shooterRight", "shooterLeft");

        RobotCommands robotCommands = new RobotCommands(conv, shooter, intake);


        // --- 2. INITIALIZE GAMEPADS ---
        GamepadEx driver = new GamepadEx(gamepad2);
        GamepadEx operator = new GamepadEx(gamepad1);

        // --- 3. DRIVER BINDINGS (Gamepad 2) ---

        // Toggles
        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> driveSubsystem.toggleGoalLock());

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> driveSubsystem.toggleReverse());

        // Reset Odometry
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> driveSubsystem.resetOdometry(FIELD_POSES.of(0, 0, 0)));

        // Speed Multipliers

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(() -> driveSubsystem.setSpeedMult(0.5))
                .whenReleased(() -> driveSubsystem.setSpeedMult(1.0));

        DoubleSupplier forward = () -> -driver.getLeftY();
        DoubleSupplier strafe = driver::getLeftX;
        DoubleSupplier turn = driver::getRightX;

        // Set the Default Drive Command
        driveSubsystem.setDefaultCommand(driveSubsystem.drive(forward, strafe, turn, () -> FIELD_CENTRIC, () -> IS_BLUE));


        // --- 4. OPERATOR BINDINGS (Gamepad 1) ---

        // Shooter Logic
        operator.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(robotCommands.stopShot(),shooter.spinUpCommand(SHOOTER_SPEED)); // Emergency stop

        // Load Logic (Dynamic power based on Right Bumper)
        operator.getGamepadButton(GamepadKeys.Button.A).whileHeld(
                robotCommands.load(() -> operator.getButton(GamepadKeys.Button.RIGHT_BUMPER) ? 0.12 : 0.08)
        );

        // Unload Logic
        operator.getGamepadButton(GamepadKeys.Button.A).whileHeld(
                new ParallelCommandGroup(
                    conv.unLoadCommand(),
                    intake.spinDownCommand(),
                    shooter.unloadShooterCommand(-0.07)
                )
        );
    }

    @Override
    public void end() {
        super.end();
        if (driveSubsystem != null) driveSubsystem.stop();
    }

    @Override
    public void run() {
        super.run(); // Executes Command Scheduler
        Stats.robotPose = driveSubsystem.getPose();

        // Telemetry
        joinedTelemetry.addData("Mode", FIELD_CENTRIC ? "Field Centric" : "Robot Centric");
        driveSubsystem.telemetryDebug(joinedTelemetry);
        joinedTelemetry.update();
    }
}
