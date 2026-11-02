package org.firstinspires.ftc.teamcode.examples;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.RobotCommandOpMode;

/** 07: Hardware-free Panels/configuration. Guide: doc/guide/08-panels-tuning.md.
 * Copy/change: class/name, fields and telemetry; remove @Disabled.
 * Edit REQUESTED in Panels: 'live' updates, 'snapshot' stays until reinitialize.
 * Check the same telemetry in Driver Station. Sloth hot loading is a separate workflow.
 */
@Disabled
@Configurable
@TeleOp(name = "07 Panels (no hardware)", group = "Examples")
public class Example07Panels extends RobotCommandOpMode {
    public static double REQUESTED = 0.2;
    private double initializedValue;
    private JoinedTelemetry joined;
    @Override public void initialize() {
        reset();
        initializedValue = REQUESTED;
        joined = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    }
    @Override public void run() {
        super.run();
        joined.addData("Live field", REQUESTED);
        joined.addData("Initialization snapshot", initializedValue);
        joined.update();
    }
}
