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
 * עברית: 07 — Panels והגדרות ללא חומרה. מדריך: doc/guide/he/08-panels-tuning.md.
 * להעתקה: שנו מחלקה, שם תצוגה, שדות וטלמטריה; הסירו @Disabled.
 * ערכו REQUESTED ב־Panels: הערך החי מתעדכן, ותמונת המצב נשארת עד אתחול מחדש.
 * בדקו אותה טלמטריה ב־Driver Station. טעינה חמה באמצעות Sloth היא תהליך נפרד.
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
