package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.RADIAN_POSES;

import static com.pedropathing.api.Paths.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.paths.Path;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Pedro 3 localization, characterization, and path tests. See doc/UPGRADE_REVIEW.md. */
@Configurable
@TeleOp(name = "Tuning", group = "Pedro Pathing")
public class Tuning extends OpMode {
    public static double TEST_POWER = 0.3;
    public static double TEST_DISTANCE = 24;
    public static double TURN_ANGLE = Math.PI / 2;
    private static final String[] MODES = {
            "Localization / offsets", "Forward velocity / coast", "Lateral velocity / coast",
            "Turn velocity / coast", "Line / translational / drive", "Triangle", "Circle / centripetal",
            "Heading hold", "Predictive braking"
    };
    private Follower follower;
    private JoinedTelemetry joined;
    private int selection;
    private boolean previousLeft, previousRight, previousA, previousB;
    private double maxForward, maxStrafe, maxTurn;
    private Pose releasePose;
    private Twist releaseVelocity;
    private long releaseTime;
    private boolean measuringCoast;
    private double coastDistance, coastAngle, coastSeconds;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(Pose.zero());
        ((com.pedropathing.revhub.drivetrains.Mecanum) follower.drivetrain).config.manualBrakeMode.set(false);
        follower.manual(0, 0, 0);
        joined = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    }

    @Override
    public void init_loop() {
        selectMode();
        showTelemetry();
    }

    private void selectMode() {
        if (gamepad1.dpad_right && !previousRight) selection = (selection + 1) % MODES.length;
        if (gamepad1.dpad_left && !previousLeft) selection = (selection + MODES.length - 1) % MODES.length;
        previousRight = gamepad1.dpad_right;
        previousLeft = gamepad1.dpad_left;
    }

    @Override
    public void loop() {
        follower.update();
        boolean aPressed = gamepad1.a && !previousA;
        boolean aReleased = !gamepad1.a && previousA;
        boolean bPressed = gamepad1.b && !previousB;
        double power = Math.max(-1, Math.min(1, TEST_POWER));
        if (gamepad1.b) {
            follower.stop();
            follower.drivetrain.stop();
            measuringCoast = false;
        } else if (selection == 0) {
            follower.manual(-gamepad1.left_stick_y * power,
                    -gamepad1.left_stick_x * power, -gamepad1.right_stick_x * power);
        } else if (selection <= 3) {
            if (gamepad1.a) {
                follower.manual(selection == 1 ? power : 0, selection == 2 ? power : 0,
                        selection == 3 ? power : 0);
            } else {
                // Zero manual powers use the drivetrain's float mode for coast measurements.
                follower.manual(0, 0, 0);
            }
            if (aReleased) {
                releasePose = follower.pose();
                releaseVelocity = follower.twist();
                releaseTime = System.nanoTime();
                measuringCoast = true;
            }
        } else if (aPressed) {
            measuringCoast = false;
            Pose start = follower.pose();
            if (selection == 7) {
                follower.hold(start.withHeading(start.heading() + TURN_ANGLE));
                follower.algorithm().reset();
            } else {
                follower.follow(testPath(selection, start));
            }
        }
        if (measuringCoast) {
            coastDistance = releasePose.distance(follower.pose());
            coastAngle = Math.abs(com.pedropathing.utils.Angle.normalizeSigned(
                    follower.pose().heading() - releasePose.heading()));
            coastSeconds = (System.nanoTime() - releaseTime) / 1e9;
            if (coastSeconds > 0.1 && follower.velocity().linSpeed() < 0.1
                    && Math.abs(follower.velocity().omega) < 0.01) measuringCoast = false;
        }
        Twist velocity = follower.twist();
        maxForward = Math.max(maxForward, Math.abs(velocity.vx));
        maxStrafe = Math.max(maxStrafe, Math.abs(velocity.vy));
        maxTurn = Math.max(maxTurn, Math.abs(velocity.omega));
        if (bPressed) {
            maxForward = maxStrafe = maxTurn = 0;
        }
        previousA = gamepad1.a;
        previousB = gamepad1.b;
        showTelemetry();
    }

    public static Path testPath(int mode, Pose start) {
        double d = TEST_DISTANCE;
        Pose end = start.plus(RADIAN_POSES.of(d, 0, 0));
        if (mode == 5) {
            Pose corner = start.plus(RADIAN_POSES.of(d / 2, d, 0));
            return path(line(start, end), line(end, corner), line(corner, start)).constant(start);
        }
        if (mode == 6) {
            // Four cubic Beziers approximate a circle, starting at the current pose.
            double k = 0.5522847498307936;
            double x = start.x(), y = start.y(), r = d / 2;
            return path(
                    curve(RADIAN_POSES.of(x,y, 0), RADIAN_POSES.of(x,y+k*r, 0), RADIAN_POSES.of(x+r-k*r,y+r, 0), RADIAN_POSES.of(x+r,y+r, 0)),
                    curve(RADIAN_POSES.of(x+r,y+r, 0), RADIAN_POSES.of(x+r+k*r,y+r, 0), RADIAN_POSES.of(x+2*r,y+k*r, 0), RADIAN_POSES.of(x+2*r,y, 0)),
                    curve(RADIAN_POSES.of(x+2*r,y, 0), RADIAN_POSES.of(x+2*r,y-k*r, 0), RADIAN_POSES.of(x+r+k*r,y-r, 0), RADIAN_POSES.of(x+r,y-r, 0)),
                    curve(RADIAN_POSES.of(x+r,y-r, 0), RADIAN_POSES.of(x+r-k*r,y-r, 0), RADIAN_POSES.of(x,y-k*r, 0), RADIAN_POSES.of(x,y, 0)))
                    .constant(start);
        }
        return line(start, end).constant(start);
    }

    private void showTelemetry() {
        joined.addData("Test (D-pad left/right during INIT)", MODES[selection]);
        joined.addData("Controls", "A: run/hold characterization; release A: coast; B: stop");
        joined.addData("Pose (inches/radians)", follower.pose());
        joined.addData("Body velocity", follower.twist());
        joined.addData("Max forward / strafe / turn", "%.3f / %.3f / %.3f", maxForward, maxStrafe, maxTurn);
        joined.addData("Coast distance / angle / seconds", "%.3f / %.3f / %.3f", coastDistance, coastAngle, coastSeconds);
        if (releaseVelocity != null && coastDistance > 0.01) {
            double v = Math.hypot(releaseVelocity.vx, releaseVelocity.vy);
            joined.addData("Natural deceleration estimate (v^2/2d)", v * v / (2 * coastDistance));
        }
        joined.addData("Follower", follower.mode());
        joined.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.stop();
            follower.drivetrain.stop();
        }
    }

    @Tuner(name = "Robot configuration checklist")
    public static Procedure configurationChecklist() {
        return new Procedure("Robot configuration checklist", "Review before characterization runs") {
            @Override
            public void run() throws InterruptedException {
                confirmation("Hardware", "Verify four unique motor names, Pinpoint offsets and directions.");
                result("Front right motor name", Constants.FRONT_RIGHT_NAME);
                result("Forward velocity estimate", Constants.FORWARD_VELOCITY);
                result("Strafe velocity estimate", Constants.STRAFE_VELOCITY);
                result("Next step", "Use the Tuning OpMode, then record measurements in Constants.java.");
            }
        };
    }
}
