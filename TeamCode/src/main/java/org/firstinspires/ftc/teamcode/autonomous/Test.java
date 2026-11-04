package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayDeque;
import java.util.Deque;


@Configurable
@Autonomous(name = "Test", group = "Auto")
public class Test extends CommandOpMode {

    private JoinedTelemetry joinedTelemetry; // Panels Telemetry instance
    private Follower follower;
    private FieldManager field;
    private static final int PATH_SEGMENTS = 100;
    private static final int MAX_TRAIL_POINTS = 1000;
    private static final double TRAIL_SPACING = 0.5;
    private static final double ROBOT_RADIUS = 9.0;
    private final Pose[] plannedPath = new Pose[PATH_SEGMENTS + 1];
    private final Deque<Pose> trail = new ArrayDeque<>();

    @Override
    public void initialize() {
        super.reset();

        joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        joinedTelemetry.addData("Status", "Initialized");
        joinedTelemetry.update();

        follower = Constants.create(hardwareMap);
        follower.setPose(RobotPath.start);
        Path path = RobotPath.path1();
        for (int i = 0; i <= PATH_SEGMENTS; i++) {
            plannedPath[i] = path.get((double) i / PATH_SEGMENTS);
        }
        trail.clear();
        trail.addLast(follower.pose());
        field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        drawField(follower.pose());


        Command fullAutoRoutine = new SequentialCommandGroup(
                new FollowPathCommand(follower, path)
        );


        schedule(fullAutoRoutine);

    }

    @Override
    public void run() {
        follower.update();

        // Must call super.run() so the CommandScheduler executes scheduled commands
        super.run();

        Pose pose = follower.pose();
        if (trail.isEmpty() || trail.peekLast().distance(pose) >= TRAIL_SPACING) {
            trail.addLast(pose);
            if (trail.size() > MAX_TRAIL_POINTS) {
                trail.removeFirst();
            }
        }
        drawField(pose);

        //Telemetry
        joinedTelemetry.addData("X", follower.pose().x());
        joinedTelemetry.addData("Y", follower.pose().y());
        joinedTelemetry.addData("Heading", follower.pose().heading());

        joinedTelemetry.update();

    }



    private void drawField(Pose pose) {
        // Panels clears its canvas after update, so redraw the complete scene.
        field.setStyle("transparent", "blue", 0.5);
        for (int i = 1; i < plannedPath.length; i++) {
            drawLine(plannedPath[i - 1], plannedPath[i]);
        }

        field.setStyle("transparent", "orange", 0.75);
        Pose previous = null;
        for (Pose point : trail) {
            if (previous != null) {
                drawLine(previous, point);
            }
            previous = point;
        }
        if (previous != null) {
            drawLine(previous, pose);
        }

        field.setStyle("transparent", "green", 1.0);
        field.moveCursor(pose.x(), pose.y());
        field.circle(ROBOT_RADIUS);
        field.line(pose.x() + ROBOT_RADIUS * Math.cos(pose.heading()),
                pose.y() + ROBOT_RADIUS * Math.sin(pose.heading()));
        field.update();
    }

    private void drawLine(Pose from, Pose to) {
        field.moveCursor(from.x(), from.y());
        field.line(to.x(), to.y());
    }

    public static class RobotPath {

        private static final PoseFactory poseFactory = PoseFactory.degrees();

        private static final Pose start = poseFactory.of(22.7996, 21.9662, 90);
        private static final Pose path1 = poseFactory.of(22.9221, 22.1563, 180);
        private static final Pose path1Control1 = poseFactory.of(83.2088, 9.5559, 0);
        private static final Pose path1Control2 = poseFactory.of(123.8529, 141.5, 0);
        private static final Pose path1Control3 = poseFactory.of(7.8674, 141.4534, 0);
        private static final Pose path1Control4 = poseFactory.of(22.8741, 115.3687, 0);

        public static Path path1() {
            return Paths.curve(start, path1Control1, path1Control2, path1Control3, path1Control4, path1).tangent();
        }
    }


}
