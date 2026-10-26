package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.FIELD_POSES;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

public class Helpers {
    public static final Pose GOAL_POSE_BLUE = FIELD_POSES.of(0, 144, 0);
    public static final Pose GOAL_POSE_RED = FIELD_POSES.of(144, 144, 0);

    public static double getAngleToBasket(Pose pose, boolean isBlue) {
        Pose goalPose = isBlue ? GOAL_POSE_BLUE : GOAL_POSE_RED;

        double angle = Math.atan2(
                goalPose.y() - pose.y(), // Y comes first!
                goalPose.x() - pose.x()  // X comes second
        );
        if (angle < 0){
            return angle + 2*Math.PI;
        }
        return angle;
    }

    public static Command TurnToPosCommand(Follower follower, Pose targetPose){
        return new CommandBase() {
            private TurnToCommand turn;

            @Override
            public void initialize() {
                Pose pose = follower.pose();
                turn = new TurnToCommand(follower, Math.atan2(
                        targetPose.y() - pose.y(), targetPose.x() - pose.x()));
                turn.initialize();
            }

            @Override
            public boolean isFinished() {
                return turn != null && turn.isFinished();
            }

            @Override
            public void end(boolean interrupted) {
                if (turn != null) turn.end(interrupted);
            }
        };
    }

}
