package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class Helpers {
    public static final Pose GOAL_POSE_BLUE = new Pose(0, 144, 0);
    public static final Pose GOAL_POSE_RED = new Pose(144, 144, 0);

    public static double getAngleToBasket(Pose pose, boolean isBlue) {
        Pose goalPose = isBlue ? GOAL_POSE_BLUE : GOAL_POSE_RED;

        double angle = Math.atan2(
                goalPose.getY() - pose.getY(), // Y comes first!
                goalPose.getX() - pose.getX()  // X comes second
        );
        if (angle < 0){
            return angle + 2*Math.PI;
        }
        return angle;
    }


}
