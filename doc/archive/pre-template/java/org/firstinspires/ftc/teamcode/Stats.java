package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

public abstract class Stats {
    public static Pose robotPose;

    public static Pose startingPose() {
        return robotPose == null ? Pose.zero() : robotPose;
    }
}
