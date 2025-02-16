// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class PathDestination {
    public final Pose2d pose;
    public final double liftHeight;
    public final double pivotAngle;

    public PathDestination(Pose2d pose, double liftHeight, double pivotAngle) {
        this.pose = pose;
        this.liftHeight = liftHeight;
        this.pivotAngle = pivotAngle;
    }
}
