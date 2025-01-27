// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    /** Width between robot wheels in meters */ 
    public static final double ROBOT_WHEEL_BASE = Units.inchesToMeters(21.5);
    public static final double MAX_DRIVE_SPEED = 1;
    public static final double MAX_ANGULAR_SPEED = 3;
    public static final double DRIVE_TOLERANCE_PERCENT = 0.03;

    public static class ModuleConstants{
        /** Overall max speed of the module in m/s */
        public static final double MAX_SPEED = 5;

        /** rotational offset in radians of modules during homing */
        public static final double[] MODULE_OFFSETS = new double[] {
            -Math.PI/4,
            Math.PI/4,
            Math.PI/4,
            -Math.PI/4
        };

        /** Gear Ratio of the drive motor */
        public static final double DRIVE_GEARING = 8;

        /**Gear ratio of the turning motor */
        public static final double TURN_GEARING = 2.89 * 2.89 * 6;

        /**Diameter of the billet wheel */
        public static final double WHEEL_DIA = 3.875;

    }

    public static class PhotonConstants {
        //Camera position relative to the bot to translate vision data to the center of the robot instead of based around the camera's reference
        public final static double CAM_PITCH = 15; //degrees

        public static final Transform3d[] CAMERAS_TO_ROBOT = {
            new Transform3d(
                new Translation3d(0, Units.inchesToMeters(5.5), -Units.inchesToMeters(11.75)), 
                new Rotation3d(Units.degreesToRadians(CAM_PITCH),0,0)
            ),
            new Transform3d(
                new Translation3d(0, Units.inchesToMeters(5.5), -Units.inchesToMeters(11.75)), 
                new Rotation3d(Units.degreesToRadians(CAM_PITCH),0,0)
            ),
            new Transform3d(
                new Translation3d(0, Units.inchesToMeters(5.5), -Units.inchesToMeters(11.75)), 
                new Rotation3d(Units.degreesToRadians(CAM_PITCH),0,0)
            )
        };

        public static final Transform3d[] ROBOT_TO_CAMERAS = {
            CAMERAS_TO_ROBOT[0].inverse(),
            CAMERAS_TO_ROBOT[1].inverse(),
            CAMERAS_TO_ROBOT[2].inverse()
        };

        //Index = ID - 1
        public static final Pose3d[] APRILTAG_LOCATIONS = {
            //Generated from a python script
            //JSON: https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2025-reefscape.json
            new Pose3d(16.697198, 0.65532, 1.4859, new Rotation3d(new Quaternion(0.4539904997395468, 0.0, 0.0, 0.8910065241883678))),
            new Pose3d(16.697198, 7.3964799999999995, 1.4859, new Rotation3d(new Quaternion(-0.45399049973954675, -0.0, 0.0, 0.8910065241883679))),
            new Pose3d(11.560809999999998, 8.05561, 1.30175, new Rotation3d(new Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476))),
            new Pose3d(9.276079999999999, 6.137656, 1.8679160000000001, new Rotation3d(new Quaternion(0.9659258262890683, 0.0, 0.25881904510252074, 0.0))),
            new Pose3d(9.276079999999999, 1.914906, 1.8679160000000001, new Rotation3d(new Quaternion(0.9659258262890683, 0.0, 0.25881904510252074, 0.0))),
            new Pose3d(13.474446, 3.3063179999999996, 0.308102, new Rotation3d(new Quaternion(-0.8660254037844387, -0.0, 0.0, 0.49999999999999994))),
            new Pose3d(13.890498, 4.0259, 0.308102, new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))),
            new Pose3d(13.474446, 4.745482, 0.308102, new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994))),
            new Pose3d(12.643358, 4.745482, 0.308102, new Rotation3d(new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386))),
            new Pose3d(12.227305999999999, 4.0259, 0.308102, new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0))),
            new Pose3d(12.643358, 3.3063179999999996, 0.308102, new Rotation3d(new Quaternion(-0.4999999999999998, -0.0, 0.0, 0.8660254037844387))),
            new Pose3d(0.851154, 0.65532, 1.4859, new Rotation3d(new Quaternion(0.8910065241883679, 0.0, 0.0, 0.45399049973954675))),
            new Pose3d(0.851154, 7.3964799999999995, 1.4859, new Rotation3d(new Quaternion(-0.8910065241883678, -0.0, 0.0, 0.45399049973954686))),
            new Pose3d(8.272272, 6.137656, 1.8679160000000001, new Rotation3d(new Quaternion(5.914589856893349e-17, -0.25881904510252074, 1.5848095757158825e-17, 0.9659258262890683))),
            new Pose3d(8.272272, 1.914906, 1.8679160000000001, new Rotation3d(new Quaternion(5.914589856893349e-17, -0.25881904510252074, 1.5848095757158825e-17, 0.9659258262890683))),
            new Pose3d(5.9875419999999995, -0.0038099999999999996, 1.30175, new Rotation3d(new Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476))),
            new Pose3d(4.073905999999999, 3.3063179999999996, 0.308102, new Rotation3d(new Quaternion(-0.4999999999999998, -0.0, 0.0, 0.8660254037844387))),
            new Pose3d(3.6576, 4.0259, 0.308102, new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0))),
            new Pose3d(4.073905999999999, 4.745482, 0.308102, new Rotation3d(new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386))),
            new Pose3d(4.904739999999999, 4.745482, 0.308102, new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994))),
            new Pose3d(5.321046, 4.0259, 0.308102, new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))),
            new Pose3d(4.904739999999999, 3.3063179999999996, 0.308102, new Rotation3d(new Quaternion(-0.8660254037844387, -0.0, 0.0, 0.49999999999999994))),
        };
    }
}
