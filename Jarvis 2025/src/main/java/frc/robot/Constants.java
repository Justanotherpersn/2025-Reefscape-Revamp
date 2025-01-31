// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Util.Gains;

/** Add your docs here. */
public class Constants {
    public enum CAN_DEVICES {
        // These are assigned algorithmically in Drivetrain; do not use for anything else
        // DRIVE_FRONT_LEFT(1),
        // DRIVE_FRONT_RIGHT(2),
        // DRIVE_BACK_LEFT(3),
        // DRIVE_BACK_RIGHT(4),
        // TURN_FRONT_LEFT(5),
        // TURN_FRONT_RIGHT(6),
        // TURN_BACK_LEFT(7),
        // TURN_BACK_RIGHT(8),

        ROBORIO(0),
        ELEVATOR_MOTOR(9),
        PIGEON_2(20);

        public int id;
        private CAN_DEVICES(int id) {
            this.id = id;
        }
    }

    public static class GAINS {
        public static Gains DRIVE = new Gains(5, 0, 0.15, 2.65, 12);
        public static Gains TURN = new Gains(.6, 1);
        public static Gains ELEVATOR = new Gains(20, 0, 0, 0, 0, 12);
    }

    /** Width between robot wheels in meters */ 
    public static final double ROBOT_WHEEL_BASE = Units.inchesToMeters(21.5);
    public static final double MAX_DRIVE_SPEED = 1;
    public static final double MAX_ANGULAR_SPEED = 3;
    public static final double DRIVE_TOLERANCE_PERCENT = 0.05;
    public static final double MAX_POSE_TARGET_DISTANCE = 0.1;
    public static final PathConstraints NAV_PATHING_CONSTRAINTS = new PathConstraints(
        MAX_DRIVE_SPEED,
        10,
        MAX_ANGULAR_SPEED,
        10,
        12
    );

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

        public static final Translation2d[] MODULE_POSITIONS = {
            new Translation2d(Constants.ROBOT_WHEEL_BASE/2 , Constants.ROBOT_WHEEL_BASE/2),
            new Translation2d(Constants.ROBOT_WHEEL_BASE/2 , -Constants.ROBOT_WHEEL_BASE/2),
            new Translation2d(-Constants.ROBOT_WHEEL_BASE/2 , Constants.ROBOT_WHEEL_BASE/2),
            new Translation2d(-Constants.ROBOT_WHEEL_BASE/2 , -Constants.ROBOT_WHEEL_BASE/2)
        };
    }

    public static class ElevatorConstants{

        //Max extension of the elevator relative to itself in meters
        public static final double MAX_ELEVATOR_EXTENSION = Units.inchesToMeters(5);

        //Diameter of the elevator drive sprocket in meters
        public static final double SPROCKET_DIA = Units.inchesToMeters(5);

        public static final double SPROKET_CIRCUMFERENCE = (2 * Math.PI) * SPROCKET_DIA;

        //Gear ratio of the elevator drive motor
        public static final double ELEVATOR_GEARING = 1;

        //Distance from floor to bottom of elevator
        public static final double FLOOR_OFFSET = Units.inchesToMeters(1.25);

        //Offset of manipulator
        public static final double HEAD_OFFSET = Units.inchesToMeters(3);

        public static final double[] levelHeight = {
            Units.inchesToMeters(17.88) - FLOOR_OFFSET - HEAD_OFFSET, //L1
            Units.inchesToMeters(31.72) - FLOOR_OFFSET - HEAD_OFFSET, //L2
            Units.inchesToMeters(47.59) - FLOOR_OFFSET - HEAD_OFFSET, //L3
            Units.inchesToMeters(71.87) - FLOOR_OFFSET - HEAD_OFFSET  //L4
        };

        
    }

    public static class PhotonConstants {
        public final static double CAMS_PITCH = Math.toRadians(15);
        public final static double CAMS_HEIGHT = 0.1;

        public static final Transform3d[] ROBOT_TO_CAMERAS = {
            new Transform3d(
                new Translation3d(ModuleConstants.MODULE_POSITIONS[0].getX(), ModuleConstants.MODULE_POSITIONS[0].getY(), CAMS_HEIGHT), 
                new Rotation3d(CAMS_PITCH, 0, Math.PI / 4)
            ),
            new Transform3d(
                new Translation3d(ModuleConstants.MODULE_POSITIONS[1].getX(), ModuleConstants.MODULE_POSITIONS[1].getY(), CAMS_HEIGHT), 
                new Rotation3d(CAMS_PITCH, 0, 3 * Math.PI / 4)
            ),
            new Transform3d(
                new Translation3d(ModuleConstants.MODULE_POSITIONS[2].getX(), ModuleConstants.MODULE_POSITIONS[2].getY(), CAMS_HEIGHT), 
                new Rotation3d(CAMS_PITCH, 0, -3 * Math.PI / 4)
            ),
            new Transform3d(
                new Translation3d(ModuleConstants.MODULE_POSITIONS[3].getX(), ModuleConstants.MODULE_POSITIONS[3].getY(), CAMS_HEIGHT), 
                new Rotation3d(CAMS_PITCH, 0, -Math.PI / 4)
            )
        };

        public static final Transform3d[] CAMERAS_TO_ROBOT = {
            ROBOT_TO_CAMERAS[0].inverse(),
            ROBOT_TO_CAMERAS[1].inverse(),
            ROBOT_TO_CAMERAS[2].inverse(),
            ROBOT_TO_CAMERAS[3].inverse()
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
