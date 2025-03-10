// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
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
        END_EFFECTOR(10),
        PIVOT(11),
        CLIMBER(12),
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
        public static Gains END_EFFECTOR = new Gains(0.005, 0, 0, 0, 12);
        public static Gains PIVOT = new Gains(3, 0, 0, 0, 12);
        public static Gains CLIMBER = new Gains(100, 0, 0, 0, 12);
    }

    public static class DrivetrainConstants {
        public static final double WHEEL_BASE = Units.inchesToMeters(22.5);
        public static final double MAX_DRIVE_SPEED = 1;
        public static final double MAX_ANGULAR_SPEED = 3;
        public static final double DRIVE_TOLERANCE_PERCENT = 0.05;
    }

    public static class ModuleConstants{
        /** Overall max speed of the module in m/s */
        public static final double MAX_SPEED = DrivetrainConstants.MAX_DRIVE_SPEED;

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
        public static final double WHEEL_DIA = Units.inchesToMeters(3.875);

        public static final Translation2d[] MODULE_POSITIONS = {
            new Translation2d(DrivetrainConstants.WHEEL_BASE/2 , DrivetrainConstants.WHEEL_BASE/2),
            new Translation2d(DrivetrainConstants.WHEEL_BASE/2 , -DrivetrainConstants.WHEEL_BASE/2),
            new Translation2d(-DrivetrainConstants.WHEEL_BASE/2 , DrivetrainConstants.WHEEL_BASE/2),
            new Translation2d(-DrivetrainConstants.WHEEL_BASE/2 , -DrivetrainConstants.WHEEL_BASE/2)
        };
    }

    public static class ElevatorConstants{
        public static final double TOP_LIMIT_POSITION = Units.inchesToMeters(65.25);
        public static final double BOTTOM_LIMIT_POSITION = Units.inchesToMeters(39.25);

        public static final double MAX_ELEVATOR_EXTENSION = TOP_LIMIT_POSITION;
        public static final double MIN_ELEVATOR_EXTENSION = BOTTOM_LIMIT_POSITION;

        //Diameter of the elevator drive sprocket in meters
        public static final double SPROCKET_DIA = Units.inchesToMeters(1.5);
        public static final double SPROKET_CIRCUMFERENCE = Math.PI * SPROCKET_DIA;

        //Gear ratio of the elevator drive motor
        public static final double GEARING = 9 * 3;

        //Distance from floor to bottom of elevator
        public static final double FLOOR_OFFSET = Units.inchesToMeters(1.25);
        //Tolerence for elevator height
        public static final double SETPOINT_RANGE = .25;
        
        public static final double LINEAR_SPEED = 1;

        public static final double[] PRESET_HEIGHTS = {
            1,
            1.25,
            1.1,
            TOP_LIMIT_POSITION
        };

        public static final double CORAL_INTAKE_HEIGHT = 1.15;
    }

    public static class PivotConstants {
        public static final double GEARING = 90 * 5 / 1.5;
        /**The angle between the arm and a loaded coral, with positive x pointing radially outwards*/
        public static final Rotation2d END_MOUNT_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d POSITION_TOLERANCE = Rotation2d.fromDegrees(15);
        public static final Rotation2d CORAL_DEPOSIT_ANGLES[] = {
            Rotation2d.fromDegrees(-125),
            Rotation2d.fromDegrees(-115),
            Rotation2d.fromDegrees(-223),
            Rotation2d.fromDegrees(-223),
        };
        public static final Rotation2d CORAL_INTAKE_ANGLE = Rotation2d.fromDegrees(-40);
        public static final Rotation2d ANGULAR_SPEED = Rotation2d.fromDegrees(45);
        /**Length from pivot axis to center of coral when loaded*/
        public static final double LENGTH = 1;
    }

    public static class EndEffectorConstants {
        public static final double CORAL_PROTRUSION_LENGTH = 0.5;
        public static final Rotation2d ACCEPTABLE_SCORING_RANGE = Rotation2d.fromDegrees(45);
        public static final double GEARING = 5 * 3 * 3 * 1.5;
        public static final double GEARING_TO_PIVOT = 42.0 / 16.0;
        public static final double INTAKE_RPM = 200;
        public static final double OUTAKE_RPM = -200;
    }

    public static class ClimberConstants {
        public static final Rotation2d MAX_ROTATION = Rotation2d.fromDegrees(170);
        public static final Rotation2d MIN_ROTATION = Rotation2d.fromDegrees(10);
        public static final Rotation2d SETPOINT_RANGE = Rotation2d.fromDegrees(1);
        public static final double GEARING = 9 * 5 * 4 * 4;
    }

    public static class PhotonConstants {
        public final static double CAMS_PITCH = Math.toRadians(-31);
        public final static Translation3d CAM_OFFSET = new Translation3d(0.05, 0.05, 0.1);

        public static final Transform3d[] ROBOT_TO_CAMERAS = {
            new Transform3d(
                new Translation3d(ModuleConstants.MODULE_POSITIONS[0].getX() + CAM_OFFSET.getX(), ModuleConstants.MODULE_POSITIONS[0].getY() + CAM_OFFSET.getY(), CAM_OFFSET.getZ()), 
                new Rotation3d(0, CAMS_PITCH, Math.PI / 4)
            ),
            new Transform3d(
                new Translation3d(ModuleConstants.MODULE_POSITIONS[1].getX() + CAM_OFFSET.getX(), ModuleConstants.MODULE_POSITIONS[1].getY() - CAM_OFFSET.getY(), CAM_OFFSET.getZ()), 
                new Rotation3d(0, CAMS_PITCH, - Math.PI / 4)
            ),
            new Transform3d(
                new Translation3d(ModuleConstants.MODULE_POSITIONS[2].getX() - CAM_OFFSET.getX(), ModuleConstants.MODULE_POSITIONS[2].getY() + CAM_OFFSET.getY(), CAM_OFFSET.getZ()), 
                new Rotation3d(0, CAMS_PITCH, 3 * Math.PI / 4)
            ),
            new Transform3d(
                new Translation3d(ModuleConstants.MODULE_POSITIONS[3].getX() - CAM_OFFSET.getX(), ModuleConstants.MODULE_POSITIONS[3].getY() - CAM_OFFSET.getX(), CAM_OFFSET.getZ()), 
                new Rotation3d(0, CAMS_PITCH, -3 * Math.PI / 4)
            )
        };

        public static final Transform3d[] CAMERAS_TO_ROBOT = {
            ROBOT_TO_CAMERAS[0].inverse(),
            ROBOT_TO_CAMERAS[1].inverse(),
            ROBOT_TO_CAMERAS[2].inverse(),
            ROBOT_TO_CAMERAS[3].inverse()
        };

        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

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

    public static class NavigationConstants {
        public static final PathConstraints PATHING_CONSTRAINTS = new PathConstraints(
            DrivetrainConstants.MAX_DRIVE_SPEED,
            5,
            Math.toRadians(540),
            Math.toRadians(720),
            12
        );

        public static final double DESTINATION_TOLERANCE = 0.01;
        public static final double SECONDARY_DESTINATION_TOLERANCE = 0.005;

        public static final String[] REEF_LOCATIONS = {
            "1-L Lineup",
            "1-R Lineup",
            "2-L Lineup",
            "2-R Lineup",
            "3-L Lineup",
            "3-R Lineup",
            "4-L Lineup",
            "4-R Lineup",
            "5-L Lineup",
            "5-R Lineup",
            "6-L Lineup",
            "6-R Lineup",
        };
        public static final String CORAL_STATIONS[] = {
            "Lineup Left Station",
            "Lineup Right Station"
        };

        public static final double OPERATION_RADIUS = 0.5;
    }
}