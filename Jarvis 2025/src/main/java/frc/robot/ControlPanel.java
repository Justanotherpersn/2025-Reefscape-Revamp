package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.JoystickDrive;
import frc.robot.Subsystems.Drivetrain;

public class ControlPanel {
    private static final Joystick controller = new Joystick(0);
    private static final Joystick controller2 = new Joystick(1);
    
    private static String[] heightEntries = {
        "L0", "L1", "L2", "L3"
    };

    private static String[] locationEntries = {
        "1L", "1R", "2L", "2R", "3L", "3R", "4L", "4R", "5L", "5R", "6L", "6R"
    };

    private static final int[] buttonLookup = {14, 0, 8, 13, 12, 2, 3, 11, 10, 9, 1, 15, 4, 5, 7, 6};

    private static Drivetrain drivetrain;

    public static void configureBinding(Drivetrain drivetrain/*, Elevator elevator, Pivot pivot, EndEffector endEffector*/) {
        drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, controller));

        new JoystickButton(controller, 1).whileTrue(drivetrain.homeCommand());
        new JoystickButton(controller, 2).whileTrue(drivetrain.pathingCommand(drivetrain.getPose().plus(new Transform2d(1, 1, new Rotation2d())), 0));
        // fnew JoystickButton(controller, 3).whileTrue(UniversalCommandFactory.reefCycle(drivetrain, elevator, pivot, endEffector));

        for (int i = 0; i < 16; i++) {
            final int buttonID = buttonLookup[i];
            SmartDashboard.putBoolean(buttonID < 4 ? heightEntries[buttonID] : locationEntries[buttonID - 4], false);
            new JoystickButton(controller2, i + 1).onTrue(buttonID < 4 ? ReefCycle.setHeight(buttonID) : ReefCycle.setPosition(buttonID - 4));
        }
        ControlPanel.drivetrain = drivetrain;
    }

    public static class ReefCycle {
        private static int targetPosition;
        private static int targetHeight;

        private static boolean depositing;
        private static Pose2d previousLocation;

        private static Command setPosition(int positionIndex) {
            return new InstantCommand(() -> {
                SmartDashboard.putBoolean(locationEntries[targetPosition], false);
                SmartDashboard.putBoolean(locationEntries[positionIndex], true);
                targetPosition = positionIndex;
            });
        }
    
        private static Command setHeight(int heightIndex) {
            return new InstantCommand(() -> {
                SmartDashboard.putBoolean(heightEntries[targetHeight], false);
                SmartDashboard.putBoolean(heightEntries[heightIndex], true);
                targetHeight = heightIndex;
            });
        }
    
        public static void setTravelState(boolean _depositing) {
            previousLocation = getLocation();
            depositing = _depositing;
            //update display?
        }

        public static boolean getTravelState() {
            return depositing;
        }
    
        public static Pose2d getLocation() {
            return depositing ? Constants.NavigationConstants.REEF_LOCATIONS[targetPosition] :  
                Constants.NavigationConstants.CORAL_STATIONS[
                    Math.abs(drivetrain.getPose().getY() - Constants.NavigationConstants.CORAL_STATIONS[0].getY()) 
                    < Math.abs(drivetrain.getPose().getY() - Constants.NavigationConstants.CORAL_STATIONS[1].getY()) ? 0 : 1
                ];
        }
    
        public static Pose2d getPreviousLocation() {
            return previousLocation;
        }
    
        //TODO FIX
        public static double getHeight() {
            return depositing ? targetHeight : 0;
        }
    
        public static Rotation2d getAngle() {
            return depositing ? Constants.PivotConstants.CORAL_DEPOSIT_ANGLES[targetHeight] : Constants.PivotConstants.CORAL_INTAKE_ANGLE;
        }
    }
}
