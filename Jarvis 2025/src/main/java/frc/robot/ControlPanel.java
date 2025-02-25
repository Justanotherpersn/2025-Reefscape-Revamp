package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.JoystickDrive;
import frc.robot.Subsystems.Drivetrain;

public class ControlPanel {
    private static final Joystick controller = new Joystick(0);
    private static final Joystick controller2 = new Joystick(1);

    private static final NetworkTable nTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/Reef Locations");
    
    private static GenericEntry[] heightEntries = {
        nTable.getTopic("L0").getGenericEntry(), 
        nTable.getTopic("L1").getGenericEntry(),
        nTable.getTopic("L2").getGenericEntry(),
        nTable.getTopic("L3").getGenericEntry()
    };

    private static GenericEntry[] locationEntries = {
        nTable.getTopic("1L").getGenericEntry(), 
        nTable.getTopic("1R").getGenericEntry(),
        nTable.getTopic("2L").getGenericEntry(),
        nTable.getTopic("2R").getGenericEntry(),
        nTable.getTopic("3L").getGenericEntry(), 
        nTable.getTopic("3R").getGenericEntry(),
        nTable.getTopic("4L").getGenericEntry(),
        nTable.getTopic("4R").getGenericEntry(),
        nTable.getTopic("5L").getGenericEntry(), 
        nTable.getTopic("5R").getGenericEntry(),
        nTable.getTopic("6L").getGenericEntry(),
        nTable.getTopic("6R").getGenericEntry()
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
            if (buttonID < 4) heightEntries[buttonID].setBoolean(false);
            else locationEntries[buttonID - 4].setBoolean(false);
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
                locationEntries[targetPosition].setBoolean(false);
                locationEntries[positionIndex].setBoolean(true);
                targetPosition = positionIndex;
            });
        }
    
        private static Command setHeight(int heightIndex) {
            return new InstantCommand(() -> {
                heightEntries[targetHeight].setBoolean(false);
                heightEntries[heightIndex].setBoolean(true);
                targetHeight = heightIndex;
            });
        }
    
        public static void setTravelState(boolean _depositing) {
            previousLocation = getLocation();
            depositing = _depositing;
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
