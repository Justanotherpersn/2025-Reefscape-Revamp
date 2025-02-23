package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.JoystickDrive;
import frc.robot.Subsystems.Drivetrain;

public class ControlPanel {
    private static final Joystick controller = new Joystick(0);
    private static final Joystick controller2 = new Joystick(1);

    private static ShuffleboardTab reefTab = Shuffleboard.getTab("Reef Tab");
    
    private static GenericEntry[] heightEntries = {
        reefTab.add("L0", false).withPosition(8, 3).getEntry(),
        reefTab.add("L1", false).withPosition(8, 2).getEntry(),
        reefTab.add("L2", false).withPosition(8, 1).getEntry(),
        reefTab.add("L3", false).withPosition(8, 0).getEntry(),
    };

    private static GenericEntry[] locationEntries = {
        reefTab.add("1L", false).withPosition(3, 4).getEntry(),
        reefTab.add("1R", false).withPosition(4, 4).getEntry(),
        reefTab.add("2L", false).withPosition(6, 4).getEntry(),
        reefTab.add("2R", false).withPosition(7, 3).getEntry(),
        reefTab.add("3L", false).withPosition(7, 1).getEntry(),
        reefTab.add("3R", false).withPosition(6, 0).getEntry(),
        reefTab.add("4L", false).withPosition(4, 0).getEntry(),
        reefTab.add("4R", false).withPosition(3, 0).getEntry(),
        reefTab.add("5L", false).withPosition(1, 0).getEntry(),
        reefTab.add("5R", false).withPosition(0, 1).getEntry(),
        reefTab.add("6L", false).withPosition(0, 3).getEntry(),
        reefTab.add("6R", false).withPosition(1, 4).getEntry()
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
            System.out.println("buttonID: " + buttonID + ", id: " + i);
            new JoystickButton(controller2, i).onTrue(buttonID < 4 ? ReefCycle.setHeight(buttonID) : ReefCycle.setPosition(buttonID - 4));
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
                System.out.println("Pos: " + (positionIndex + 4));
                for (GenericEntry locationEntry : locationEntries)
                    locationEntry.setBoolean(false);
                locationEntries[positionIndex].setBoolean(true);
                targetPosition = positionIndex;
            });
        }
    
        private static Command setHeight(int heightIndex) {
            return new InstantCommand(() -> {
                System.out.println(heightIndex);
                for (GenericEntry heightEntry : heightEntries)
                    heightEntry.setBoolean(false);
                heightEntries[heightIndex].setBoolean(true);
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
