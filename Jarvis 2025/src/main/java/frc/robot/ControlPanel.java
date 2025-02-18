package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.JoystickDrive;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.Pivot;
import frc.robot.Util.UniversalCommandFactory;

public class ControlPanel {
    private static final Joystick controller = new Joystick(0);
    private static final Joystick controller2 = new Joystick(1);

    public static void configureBinding(Drivetrain drivetrain, Elevator elevator, Pivot pivot, EndEffector endEffector) {
        drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, controller));

        new JoystickButton(controller, 1).whileTrue(drivetrain.homeCommand());
        new JoystickButton(controller, 2).whileTrue(drivetrain.pathingCommand(drivetrain.getPose().plus(new Transform2d(1, 1, new Rotation2d())), 0));
        new JoystickButton(controller, 3).whileTrue(UniversalCommandFactory.reefCycle(drivetrain, elevator, pivot, endEffector));

        for (int i = 0; i < 16; i++) {
            new JoystickButton(controller2, i).onTrue(i < 4 ? ReefCycle.setHeight(i) : ReefCycle.setPosition(i));
        }
    }

    public static class ReefCycle {
        private static int targetPosition;
        private static int targetHeight;

        private static boolean depositing;
        private static Pose2d previousLocation;

        private static Command setPosition(int positionIndex) {
            return new SequentialCommandGroup(
                new InstantCommand(() -> targetPosition = positionIndex)
                //update display
            );
        }
    
        private static Command setHeight(int heightIndex) {
            return new SequentialCommandGroup(
                new InstantCommand(() -> targetHeight = heightIndex)
                //update display
            );
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
            return depositing ? Constants.NavigationConstants.REEF_LOCATIONS[targetPosition] : Constants.NavigationConstants.CORAL_STATION;
        }
    
        public static Pose2d getPreviousLocation() {
            return previousLocation;
        }
    
        public static double getHeight() {
            return depositing ? Constants.ElevatorConstants.LEVEL_HEIGHT[targetHeight] : 0;
        }
    
        public static Rotation2d getAngle() {
            return depositing ? Constants.PivotConstants.CORAL_DEPOSIT_ANGLES[targetHeight] : Constants.PivotConstants.CORAL_INTAKE_ANGLE;
        }
    }
}
