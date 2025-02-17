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

    private static int targetReefPosition;
    private static int targetReefHeight;

    public static void configureBinding(Drivetrain drivetrain, Elevator elevator, Pivot pivot, EndEffector endEffector) {
        drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, controller));

        new JoystickButton(controller, 1).whileTrue(drivetrain.homeCommand());
        new JoystickButton(controller, 2).whileTrue(drivetrain.pathingCommand(drivetrain.getPose().plus(new Transform2d(1, 1, new Rotation2d())), 0));
        new JoystickButton(controller, 3).whileTrue(UniversalCommandFactory.reefCycle(drivetrain, elevator, pivot, endEffector));

        for (int i = 0; i < 16; i++) {
            new JoystickButton(controller2, i).onTrue(i < 4 ? setTargetReefHeight(i) : setTargetReefPosition(i));
        }
    }

    private static Command setTargetReefPosition(int positionIndex) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> targetReefPosition = positionIndex)
            //update display
        );
    }

    private static Command setTargetReefHeight(int heightIndex) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> targetReefHeight = heightIndex)
            //update display
        );
    }

    public static Pose2d getReefLocation() {
        return Constants.NavigationConstants.REEF_LOCATIONS[targetReefPosition];
    }

    public static double getReefHeight() {
        return Constants.ElevatorConstants.LEVEL_HEIGHT[targetReefHeight];
    }

    public static Rotation2d getReefAngle() {
        return Constants.PivotConstants.CORAL_DEPOSIT_ANGLES[targetReefHeight];
    }
}
