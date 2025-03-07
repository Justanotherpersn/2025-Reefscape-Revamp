package frc.robot;

import java.util.Set;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.JoystickDrive;
import frc.robot.Commands.UniversalCommandFactory;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.Pivot;

public class ControlPanel {
    private static final Joystick controller = new Joystick(0);
    private static final Joystick controller2 = new Joystick(1);

    private static final NetworkTable nTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/Reef Locations");

    private static GenericEntry reefDisplay = nTable.getStringTopic("Reef Display").getGenericEntry();

    private static final int[] buttonLookup = {14, 0, 8, 13, 12, 2, 3, 11, 10, 9, 1, 15, 4, 5, 7, 6};

    private static Drivetrain drivetrain;

    public static void configureBinding(Drivetrain drivetrain, Elevator elevator, Pivot pivot, EndEffector endEffector, Climber climber) {
        drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, controller));

        new JoystickButton(controller, 1).whileTrue(drivetrain.homeCommand());

        //new JoystickButton(controller, 3).onTrue(elevator.homeCommand());

        new JoystickButton(controller, 2).whileTrue(new ParallelCommandGroup(
            UniversalCommandFactory.pivotAngleCommand(Rotation2d.fromDegrees(-90), false, pivot, endEffector),
            elevator.moveCommand(Constants.ElevatorConstants.MIN_ELEVATOR_EXTENSION)
        ));

        new JoystickButton(controller, 4).whileTrue(new DeferredCommand(() -> new ParallelCommandGroup(
            UniversalCommandFactory.pivotAngleCommand(ControlPanel.ReefCycle.getAngle(), true, pivot, endEffector),
            elevator.moveCommand(ControlPanel.ReefCycle.getHeight())
        ), Set.of(elevator, pivot)));

        new JoystickButton(controller, 3).whileTrue(new ParallelCommandGroup(
            UniversalCommandFactory.pivotAngleCommand(Constants.PivotConstants.CORAL_INTAKE_ANGLE, true, pivot, endEffector),
            elevator.moveCommand(Constants.ElevatorConstants.CORAL_INTAKE_HEIGHT)
        ));

        new JoystickButton(controller, 5).whileTrue(climber.climbCommand(Constants.ClimberConstants.MAX_ROTATION));
        new JoystickButton(controller, 6).whileTrue(climber.climbCommand(Constants.ClimberConstants.MIN_ROTATION)); 

        new Trigger(() -> controller.getRawAxis(3) > 0.5).whileTrue(endEffector.moveCoralCommand(true));
        new Trigger(() -> controller.getRawAxis(2) > 0.5).whileTrue(endEffector.moveCoralCommand(false));

        new JoystickButton(controller, 7).whileTrue(UniversalCommandFactory.reefCycle(drivetrain, elevator, pivot, endEffector));

        //new JoystickButton(controller, 5).whileTrue(UniversalCommandFactory.pivotAngleCommand(Rotation2d.fromDegrees(0), false, pivot, endEffector));
        //new JoystickButton(controller, 6).whileTrue(UniversalCommandFactory.pivotAngleCommand(Rotation2d.fromDegrees(-90), false, pivot, endEffector));
        //new JoystickButton(controller, 5).whileTrue(UniversalCommandFactory.reefCycle(drivetrain, elevator, pivot, endEffector));

        // new JoystickButton(controller, 3).whileTrue(UniversalCommandFactory.reefCycle(drivetrain, elevator, pivot, endEffector));
        // new JoystickButton(controller, 3).whileTrue(drivetrain.pathingCommand(new Pose2d(6, 6, Rotation2d.fromDegrees(90)), 0));

        for (int i = 0; i < 16; i++) {
            final int buttonID = buttonLookup[i];
            new JoystickButton(controller2, i + 1).onTrue(
                (buttonID < 4 ? ReefCycle.setHeight(buttonID) : ReefCycle.setPosition(buttonID - 4))
                    .andThen(() -> displayReefLocation(ReefCycle.targetPosition, ReefCycle.targetHeight))
            );
        }

        displayReefLocation(ReefCycle.targetPosition, ReefCycle.targetHeight);
        ControlPanel.drivetrain = drivetrain;
    }

    private static void displayReefLocation(int locationIndex, int height) {
        /*
              [ ] [ ]
          [ ]         [ ]
        [ ]             [ ]

        [ ]             [ ]
          [ ]         [ ]
              [ ] [ ]
        */
        String grid = """
        .            [%s] [%s]            .
        .    [%s]                 [%s]    .
        .[%s]                         [%s].

        .[%s]                         [%s].
        .    [%s]                 [%s]    .
        .            [%s] [%s]            .
        """;
        String[] format = new String[12];
        for (int i = 0; i < format.length; i++)
            format[i] = (i == locationIndex) ? Integer.toString(height) : " ";

        reefDisplay.setString(grid.formatted(
            format[7], format[6], format[8], format[5], format[9], format[4], format[10], format[3], format[11], format[2], format[0], format[1]
        ));
    }

    public static class ReefCycle {
        private static int targetPosition = 5;
        private static int targetHeight = 3;

        private static boolean depositing = true;
        private static Pose2d previousLocation;

        private static Command setPosition(int positionIndex) {
            return new InstantCommand(() -> targetPosition = positionIndex);
        }
    
        private static Command setHeight(int heightIndex) {
            return new InstantCommand(() -> targetHeight = heightIndex);
        }
    
        public static void setTravelState(boolean _depositing) {
            previousLocation = getLocation();
            depositing = _depositing;
        }

        public static boolean getTravelState() {
            return depositing;
        }
    
        public static Pose2d getLocation() {
            Pose2d target = depositing ? Constants.NavigationConstants.REEF_LOCATIONS[targetPosition] :  
                Constants.NavigationConstants.CORAL_STATIONS[
                    Math.abs(drivetrain.getPose().getY() - Constants.NavigationConstants.CORAL_STATIONS[0].getY()) 
                    < Math.abs(drivetrain.getPose().getY() - Constants.NavigationConstants.CORAL_STATIONS[1].getY()) ? 0 : 1
                ];
            if (DriverStation.getAlliance().get().equals(Alliance.Red)) target = FlippingUtil.flipFieldPose(target);
            if (target.getRotation().getDegrees() > 0) target = new Pose2d(target.getTranslation(), target.getRotation().plus(Rotation2d.fromDegrees(180)));
            return target;
        }
    
        public static Pose2d getPreviousLocation() {
            return previousLocation;
        }
    
        //TODO FIX
        public static double getHeight() {
            return depositing ? Constants.ElevatorConstants.PRESET_HEIGHTS[targetHeight] : Constants.ElevatorConstants.CORAL_INTAKE_HEIGHT;
        }
    
        public static Rotation2d getAngle() {
            return depositing ? Constants.PivotConstants.CORAL_DEPOSIT_ANGLES[targetHeight] : Constants.PivotConstants.CORAL_INTAKE_ANGLE;
        }
    }
}
