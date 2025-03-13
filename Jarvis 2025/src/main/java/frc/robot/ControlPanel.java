package frc.robot;

import java.io.IOException;
import java.util.EnumSet;
import java.util.Set;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.JoystickDrive;
import frc.robot.Commands.Notifications;
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

    private static GenericEntry reefDisplay = nTable.getTopic("Reef Display").getGenericEntry();
    private static GenericEntry targetHeightEntry = nTable.getTopic("Level").getGenericEntry();
    private static GenericEntry targetPositionEntry = nTable.getTopic("Position").getGenericEntry();

    private static final int[] buttonLookup = {14, 0, 8, 13, 12, 2, 3, 11, 10, 9, 1, 15, 4, 5, 7, 6};

    private static Drivetrain drivetrain;

    public static void configureBinding(Drivetrain drivetrain, Elevator elevator, Pivot pivot, EndEffector endEffector, Climber climber) {
        drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, controller));
        ControlPanel.drivetrain = drivetrain;

        new JoystickButton(controller, 7).whileTrue(drivetrain.homeCommand());
        new JoystickButton(controller, 8).onTrue(new InstantCommand(() -> drivetrain.resetIMU()));

        //A: Go to selected height
        new JoystickButton(controller, 1).whileTrue(new DeferredCommand(() -> new ParallelCommandGroup(
            UniversalCommandFactory.pivotAngleCommand(ControlPanel.ReefCycle.getAngle(), pivot, endEffector),
            elevator.moveCommand(ControlPanel.ReefCycle.getHeight())
        ), Set.of(elevator, pivot)));

        //B: Height travel
        new JoystickButton(controller, 2).whileTrue(new ParallelCommandGroup(
            UniversalCommandFactory.pivotAngleCommand(Constants.PivotConstants.TRAVEL_POSITION, pivot, endEffector),
            elevator.moveCommand(Constants.ElevatorConstants.MIN_ELEVATOR_EXTENSION)
        ));

        //Y: Reef cycle
        new JoystickButton(controller, 4).whileTrue(UniversalCommandFactory.reefCycle(drivetrain, elevator, pivot, endEffector));

        //X: Intake height
        new JoystickButton(controller, 3).whileTrue(new ParallelCommandGroup(
            UniversalCommandFactory.pivotAngleCommand(Constants.PivotConstants.CORAL_INTAKE_ANGLE, pivot, endEffector),
            elevator.moveCommand(Constants.ElevatorConstants.CORAL_INTAKE_HEIGHT),
            endEffector.moveCoralCommand(true)
        ));

        //LB: Climb out
        new JoystickButton(controller, 5).whileTrue(climber.climbCommand(Constants.ClimberConstants.MIN_ROTATION));
        //RB: Climb in
        new JoystickButton(controller, 6).whileTrue(climber.climbCommand(Constants.ClimberConstants.MAX_ROTATION)); 

        //R Trigger: intake
        new Trigger(() -> controller.getRawAxis(3) > 0.5).whileTrue(endEffector.moveCoralCommand(true));
        //L Trigger: reverse intake
        new Trigger(() -> controller.getRawAxis(2) > 0.5).whileTrue(endEffector.moveCoralCommand(false));

        //Control panel height and position selector
        for (int i = 0; i < 16; i++) {
            final int buttonID = buttonLookup[i];
            new JoystickButton(controller2, i + 1).onTrue(
                (buttonID < 4 ? ReefCycle.setHeight(buttonID) : ReefCycle.setPosition(buttonID - 4))
                    .andThen(() -> ReefCycle.updateReefDisplay())
            );
        }

        ReefCycle.updateReefDisplay();
        
        targetHeightEntry.setInteger(ReefCycle.targetHeight);
        targetPositionEntry.setInteger(ReefCycle.targetPosition);

        NetworkTableListener.createListener(targetHeightEntry.getTopic(), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            int value = (int)event.valueData.value.getInteger();
            if (value >= 0 && value <= 3) ReefCycle.targetHeight = value;
            else Notifications.CONTROL_INVALID_INDEX.sendImmediate(value, ReefCycle.targetHeight);
            ReefCycle.updateReefDisplay();
        });
        
        NetworkTableListener.createListener(targetPositionEntry.getTopic(), EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            int value = (int)event.valueData.value.getInteger();
            if (value >= 0 && value <= 11) ReefCycle.targetPosition = value;
            else Notifications.CONTROL_INVALID_INDEX.sendImmediate(value, ReefCycle.targetPosition);
            ReefCycle.updateReefDisplay();
        });
    }

    public static class ReefCycle {
        public static int targetHeight = 3;
        public static int targetPosition = 0;
        private static boolean depositing = true;
        private static boolean mutatePath, pathEnd;
        private static Pose2d previousLocation;

        private static Command setPosition(int positionIndex) {
            return new InstantCommand(() -> {
                targetPosition = positionIndex;
                if (depositing) mutatePath = true;
            });
        }
    
        private static Command setHeight(int heightIndex) {
            return new InstantCommand(() -> {
                targetHeight = heightIndex;
                if (depositing) mutatePath = true;
            });
        }

        public static void setTravelState(boolean _depositing) {
            depositing = _depositing;
        }

        public static boolean getTravelState() {
            return depositing;
        }
    
        public static PathPlannerPath getLineupPath() {
            PathPlannerPath target = null;
            try {
                target = PathPlannerPath.fromPathFile(depositing ? Constants.NavigationConstants.REEF_LOCATIONS[targetPosition] :  
                    Constants.NavigationConstants.CORAL_STATIONS[
                        Math.abs(drivetrain.getPose().getY() - 1.028) 
                        < Math.abs(drivetrain.getPose().getY() - 7.000) ? 1 : 0
                    ]);
            } catch (FileVersionException | IOException | ParseException e) {
                e.printStackTrace();
            }
            if (!RobotContainer.isBlue()) target.flipPath();
            return target;
        }
    
        public static Pose2d getPreviousLocation() {
            return previousLocation;
        }
    
        public static double getHeight() {
            return depositing ? Constants.ElevatorConstants.PRESET_HEIGHTS[targetHeight] : Constants.ElevatorConstants.CORAL_INTAKE_HEIGHT;
        }
    
        public static Rotation2d getAngle() {
            return depositing ? Constants.PivotConstants.CORAL_DEPOSIT_ANGLES[targetHeight] : Constants.PivotConstants.CORAL_INTAKE_ANGLE;
        }

        private static void updateReefDisplay() {
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
                format[i] = (i == ReefCycle.targetPosition) ? Integer.toString(ReefCycle.targetHeight) : " ";
    
            reefDisplay.setString(grid.formatted(
                format[7], format[6], format[8], format[5], format[9], format[4], format[10], format[3], format[11], format[2], format[0], format[1]
            ));
        }
        
        /**Pathfind to the next location, updating according to inputs from the control panel
         */
        public static Command mutatingPathCommand() {
            return new SequentialCommandGroup(
                new InstantCommand(() -> mutatePath = pathEnd = false),
                new ParallelDeadlineGroup(
                    new WaitUntilCommand(() -> pathEnd), 
                    new ParallelRaceGroup(
                        new SequentialCommandGroup(
                            new WaitUntilCommand(() -> mutatePath && !pathEnd),
                            new InstantCommand(() -> mutatePath = false)
                        ),
                        new SequentialCommandGroup(
                            new DeferredCommand(() -> drivetrain.pathingCommand(ControlPanel.ReefCycle.getLineupPath()), Set.of(drivetrain)),
                            new InstantCommand(() -> pathEnd = true)
                        )
                    ).repeatedly()
                )
            );
        }
    }
}
