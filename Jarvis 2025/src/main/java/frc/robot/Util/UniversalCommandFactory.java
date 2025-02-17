package frc.robot.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.ControlPanel;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.Pivot;

public class UniversalCommandFactory {
    public static final Command reefCycle(Drivetrain drivetrain, Elevator elevator, Pivot pivot, EndEffector endEffector) {
        return new RepeatCommand(
            new ParallelCommandGroup(
                drivetrain.pathingCommand(drivetrain.getReefCycleDestination(endEffector.coralPresent()), 0),

                new WaitUntilCommand(() ->
                    drivetrain.getPose().getTranslation().getDistance(drivetrain.getReefCycleDestination(!endEffector.coralPresent()).getTranslation()) 
                    > Constants.NavigationConstants.OPERATION_RADIUS)
                .andThen(
                    new ParallelCommandGroup(
                        elevator.elevatorHeight(0),
                        pivot.setAngleCommand(Rotation2d.fromDegrees(-90), false)
                    )
                ),

                new WaitUntilCommand(() -> true/*within certain radius of target location*/)
                .andThen(endEffector.coralPresent() ? ControlPanel.getReefAngle() : Constants.PivotConstants.CORAL_INTAKE_ANGLE, false)), //move pivot to target
                
                new WaitUntilCommand(() -> true/*within certain radius of target location*/).andThen(null) //move elevator to target
            ).andThen(endEffector.moveCoralCommand(endEffector.coralPresent()))
        );
    }
}