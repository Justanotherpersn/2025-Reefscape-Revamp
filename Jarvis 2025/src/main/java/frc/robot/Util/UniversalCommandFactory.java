package frc.robot.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
            new SequentialCommandGroup(
                new InstantCommand(() -> ControlPanel.ReefCycle.setTravelState(endEffector.coralPresent())),
                new ParallelCommandGroup(
                    drivetrain.pathingCommand(ControlPanel.ReefCycle.getLocation(), 0),

                    new WaitUntilCommand(() ->
                        drivetrain.getPose().getTranslation().getDistance(ControlPanel.ReefCycle.getPreviousLocation().getTranslation())
                        > Constants.NavigationConstants.OPERATION_RADIUS
                    ).andThen(
                        new ParallelCommandGroup(
                            elevator.elevatorHeight(0),
                            pivot.setAngleCommand(Rotation2d.fromDegrees(-90), false)
                        )
                    ),

                    //Potentially add time to traverse operation radius?
                    new WaitUntilCommand(() -> pivot.timeToReach(ControlPanel.ReefCycle.getAngle()) < drivetrain.timeToReach(ControlPanel.ReefCycle.getLocation()))
                    .andThen(pivot.setAngleCommand(ControlPanel.ReefCycle.getAngle(), true)),
                    
                    new WaitUntilCommand(() -> true/*within certain radius of target location*/)
                    .andThen(elevator.elevatorHeight(ControlPanel.ReefCycle.getHeight()))
                ).andThen(endEffector.moveCoralCommand(ControlPanel.ReefCycle.getTravelState()))
            )
        );
    }
}