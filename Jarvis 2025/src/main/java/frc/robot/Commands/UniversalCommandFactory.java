package frc.robot.Commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
                    ControlPanel.ReefCycle.mutatingPathCommand(),
                    
                    new SequentialCommandGroup(
                        new WaitCommand(0.2),
                        new ParallelCommandGroup(
                            elevator.moveCommand(Constants.ElevatorConstants.MIN_ELEVATOR_EXTENSION),
                            pivot.setAngleCommand(Constants.PivotConstants.TRAVEL_POSITION)
                        ),
                        new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                new WaitUntilCommand(() -> pivot.timeToReach(ControlPanel.ReefCycle.getTargetAngle()) 
                                    > drivetrain.timeToReach(ControlPanel.ReefCycle.getLineupPath().getStartingHolonomicPose().get())),
                                new DeferredCommand(() -> pivot.setAngleCommand(ControlPanel.ReefCycle.getTargetAngle()), Set.of(pivot))
                            ),
                            new SequentialCommandGroup(
                                new WaitUntilCommand(() -> elevator.timeToReach(ControlPanel.ReefCycle.getTargetHeight()) 
                                    > drivetrain.timeToReach(ControlPanel.ReefCycle.getLineupPath().getStartingHolonomicPose().get())),
                                new DeferredCommand(() -> elevator.moveCommand(ControlPanel.ReefCycle.getTargetHeight()), Set.of(elevator))
                            )
                        )
                    )
                ),
                new WaitUntilCommand(() -> false)
                //new DeferredCommand(() -> endEffector.moveCoralCommand(ControlPanel.ReefCycle.getTravelState()), Set.of(endEffector))
            )
        );
    }
}