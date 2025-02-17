package frc.robot.Util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.EndEffector;

public class UniversalCommandFactory {

    public static final Command reefCycle(Drivetrain drivetrain, Elevator elevator, EndEffector endEffector) {
        return new RepeatCommand(
            new ParallelCommandGroup(
                drivetrain.pathingCommand(drivetrain.getReefCycleDestination(endEffector.coralPresent()), 0),
                new WaitUntilCommand(() -> 
                    drivetrain.getPose().getTranslation().getDistance(drivetrain.getReefCycleDestination(!endEffector.coralPresent()).getTranslation()) 
                    > Constants.NavigationConstants.OPERATION_RADIUS).andThen(null),//set pivot and elevator to default(down) position
                new WaitUntilCommand(() -> true/*within certain radius of target location*/).andThen(null), //move pivot to target
                new WaitUntilCommand(() -> true/*within certain radius of target location*/).andThen(null) //move elevator to target
            ).andThen(endEffector.moveCoralCommand(endEffector.coralPresent()))
        );
    }
}