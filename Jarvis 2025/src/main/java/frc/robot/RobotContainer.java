// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.Notifications;
import frc.robot.Commands.UniversalCommandFactory;
import frc.robot.Subsystems.ChassisVisionLocalizer;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.Pivot;
import frc.robot.Util.PIDDisplay;

public class RobotContainer {
  private static final Drivetrain drivetrain = new Drivetrain();
  private static final Elevator elevator = new Elevator();
  private static final Pivot pivot = new Pivot();
  private static final Climber climber = new Climber();
  private static final EndEffector endEffector = new EndEffector();
  SendableChooser<Command> autoChooser;

  int elevatorHeight;
  int pivotAngle;

  public RobotContainer() {

    NamedCommands.registerCommand("Set Elevator Height", elevator.moveCommand(Constants.ElevatorConstants.PRESET_HEIGHTS[elevatorHeight]));
  

    NamedCommands.registerCommand("Set Pivot L1", null);
    NamedCommands.registerCommand("Set Pivot L2", null);
    NamedCommands.registerCommand("Set Pivot L3", UniversalCommandFactory.pivotAngleCommand(Rotation2d.fromDegrees(0), false, pivot, endEffector));
    NamedCommands.registerCommand("Set Pivot L4", null);

    NamedCommands.registerCommand("Deposit Coral", null);
    NamedCommands.registerCommand("Load Coral", null);

    NamedCommands.registerCommand("Home Swerve", drivetrain.homeCommand());
    NamedCommands.registerCommand("Home Elevator", elevator.homeCommand());

    new PIDDisplay();
    new ChassisVisionLocalizer();

    ControlPanel.configureBinding(drivetrain, elevator, pivot, endEffector, climber);
    configureAuto();

    PIDDisplay.Init();
  }

  public void onTeleopEnabled() { 
      // Notifications.GENERAL.send("Starting test sequence").andThen(new RepeatCommand(new SequentialCommandGroup(
      //   new WaitCommand(1),
      //   UniversalCommandFactory.reefCycle(drivetrain, elevator, pivot, endEffector)
      // ))).schedule();
  }

  private void configureAuto() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      drivetrain.homeCommand(),
      AutoBuilder.buildAuto(autoChooser.getSelected().getName())
    );
  }
}
