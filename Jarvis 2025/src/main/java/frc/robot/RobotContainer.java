// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ChassisVisionLocalizer;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.Pivot;
import frc.robot.Util.PIDDisplay;

public class RobotContainer {
  private static final Drivetrain drivetrain = new Drivetrain();
  private static final Elevator elevator = new Elevator();
  private static final Pivot pivot = new Pivot();
  private static final EndEffector endEffector = new EndEffector();

  ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
  SendableChooser<Command> autoChooser;

  public RobotContainer() {
    new PIDDisplay();
    new ChassisVisionLocalizer();

    ControlPanel.configureBinding(drivetrain, elevator, pivot, endEffector);
    configureAuto();

    PIDDisplay.Init();
  }

  private void configureAuto() {
    NamedCommands.registerCommand("Elevator Height 0", elevator.elevatorHeight(0));
    NamedCommands.registerCommand("Elevator Height 1", elevator.elevatorHeight(1));
    NamedCommands.registerCommand("Elevator Height 2", elevator.elevatorHeight(2));
    NamedCommands.registerCommand("Elevator Height 3", elevator.elevatorHeight(3));

    autoChooser = AutoBuilder.buildAutoChooser();
    autoTab.add("Auto Chooser", autoChooser).withPosition(0, 0).withPosition(5, 1);
  }

  public Command getAutonomousCommand() {
    return
      drivetrain.homeCommand()
      .andThen(autoChooser.getSelected());
  }
}
