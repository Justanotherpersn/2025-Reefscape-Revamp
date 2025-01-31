// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.JoystickDrive;
import frc.robot.Subsystems.ChassisVisionLocalizer;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Util.PIDDisplay;

public class RobotContainer {
  /**PS4-ish Controller */
  private static final Joystick controller = new Joystick(0);
  private static final Joystick controller2 = new Joystick(1);

  private static final Drivetrain drivetrain = new Drivetrain();
  private static final Elevator elevator = new Elevator();
  private static final JoystickDrive joystickDrive = new JoystickDrive(drivetrain, controller);

  ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
  SendableChooser<Command> autoChooser;

  public RobotContainer() {
    new PIDDisplay();
    new ChassisVisionLocalizer();

    configureBindings();
    configureAuto();

    drivetrain.setDefaultCommand(joystickDrive);
    PIDDisplay.Init();
  }

  private void configureBindings() {
    new JoystickButton(controller, 1).whileTrue(drivetrain.homeCommand());
    new JoystickButton(controller, 2).whileTrue(drivetrain.pathingCommand(drivetrain.getPose().plus(new Transform2d(1, 1, new Rotation2d())), 0));

    new JoystickButton(controller2, 1).onTrue(elevator.elevatorHeight(0)); //"A" Button
    new JoystickButton(controller2, 3).onTrue(elevator.elevatorHeight(1)); //"X" Button
    new JoystickButton(controller2, 2).onTrue(elevator.elevatorHeight(2)); //"B" Button
    new JoystickButton(controller2, 4).onTrue(elevator.elevatorHeight(3)); //"Y" Button
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
