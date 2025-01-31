// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  public RobotContainer() {
    new PIDDisplay();
    new ChassisVisionLocalizer();

    configureBindings();
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

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
