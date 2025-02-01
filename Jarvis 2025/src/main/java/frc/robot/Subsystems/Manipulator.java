// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.SparkMaxSetter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
  private SparkMax coralNeo, coralNeoFollow;
  private SparkClosedLoopController neoClosedLoopController, neoFollowClosedLoopController;
  private SparkMaxConfig neoConfig, neoFollowConfig;
  
  private double targetRPM;

  public Manipulator() {
    coralNeo = new SparkMax(Constants.CAN_DEVICES.MANIPULATOR_NEO.id, MotorType.kBrushless);
    coralNeoFollow = new SparkMax(Constants.CAN_DEVICES.MANIPULATOR_NEO_FOLLOW.id, MotorType.kBrushless);

    neoClosedLoopController = coralNeo.getClosedLoopController();
    neoFollowClosedLoopController = coralNeoFollow.getClosedLoopController();

    //coral manipulator neo550
    neoConfig = new SparkMaxConfig();
    neoConfig
    .inverted(false)
    .idleMode(IdleMode.kCoast);
    //second coral manipulator neo550
    neoFollowConfig = new SparkMaxConfig();
    neoFollowConfig
    .inverted(true)
    .idleMode(IdleMode.kCoast);

    SparkMaxSetter neoClosedLoopSetter = new SparkMaxSetter(neoConfig, neoFollowConfig);
    neoClosedLoopSetter.setPID(null);
    PIDDisplay.PIDList.addOption("Manipulator Motors", neoClosedLoopSetter);
  }

  public void setRPM(double value){
    targetRPM = value;
    neoClosedLoopController.setReference(targetRPM, ControlType.kVelocity);
    neoFollowClosedLoopController.setReference(targetRPM, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
