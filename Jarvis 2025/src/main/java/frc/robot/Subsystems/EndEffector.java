// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.SparkBaseSetter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class EndEffector extends SubsystemBase {
  private final SparkMax coral;
  private final SparkMaxConfig coralConfig;
  private final SparkClosedLoopController coralController;
  private final DigitalInput sensor = new DigitalInput(8);

  private final NetworkTable nTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/End Effector");
  private final GenericEntry targetSpeedEntry = nTable.getTopic("Target").getGenericEntry();
  private final GenericEntry encoderEntry = nTable.getTopic("Encoder").getGenericEntry();
  private final GenericEntry coralSwitchEntry = nTable.getTopic("Coral Switch").getGenericEntry();
  
  public EndEffector() {
    coral = new SparkMax(Constants.CAN_DEVICES.END_EFFECTOR.id, MotorType.kBrushless);

    coralController = coral.getClosedLoopController();

    coralConfig = new SparkMaxConfig();
    coralConfig
      .inverted(true)
      .idleMode(IdleMode.kCoast)
      .voltageCompensation(12);
    coralConfig.encoder
      .velocityConversionFactor(1 / Constants.EndEffectorConstants.GEARING);
    coralConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .positionWrappingEnabled(false)
      .outputRange(-Constants.GAINS.END_EFFECTOR.peakOutput, Constants.GAINS.END_EFFECTOR.peakOutput);

    targetSpeedEntry.setDouble(0);
    encoderEntry.setDouble(0);
    coralSwitchEntry.setBoolean(false);

    SparkBaseSetter closedLoopSetter = new SparkBaseSetter(new SparkBaseSetter.SparkConfiguration(coral, coralConfig));
    closedLoopSetter.setPID(Constants.GAINS.END_EFFECTOR);
    PIDDisplay.PIDList.addOption("End Effector", closedLoopSetter);
  }

  public void setRPM(double speed){
    if( Math.abs(speed) == Constants.EndEffectorConstants.INTAKE_RPM)
    setSpeed(Math.signum(speed));
    else
    setSpeed(speed);
    // targetSpeedEntry.setDouble(speed);
    // coralController.setReference(speed, ControlType.kVelocity);
  }

  public void setSpeed(double speed) {
    //revert and uncomment above
    targetSpeedEntry.setDouble(speed * 0.01);
    coral.set(speed);
  }

  public boolean coralPresent() {
    //return !sensor.get();
    return true;
  }

  public Command moveCoralCommand(boolean intake) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> setRPM(intake ? Constants.EndEffectorConstants.INTAKE_RPM : Constants.EndEffectorConstants.OUTAKE_RPM)),
        new WaitUntilCommand(() -> intake == coralPresent()),
        new WaitCommand(0.5)
    ).finallyDo(() -> setRPM(0));
  }

  public Command testDepositCoral() {
    return new RunCommand(() -> setSpeed(Constants.EndEffectorConstants.OUTAKE_RPM)).finallyDo(() -> setSpeed(0));
  }

  public Command testIntakeCoral() {
    return new RunCommand(() -> setSpeed(Constants.EndEffectorConstants.INTAKE_RPM)).finallyDo(() -> setSpeed(0));
  }

  public Command velocityCoralCommand(double velocity) {
    return new InstantCommand(() -> setSpeed(velocity));
  }

  @Override
  public void periodic() {
    encoderEntry.setDouble(coral.getOutputCurrent());
    coralSwitchEntry.setBoolean(coralPresent());
  }
}
