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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class EndEffector extends SubsystemBase {
  private final SparkMax coral;
  private final SparkMaxConfig coralConfig;
  private final SparkClosedLoopController coralController;
  private final DigitalInput sensor = new DigitalInput(6);
  
  public EndEffector() {
    coral = new SparkMax(Constants.CAN_DEVICES.END_EFFECTOR.id, MotorType.kBrushless);

    coralController = coral.getClosedLoopController();

    coralConfig = new SparkMaxConfig();
    coralConfig
      .inverted(false)
      .idleMode(IdleMode.kCoast)
      .voltageCompensation(12);
    coralConfig.encoder
      .velocityConversionFactor(1 / Constants.EndEffectorConstants.GEARING);
    coralConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .positionWrappingEnabled(false)
      .outputRange(-Constants.GAINS.END_EFFECTOR.peakOutput, Constants.GAINS.END_EFFECTOR.peakOutput);

    SparkBaseSetter closedLoopSetter = new SparkBaseSetter(new SparkBaseSetter.SparkConfiguration(coral, coralConfig));
    closedLoopSetter.setPID(null);
    PIDDisplay.PIDList.addOption("End Effector", closedLoopSetter);
  }

  public void setRPM(double speed){
    coralController.setReference(speed, ControlType.kVelocity);
  }

  public boolean coralPresent() {
    return sensor.get();
  }

  public Command moveCoralCommand(boolean intake) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> setRPM(intake ? 1 : -1)),
        new WaitUntilCommand(() -> coralPresent()),
        new WaitCommand(0.5),
        new InstantCommand(() -> setRPM(0))
    );
  }
}
