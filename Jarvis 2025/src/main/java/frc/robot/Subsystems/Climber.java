package frc.robot.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.SparkBaseSetter;

public class Climber extends SubsystemBase {
  private final SparkMax climber;
  private final SparkMaxConfig climberConfig;
  private final SparkClosedLoopController climberController;
  private final DigitalInput positiveLimit = new DigitalInput(-1);
  private final DigitalInput negativeLimit = new DigitalInput(-1);
  private Rotation2d targetPosition;

  public Climber() {
    climber = new SparkMax(Constants.CAN_DEVICES.CLIMBER.id, MotorType.kBrushless);

    climberController = climber.getClosedLoopController();

    climberConfig = new SparkMaxConfig();
    climberConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .voltageCompensation(12);
    climberConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .positionWrappingEnabled(false)
      .outputRange(-Constants.GAINS.CLIMBER.peakOutput, Constants.GAINS.CLIMBER.peakOutput);

    SparkBaseSetter closedLoopSetter = new SparkBaseSetter(new SparkBaseSetter.SparkConfiguration(climber, climberConfig));
    closedLoopSetter.setPID(Constants.GAINS.PIVOT);
    PIDDisplay.PIDList.addOption("Climber", closedLoopSetter);
  }

  public void setPosition(Rotation2d target) {
    climberController.setReference(target.getRotations(), ControlType.kPosition);
  }

  public Command climbCommand(Rotation2d target) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setPosition(target)),
      new WaitUntilCommand(() -> Math.abs(climber.getAlternateEncoder().getPosition() - targetPosition.getRotations()) < Constants.ClimberConstants.SETPOINT_RANGE.getRotations())
    );
  }

  @Override
  public void periodic() {
    boolean positive = positiveLimit.get();
    if (positive || negativeLimit.get()) {
      targetPosition = positive ? Constants.ClimberConstants.MAX_ROTATION : Constants.ClimberConstants.MIN_ROTATION;
      climber.getAlternateEncoder().setPosition(targetPosition.getRotations());
      setPosition(targetPosition.plus(positive ? Constants.ClimberConstants.SETPOINT_RANGE.unaryMinus() : Constants.ClimberConstants.SETPOINT_RANGE));
    }
  }
}
