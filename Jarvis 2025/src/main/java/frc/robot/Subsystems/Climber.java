package frc.robot.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  private final DigitalInput positiveLimit = new DigitalInput(6);
  private final DigitalInput negativeLimit = new DigitalInput(7);

  private boolean previousPositiveLimit, previousNegativeLimit;

  private final NetworkTable nTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/Climber");
  private final GenericEntry targetPositionEntry = nTable.getTopic("Target").getGenericEntry();
  private final GenericEntry outputEntry = nTable.getTopic("Output").getGenericEntry();
  private final GenericEntry positiveSwitchEntry = nTable.getTopic("Positive Switch").getGenericEntry();
  private final GenericEntry negativeSwitchEntry = nTable.getTopic("Negative Switch").getGenericEntry();
  private final GenericEntry encoderEntry = nTable.getTopic("Encoder").getGenericEntry();

  public Climber() {
    climber = new SparkMax(Constants.CAN_DEVICES.CLIMBER.id, MotorType.kBrushless);

    climberController = climber.getClosedLoopController();

    climberConfig = new SparkMaxConfig();
    climberConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .voltageCompensation(12);
    climberConfig.encoder
      .positionConversionFactor(1 / Constants.ClimberConstants.GEARING)
      .velocityConversionFactor(1 / Constants.ClimberConstants.GEARING);
    climberConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .positionWrappingEnabled(false)
      .outputRange(-Constants.GAINS.CLIMBER.peakOutput, Constants.GAINS.CLIMBER.peakOutput);

    outputEntry.setDouble(0);
    targetPositionEntry.setDouble(0);
    positiveSwitchEntry.setBoolean(false);
    negativeSwitchEntry.setBoolean(false);
    encoderEntry.setDouble(0);

    climber.getEncoder().setPosition(Constants.ClimberConstants.MIN_ROTATION.getRotations());

    SparkBaseSetter closedLoopSetter = new SparkBaseSetter(new SparkBaseSetter.SparkConfiguration(climber, climberConfig));
    closedLoopSetter.setPID(Constants.GAINS.PIVOT);
    PIDDisplay.PIDList.addOption("Climber", closedLoopSetter);
  }

  public void setPosition(Rotation2d target) {
    targetPositionEntry.setDouble(target.getDegrees());
    climberController.setReference(target.getRotations(), ControlType.kPosition);
  }

  public Command climbCommand(Rotation2d target) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setPosition(target)),
      new WaitUntilCommand(() -> Math.abs(climber.getEncoder().getPosition() - target.getRotations()) < Constants.ClimberConstants.SETPOINT_RANGE.getRotations())
    );
  }

  @Override
  public void periodic() {
    if (!positiveLimit.get() && !previousPositiveLimit) climber.getEncoder().setPosition(Constants.ClimberConstants.MAX_ROTATION.getRotations());
    if (!negativeLimit.get() && !previousNegativeLimit) climber.getEncoder().setPosition(Constants.ClimberConstants.MIN_ROTATION.getRotations());
    previousPositiveLimit = !positiveLimit.get();
    previousNegativeLimit = !negativeLimit.get();

    outputEntry.setDouble(climber.get());
    positiveSwitchEntry.setBoolean(previousPositiveLimit);
    negativeSwitchEntry.setBoolean(previousNegativeLimit);
    encoderEntry.setDouble(360 * climber.getEncoder().getPosition());
  }
}
