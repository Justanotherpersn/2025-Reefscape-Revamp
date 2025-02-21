package frc.robot.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.SparkBaseSetter;
import frc.robot.Util.SparkBaseSetter.SparkConfiguration;


public class Elevator extends SubsystemBase {
  private final DigitalInput topLimit = new DigitalInput(4);
  private final DigitalInput bottomLimit = new DigitalInput(5);

  private SparkFlex elevator;
  private SparkFlexConfig elevatorConfig;
  private SparkClosedLoopController elevatorController;

  private double setPoint;
  
  public Elevator(){
    elevator = new SparkFlex(Constants.CAN_DEVICES.ELEVATOR_MOTOR.id, MotorType.kBrushless);
    elevatorController = elevator.getClosedLoopController();

    elevatorConfig = new SparkFlexConfig();
    elevatorConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .voltageCompensation(12);
    elevatorConfig.encoder
      .positionConversionFactor(Constants.ElevatorConstants.SPROKET_CIRCUMFERENCE)
      .velocityConversionFactor(Constants.ElevatorConstants.SPROKET_CIRCUMFERENCE);
    elevatorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .positionWrappingEnabled(false)
      .outputRange(-Constants.GAINS.ELEVATOR.peakOutput, Constants.GAINS.ELEVATOR.peakOutput);

    SparkBaseSetter motorClosedLoopSetter = new SparkBaseSetter(new SparkConfiguration(elevator, elevatorConfig));
    motorClosedLoopSetter.setPID(Constants.GAINS.ELEVATOR);
    PIDDisplay.PIDList.addOption("Elevator Motors", motorClosedLoopSetter);
  }
  
  public void move(double targetPosition){
    setPoint = targetPosition;
    elevatorController.setReference(setPoint, SparkFlex.ControlType.kPosition);
  }

  public Command elevatorHeight(double targetPosition) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> move(targetPosition)),
      new WaitUntilCommand(() -> Math.abs(elevator.getEncoder().getPosition() - targetPosition) < Constants.ElevatorConstants.SETPOINT_RANGE)
    );
  }

  @Override
  public void periodic() {
    // boolean top = topLimit.get();
    // if (top || bottomLimit.get()) {
    //   setPoint = top ? Constants.ElevatorConstants.MAX_ELEVATOR_EXTENSION : 0;
    //   climber.getAlternateEncoder().setPosition(targetPosition.getRotations());
    //   setPosition(targetPosition.plus(positive ? Constants.ClimberConstants.SETPOINT_RANGE.unaryMinus() : Constants.ClimberConstants.SETPOINT_RANGE));
    // }
  }
}