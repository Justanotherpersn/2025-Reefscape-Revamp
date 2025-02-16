package frc.robot.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  
  public void move(int targetPosition){
    setPoint = Constants.ElevatorConstants.LEVEL_HEIGHT[targetPosition];
    elevatorController.setReference(setPoint, SparkFlex.ControlType.kPosition);
  }

  public Command elevatorHeight(int targetPosition){
    return new RunCommand(() -> {
        move(targetPosition);
    }, this)
    .until(() -> {
      return Math.abs(elevator.getEncoder().getPosition()) < 0.1;
      // || topLimit.get() || bottomLimit.get())
    })
    .andThen(() -> {
      elevatorController.setReference(elevator.getEncoder().getPosition(), SparkFlex.ControlType.kPosition);
      if (topLimit.get()) elevator.getEncoder().setPosition(Constants.ElevatorConstants.LEVEL_HEIGHT.length - 1);
      if (bottomLimit.get()) elevator.getEncoder().setPosition(0);
    });
  }
}