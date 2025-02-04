package frc.robot.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.SparkFlexSetter;


public class Elevator extends SubsystemBase {
  private final DigitalInput topLimit = new DigitalInput(4);
  private final DigitalInput bottomLimit = new DigitalInput(5);

  private SparkFlex elevatorMotor, elevatorMotorFollow;
  private SparkClosedLoopController elevatorPIDController;
  private SparkFlexConfig motorConfig, motorConfigFollow;

  private double setPoint;
  
  public Elevator(){
    elevatorMotor = new SparkFlex(Constants.CAN_DEVICES.ELEVATOR_MOTOR.id, MotorType.kBrushless);
    elevatorPIDController = elevatorMotor.getClosedLoopController();
    motorConfig = new SparkFlexConfig();
    motorConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .voltageCompensation(12);
    motorConfig.encoder
      .positionConversionFactor(Constants.ElevatorConstants.SPROKET_CIRCUMFERENCE)
      .velocityConversionFactor(Constants.ElevatorConstants.SPROKET_CIRCUMFERENCE);
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .positionWrappingEnabled(false)
      .outputRange(-Constants.GAINS.ELEVATOR.peakOutput, Constants.GAINS.ELEVATOR.peakOutput);

    SparkFlexSetter motorClosedLoopSetter = new SparkFlexSetter(motorConfig);
    motorClosedLoopSetter.setPID(Constants.GAINS.ELEVATOR);
    PIDDisplay.PIDList.addOption("Elevator Motors", motorClosedLoopSetter);

    elevatorMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    elevatorMotorFollow = new SparkFlex(Constants.CAN_DEVICES.ELEVATOR_MOTOR_FOLLOW.id, MotorType.kBrushless);
    motorConfigFollow = new SparkFlexConfig();
    motorConfigFollow
      .follow(Constants.CAN_DEVICES.ELEVATOR_MOTOR.id, true);

    elevatorMotorFollow.configure(motorConfigFollow, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
  
  public void move(int targetPosition){
    setPoint = Constants.ElevatorConstants.LEVEL_HEIGHT[targetPosition];
    elevatorPIDController.setReference(setPoint, SparkFlex.ControlType.kPosition);
    System.out.println(elevatorMotor.get());
  }

  public Command elevatorHeight(int targetPosition){
      return new RunCommand(() -> {
          move(targetPosition);
      }, this)
      .until(() -> {
        return Math.abs(elevatorMotor.getEncoder().getPosition()) < 0.1;
        // || topLimit.get() || bottomLimit.get())
      })
      .andThen(() -> {
        elevatorPIDController.setReference(elevatorMotor.getEncoder().getPosition(), SparkFlex.ControlType.kPosition);
        if (topLimit.get()) elevatorMotor.getEncoder().setPosition(Constants.ElevatorConstants.LEVEL_HEIGHT.length - 1);
        if (bottomLimit.get()) elevatorMotor.getEncoder().setPosition(0);
      });
  }
}