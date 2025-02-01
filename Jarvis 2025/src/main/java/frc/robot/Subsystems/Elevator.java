package frc.robot.Subsystems;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.SparkFlexSetter;


public class Elevator extends SubsystemBase {
  private final DigitalInput topLimit = new DigitalInput(1);
  private final DigitalInput bottomLimit = new DigitalInput(1);

  private SparkFlex elevatorMotor, elevatorMotorFollow;
  private SparkClosedLoopController elevatorPIDController, elevatorFollowPIDController;
  private SparkFlexConfig motorConfig, motorConfigFollow;

  private double angularSetpoint;
  
  public Elevator(){
    elevatorMotor = new SparkFlex(Constants.CAN_DEVICES.ELEVATOR_MOTOR.id, MotorType.kBrushless);
    elevatorPIDController = elevatorMotor.getClosedLoopController();
    
    motorConfig = new SparkFlexConfig();

    motorConfig
    .inverted(false)
    .idleMode(IdleMode.kCoast);

    elevatorMotorFollow = new SparkFlex(Constants.CAN_DEVICES.ELEVATOR_MOTOR_FOLLOW.id, MotorType.kBrushless);
    elevatorFollowPIDController = elevatorMotorFollow.getClosedLoopController();
    
    motorConfigFollow = new SparkFlexConfig();

    motorConfigFollow
    .inverted(true)
    .idleMode(IdleMode.kCoast);


    SparkFlexSetter motorClosedLoopSetter = new SparkFlexSetter(motorConfig, motorConfigFollow);
    motorClosedLoopSetter.setPID(Constants.GAINS.ELEVATOR);
    PIDDisplay.PIDList.addOption("Elevator Motors", motorClosedLoopSetter);
  }
  
  public void move(int targetPosition){
    //double maxRotations = Constants.ElevatorConstants.MAX_ELEVATOR_EXTENSION/Constants.ElevatorConstants.SPROKET_CIRCUMFERENCE;
    angularSetpoint = Constants.ElevatorConstants.levelHeight[targetPosition]/Constants.ElevatorConstants.SPROKET_CIRCUMFERENCE;
    elevatorPIDController.setReference(angularSetpoint, SparkFlex.ControlType.kPosition);
    elevatorFollowPIDController.setReference(angularSetpoint, SparkFlex.ControlType.kPosition);
  }

  public Command elevatorHeight(int targetPosition){
      return new RunCommand(() -> {
          move(targetPosition);
      }, this).until(() -> Math.abs(elevatorMotor.getEncoder().getPosition() - angularSetpoint) < 0.1 || topLimit.get() || bottomLimit.get());
  }
}