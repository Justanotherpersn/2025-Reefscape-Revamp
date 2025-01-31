package frc.robot.Subsystems;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.SparkFlexSetter;


public class Elevator extends SubsystemBase {
  private SparkFlex elevatorMotor;
  private SparkClosedLoopController elevatorPIDController;
  private SparkFlexConfig motorConfig;
  
  public Elevator(){
    elevatorMotor = new SparkFlex(Constants.CAN_DEVICES.ELEVATOR_MOTOR.id, MotorType.kBrushless);
    elevatorPIDController = elevatorMotor.getClosedLoopController();
    
    motorConfig = new SparkFlexConfig();

    motorConfig
    .inverted(false)
    .idleMode(IdleMode.kCoast);

    SparkFlexSetter motorClosedLoopSetter = new SparkFlexSetter(motorConfig);
    motorClosedLoopSetter.setPID(Constants.GAINS.ELEVATOR);
    PIDDisplay.PIDList.addOption("Elevator Motors", motorClosedLoopSetter);
  }
  
  public void move(int targetPosition){
    //double maxRotations = Constants.ElevatorConstants.MAX_ELEVATOR_EXTENSION/Constants.ElevatorConstants.SPROKET_CIRCUMFERENCE;
    double setPoint = Constants.ElevatorConstants.levelHeight[targetPosition]/Constants.ElevatorConstants.SPROKET_CIRCUMFERENCE;
    elevatorPIDController.setReference(setPoint, SparkFlex.ControlType.kPosition);
  }

  public Command elevatorHeight(int position){
      return new InstantCommand(() -> {
          move(position);
      }, this);
  }
}