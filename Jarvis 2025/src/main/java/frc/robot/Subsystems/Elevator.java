package frc.robot.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
import frc.robot.Util.SparkBaseSetter.SparkConfiguration;


public class Elevator extends SubsystemBase {
  private final DigitalInput topLimit = new DigitalInput(4);
  private final DigitalInput bottomLimit = new DigitalInput(5);
  private boolean previousTopLimit, previousBottomLimit;

  private SparkFlex elevator;
  private SparkFlexConfig elevatorConfig;
  private SparkClosedLoopController elevatorController;

  private double setPoint;

  private final NetworkTable nTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/Elevator");
  private final GenericEntry targetPositionEntry = nTable.getTopic("Target").getGenericEntry();
  private final GenericEntry outputEntry = nTable.getTopic("Output").getGenericEntry();
  private final GenericEntry topSwitchEntry = nTable.getTopic("Top Switch").getGenericEntry();
  private final GenericEntry bottomSwitchEntry = nTable.getTopic("Bottom Switch").getGenericEntry();
  
  public Elevator(){
    elevator = new SparkFlex(Constants.CAN_DEVICES.ELEVATOR_MOTOR.id, MotorType.kBrushless);
    elevatorController = elevator.getClosedLoopController();

    elevatorConfig = new SparkFlexConfig();
    elevatorConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .voltageCompensation(12);
    elevatorConfig.encoder
      .positionConversionFactor(Constants.ElevatorConstants.SPROKET_CIRCUMFERENCE / Constants.ElevatorConstants.GEARING)
      .velocityConversionFactor(Constants.ElevatorConstants.SPROKET_CIRCUMFERENCE / Constants.ElevatorConstants.GEARING);
    elevatorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .positionWrappingEnabled(false)
      .outputRange(-Constants.GAINS.ELEVATOR.peakOutput, Constants.GAINS.ELEVATOR.peakOutput);

    outputEntry.setDouble(0);
    targetPositionEntry.setDouble(0);
    topSwitchEntry.setBoolean(false);
    bottomSwitchEntry.setBoolean(false);

    SparkBaseSetter motorClosedLoopSetter = new SparkBaseSetter(new SparkConfiguration(elevator, elevatorConfig));
    motorClosedLoopSetter.setPID(Constants.GAINS.ELEVATOR);
    PIDDisplay.PIDList.addOption("Elevator Motors", motorClosedLoopSetter);

    elevator.getEncoder().setPosition(Constants.ElevatorConstants.MIN_ELEVATOR_EXTENSION);
  }
  
  public void move(double targetPosition){
    if (targetPosition < Constants.ElevatorConstants.MIN_ELEVATOR_EXTENSION || targetPosition > Constants.ElevatorConstants.MAX_ELEVATOR_EXTENSION) return;
    setPoint = targetPosition;
    elevatorController.setReference(setPoint, SparkFlex.ControlType.kPosition);
    targetPositionEntry.setDouble(targetPosition);
  }

  public Command elevatorHeight(double targetPosition) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> move(targetPosition)),
      new WaitUntilCommand(() -> Math.abs(elevator.getEncoder().getPosition() - targetPosition) < Constants.ElevatorConstants.SETPOINT_RANGE)
    );
  }

  @Override
  public void periodic() {
    if (topLimit.get() && !previousTopLimit) elevator.getEncoder().setPosition(Constants.ElevatorConstants.MAX_ELEVATOR_EXTENSION);
    if (bottomLimit.get() && !previousBottomLimit) elevator.getEncoder().setPosition(Constants.ElevatorConstants.MIN_ELEVATOR_EXTENSION);
    previousTopLimit = topLimit.get();
    previousBottomLimit = bottomLimit.get();

    outputEntry.setDouble(elevator.get());
    topSwitchEntry.setBoolean(previousTopLimit);
    bottomSwitchEntry.setBoolean(previousBottomLimit);
  }
}