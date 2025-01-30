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
import frc.robot.Util.Gains;


public class Elevator extends SubsystemBase {

    // Elevator Gains
    private Gains elevatorGains = new Gains(20, 0, 0, 0, 0, 12);

    private SparkFlex elevatorMotor;
    private SparkClosedLoopController elevatorPIDController;
    private SparkFlexConfig motorConfig;
    
        public Elevator(){
            elevatorMotor = new SparkFlex(8, MotorType.kBrushless);
            elevatorPIDController = elevatorMotor.getClosedLoopController();
            
            motorConfig = new SparkFlexConfig();

            motorConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast);

            motorConfig.closedLoop
            .p(elevatorGains.P)
            .d(elevatorGains.D);        
    
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