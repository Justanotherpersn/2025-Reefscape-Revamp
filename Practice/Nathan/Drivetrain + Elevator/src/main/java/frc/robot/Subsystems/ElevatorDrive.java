package frc.robot.Subsystems;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Gains;


public class ElevatorDrive extends SubsystemBase {

    private Gains elevatorGains = new Gains(3, 0, 0, 0, 0, 0);

    private SparkFlex elevatorMotor;
    private SparkClosedLoopController elevatorPIDController;
    private SparkFlexConfig motorConfig;
    
       
    
        public ElevatorDrive(){
            elevatorMotor = new SparkFlex(5, MotorType.kBrushless);
            elevatorPIDController = elevatorMotor.getClosedLoopController();
            elevatorPIDController.setReference(0, ControlType.kPosition);
            
            motorConfig = new SparkFlexConfig();
            motorConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
            motorConfig.closedLoop
            .p(elevatorGains.P)
            .d(elevatorGains.D);        
    
        }
            //sproket radius = .03175m
            //Extension length .8382m
            //angle in radians(radius)

            public void move(double yValue){
            //double extenionMax = .8382; // Max elevator extension is .8382m
            //double sproketRadius = .03175; // Sprocket radius is .03175m
            double moveTest = 1 * Math.abs(yValue);
            elevatorPIDController.setReference(moveTest, ControlType.kPosition);
            }

}
