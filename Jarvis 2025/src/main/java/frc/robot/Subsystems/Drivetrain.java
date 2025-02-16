// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.stream.Collectors;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Util.PIDDisplay;

public class Drivetrain extends SubsystemBase {
  Pigeon2 IMU = new Pigeon2(Constants.CAN_DEVICES.PIGEON_2.id);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.ModuleConstants.MODULE_POSITIONS);

  private static SwerveDrivePoseEstimator poseEstimator;

  private static final ShuffleboardTab telemTab = Shuffleboard.getTab("Telemetry");
  private static GenericEntry headingEntry = telemTab.add("Robot Heading", 0).withPosition(5, 0).getEntry();
  private static Field2d fieldEntry = new Field2d();

  private int targetReefLocation;

  public Drivetrain() {
    poseEstimator = new SwerveDrivePoseEstimator(
      kinematics, 
      new Rotation2d(), 
      MODULES.collectProperty(SwerveModule::getPosition, SwerveModulePosition.class),
      new Pose2d(0,0, new Rotation2d()),
      MatBuilder.fill(Nat.N3(), Nat.N1(),0.05,0.05,0.05), //Standard deviations for state estimate, (m,m,rad). Increase to trust less
      MatBuilder.fill(Nat.N3(), Nat.N1(),0.9,0.9,0.9) //Standard deviations for vision estimate, (m,m,rad). Increase to trust less
    );

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      System.out.println("PathPlanner failed to read robot config from GUI");
      e.printStackTrace();
      config = new RobotConfig(60, 7, new ModuleConfig(
        Constants.ModuleConstants.WHEEL_DIA, 
        Constants.DrivetrainConstants.MAX_DRIVE_SPEED, 
        1, 
        DCMotor.getKrakenX60(1), 
        1, 
        1), 
        kinematics.getModules());
    }

    AutoBuilder.configure(
      this::getPose,
      this::setPose,
      () -> {return kinematics.toChassisSpeeds(MODULES.collectProperty(SwerveModule::getState, SwerveModuleState.class));},
      (speeds, feedforwards) -> drive(speeds, false),
      new PPHolonomicDriveController(new PIDConstants(4), new PIDConstants(4)),
      config,
      () -> {return DriverStation.getAlliance().get().equals(Alliance.Red);},
      this
    );

    Pigeon2Configuration IMUconfig = new Pigeon2Configuration();
    IMU.getConfigurator().apply(IMUconfig);
    resetIMU();

    setPose(new Pose2d()); //autonomous will reset this when starting 
    SwerveModule.driveSetters.setPID(Constants.GAINS.DRIVE);
    SwerveModule.turnSetters.setPID(Constants.GAINS.TURN);
    PIDDisplay.PIDList.addOption("Swerve Drive Motors", SwerveModule.driveSetters);
    PIDDisplay.PIDList.addOption("Swerve Turn Motors", SwerveModule.turnSetters);

    telemTab.add("Robot Pose", fieldEntry).withSize(5, 3);
  }

  enum MODULES {
    FRONT_LEFT(0),
    FRONT_RIGHT(1),
    BACK_LEFT(2),
    BACK_RIGHT(3);

    public SwerveModule base;
    private MODULES(int moduleID) {
      this.base = new SwerveModule(
        moduleID, 
        moduleID + 1, 
        moduleID + 5, 
        ModuleConstants.MODULE_OFFSETS[moduleID], 
        moduleID % 2 == 0 ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive,
        moduleID
      );
    }

    /**
     * Iterates through the list of swerve modules and returns an array containing the result of executing a function on each module
     * @param <T> Property type
     * @param map The function to run on each swerve module
     * @return The resulting list
     */
    @SuppressWarnings("unchecked")
    public static <T> T[] collectProperty(Function<SwerveModule, T> map, Class<T> returnType) {
      List<T> result = List.of(MODULES.values()).stream().map(m -> map.apply(m.base)).collect(Collectors.toList());
      return result.toArray((T[]) Array.newInstance(returnType, result.size()));
    }

    public static void forAll(Consumer<SwerveModule> lambda) {
      for (MODULES modules : MODULES.values()) {
        lambda.accept(modules.base);
      }
    }
  }

  /**
   * Sets the desired module states for all of the modules - desaturates the max speeds first
   * @param moduleStates array of target module states
   */
  public static void setModuleStates(SwerveModuleState[] moduleStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, ModuleConstants.MAX_SPEED);
    MODULES.forAll(m -> m.setDesiredState(moduleStates[m.moduleID]));
  }

  @Override
  public void periodic() {
    headingEntry.setDouble(getPose().getRotation().getDegrees());
    fieldEntry.setRobotPose(getPose());
    poseEstimator.update(getHeadingRaw(), MODULES.collectProperty(SwerveModule::getPosition, SwerveModulePosition.class));
  }

  /**
   * Primary drive method for the robot
   * @param speeds Speeds that the robot should be moving in - positive x is foreward, y is left, omega is CCW
   * @param isFieldRelative true if running in field relative mode
   */
  public void drive(ChassisSpeeds speeds, boolean isFieldRelative){
    if(isFieldRelative)
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());

    setModuleStates(kinematics.toSwerveModuleStates(speeds));
  }

  public void stop(){
    drive(new ChassisSpeeds(), false);
  }

  /**
   * Reset robot pose to new pose
   * @param newPose new pose
   */
  public void setPose(Pose2d newPose){
    poseEstimator.resetPosition(getHeadingRaw(), MODULES.collectProperty(SwerveModule::getPosition, SwerveModulePosition.class), newPose);
  }

  /**
   * @return Current pose of the robot on the field
   */
  public Pose2d getPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public void resetIMU() {
    IMU.reset();
  }

  /**
   * @return The heading directly from the gyroscope
   */
  public Rotation2d getHeadingRaw() {
    return Rotation2d.fromDegrees(IMU.getYaw().getValueAsDouble());
  }

  public static void addVisionMeasurement(Pose2d visionPose, double timestamp){
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  public void setReefLocation(int locationIndex) {
    targetReefLocation = locationIndex;
  }

  public Pose2d getReefCycleDestination(boolean coralPresent) {
    return coralPresent ? Constants.NavigationConstants.REEF_LOCATIONS[targetReefLocation] : Constants.NavigationConstants.CORAL_STATION;
  }

  public double timeToReach(Pose2d pose) {
    return Math.max(getPose().getTranslation().getDistance(pose.getTranslation()) * Constants.DrivetrainConstants.MAX_DRIVE_SPEED, 
      getPose().getRotation().plus(pose.getRotation().unaryMinus()).getRadians() * Constants.DrivetrainConstants.MAX_ANGULAR_SPEED);
  }
 
  public Command homeCommand() {
    return new
      InstantCommand(() -> MODULES.forAll(m -> m.setHomed(false)))
      .alongWith(new RunCommand(() -> MODULES.forAll(SwerveModule::home), this))
      .until(() -> !Arrays.asList(MODULES.collectProperty(m -> m.homed, Boolean.class)).contains(false));
  }

  public Command pathingCommand(Pose2d destination, double endSpeed) {
    return AutoBuilder.pathfindToPose(
      destination,
      Constants.NavigationConstants.PATHING_CONSTRAINTS,
      endSpeed
    );
  }
}
