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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Util.PIDDisplay;

public class Drivetrain extends SubsystemBase {
  Pigeon2 IMU = new Pigeon2(20);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(Constants.ROBOT_WHEEL_BASE/2 , Constants.ROBOT_WHEEL_BASE/2), 
    new Translation2d(Constants.ROBOT_WHEEL_BASE/2, -Constants.ROBOT_WHEEL_BASE/2), 
    new Translation2d(-Constants.ROBOT_WHEEL_BASE/2,Constants.ROBOT_WHEEL_BASE/2),
    new Translation2d(-Constants.ROBOT_WHEEL_BASE/2, -Constants.ROBOT_WHEEL_BASE/2)
  );

  private static SwerveDrivePoseEstimator poseEstimator;

  private static final ShuffleboardTab telemTab = Shuffleboard.getTab("Telemetry");

  private static GenericEntry headingEntry = telemTab.add("Robot Heading", 0).withPosition(9, 0).getEntry();

  public Drivetrain() {
    poseEstimator = new SwerveDrivePoseEstimator(
      kinematics, 
      new Rotation2d(), 
      MODULES.collectProperty(SwerveModule::getPosition, SwerveModulePosition.class),
      new Pose2d(0,0, new Rotation2d()),
      MatBuilder.fill(Nat.N3(), Nat.N1(),0.05,0.05,0.05), //Standard deviations for state estimate, (m,m,rad). Increase to trust less
      MatBuilder.fill(Nat.N3(), Nat.N1(),0.9,0.9,0.9) //Standard deviations for vision estimate, (m,m,rad). Increase to trust less
    );

    Pigeon2Configuration IMUconfig = new Pigeon2Configuration();
    IMUconfig.MountPose.MountPoseYaw = 0;
    IMUconfig.MountPose.MountPosePitch = 0;
    IMUconfig.MountPose.MountPoseRoll = 90;
    IMU.getConfigurator().apply(IMUconfig);

    setPose(new Pose2d()); //autonomous will reset this when starting 
    PIDDisplay.PIDList.addOption("Swerve Drive Motors", SwerveModule.driveSetters);
    PIDDisplay.PIDList.addOption("Swerve Turn Motors", SwerveModule.turnSetters);
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

    // //The entire purpose of this subclass is to convert a List to an Array because Java has a mental breakdown trying to do it with reflection
    // public static class CommonArrays {
    //   private static final SwerveModulePosition swerveDrivePositions[] = new SwerveModulePosition[MODULES.values().length];

    //   private static void init() {
    //     for (MODULES module : MODULES.values()) {
    //       int i = module.ordinal();
    //       swerveDrivePositions[i] = 
    //       module.base.getPosition();
    //     }
    //   }
    // }

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

  //#region Module Interface
  /**
   * Sets the desired module states for all of the modules - desaturates the max speeds first
   * @param moduleStates array of target module states
   */
  public static void setModuleStates(SwerveModuleState[] moduleStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, ModuleConstants.MAX_SPEED);
    MODULES.forAll(m -> m.setDesiredState(moduleStates[m.moduleID]));
  }

  public void homeAllModules() {
    MODULES.forAll(m -> m.home());
  }

  public void resetHomeStatus() {
    MODULES.forAll(m -> m.setHomed(false));
  }

  public boolean allModulesHomed() {
    return !Arrays.asList(MODULES.collectProperty(m -> m.homed, Boolean.class)).contains(false);
  }
  //#endregion

  @Override
  public void periodic() {
    headingEntry.setDouble(getPose().getRotation().getDegrees());

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
    return new Rotation2d(IMU.getYaw().getValueAsDouble());
  }

  public static void addVisionMeasurement(Pose2d visionPose, double timestamp){
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }
}
