// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.Notifications;

public class ChassisVisionLocalizer extends SubsystemBase {
  public boolean enabled = true;
  private int overrunCount = 0;
  private double lastPeriodicStart = 0;

  private class PoseEntry {
    public final Pose2d pose;
    public final double timeStamp;

    public PoseEntry(Pose2d pose, double timeStamp) {
      this.pose = pose;
      this.timeStamp = timeStamp;
    }
  }

  private class NavCam {
    public final int index;
    private final PhotonCamera cam;
    private final PhotonPoseEstimator poseEstimator;
    private final Field2d field = new Field2d();
    private final GenericEntry trustEntry;
    
    public NavCam(int index) {
      this.index = index;
      cam = new PhotonCamera(getName());
      trustEntry = nTable.getTopic("Trusting %s".formatted(index)).getGenericEntry();

      poseEstimator = new PhotonPoseEstimator(
        Constants.PhotonConstants.APRIL_TAG_FIELD_LAYOUT, 
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
        Constants.PhotonConstants.ROBOT_TO_CAMERAS[index]
      );
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      trustEntry.setBoolean(false);
      SmartDashboard.putData("Drivetrain/Vision/" + getName(), field);
    }

    public List<PoseEntry> getRobotPoses() {
      List<PoseEntry> poses = new ArrayList<PoseEntry>();
      cam.getAllUnreadResults().forEach(r -> {
        if (r.hasTargets() && r.getBestTarget().getPoseAmbiguity() <= 0.05)
          poses.add(new PoseEntry(poseEstimator.update(r).get().estimatedPose.toPose2d(), r.getTimestampSeconds()));
      });

      trustEntry.setBoolean(poses.size() > 0);
      if (poses.size() > 0) field.setRobotPose(poses.get(poses.size() - 1).pose);
      SmartDashboard.putData("Drivetrain/Vision/" + getName(), field);

      return poses;
    }

    public String getName() {
      return "navCam%s".formatted(index);
    }
  }

  private final NetworkTable nTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/Drivetrain/Vision");

  private final GenericEntry enabledEntry = nTable.getTopic("Enabled").getGenericEntry();

  private NavCam[] navCams = {
    new NavCam(0),
    new NavCam(1),
    new NavCam(2),
    new NavCam(3)
  };

  public ChassisVisionLocalizer() {
    PortForwarder.add(5800, "navCams01.local", 5800);
    PortForwarder.add(5800, "navCams23.local", 5800);
    enabledEntry.setBoolean(enabled);
  }

  @Override
  public void periodic() {
    if (!enabled) return;
    if (Timer.getFPGATimestamp() - lastPeriodicStart > 0.2 && lastPeriodicStart != 0) overrunCount++;
    if (overrunCount > 2) {
      enabled = false;
      enabledEntry.setBoolean(false);
      Notifications.VISION_TIMER_EXCEEDED.sendImmediate();
    }
    lastPeriodicStart = Timer.getFPGATimestamp();

    for (NavCam navCam : navCams) {
      navCam.getRobotPoses().forEach(pose -> {
        Drivetrain.addVisionMeasurement(pose.pose, pose.timeStamp);
      });
    }
  }
}
