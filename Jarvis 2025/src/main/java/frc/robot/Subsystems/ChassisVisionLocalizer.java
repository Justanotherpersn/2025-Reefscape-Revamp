// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonConstants;

public class ChassisVisionLocalizer extends SubsystemBase {
  public boolean enabled = true;

  private final NetworkTable nTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/Drivetrain/Vision");
  private final GenericEntry[] trustEntries = new GenericEntry[] {
    nTable.getTopic("Trusting 0").getGenericEntry(),
    nTable.getTopic("Trusting 1").getGenericEntry(),
    nTable.getTopic("Trusting 2").getGenericEntry(),
    nTable.getTopic("Trusting 3").getGenericEntry()
  };

  private static final Field2d[] fields = {new Field2d(), new Field2d(), new Field2d(), new Field2d()};

  private static final PhotonCamera[] photonCameras = new PhotonCamera[] {
    new PhotonCamera("navCam0"),
    new PhotonCamera("navCam1"),
    new PhotonCamera("navCam2"),
    new PhotonCamera("navCam3")
  };

  public ChassisVisionLocalizer() {
    PortForwarder.add(5800, "navCams01.local", 5800);
    PortForwarder.add(5800, "navCams23.local", 5800);

    for (int i = 0; i < 4; i++) {
      trustEntries[i].setBoolean(false);
      SmartDashboard.putData("Drivetrain/Vision/" + photonCameras[i].getName(), fields[i]);
    }
  }

  @Override
  public void periodic() {
    if (!enabled) return;
    for (int i = 0; i < photonCameras.length; i++) {
      PhotonCamera camera = photonCameras[i];
      final Transform3d cameraToRobot = PhotonConstants.CAMERAS_TO_ROBOT[i];
      final Field2d camField = fields[i];
      final GenericEntry trustEntry = trustEntries[i];

      camera.getAllUnreadResults().forEach(pipelineResult -> {
        PhotonTrackedTarget target = pipelineResult.getBestTarget();

        if (target != null && target.getPoseAmbiguity() <= .025) {
          Transform3d camToTarget = target.getBestCameraToTarget();
          Transform3d targetToCamera = camToTarget.inverse();

          Pose3d targetPose = PhotonConstants.APRILTAG_LOCATIONS[target.getFiducialId() - 1];
          Pose3d camPose = targetPose.transformBy(targetToCamera);

          Pose2d visionMeasurement = camPose.transformBy(cameraToRobot).toPose2d();
          Drivetrain.addVisionMeasurement(visionMeasurement, pipelineResult.getTimestampSeconds());
          
          camField.setRobotPose(visionMeasurement);
          SmartDashboard.putData("Drivetrain/Vision/" + camera.getName(), camField);
          trustEntry.setBoolean(true);
        }
        else {
          trustEntry.setBoolean(false);
        }
      });
    }
  }
}
