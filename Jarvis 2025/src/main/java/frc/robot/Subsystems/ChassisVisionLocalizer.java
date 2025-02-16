// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ChassisVisionLocalizer extends SubsystemBase {
  private static final ShuffleboardTab visionTab = Shuffleboard.getTab("Vision Localizer");

  private static final Field2d[] fields = new Field2d[4];

  private static final PhotonCamera[] photonCameras = new PhotonCamera[] {
    new PhotonCamera("navCam0"),
    new PhotonCamera("navCam1"),
    new PhotonCamera("navCam2"),
    new PhotonCamera("navCam3")
  };

  public ChassisVisionLocalizer() {
    PortForwarder.add(5800, "navCams01.local", 5800);
    PortForwarder.add(5800, "navCams23.local", 5800);

    for (int i = 0; i < fields.length; i++) {
      fields[i] = new Field2d();
      visionTab.add(photonCameras[i].getName(), fields[i]).withSize(4, 2).withPosition((i % 2) * 4, (i / 2) * 2);
    }
  }

  @Override
  public void periodic() {
  //   for (int i = 0; i < photonCameras.length; i++) {
  //     PhotonCamera camera = photonCameras[i];
  //     final Transform3d cameraToRobot = PhotonConstants.CAMERAS_TO_ROBOT[i];
  //     final Field2d camField = fields[i];

  //     camera.getAllUnreadResults().forEach(pipelineResult -> {
  //       PhotonTrackedTarget target = pipelineResult.getBestTarget();

  //       if (target.getPoseAmbiguity() <= .05) {
  //         Transform3d camToTarget = target.getBestCameraToTarget();
  //         Transform3d targetToCamera = camToTarget.inverse();

  //         Pose3d targetPose = PhotonConstants.APRILTAG_LOCATIONS[target.getFiducialId()];
  //         Pose3d camPose = targetPose.transformBy(targetToCamera);

  //         Pose2d visionMeasurement = camPose.transformBy(cameraToRobot).toPose2d();
  //         camField.setRobotPose(visionMeasurement);
  //         Drivetrain.addVisionMeasurement(visionMeasurement, pipelineResult.getTimestampSeconds());
  //       }
  //     });
  //   }
  }
}
