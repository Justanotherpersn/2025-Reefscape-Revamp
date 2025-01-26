// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonConstants;

public class ChassisVisionLocalizer extends SubsystemBase {
  private double previousPipelineTimestamp = 0;

  private static final PhotonCamera[] photonCameras = new PhotonCamera[] {
    new PhotonCamera("navCam0"),
    new PhotonCamera("navCam1"),
    new PhotonCamera("navCam2")
  };

  public ChassisVisionLocalizer() {}

  @Override
  public void periodic() {
    for (int i = 0; i < photonCameras.length; i++) {
      PhotonCamera camera = photonCameras[i];
      PhotonPipelineResult pipelineResult = camera.getLatestResult();
      double resultTimestamp = pipelineResult.getTimestampSeconds();

      if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
        previousPipelineTimestamp = resultTimestamp;
        var target = pipelineResult.getBestTarget();

        if (target.getPoseAmbiguity() <= .05) {
          Transform3d camToTarget = target.getBestCameraToTarget();
          Transform3d targetToCamera = camToTarget.inverse();

          Pose3d targetPose = PhotonConstants.APRILTAG_LOCATIONS[target.getFiducialId()];
          Pose3d camPose = targetPose.transformBy(targetToCamera);

          Pose2d visionMeasurement = camPose.transformBy(PhotonConstants.CAMERAS_TO_ROBOT[i]).toPose2d();

          Drivetrain.addVisionMeasurement(visionMeasurement, resultTimestamp);
        }
      }
    }
  }
}
