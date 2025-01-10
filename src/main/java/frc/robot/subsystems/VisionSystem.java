// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSystem extends SubsystemBase {
  /** Creates a new VisionSystem. */
  PhotonCamera frontCamera = new PhotonCamera("front-camera");

  public VisionSystem() {}

  public Transform3d getTransformToTarget(){
    PhotonPipelineResult result = frontCamera.getLatestResult();
    if(!result.hasTargets()){
      return null; //TODO: Fill out later
    }
    PhotonTrackedTarget target = result.getBestTarget();
    Transform3d pose = target.getBestCameraToTarget();
    return pose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
