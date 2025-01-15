// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Creates a new VisionSystem. */
public class VisionSystem extends SubsystemBase {
  private final Telemetry logger = new Telemetry(10);
  Pose3d savedResult = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
  private static PhotonCamera frontCamera = new PhotonCamera("front-camera");
  private static PhotonCamera secondCamera = new PhotonCamera("second-camera");
  private PhotonCamera camera;
  private PhotonPipelineResult pipeline;
  // load 2025 april tag field layout
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
  PhotonPoseEstimator photonPoseEstimator;
  // robot relative to camera **FIX FOR NEW ROBOT**
  Transform3d camToRobot =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(-13), 0, Units.inchesToMeters(7.027)),
          new Rotation3d(0, -Units.degreesToRadians(37), Math.PI));

  private static VisionSystem frontSystem;

  public VisionSystem(String name) {
    camera = new PhotonCamera(name);
    photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camToRobot);
    pipeline = camera.getLatestResult();
  }

  public static VisionSystem getInstance(String name) {
    if (name == "front-camera") {
      if (frontSystem == null) {
        frontSystem = new VisionSystem("front-camera");
        return frontSystem;

      } else {
        return frontSystem;
      }
    }
       if (name == "second-camera") {
      if (frontSystem == null) {
        frontSystem = new VisionSystem("second-camera");
        return frontSystem;

      } else {
        return frontSystem;
      }
    }
    return frontSystem;
  }

  public Transform3d getTransformToTarget() {
    PhotonPipelineResult result = frontCamera.getLatestResult();
    if (!result.hasTargets()) {
      return new Transform3d(0, 0, 0, new Rotation3d());
    }
    PhotonTrackedTarget target = result.getBestTarget();
    Transform3d pose = target.getBestCameraToTarget();
    return pose;
  }

  // use this when feeding into drivetrain
  public Optional<EstimatedRobotPose> getMultiTagPose3d(Pose2d previousRobotPose) {
    photonPoseEstimator.setReferencePose(previousRobotPose);
    return photonPoseEstimator.update(pipeline);
  }

  // use this when feeding into drivetrain withouth previous pose
  public Optional<EstimatedRobotPose> noRefPose3d() {

    return photonPoseEstimator.update(pipeline);
  }

  // without multitag, prolly not going to use this
  public Pose3d getRobotPose3d() {
    if (!pipeline.hasTargets()) {
      return savedResult;
    }
    Optional<Pose3d> tagPose =
        aprilTagFieldLayout.getTagPose(pipeline.getBestTarget().getFiducialId());
    if (tagPose.isEmpty()) {
      return savedResult;
    } else {
      savedResult =
          PhotonUtils.estimateFieldToRobotAprilTag(
              getTransformToTarget(), tagPose.get(), camToRobot);
    }
    return savedResult;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (frontSystem.getRobotPose3d()) {
    //   SmartDashboard.putBoolean("No Pose", true);
    // } else {
    //   SmartDashboard.putBoolean("No Pose", false);
    // }
    logger.logVisionPose(frontSystem.getMultiTagPose3d(new Pose2d()));
  }
}
