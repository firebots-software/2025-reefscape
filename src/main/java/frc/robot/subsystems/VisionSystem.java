// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/** Creates a new VisionSystem. */
public class VisionSystem extends SubsystemBase {
 
  Pose2d savedResult = new Pose2d(0, 0, new Rotation2d(0.01, 0.01));
  private static VisionSystem[] systemList =
      new VisionSystem[Constants.Vision.Cameras.values().length];
  private Transform3d[] camToRobots = {
    // Front Camera transform
    new Transform3d(
        new Translation3d(Units.inchesToMeters(-9.5), Units.inchesToMeters(6.5), 0),
        new Rotation3d(0, Units.degreesToRadians(25.37693353), Math.PI)),
    // Back Camera
    new Transform3d(
        new Translation3d(Units.inchesToMeters(8), Units.inchesToMeters(5.75), 0),
        new Rotation3d(0, 0, 0))
  };
  private PhotonCamera camera;
  private PhotonPipelineResult pipeline;
  AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  PhotonPoseEstimator photonPoseEstimator;

  public VisionSystem(String name, int index) {

    camera = new PhotonCamera(name);
    photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camToRobots[index]);
  }

  public static VisionSystem getInstance(Constants.Vision.Cameras name) {
    if (systemList[name.ordinal()] == null) {
      systemList[0] =
          new VisionSystem(
              Constants.Vision.Cameras.RIGHT_CAM.toString(),
              Constants.Vision.Cameras.RIGHT_CAM.ordinal());
      systemList[1] =
          new VisionSystem(
              Constants.Vision.Cameras.LEFT_CAM.toString(),
              Constants.Vision.Cameras.LEFT_CAM.ordinal());
    }

    return systemList[name.ordinal()];
  }

  // use this when feeding into drivetrain
  public Optional<EstimatedRobotPose> getMultiTagPose3d(Pose2d previousRobotPose) {
    pipeline = camera.getLatestResult();
    photonPoseEstimator.setReferencePose(previousRobotPose);
    return photonPoseEstimator.update(pipeline);
  }

  public Pose2d getPose2d() {
    Optional<EstimatedRobotPose> pose3d = getMultiTagPose3d(savedResult);
    if (pose3d.isEmpty()) return null;
    savedResult = pose3d.get().estimatedPose.toPose2d();
    return savedResult;
  }

  public Pose2d getSaved() {
    return savedResult;
  }

  public void setReference(Pose2d newPose) {
    if (newPose == null) {
      return;
    }
    savedResult = newPose;
  }

  public static Pose2d getAverageForOffBotTesting(Pose2d one, Pose2d two) {
    if (one == null) return two;
    if (two == null) return one;

    return new Pose2d(
        (one.getX() + two.getX()) / 2, (one.getY() + two.getY()) / 2, one.getRotation());
  }

  public AprilTagFieldLayout gAprilTagFieldLayout() {
    return this.aprilTagFieldLayout;
  }

  public PhotonPipelineResult gPipelineResult() {
    return camera.getLatestResult();
  }

  @Override
  public void periodic() {}
}
