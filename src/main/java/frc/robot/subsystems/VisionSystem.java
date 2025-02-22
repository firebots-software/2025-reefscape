// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
    // right Camera transform
    new Transform3d(
        new Translation3d(
            Constants.Vision.RIGHT_CAM_TO_ROBOT_TRANSLATION_X,
            Constants.Vision.RIGHT_CAM_TO_ROBOT_TRANSLATION_Y,
            Constants.Vision.RIGHT_CAM_TO_ROBOT_TRANSLATION_Z),
        new Rotation3d(
            Constants.Vision.RIGHT_CAM_TO_ROBOT_ROTATION_ROLL,
            Constants.Vision.RIGHT_CAM_TO_ROBOT_ROTATION_PITCH,
            Constants.Vision.RIGHT_CAM_TO_ROBOT_ROTATION_YAW)),
    // left Camera
    new Transform3d(
        new Translation3d(
            Constants.Vision.LEFT_CAM_TO_ROBOT_TRANSLATION_X,
            Constants.Vision.LEFT_CAM_TO_ROBOT_TRANSLATION_Y,
            Constants.Vision.LEFT_CAM_TO_ROBOT_TRANSLATION_Z),
        new Rotation3d(
            Constants.Vision.LEFT_CAM_TO_ROBOT_ROTATION_ROLL,
            Constants.Vision.LEFT_CAM_TO_ROBOT_ROTATION_PITCH,
            Constants.Vision.LEFT_CAM_TO_ROBOT_ROTATION_YAW)),
  };
  private PhotonCamera camera;
  private PhotonPipelineResult pipeline;
  AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  PhotonPoseEstimator photonPoseEstimator;
  private SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();

  public VisionSystem(String name, int index) {

    camera = new PhotonCamera(name);
    photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camToRobots[index]);

    for (var x : camera.getAllUnreadResults()) {
      pipeline = x;
    }
  }

  public Double getDistance() {
    return this.getAprilTagFieldLayout()
        .getTagPose(getPipelineResult().getBestTarget().getFiducialId())
        .get()
        .getTranslation()
        .getDistance(
            new Translation3d(
                driveTrain.getState().Pose.getX(), driveTrain.getState().Pose.getY(), 0.0));
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

  public void DoKalman(VisionSystem secondCam){
    double leastPoseAmbDist;
    Optional<EstimatedRobotPose> firstCamPose = this.getMultiTagPose3d(driveTrain.getState().Pose);
    Optional<EstimatedRobotPose> secondRobotPose =  secondCam.getMultiTagPose3d(driveTrain.getState().Pose);

    Optional<EstimatedRobotPose> bestRobotPose;

    PhotonPipelineResult pipeline1 = this.getPipelineResult();
    PhotonPipelineResult pipeline2 = secondCam.getPipelineResult();

    
    // if both present, else if right present, else if left present
    if (this.hasTarget(pipeline1)
        && firstCamPose.isPresent()
        && secondCam.hasTarget(pipeline2)
        && secondRobotPose.isPresent()) {
      double thisPoseAmb = pipeline1.getBestTarget().getPoseAmbiguity();
      double otherPoseAmb = pipeline2.getBestTarget().getPoseAmbiguity();
      if (thisPoseAmb < otherPoseAmb) {
        leastPoseAmbDist = this.getDistance();
        bestRobotPose = firstCamPose;
      }
      leastPoseAmbDist = secondCam.getDistance();
      bestRobotPose = secondRobotPose;

    } else if (this.hasTarget(pipeline1) && firstCamPose.isPresent()) {

      leastPoseAmbDist = this.getDistance();
      bestRobotPose = firstCamPose;
  
    } else if (secondCam.hasTarget(pipeline2) && secondRobotPose.isPresent()) {
      leastPoseAmbDist = secondCam.getDistance();
      bestRobotPose = secondRobotPose;
    } else {
      return;
    }

    double xKalman = 0.1 * Math.pow(1.15, leastPoseAmbDist);
    double yKalman = 0.1 * Math.pow(1.4, leastPoseAmbDist);

    Matrix<N3, N1> visionMatrix = VecBuilder.fill(xKalman, yKalman, 100d);
    Pose2d bestRobotPose2d = bestRobotPose.get().estimatedPose.toPose2d();
    Pose2d rotationLess =
        new Pose2d(bestRobotPose2d.getTranslation(), driveTrain.getState().Pose.getRotation());

    driveTrain.addVisionMeasurement(
        rotationLess, pipeline1.getTimestampSeconds(), visionMatrix);

  }

  // use this when feeding into drivetrain
  public Optional<EstimatedRobotPose> getMultiTagPose3d(Pose2d previousRobotPose) {
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

  public boolean hasTarget(PhotonPipelineResult pipeline) {
    return pipeline.hasTargets();
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

  public AprilTagFieldLayout getAprilTagFieldLayout() {
    return this.aprilTagFieldLayout;
  }

  public PhotonPipelineResult getPipelineResult() {
    return pipeline;
  }

  @Override
  public void periodic() {
    for (var x : camera.getAllUnreadResults()) {
      pipeline = x;
    }
  }
}
