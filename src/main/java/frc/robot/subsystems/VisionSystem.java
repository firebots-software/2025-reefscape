// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MiscUtils;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Creates a new VisionSystem. */
public class VisionSystem extends SubsystemBase {

  public static int numThrowaways = 0;
  public static int numNotThrowaways = 0;
  List<Integer> reefIDs =
      new ArrayList<Integer>(Arrays.asList(19, 20, 21, 22, 17, 18, 6, 7, 8, 9, 10, 11));

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
  private Constants.Vision.Cameras cameraEnum;
  private PhotonPipelineResult pipeline;
  AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  PhotonPoseEstimator photonPoseEstimator;
  private SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();

  public VisionSystem(Constants.Vision.Cameras cameraEnum) {
    this.cameraEnum = cameraEnum;
    String name = cameraEnum.toString();
    int index = cameraEnum.ordinal();
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
      systemList[name.ordinal()] = new VisionSystem(name);
    }

    return systemList[name.ordinal()];
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
    if (pipeline == null) {
      return false;
    }
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

  public void addFilteredPose() {
    String camName = cameraEnum.getLoggingName();
    PhotonPipelineResult pipelineResult = getPipelineResult();

    DogLog.log("KalmanDebug/" + camName + "PiplineNull", pipelineResult == null);
    DogLog.log("KalmanDebug/" + camName + "PipelineHasTarget", hasTarget(pipelineResult));

    if (pipelineResult != null && hasTarget(pipelineResult)) {

      List<PhotonTrackedTarget> targets = pipelineResult.getTargets();

      boolean hasReefTag = true;
      double poseAmbiguity = pipelineResult.getBestTarget().poseAmbiguity;
      double distance = getDistance();

      DogLog.log("KalmanDebug/" + camName + "PoseAmbiguity", poseAmbiguity);
      DogLog.log("KalmanDebug/" + camName + "DistToAprilTag", distance);

      for (PhotonTrackedTarget target : targets) {
        if (!reefIDs.contains(target.getFiducialId())) {
          hasReefTag = false;
        }
      }

      if (hasReefTag) {
        double speedMultiplier =
            (Math.sqrt(
                        Math.pow(driveTrain.getRobotSpeeds().vxMetersPerSecond, 2)
                            + Math.pow(driveTrain.getRobotSpeeds().vyMetersPerSecond, 2)))
                    / 2
                + 1.0;
        double xKalman = MiscUtils.lerp((distance - 0.62) / 3, 0.03, 0.3, 1.0) * speedMultiplier;
        double yKalman = MiscUtils.lerp((distance - 0.62) / 3, 0.03, 0.3, 1.0) * speedMultiplier;
        // double speedMultiplier = 1;
        // if (Math.sqrt(
        //         Math.pow(driveTrain.getRobotSpeeds().vxMetersPerSecond, 2)
        //             + Math.pow(driveTrain.getRobotSpeeds().vyMetersPerSecond, 2))
        //     > 1) {
        //   speedMultiplier = 2;
        // }
        // speedMultiplier =
        //     (Math.sqrt(
        //                 Math.pow(driveTrain.getRobotSpeeds().vxMetersPerSecond, 2)
        //                     + Math.pow(driveTrain.getRobotSpeeds().vyMetersPerSecond, 2)))
        //             / 2
        //         + 1.0;
        // double xKalman = MiscUtils.lerp((distance - 0.6) / 2.4, 0.04, 0.5, 1.0) *
        // speedMultiplier;
        // double yKalman = MiscUtils.lerp((distance - 0.6) / 2.4, 0.04, 0.5, 1.0) *
        // speedMultiplier;
        double rotationKalman = MiscUtils.lerp((distance - 0.6) / 1.4, 0.4, 5, 30) / 10;

        DogLog.log("KalmanDebug/" + camName + "TranslationStandardDeviation", xKalman);
        DogLog.log("KalmanDebug/" + camName + "RotationStandardDeviation", rotationKalman);

        Matrix<N3, N1> visionMatrix = VecBuilder.fill(xKalman, yKalman, rotationKalman);
        Pose2d bestRobotPose2d = getPose2d();

        if (bestRobotPose2d == null) {
          VisionSystem.numThrowaways++;
          DogLog.log("KalmanDebug/" + camName + "visionUsed", false);
          DogLog.log("KalmanDebug/" + camName + "throwaways", VisionSystem.numThrowaways);
          return;
        }
        Pose2d rotationLess =
            new Pose2d(bestRobotPose2d.getTranslation(), driveTrain.getState().Pose.getRotation());
        DogLog.log("KalmanDebug/" + camName + "Rotationless", rotationLess);
        DogLog.log("KalmanDebug/" + camName + "RobotPoseIsPresent", bestRobotPose2d != null);
        DogLog.log("KalmanDebug/" + camName + "VisionPose", bestRobotPose2d);

        double xDifference = Math.abs(driveTrain.getPose().getX() - bestRobotPose2d.getX());
        double yDifference = Math.abs(driveTrain.getPose().getY() - bestRobotPose2d.getY());
        double rotDifference =
            Math.abs(
                driveTrain.getPose().getRotation().getDegrees()
                    - bestRobotPose2d.getRotation().getDegrees());
        Translation2d transDifference = new Translation2d(xDifference, yDifference);
        Rotation2d rot2dDifference = new Rotation2d(rotDifference);

        Pose2d visionDiff = new Pose2d(transDifference, rot2dDifference);
        double timeDiff = Math.abs(pipelineResult.getTimestampSeconds() - Timer.getTimestamp());
        DogLog.log("KalmanDebug/" + camName + "visionDiff", visionDiff);
        DogLog.log("KalmanDebug/" + camName + "diffInTimestamps", timeDiff);

        driveTrain.addVisionMeasurement(
            bestRobotPose2d,
            timeDiff > 5 ? Timer.getTimestamp() - 0.070 : pipelineResult.getTimestampSeconds(),
            visionMatrix);
        VisionSystem.numNotThrowaways++;
        DogLog.log("KalmanDebug/" + camName + "numNotThrowaway", VisionSystem.numNotThrowaways);
        DogLog.log("KalmanDebug/" + camName + "visionUsed", true);
      } else {
        DogLog.log("KalmanDebug/" + camName + "visionUsed", false);
      }
    } else {
      DogLog.log("KalmanDebug/" + camName + "visionUsed", false);
    }
  }

  @Override
  public void periodic() {
    for (var x : camera.getAllUnreadResults()) {
      pipeline = x;
    }
  }
}
