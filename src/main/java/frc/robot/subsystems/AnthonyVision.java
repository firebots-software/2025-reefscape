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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * VisionSystem fuses multi-tag AprilTag measurements from PhotonVision with swerve odometry by
 * dynamically computing measurement noise based on tag count, distance, viewing angle, and robot
 * speed.
 */
public class AnthonyVision extends SubsystemBase {
  // static member of AnthonyVision that contains array of all existing AnthonyVision systems
  private static AnthonyVision[] systemList =
      new AnthonyVision[Constants.Vision.Cameras.values().length];

  // Data type is "Cameras", an enum defined in Constants.java with only two options (left, right)
  private final Constants.Vision.Cameras cameraId;

  // Reef tag IDs for each side of the field
  private static final List<Integer> BLUE_SIDE_TAG_IDS = List.of(19, 20, 21, 22, 17, 18);
  private static final List<Integer> RED_SIDE_TAG_IDS = List.of(6, 7, 8, 9, 10, 11);

  // Noise parameters
  private double calibrationFactor = 1.0; // constant multiplier to everything
  private double baseNoiseX = 0.0008; // meters
  private double baseNoiseY = 0.0008;
  private double baseNoiseTheta = 0.5; // radians

  // private double distanceCoefficientX = 0.06;
  // private double distanceCoefficientY = 0.06;

  private double distanceExponentialCoefficientX = 0.00046074;
  private double distanceExponentialBaseX = 2.97294;
  private double distanceExponentialCoefficientY = 0.00046074;
  private double distanceExponentialBaseY = 2.97294;

  private double distanceCoefficientTheta = 0.9;

  private double angleCoefficientX = 0.5; // noise growth per radian of viewing angle
  private double angleCoefficientY = 0.5;
  private double angleCoefficientTheta = 0.5;

  private double speedCoefficientX = 0.5; // noise growth per fraction of max speed
  private double speedCoefficientY = 0.5;
  private double speedCoefficientTheta = 0.5;

  // Maximums for normalization
  private double maximumRobotSpeed = 5; // meters per second
  private double maximumAllowedDistance = 15.0; // meters, beyond which readings are dropped

  // PhotonVision and odometry references
  private final PhotonCamera photonCamera;
  private final PhotonPoseEstimator poseEstimator; // MULTI_TAG_PNP_ON_COPROCESSOR
  private PhotonPipelineResult latestVisionResult;
  private final BooleanSupplier isRedSide;
  private SwerveSubsystem swerveDrive;
  private final AprilTagFieldLayout fieldLayout;

  public AnthonyVision(Constants.Vision.Cameras cameraId, BooleanSupplier isRedSide) {
    this.isRedSide = isRedSide;
    this.cameraId = cameraId;
    photonCamera = new PhotonCamera(cameraId.toString());
    Transform3d cameraToRobot = Constants.Vision.getCameraTransform(cameraId);

    // Initialize field layout
    this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    // Initialize both pose estimators
    poseEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraToRobot);

    latestVisionResult = null;
  }

  // Get instance of AnthonyVision
  public static AnthonyVision getInstance(
      Constants.Vision.Cameras cameraId, BooleanSupplier isRedSide) {
    int idx = cameraId.ordinal();
    if (systemList[idx] == null) {
      systemList[idx] = new AnthonyVision(cameraId, isRedSide);
    }
    return systemList[idx];
  }

  @Override
  public void periodic() {
    // Initialize swerve drive if not already done
    if (swerveDrive == null) {
      swerveDrive = SwerveSubsystem.getInstance();
    }

    // Check camera connection
    boolean cameraConnected = photonCamera.isConnected();

    // If the current camera isn't connected, there's nothing to do here
    if (!cameraConnected) {
      return;
    }

    // Get all unread results
    List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();

    // Go through all results (if there are any) and update the latest result with the last
    for (var result : results) {
      latestVisionResult = result;
    }
  }

  /**
   * Internal method to handle pose estimation with optional confidence checking.
   *
   * @param useTrigsolve If true, uses trigsolve strategy
   * @param forceAdd If true, bypasses confidence checking (used for non-trigsolve or single camera)
   * @return PoseEstimateResult containing the pose and confidence data, or null if failed
   */
  public void addFilteredPose() {
    PhotonPoseEstimator selectedEstimator = poseEstimator;
    String camTitle = cameraId.getLoggingName();
    if (latestVisionResult == null || !latestVisionResult.hasTargets()) {
      return;
    }

    double averageDistance =
        latestVisionResult.getTargets().stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(Double.NaN);

    double minDistance =
        latestVisionResult.getTargets().stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .min()
            .orElse(Double.NaN);

    // Filter tags to only those on the active side
    List<PhotonTrackedTarget> validTags =
        latestVisionResult.getTargets().stream()
            .filter(t -> isTagOnActiveSide(t.getFiducialId()))
            .filter(t -> isNotChopped(t.getYaw()))
            .collect(Collectors.toList());

    DogLog.log("Vision/" + camTitle + "/numValidTags", validTags.size());

    for (PhotonTrackedTarget tag : validTags) {
      DogLog.log("Vision/" + camTitle + "/Area", tag.getArea());
      DogLog.log("Vision/" + camTitle + "/Yaw", tag.getYaw());
    }

    // Log all detected tags for debugging
    String allTagIds =
        latestVisionResult.getTargets().stream()
            .map(t -> Integer.toString(t.getFiducialId()))
            .collect(Collectors.joining(","));

    DogLog.log("Vision/" + camTitle + "/allTagIds", allTagIds);

    if (validTags.isEmpty()) {
      return;
    }

    // Log all tags that haven't been thrown out
    int tagCount = validTags.size();

    // Compute effective metrics for solution
    // Use camera→target distance from PV (avoids odometry dependence)

    // nothing to do if rejected based on the minDistance or if no min dist has been found
    if (Double.isNaN(minDistance) || minDistance > maximumAllowedDistance) {
      return;
    }

    // find the current speed
    double currentSpeed =
        Math.hypot(
            swerveDrive.getRobotSpeeds().vxMetersPerSecond,
            swerveDrive.getRobotSpeeds().vyMetersPerSecond);

    // Get the pose from PhotonVision
    Optional<EstimatedRobotPose> maybePose = selectedEstimator.update(latestVisionResult);
    if (maybePose.isEmpty()) {
      return;
    }

    EstimatedRobotPose estimatedPose = maybePose.get();
    Pose2d measuredPose = estimatedPose.estimatedPose.toPose2d();

    double nX =
        computeNoiseXY(
            baseNoiseX,
            distanceExponentialCoefficientX,
            distanceExponentialBaseX,
            angleCoefficientX,
            speedCoefficientX,
            averageDistance,
            currentSpeed,
            tagCount);
    double nY =
        computeNoiseXY(
            baseNoiseY,
            distanceExponentialCoefficientY,
            distanceExponentialBaseY,
            angleCoefficientY,
            speedCoefficientY,
            averageDistance,
            currentSpeed,
            tagCount);
    double nTH =
        computeNoiseHeading(
            baseNoiseTheta,
            distanceCoefficientTheta,
            angleCoefficientTheta,
            speedCoefficientTheta,
            averageDistance,
            currentSpeed,
            tagCount);

    DogLog.log("Vision/" + camTitle + "/speed", currentSpeed);
    DogLog.log("Vision/" + camTitle + "/nX", nX);
    DogLog.log("Vision/" + camTitle + "/nY", nY);
    DogLog.log("Vision/" + camTitle + "/nTH", nTH);
    DogLog.log("Vision/" + camTitle + "/Pose", measuredPose);
    DogLog.log("Vision/" + camTitle + "/averageDistance", averageDistance);

    Matrix<N3, N1> noiseVector = VecBuilder.fill(nX, nY, nTH);
    // Process locally (no cross-camera comparison)
    processPoseEstimate(
        measuredPose,
        averageDistance,
        currentSpeed,
        tagCount,
        latestVisionResult.getTimestampSeconds(),
        noiseVector);
  }

  /** Final processing and addition of pose estimate to odometry. */
  private void processPoseEstimate(
      Pose2d measuredPose,
      double averageDistance,
      double currentSpeed,
      int tagCount,
      double timestamp,
      Matrix<N3, N1> noiseVector) {
    // Choose timestamp: use vision timestamp unless it differs too much from FPGA
    double fpgaTimestamp = Timer.getFPGATimestamp();
    double timestampDifference = Math.abs(timestamp - fpgaTimestamp);
    double chosenTimestamp = (timestampDifference > 0.5) ? fpgaTimestamp - 0.03 : timestamp;

    // Build the noise vector and add the vision measurement

    swerveDrive.addVisionMeasurement(measuredPose, chosenTimestamp, noiseVector);
  }

  private boolean isTagOnActiveSide(int tagId) {
    return isRedSide.getAsBoolean()
        ? RED_SIDE_TAG_IDS.contains(tagId)
        : BLUE_SIDE_TAG_IDS.contains(tagId);
  }

  private boolean isNotChopped(double yaw) {
    return (Math.abs(yaw) < 60d);
  }

  private double computeNoiseXY(
      double baseNoise,
      double distanceExponentialCoefficient,
      double distanceExponentialBase,
      double angleCoefficient,
      double speedCoefficient,
      double distance,
      double robotSpeed,
      int tagCount) {

    // Tag count factor (diminishing returns; cap at 4)
    int effectiveTags = Math.min(tagCount, 4);
    double tagFactor = 1.0 / Math.sqrt(effectiveTags);

    // Distance term (keep as d^2)
    double distanceFactor = baseNoise + distanceExponentialCoefficient*Math.pow(distanceExponentialBase, distance);

    // Speed term (quadratic, saturated)
    double vNorm = Math.min(robotSpeed, maximumRobotSpeed) / maximumRobotSpeed;
    double speedFactor = 1.0 + speedCoefficient * (vNorm * vNorm);
    DogLog.log("Vision/calibrationFactor", calibrationFactor);
    DogLog.log("Vision/tagFactor", tagFactor);
    DogLog.log("Vision/distanceFactor", distanceFactor);
    DogLog.log("Vision/speedFactor", speedFactor);
    
    double computedStdDevs = calibrationFactor * tagFactor * distanceFactor * speedFactor;
    return computedStdDevs;
  }


  private double computeNoiseHeading(
      double baseNoise,
      double distanceCoefficient,
      double angleCoefficient,
      double speedCoefficient,
      double distance,
      double robotSpeed,
      int tagCount) {

    // Tag count factor (diminishing returns; cap at 4)
    int effectiveTags = Math.min(tagCount, 4);
    double tagFactor = 1.0 / Math.sqrt(effectiveTags);

    // Distance term (keep as d^2)
    double distanceFactor = baseNoise + distanceCoefficient * distance * distance;

    // Speed term (quadratic, saturated)
    double vNorm = Math.min(robotSpeed, maximumRobotSpeed) / maximumRobotSpeed;
    double speedFactor = 1.0 + speedCoefficient * (vNorm * vNorm);

    double computedStdDevs = calibrationFactor * tagFactor * distanceFactor * speedFactor;
    return computedStdDevs;
  }
}
