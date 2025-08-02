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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
public class VisionSystem extends SubsystemBase {
  private static VisionSystem[] systemList =
      new VisionSystem[Constants.Vision.Cameras.values().length];

  private final Constants.Vision.Cameras cameraId;
  // Reef tag IDs for each side of the field
  private static final List<Integer> BLUE_SIDE_TAG_IDS = List.of(19, 20, 21, 22, 17, 18);
  private static final List<Integer> RED_SIDE_TAG_IDS = List.of(6, 7, 8, 9, 10, 11);

  // Base noise tuning parameters (tweakable)
  private double calibrationFactor = 1.0;
  private double baseNoiseX = 0.02; // meters
  private double baseNoiseY = 0.02;
  private double baseNoiseTheta = 0.5; // radians

  private double distanceCoefficientX = 0.4; // noise growth per meter
  private double distanceCoefficientY = 0.4;
  private double distanceCoefficientTheta = 1;

  private double angleCoefficientX = 0.5; // noise growth per radian of viewing angle
  private double angleCoefficientY = 0.5;
  private double angleCoefficientTheta = 0.5;

  private double speedCoefficientX = 0.5; // noise growth per fraction of max speed
  private double speedCoefficientY = 0.5;
  private double speedCoefficientTheta = 0.5;

  // Maximums for normalization
  private double maximumViewingAngle = Math.toRadians(90.0);
  private double maximumRobotSpeed = 5; // meters per second
  private double maximumAllowedDistance = 15.0; // meters, beyond which readings are dropped

  // PhotonVision and odometry references
  private final PhotonCamera photonCamera;
  private final PhotonPoseEstimator poseEstimator; // MULTI_TAG_PNP_ON_COPROCESSOR
  private final PhotonPoseEstimator trigsolvePoseEstimator; // LOWEST_AMBIGUITY with trigsolve
  private PhotonPipelineResult latestVisionResult;
  private final BooleanSupplier isRedSide;
  private Pose2d lastKnownPose = new Pose2d(0, 0, new Rotation2d());
  private Pose2d lastKnownTrigsolvePose = new Pose2d(0, 0, new Rotation2d());
  private final SwerveSubsystem swerveDrive = SwerveSubsystem.getInstance();
  private final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  public VisionSystem(Constants.Vision.Cameras cameraId, BooleanSupplier isRedSide) {
    this.isRedSide = isRedSide;
    this.cameraId = cameraId;
    photonCamera = new PhotonCamera(cameraId.toString());
    Transform3d cameraToRobot = Constants.Vision.getCameraTransform(cameraId);
    
    // Initialize both pose estimators
    poseEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraToRobot);
    
    trigsolvePoseEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, cameraToRobot);
    
    latestVisionResult = null;
  }

  public static VisionSystem getInstance(
      Constants.Vision.Cameras cameraId, BooleanSupplier isRedSide) {
    int idx = cameraId.ordinal();
    if (systemList[idx] == null) {
      systemList[idx] = new VisionSystem(cameraId, isRedSide);
    }
    return systemList[idx];
  }

  @Override
  public void periodic() {
    for (var x : photonCamera.getAllUnreadResults()) {
      latestVisionResult = x;
    }
  }

  /**
   * Attempts to fuse a vision measurement into the swerve pose estimator using MULTI_TAG_PNP strategy,
   * dropping readings that fail validity checks, and computing noise dynamically via computeMeasurementNoise().
   */
  public void addFilteredPose() {
    addFilteredPose(false);
  }

  /**
   * Attempts to fuse a vision measurement into the swerve pose estimator, dropping readings that
   * fail validity checks, and computing noise dynamically via computeMeasurementNoise().
   * 
   * @param useTrigsolve If true, uses the trigsolve pose estimator (PNP_DISTANCE_TRIG_SOLVE strategy).
   *                     If false, uses the multi-tag PnP pose estimator (MULTI_TAG_PNP_ON_COPROCESSOR strategy).
   */
  public void addFilteredPose(boolean useTrigsolve) {
    addFilteredPoseInternal(useTrigsolve, false);
  }

  /**
   * Internal method to handle pose estimation with optional confidence checking.
   * 
   * @param useTrigsolve If true, uses trigsolve strategy
   * @param forceAdd If true, bypasses confidence checking (used for non-trigsolve or single camera)
   * @return PoseEstimateResult containing the pose and confidence data, or null if failed
   */
  private PoseEstimateResult addFilteredPoseInternal(boolean useTrigsolve, boolean forceAdd) {
    if (latestVisionResult == null || !latestVisionResult.hasTargets()) {
      if (forceAdd) return null;
      DogLog.log("Vision/MeasurementUsed", false);
      return null;
    }

    // Filter tags to only those on the active side
    List<PhotonTrackedTarget> validTags =
        latestVisionResult.getTargets().stream()
            .filter(t -> isTagOnActiveSide(t.getFiducialId()))
            .collect(Collectors.toList());
    if (validTags.isEmpty()) {
      if (forceAdd) return null;
      DogLog.log("Vision/TagFilter", false);
      return null;
    }

    // Compute effective metrics for solution
    int tagCount = validTags.size();
    double averageDistance =
        validTags.stream()
            .mapToDouble(t -> getDistanceToTag(t.getFiducialId()))
            .average()
            .orElse(Double.NaN);
    if (Double.isNaN(averageDistance) || averageDistance > maximumAllowedDistance) {
      if (forceAdd) return null;
      DogLog.log("Vision/DistanceFilter", true);
      return null;
    }
    double averageAngle =
        validTags.stream()
            .mapToDouble(PhotonTrackedTarget::getYaw)
            .map(Math::toRadians)
            .map(Math::abs)
            .average()
            .orElse(0.0);
    double currentSpeed =
        Math.hypot(
            swerveDrive.getRobotSpeeds().vxMetersPerSecond,
            swerveDrive.getRobotSpeeds().vyMetersPerSecond);

    // Choose the appropriate pose estimator and reference pose
    PhotonPoseEstimator selectedEstimator = useTrigsolve ? trigsolvePoseEstimator : poseEstimator;
    Pose2d referencePos = useTrigsolve ? lastKnownTrigsolvePose : lastKnownPose;
    
    // Get the pose from PhotonVision
    selectedEstimator.setReferencePose(referencePos);
    Optional<EstimatedRobotPose> maybePose = selectedEstimator.update(latestVisionResult);
    if (maybePose.isEmpty()) {
      if (forceAdd) return null;
      String strategyPrefix = useTrigsolve ? "Trigsolve" : "MultiTag";
      DogLog.log("Vision/" + strategyPrefix + "PoseEstimateFailed", true);
      return null;
    }
    
    EstimatedRobotPose estimatedPose = maybePose.get();
    Pose2d measuredPose = estimatedPose.estimatedPose.toPose2d();
    
    // Calculate confidence metric for trigsolve based on measurement quality
    double confidence = 1.0; // Default confidence for multi-tag
    if (useTrigsolve) {
      // For trigsolve, calculate confidence based on the measurement noise multipliers
      // Lower total noise = higher confidence
      double noiseX =
          computeMeasurementNoise(
              baseNoiseX,
              distanceCoefficientX,
              angleCoefficientX,
              speedCoefficientX,
              averageDistance,
              averageAngle,
              currentSpeed,
              tagCount);
      double noiseY =
          computeMeasurementNoise(
              baseNoiseY,
              distanceCoefficientY,
              angleCoefficientY,
              speedCoefficientY,
              averageDistance,
              averageAngle,
              currentSpeed,
              tagCount);
      double noiseTheta =
          computeMeasurementNoise(
              baseNoiseTheta,
              distanceCoefficientTheta,
              angleCoefficientTheta,
              speedCoefficientTheta,
              averageDistance,
              averageAngle,
              currentSpeed,
              tagCount);
      
      // Calculate overall measurement quality (lower noise = higher confidence)
      double totalNoise = Math.sqrt(noiseX * noiseX + noiseY * noiseY + noiseTheta * noiseTheta);
      confidence = 1.0 / (1.0 + totalNoise);
    }
    
    // If this is just for confidence comparison, return the result without adding to odometry
    if (forceAdd) {
      return new PoseEstimateResult(measuredPose, confidence, averageDistance, averageAngle, 
                                   currentSpeed, tagCount, latestVisionResult.getTimestampSeconds());
    }

    // For non-trigsolve or when not doing confidence comparison, proceed normally
    if (!useTrigsolve) {
      return processPoseEstimate(measuredPose, useTrigsolve, averageDistance, averageAngle, 
                                currentSpeed, tagCount, latestVisionResult.getTimestampSeconds());
    }

    // For trigsolve, check confidence against other cameras
    return processTrigsolveWithConfidenceCheck(measuredPose, confidence, averageDistance, 
                                              averageAngle, currentSpeed, tagCount, 
                                              latestVisionResult.getTimestampSeconds());
  }

  /**
   * Processes trigsolve pose estimation with confidence checking across all cameras.
   */
  private PoseEstimateResult processTrigsolveWithConfidenceCheck(Pose2d measuredPose, double confidence,
      double averageDistance, double averageAngle, double currentSpeed, int tagCount, double timestamp) {
    
    // Get results from all other cameras for confidence comparison
    double bestConfidence = confidence;
    PoseEstimateResult bestResult = new PoseEstimateResult(measuredPose, confidence, averageDistance, 
                                                          averageAngle, currentSpeed, tagCount, timestamp);
    
    // Check all other camera instances
    for (Constants.Vision.Cameras otherCamera : Constants.Vision.Cameras.values()) {
      if (otherCamera == this.cameraId) continue; // Skip self
      
      VisionSystem otherSystem = systemList[otherCamera.ordinal()];
      if (otherSystem != null) {
        PoseEstimateResult otherResult = otherSystem.addFilteredPoseInternal(true, true);
        if (otherResult != null && otherResult.confidence > bestConfidence) {
          bestConfidence = otherResult.confidence;
          bestResult = otherResult;
        }
      }
    }
    
    // Only proceed if this camera has the best confidence
    if (bestResult.pose == measuredPose) { // This camera won
      return processPoseEstimate(measuredPose, true, averageDistance, averageAngle, 
                                currentSpeed, tagCount, timestamp);
    } else {
      // Another camera had better confidence, don't add measurement
      DogLog.log("Vision/TrigsolveLowerConfidence", true);
      DogLog.log("Vision/TrigsolveOurConfidence", confidence);
      DogLog.log("Vision/TrigsolveBestConfidence", bestConfidence);
      return null;
    }
  }

  /**
   * Final processing and addition of pose estimate to odometry.
   */
  private PoseEstimateResult processPoseEstimate(Pose2d measuredPose, boolean useTrigsolve,
      double averageDistance, double averageAngle, double currentSpeed, int tagCount, double timestamp) {
    
    // Compute measurement noise for each axis
    double noiseX =
        computeMeasurementNoise(
            baseNoiseX,
            distanceCoefficientX,
            angleCoefficientX,
            speedCoefficientX,
            averageDistance,
            averageAngle,
            currentSpeed,
            tagCount);
    double noiseY =
        computeMeasurementNoise(
            baseNoiseY,
            distanceCoefficientY,
            angleCoefficientY,
            speedCoefficientY,
            averageDistance,
            averageAngle,
            currentSpeed,
            tagCount);
    double noiseTheta =
        computeMeasurementNoise(
            baseNoiseTheta,
            distanceCoefficientTheta,
            angleCoefficientTheta,
            speedCoefficientTheta,
            averageDistance,
            averageAngle,
            currentSpeed,
            tagCount);

    String strategyPrefix = useTrigsolve ? "Trigsolve" : "MultiTag";
    DogLog.log("Vision/" + strategyPrefix + "NoiseX", noiseX);
    DogLog.log("Vision/" + strategyPrefix + "NoiseY", noiseY);
    DogLog.log("Vision/" + strategyPrefix + "NoiseTheta", noiseTheta);
    
    // Update the appropriate reference pose
    if (useTrigsolve) {
      lastKnownTrigsolvePose = measuredPose;
    } else {
      lastKnownPose = measuredPose;
    }
    
    DogLog.log("Vision/" + cameraId.toString() + strategyPrefix + "Pose", measuredPose);
    
    // Choose timestamp: use vision timestamp unless it differs too much from FPGA
    double fpgaTimestamp = Timer.getFPGATimestamp();
    double timestampDifference = Math.abs(timestamp - fpgaTimestamp);
    double chosenTimestamp = (timestampDifference > 0.5) ? fpgaTimestamp : timestamp;

    // Build the noise vector and add the vision measurement
    Matrix<N3, N1> noiseVector = VecBuilder.fill(noiseX, noiseY, noiseTheta);
    swerveDrive.addVisionMeasurement(measuredPose, chosenTimestamp, noiseVector);
    DogLog.log("Vision/" + strategyPrefix + "MeasurementUsed", true);
    
    return new PoseEstimateResult(measuredPose, 1.0, averageDistance, averageAngle, 
                                 currentSpeed, tagCount, timestamp);
  }

  private boolean isTagOnActiveSide(int tagId) {
    return isRedSide.getAsBoolean()
        ? RED_SIDE_TAG_IDS.contains(tagId)
        : BLUE_SIDE_TAG_IDS.contains(tagId);
  }

  private double getDistanceToTag(int tagId) {
    return fieldLayout
        .getTagPose(tagId)
        .map(
            pose3d ->
                pose3d
                    .getTranslation()
                    .getDistance(
                        new Translation3d(
                            swerveDrive.getPose().getX(), swerveDrive.getPose().getY(), 0.0)))
        .orElse(Double.NaN);
  }

  /**
   * Computes a measurement noise standard deviation based on: - base noise (zero-distance, head-on)
   * - distance scaling - viewing angle scaling - robot speed scaling - tag count (noise reduction
   * by sqrt(N))
   */
  private double computeMeasurementNoise(
      double baseNoise,
      double distanceCoefficient,
      double angleCoefficient,
      double speedCoefficient,
      double distance,
      double angleRad,
      double robotSpeed,
      int tagCount) {
    double tagCountScale = 1.0 / Math.sqrt(Math.max(tagCount, 1));
    double distanceTerm = baseNoise + distanceCoefficient * distance;
    double angleTerm = 1.0 + angleCoefficient * (angleRad / maximumViewingAngle);
    double speedTerm = 1.0 + speedCoefficient * (robotSpeed / maximumRobotSpeed);
    return calibrationFactor * tagCountScale * distanceTerm * angleTerm * speedTerm;
  }

  // Setters for runtime tuning of parameters
  public void setCalibrationFactor(double factor) {
    calibrationFactor = factor;
  }

  public void setBaseNoise(double noiseX, double noiseY, double noiseTheta) {
    baseNoiseX = noiseX;
    baseNoiseY = noiseY;
    baseNoiseTheta = noiseTheta;
  }

  public void setDistanceCoefficients(double coeffX, double coeffY, double coeffTheta) {
    distanceCoefficientX = coeffX;
    distanceCoefficientY = coeffY;
    distanceCoefficientTheta = coeffTheta;
  }

  public void setAngleCoefficients(double coeffX, double coeffY, double coeffTheta) {
    angleCoefficientX = coeffX;
    angleCoefficientY = coeffY;
    angleCoefficientTheta = coeffTheta;
  }

  public void setSpeedCoefficients(double coeffX, double coeffY, double coeffTheta) {
    speedCoefficientX = coeffX;
    speedCoefficientY = coeffY;
    speedCoefficientTheta = coeffTheta;
  }

  public void setMaximums(double maxDistance, double maxSpeed, double maxAngleDegrees) {
    maximumAllowedDistance = maxDistance;
    maximumRobotSpeed = maxSpeed;
    maximumViewingAngle = Math.toRadians(maxAngleDegrees);
  }

  /**
   * Helper class to store pose estimation results with confidence metrics.
   */
  private static class PoseEstimateResult {
    public final Pose2d pose;
    public final double confidence;
    public final double averageDistance;
    public final double averageAngle;
    public final double currentSpeed;
    public final int tagCount;
    public final double timestamp;

    public PoseEstimateResult(Pose2d pose, double confidence, double averageDistance, 
                             double averageAngle, double currentSpeed, int tagCount, double timestamp) {
      this.pose = pose;
      this.confidence = confidence;
      this.averageDistance = averageDistance;
      this.averageAngle = averageAngle;
      this.currentSpeed = currentSpeed;
      this.tagCount = tagCount;
      this.timestamp = timestamp;
    }
  }
}