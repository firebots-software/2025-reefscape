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
import edu.wpi.first.math.geometry.Rotation3d;
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
  private double baseNoiseX = 0.01; // meters
  private double baseNoiseY = 0.0;
  private double baseNoiseTheta = 0.5; // radians

  private double distanceCoefficientX = 0.05; // noise growth per meter
  private double distanceCoefficientY = 0.05;
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
  private final PhotonPoseEstimator trigsolvePoseEstimator; // PNP_DISTANCE_TRIG_SOLVE
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
            fieldLayout, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, cameraToRobot);
    
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
    final String strategyPrefix = useTrigsolve ? "Trigsolve" : "MultiTag";
    if (latestVisionResult == null || !latestVisionResult.hasTargets()) {
      if (!forceAdd) {
        v(strategyPrefix + "/HasTargets", false);
        vts(strategyPrefix + "/Frame/NoTargets");
      }
      return null;
    }
    v(strategyPrefix + "/HasTargets", true);
    vts(strategyPrefix + "/Frame/Start");

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
    int tagCount = validTags.size();
    v(strategyPrefix + "/Tags/Count", tagCount);
    String tagIdsCsv = validTags.stream()
        .map(t -> Integer.toString(t.getFiducialId()))
        .sorted()
        .collect(Collectors.joining(","));
    v(strategyPrefix + "/Tags/IDs", tagIdsCsv);

    // Compute effective metrics for solution
    // Use camera→target distance from PV (avoids odometry dependence)
    double averageDistance =
        validTags.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(Double.NaN);
    double minDistance =
        validTags.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .min()
            .orElse(Double.NaN);
    v(strategyPrefix + "/Range/AvgM", averageDistance);
    v(strategyPrefix + "/Range/MinM", minDistance);
    if (Double.isNaN(minDistance) || minDistance > maximumAllowedDistance) {
      if (!forceAdd) {
        v(strategyPrefix + "/Gate/DistanceRejected", true);
        v(strategyPrefix + "/Range/MaxAllowedM", maximumAllowedDistance);
      }
      return null;
    }
    // Compute true viewing skew (how oblique the camera is to the tag face)
    double avgSkewRad =
        validTags.stream().mapToDouble(this::computeTargetSkewRad).average().orElse(0.0);
    double minSkewRad =
        validTags.stream().mapToDouble(this::computeTargetSkewRad).min().orElse(0.0);
    double maxSkewRad =
        validTags.stream().mapToDouble(this::computeTargetSkewRad).max().orElse(0.0);

    v(strategyPrefix + "/Skew/AvgDeg", Math.toDegrees(avgSkewRad));
    v(strategyPrefix + "/Skew/MinDeg", Math.toDegrees(minSkewRad));
    v(strategyPrefix + "/Skew/MaxDeg", Math.toDegrees(maxSkewRad));

    // Strategy-specific gating: trigsolve rejects only if ALL tags are too skewed; PnP rejects if ANY is too skewed
    if (useTrigsolve) {
      if (Math.toDegrees(minSkewRad) > 70.0) {
        if (!forceAdd) {
          v(strategyPrefix + "/Gate/SkewRejected", true);
          v(strategyPrefix + "/Skew/CutoffDeg", 70.0);
        }
        return null;
      }
    } else {
      if (Math.toDegrees(maxSkewRad) > 70.0) {
        if (!forceAdd) {
          v(strategyPrefix + "/Gate/SkewRejected", true);
          v(strategyPrefix + "/Skew/CutoffDeg", 70.0);
        }
        return null;
      }
    }

    // Use strategy-appropriate skew for noise scaling
    double averageAngle = useTrigsolve ? minSkewRad : avgSkewRad;
    double currentSpeed =
        Math.hypot(
            swerveDrive.getRobotSpeeds().vxMetersPerSecond,
            swerveDrive.getRobotSpeeds().vyMetersPerSecond);
    v(strategyPrefix + "/Inputs/SkewUsedDeg", Math.toDegrees(averageAngle));
    v(strategyPrefix + "/Inputs/SpeedMps", currentSpeed);

    // Choose the appropriate pose estimator and reference pose
    PhotonPoseEstimator selectedEstimator = useTrigsolve ? trigsolvePoseEstimator : poseEstimator;
    Pose2d referencePos = useTrigsolve ? lastKnownTrigsolvePose : lastKnownPose;
    
    // Get the pose from PhotonVision
    selectedEstimator.setReferencePose(referencePos);
    Optional<EstimatedRobotPose> maybePose = selectedEstimator.update(latestVisionResult);
    if (maybePose.isEmpty()) {
      if (!forceAdd) {
        v(strategyPrefix + "/Pose/EstimateFailed", true);
        vts(strategyPrefix + "/Frame/End");
      }
      return null;
    }
    
    EstimatedRobotPose estimatedPose = maybePose.get();
    Pose2d measuredPose = estimatedPose.estimatedPose.toPose2d();
    
    // Calculate confidence metric for trigsolve based on measurement quality
    double confidence = 1.0; // Default confidence for multi-tag
    if (useTrigsolve) {
      // For trigsolve, calculate confidence based on the measurement noise multipliers
      // Lower total noise = higher confidence
      NoiseComponents nxC = computeNoiseComponents(
          baseNoiseX, distanceCoefficientX, angleCoefficientX, speedCoefficientX,
          averageDistance, averageAngle, currentSpeed, tagCount);
      NoiseComponents nyC = computeNoiseComponents(
          baseNoiseY, distanceCoefficientY, angleCoefficientY, speedCoefficientY,
          averageDistance, averageAngle, currentSpeed, tagCount);
      NoiseComponents nthC = computeNoiseComponents(
          baseNoiseTheta, distanceCoefficientTheta, angleCoefficientTheta, speedCoefficientTheta,
          averageDistance, averageAngle, currentSpeed, tagCount);

      double noiseX = nxC.total;
      double noiseY = nyC.total;
      double noiseTheta = nthC.total;

      // Unitless blend for confidence
      double nxU = noiseX / Math.max(baseNoiseX, 1e-9);
      double nyU = noiseY / Math.max(baseNoiseY, 1e-9);
      double nthU = noiseTheta / Math.max(baseNoiseTheta, 1e-9);
      double totalNoise = Math.sqrt(nxU * nxU + nyU * nyU + nthU * nthU);
      confidence = 1.0 / (1.0 + totalNoise);

      // Structured logs for tuning (Trigsolve only)
      v(strategyPrefix + "/Confidence", confidence);
      v(strategyPrefix + "/Noise/X/Total", noiseX);
      v(strategyPrefix + "/Noise/Y/Total", noiseY);
      v(strategyPrefix + "/Noise/Theta/Total", noiseTheta);
      v(strategyPrefix + "/Noise/TagScale", nxC.tagScale); // same for all axes
      v(strategyPrefix + "/Noise/X/DistanceTerm", nxC.distanceTerm);
      v(strategyPrefix + "/Noise/X/AngleTerm", nxC.angleTerm);
      v(strategyPrefix + "/Noise/X/SpeedTerm", nxC.speedTerm);
      v(strategyPrefix + "/Noise/Y/DistanceTerm", nyC.distanceTerm);
      v(strategyPrefix + "/Noise/Y/AngleTerm", nyC.angleTerm);
      v(strategyPrefix + "/Noise/Y/SpeedTerm", nyC.speedTerm);
      v(strategyPrefix + "/Noise/Theta/DistanceTerm", nthC.distanceTerm);
      v(strategyPrefix + "/Noise/Theta/AngleTerm", nthC.angleTerm);
      v(strategyPrefix + "/Noise/Theta/SpeedTerm", nthC.speedTerm);
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
      v("Trigsolve/Confidence/LowerThanOtherCam", true);
      v("Trigsolve/Confidence/Ours", confidence);
      v("Trigsolve/Confidence/Best", bestConfidence);
      return null;
    }
  }

  /**
   * Final processing and addition of pose estimate to odometry.
   */
  private PoseEstimateResult processPoseEstimate(Pose2d measuredPose, boolean useTrigsolve,
      double averageDistance, double averageAngle, double currentSpeed, int tagCount, double timestamp) {
    
    // Compute measurement noise for each axis and log components
    NoiseComponents nxC = computeNoiseComponents(
        baseNoiseX, distanceCoefficientX, angleCoefficientX, speedCoefficientX,
        averageDistance, averageAngle, currentSpeed, tagCount);
    NoiseComponents nyC = computeNoiseComponents(
        baseNoiseY, distanceCoefficientY, angleCoefficientY, speedCoefficientY,
        averageDistance, averageAngle, currentSpeed, tagCount);
    NoiseComponents nthC = computeNoiseComponents(
        baseNoiseTheta, distanceCoefficientTheta, angleCoefficientTheta, speedCoefficientTheta,
        averageDistance, averageAngle, currentSpeed, tagCount);

    double noiseX = nxC.total;
    double noiseY = nyC.total;
    double noiseTheta = nthC.total;

    String strategyPrefix = useTrigsolve ? "Trigsolve" : "MultiTag";
    v(strategyPrefix + "/Noise/X/Total", noiseX);
    v(strategyPrefix + "/Noise/Y/Total", noiseY);
    v(strategyPrefix + "/Noise/Theta/Total", noiseTheta);
    v(strategyPrefix + "/Noise/TagScale", nxC.tagScale);
    v(strategyPrefix + "/Noise/X/DistanceTerm", nxC.distanceTerm);
    v(strategyPrefix + "/Noise/X/AngleTerm", nxC.angleTerm);
    v(strategyPrefix + "/Noise/X/SpeedTerm", nxC.speedTerm);
    v(strategyPrefix + "/Noise/Y/DistanceTerm", nyC.distanceTerm);
    v(strategyPrefix + "/Noise/Y/AngleTerm", nyC.angleTerm);
    v(strategyPrefix + "/Noise/Y/SpeedTerm", nyC.speedTerm);
    v(strategyPrefix + "/Noise/Theta/DistanceTerm", nthC.distanceTerm);
    v(strategyPrefix + "/Noise/Theta/AngleTerm", nthC.angleTerm);
    v(strategyPrefix + "/Noise/Theta/SpeedTerm", nthC.speedTerm);

    // Update the appropriate reference pose
    if (useTrigsolve) {
      lastKnownTrigsolvePose = measuredPose;
    } else {
      lastKnownPose = measuredPose;
    }

    // Choose timestamp: use vision timestamp unless it differs too much from FPGA
    double fpgaTimestamp = Timer.getFPGATimestamp();
    double timestampDifference = Math.abs(timestamp - fpgaTimestamp);
    double chosenTimestamp = (timestampDifference > 0.5) ? fpgaTimestamp : timestamp;

    v(strategyPrefix + "/Pose/X", measuredPose.getX());
    v(strategyPrefix + "/Pose/Y", measuredPose.getY());
    v(strategyPrefix + "/Pose/ThetaDeg", measuredPose.getRotation().getDegrees());
    v(strategyPrefix + "/Timestamp/Vision", timestamp);
    v(strategyPrefix + "/Timestamp/FPGA", fpgaTimestamp);
    v(strategyPrefix + "/Timestamp/Delta", timestampDifference);

    // Build the noise vector and add the vision measurement
    Matrix<N3, N1> noiseVector = VecBuilder.fill(noiseX, noiseY, noiseTheta);
    swerveDrive.addVisionMeasurement(measuredPose, chosenTimestamp, noiseVector);
    v(strategyPrefix + "/MeasurementUsed", true);
    vts(strategyPrefix + "/Frame/End");

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

  // ─── Vision logging helpers ──────────────────────────────────────────────────
  private String vKey(String subkey) { return "Vision/" + cameraId.toString() + "/" + subkey; }
  private void vts(String subkey) { DogLog.timestamp(vKey(subkey)); }
  private void v(String subkey, double val) { DogLog.log(vKey(subkey), val); }
  private void v(String subkey, boolean val) { DogLog.log(vKey(subkey), val); }
  private void v(String subkey, String val) { DogLog.log(vKey(subkey), val); }

  private static class NoiseComponents {
    double tagScale;     // 1/sqrt(tags)
    double distanceTerm; // base + k*d^2
    double angleTerm;    // normalized sec^2
    double speedTerm;    // 1 + k*(v/vmax)^2
    double total;        // calibrationFactor * tagScale * distanceTerm * angleTerm * speedTerm
  }

  private NoiseComponents computeNoiseComponents(
      double baseNoise,
      double distanceCoefficient,
      double angleCoefficient,
      double speedCoefficient,
      double distance,
      double angleRad,
      double robotSpeed,
      int tagCount) {
    NoiseComponents c = new NoiseComponents();

    // Tag count factor (diminishing returns; cap at 4)
    int effectiveTags = Math.max(1, Math.min(tagCount, 4));
    c.tagScale = 1.0 / Math.sqrt(effectiveTags);

    // Distance term (keep as d^2 per your model)
    c.distanceTerm = baseNoise + distanceCoefficient * distance * distance;

    // Angle term (foreshortening ~ sec^2(theta), normalized to [1, 1+coef])
    double theta = Math.max(0.0, Math.min(angleRad, 0.999 * maximumViewingAngle));
    double cosT = Math.cos(theta);
    double sec2T = 1.0 / Math.max(cosT * cosT, 1e-6);
    double cosMax = Math.cos(0.999 * maximumViewingAngle);
    double sec2Max = 1.0 / Math.max(cosMax * cosMax, 1e-6);
    double normalizedAngle = (sec2T - 1.0) / Math.max(sec2Max - 1.0, 1e-6);
    normalizedAngle = Math.max(0.0, Math.min(normalizedAngle, 1.0));
    c.angleTerm = 1.0 + angleCoefficient * normalizedAngle;

    // Speed term (quadratic, saturated)
    double vNorm = Math.max(0.0, Math.min(robotSpeed, maximumRobotSpeed))
        / Math.max(maximumRobotSpeed, 1e-6);
    c.speedTerm = 1.0 + speedCoefficient * (vNorm * vNorm);

    c.total = calibrationFactor * c.tagScale * c.distanceTerm * c.angleTerm * c.speedTerm;
    return c;
  }

  private double computeMeasurementNoise(
      double baseNoise,
      double distanceCoefficient,
      double angleCoefficient,
      double speedCoefficient,
      double distance,
      double angleRad,
      double robotSpeed,
      int tagCount) {
    NoiseComponents c = computeNoiseComponents(
        baseNoise,
        distanceCoefficient,
        angleCoefficient,
        speedCoefficient,
        distance,
        angleRad,
        robotSpeed,
        tagCount);
    return c.total;
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
  /**
   * Computes the camera's off-axis viewing skew for a single tag, in radians.
   * Uses the rotation from camera -> target; pitch (Y) and yaw (Z) near zero imply head-on view.
   */
  private double computeTargetSkewRad(PhotonTrackedTarget t) {
    Rotation3d r = t.getBestCameraToTarget().getRotation();
    // r.getY() and r.getZ() are radians (pitch and yaw).
    // Combined off-axis = sqrt(pitch^2 + yaw^2)
    return Math.hypot(r.getY(), r.getZ());
  }
}