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
  private final PhotonPoseEstimator poseEstimator;
  private PhotonPipelineResult latestVisionResult;
  private final BooleanSupplier isRedSide;
  private Pose2d lastKnownPose = new Pose2d(0, 0, new Rotation2d());
  private final SwerveSubsystem swerveDrive = SwerveSubsystem.getInstance();
  private final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  public VisionSystem(Constants.Vision.Cameras cameraId, BooleanSupplier isRedSide) {
    this.isRedSide = isRedSide;
    this.cameraId = cameraId;
    photonCamera = new PhotonCamera(cameraId.toString());
    Transform3d cameraToRobot = Constants.Vision.getCameraTransform(cameraId);
    poseEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraToRobot);
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
   * Attempts to fuse a vision measurement into the swerve pose estimator, dropping readings that
   * fail validity checks, and computing noise dynamically via computeMeasurementNoise().
   */
  public void addFilteredPose() {
    if (latestVisionResult == null || !latestVisionResult.hasTargets()) {
      DogLog.log("Vision/MeasurementUsed", false);
      return;
    }

    // Filter tags to only those on the active side
    List<PhotonTrackedTarget> validTags =
        latestVisionResult.getTargets().stream()
            .filter(t -> isTagOnActiveSide(t.getFiducialId()))
            .collect(Collectors.toList());
    if (validTags.isEmpty()) {
      DogLog.log("Vision/TagFilter", false);
      return;
    }

    // Compute effective metrics for multi-tag solution
    int tagCount = validTags.size();
    double averageDistance =
        validTags.stream()
            .mapToDouble(t -> getDistanceToTag(t.getFiducialId()))
            .average()
            .orElse(Double.NaN);
    if (Double.isNaN(averageDistance) || averageDistance > maximumAllowedDistance) {
      DogLog.log("Vision/DistanceFilter", true);
      return;
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

    DogLog.log("Vision/NoiseX", noiseX);
    DogLog.log("Vision/NoiseY", noiseY);
    DogLog.log("Vision/NoiseTheta", noiseTheta);

    // Get the pose from PhotonVision
    poseEstimator.setReferencePose(lastKnownPose);
    Optional<EstimatedRobotPose> maybePose = poseEstimator.update(latestVisionResult);
    if (maybePose.isEmpty()) {
      DogLog.log("Vision/PoseEstimateFailed", true);
      return;
    }
    Pose2d measuredPose = maybePose.get().estimatedPose.toPose2d();
    lastKnownPose = measuredPose;
    DogLog.log("Vision/" + cameraId.toString() + "Pose", measuredPose);
    // Choose timestamp: use vision timestamp unless it differs too much from FPGA
    double visionTimestamp = latestVisionResult.getTimestampSeconds();
    double fpgaTimestamp = Timer.getFPGATimestamp();
    double timestampDifference = Math.abs(visionTimestamp - fpgaTimestamp);
    double chosenTimestamp = (timestampDifference > 0.5) ? fpgaTimestamp : visionTimestamp;

    // Build the noise vector and add the vision measurement
    Matrix<N3, N1> noiseVector = VecBuilder.fill(noiseX, noiseY, noiseTheta);
    swerveDrive.addVisionMeasurement(measuredPose, chosenTimestamp, noiseVector);
    DogLog.log("Vision/MeasurementUsed", true);
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
}
