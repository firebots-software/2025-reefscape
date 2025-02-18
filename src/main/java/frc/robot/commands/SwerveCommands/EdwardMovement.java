package frc.robot.commands.SwerveCommands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class EdwardMovement extends Command {
  private final SwerveSubsystem swerve;

  private Supplier<Translation2d> targetTranslationSupplier = null;
  private Translation2d targetTranslation = null;
  private Pose2d targetPose;

  public EdwardMovement(SwerveSubsystem swerve, Supplier<Translation2d> targetPoseSupplier) {
    this.swerve = swerve;
    this.targetTranslationSupplier = targetPoseSupplier;
    addRequirements(swerve);
  }

  public EdwardMovement(SwerveSubsystem swerve, Translation2d targetTranslation) {
    this.swerve = swerve;
    this.targetTranslation = targetTranslation;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    if (targetTranslationSupplier != null) {
      targetTranslation = targetTranslationSupplier.get();
    }
    targetPose = new Pose2d(targetTranslation, swerve.getCurrentState().Pose.getRotation());
    swerve.resetProfiledPIDs();
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = swerve.calculateRequiredOneDirectionalChassisSpeeds(targetPose);

    DogLog.log("EdwardMovement/TargetPoseX(m)", targetPose.getX());
    DogLog.log("EdwardMovement/TargetPoseY(m)", targetPose.getY());
    DogLog.log("EdwardMovement/TargetPoseHeading(deg)", targetPose.getRotation().getDegrees());

    DogLog.log("EdwardMovement/DesiredChassisSpeedsX(mps)", speeds.vxMetersPerSecond);
    DogLog.log("EdwardMovement/DesiredChassisSpeedsY(mps)", speeds.vyMetersPerSecond);
    DogLog.log("EdwardMovement/DesiredChassisSpeedsX(radps)", speeds.omegaRadiansPerSecond);

    swerve.setFieldSpeeds(speeds);
  }

  @Override
  public boolean isFinished() {
    double currRot = swerve.getCurrentState().Pose.getRotation().getRadians();
    currRot = ((2.0 * Math.PI) + (currRot % (2.0 * Math.PI))) % (2.0 * Math.PI);
    double targetRot = targetPose.getRotation().getRadians();
    targetRot = ((2.0 * Math.PI) + (targetRot % (2.0 * Math.PI))) % (2.0 * Math.PI);

    if (swerve.getCurrentState().Pose.getTranslation().getDistance(targetTranslation) < 0.02
        && (Math.abs(targetRot - currRot) < 0.04)) {
      return true;
    } else return false;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setRobotSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  public static EdwardMovement toClosestLeftBranch(
      SwerveSubsystem swerve, BooleanSupplier redSide) {

    Supplier<Pose2d> targetPose =
        () -> {
          Translation2d currPosition = swerve.getCurrentState().Pose.getTranslation();
          if (redSide.getAsBoolean()) {
            double minDist = currPosition.getDistance(Constants.Landmarks.LEFT_LINEUP_RED[0]);
            int sideOfMinDist = 0;
            for (int i = 1; i < 6; i++) {
              if (currPosition.getDistance(Constants.Landmarks.LEFT_LINEUP_RED[i]) < minDist) {
                minDist = currPosition.getDistance(Constants.Landmarks.LEFT_LINEUP_RED[i]);
                sideOfMinDist = i;
              }
            }

            Pose2d target = new Pose2d(Constants.Landmarks.LEFT_LINEUP_RED[sideOfMinDist], Constants.Landmarks.reefFacingAngleRed[sideOfMinDist]);

            DogLog.log("JamesHardenMovement/toClosestLeftBranch/sideOfMinDist(m)", sideOfMinDist);
            DogLog.log("JamesHardenMovement/toClosestLeftBranch/minDist(m)", minDist);

            return target;
          } else {
            double minDist = currPosition.getDistance(Constants.Landmarks.LEFT_LINEUP_BLUE[0]);
            int sideOfMinDist = 0;
            for (int i = 1; i < 6; i++) {
              if (currPosition.getDistance(Constants.Landmarks.LEFT_LINEUP_BLUE[i]) < minDist) {
                minDist = currPosition.getDistance(Constants.Landmarks.LEFT_LINEUP_BLUE[i]);
                sideOfMinDist = i;
              }
            }

            Pose2d target = new Pose2d(Constants.Landmarks.LEFT_LINEUP_BLUE[sideOfMinDist], Constants.Landmarks.reefFacingAngleBlue[sideOfMinDist]);

            DogLog.log("JamesHardenMovement/toClosestLeftBranch/sideOfMinDist(m)", sideOfMinDist);
            DogLog.log("JamesHardenMovement/toClosestLeftBranch/minDist(m)", minDist);

            return target;
          }
        };

    return new EdwardMovement(swerve, targetPose);
  }

  public static EdwardMovement toClosestRightBranch(
      SwerveSubsystem swerve, BooleanSupplier redSide) {
    Supplier<Pose2d> targetPose =
        () -> {
          Translation2d currPosition = swerve.getCurrentState().Pose.getTranslation();
          if (redSide.getAsBoolean()) {
            double minDist = currPosition.getDistance(Constants.Landmarks.RIGHT_LINEUP_RED[0]);
            int sideOfMinDist = 0;
            for (int i = 1; i < 6; i++) {
              if (currPosition.getDistance(Constants.Landmarks.RIGHT_LINEUP_RED[i]) < minDist) {
                minDist = currPosition.getDistance(Constants.Landmarks.RIGHT_LINEUP_RED[i]);
                sideOfMinDist = i;
              }
            }

            Pose2d target = new Pose2d(Constants.Landmarks.RIGHT_LINEUP_RED[sideOfMinDist], Constants.Landmarks.reefFacingAngleRed[sideOfMinDist]);

            DogLog.log("JamesHardenMovement/toClosestRightBranch/sideOfMinDist(m)", sideOfMinDist);
            DogLog.log("JamesHardenMovement/toClosestRightBranch/minDist(m)", minDist);

            return target;
          } else {
            double minDist = currPosition.getDistance(Constants.Landmarks.RIGHT_LINEUP_BLUE[0]);
            int sideOfMinDist = 0;
            for (int i = 1; i < 6; i++) {
              if (currPosition.getDistance(Constants.Landmarks.RIGHT_LINEUP_BLUE[i]) < minDist) {
                minDist = currPosition.getDistance(Constants.Landmarks.RIGHT_LINEUP_BLUE[i]);
                sideOfMinDist = i;
              }
            }

            Pose2d target = new Pose2d(Constants.Landmarks.RIGHT_LINEUP_BLUE[sideOfMinDist], Constants.Landmarks.reefFacingAngleBlue[sideOfMinDist]);

            DogLog.log("JamesHardenMovement/toClosestRightBranch/sideOfMinDist(m)", sideOfMinDist);
            DogLog.log("JamesHardenMovement/toClosestRightBranch/minDist(m)", minDist);

            return target;
          }
        };

    return new EdwardMovement(swerve, targetPose);
  }

}
