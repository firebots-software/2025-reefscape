package frc.robot.commands.SwerveCommands;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class JamesHardenMovement extends Command {

  private final SwerveSubsystem swerve;

  private Supplier<Pose2d> targetPoseSupplier = null;
  private Pose2d targetPose = null;
  private boolean isInAuto;

  public JamesHardenMovement(SwerveSubsystem swerve, Supplier<Pose2d> targetPoseSupplier, boolean isInAuto) {
    this.swerve = swerve;
    this.targetPoseSupplier = targetPoseSupplier;
    this.isInAuto = isInAuto;
    addRequirements(swerve);
  }

  public JamesHardenMovement(SwerveSubsystem swerve, Pose2d targetPose, boolean isInAuto) {
    this.swerve = swerve;
    this.targetPose = targetPose;
    this.isInAuto = isInAuto;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    if (targetPoseSupplier != null) {
      targetPose = targetPoseSupplier.get();
    }
    swerve.resetProfiledPIDs();
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = swerve.calculateRequiredComponentChassisSpeeds(targetPose);

    DogLog.log("JamesHardenMovement/TargetPoseX(m)", targetPose.getX());
    DogLog.log("JamesHardenMovement/TargetPoseY(m)", targetPose.getY());
    DogLog.log("JamesHardenMovement/TargetPoseHeading(deg)", targetPose.getRotation().getRadians());

    DogLog.log("JamesHardenMovement/DesiredChassisSpeedsX(mps)", speeds.vxMetersPerSecond);
    DogLog.log("JamesHardenMovement/DesiredChassisSpeedsY(mps)", speeds.vyMetersPerSecond);
    DogLog.log("JamesHardenMovement/DesiredChassisSpeedsX(radps)", speeds.omegaRadiansPerSecond);

    swerve.setFieldSpeeds(speeds);
  }

  @Override
  public boolean isFinished() {
    double currRot = swerve.getCurrentState().Pose.getRotation().getRadians();
    currRot = ((2.0 * Math.PI) + (currRot % (2.0 * Math.PI))) % (2.0 * Math.PI);
    double targetRot = targetPose.getRotation().getRadians();
    targetRot = ((2.0 * Math.PI) + (targetRot % (2.0 * Math.PI))) % (2.0 * Math.PI);

    if ((Math.abs(swerve.getCurrentState().Pose.getX() - targetPose.getX()) < 0.02)
        && (Math.abs(swerve.getCurrentState().Pose.getY() - targetPose.getY()) < 0.02)
        && (Math.abs(targetRot - currRot) < 0.0075)) {
      return true;
    } else return false;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setRobotSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  public static JamesHardenMovement toClosestLeftBranch(
      SwerveSubsystem swerve, BooleanSupplier redSide, boolean isInAuto) {

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

            Pose2d target =
                new Pose2d(
                    Constants.Landmarks.LEFT_LINEUP_RED[sideOfMinDist],
                    Constants.Landmarks.reefFacingAngleRed[sideOfMinDist]);

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

            Pose2d target =
                new Pose2d(
                    Constants.Landmarks.LEFT_LINEUP_BLUE[sideOfMinDist],
                    Constants.Landmarks.reefFacingAngleBlue[sideOfMinDist]);

            DogLog.log("JamesHardenMovement/toClosestLeftBranch/sideOfMinDist(m)", sideOfMinDist);
            DogLog.log("JamesHardenMovement/toClosestLeftBranch/minDist(m)", minDist);

            return target;
          }
        };

    return new JamesHardenMovement(swerve, targetPose, isInAuto);
  }

  public static JamesHardenMovement toClosestRightBranch(
      SwerveSubsystem swerve, BooleanSupplier redSide, boolean isInAuto) {
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

            Pose2d target =
                new Pose2d(
                    Constants.Landmarks.RIGHT_LINEUP_RED[sideOfMinDist],
                    Constants.Landmarks.reefFacingAngleRed[sideOfMinDist]);

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

            Pose2d target =
                new Pose2d(
                    Constants.Landmarks.RIGHT_LINEUP_BLUE[sideOfMinDist],
                    Constants.Landmarks.reefFacingAngleBlue[sideOfMinDist]);

            DogLog.log("JamesHardenMovement/toClosestRightBranch/sideOfMinDist(m)", sideOfMinDist);
            DogLog.log("JamesHardenMovement/toClosestRightBranch/minDist(m)", minDist);

            return target;
          }
        };

    return new JamesHardenMovement(swerve, targetPose, isInAuto);
  }

  public static JamesHardenMovement toClosestLeftOutpost(
      SwerveSubsystem swerve, BooleanSupplier redSide, boolean isInAuto) {

    Supplier<Pose2d> targetPose =
        () -> {
          Translation2d currPosition = swerve.getCurrentState().Pose.getTranslation();
          if (redSide.getAsBoolean()) {
            double minDist = currPosition.getDistance(Constants.Landmarks.LEFT_OUTPOST_RED[0]);
            int sideOfMinDist = 0;
            for (int i = 1; i < 6; i++) {
              if (currPosition.getDistance(Constants.Landmarks.LEFT_OUTPOST_RED[i]) < minDist) {
                minDist = currPosition.getDistance(Constants.Landmarks.LEFT_OUTPOST_RED[i]);
                sideOfMinDist = i;
              }
            }

            Pose2d target =
                new Pose2d(
                    Constants.Landmarks.LEFT_OUTPOST_RED[sideOfMinDist],
                    Constants.Landmarks.reefFacingAngleRed[sideOfMinDist]);

            DogLog.log("JamesHardenMovement/toClosestLeftOutpost/sideOfMinDist(m)", sideOfMinDist);
            DogLog.log("JamesHardenMovement/toClosestLeftOutpost/minDist(m)", minDist);

            return target;
          } else {
            double minDist = currPosition.getDistance(Constants.Landmarks.LEFT_OUTPOST_BLUE[0]);
            int sideOfMinDist = 0;
            for (int i = 1; i < 6; i++) {
              if (currPosition.getDistance(Constants.Landmarks.LEFT_OUTPOST_BLUE[i]) < minDist) {
                minDist = currPosition.getDistance(Constants.Landmarks.LEFT_OUTPOST_BLUE[i]);
                sideOfMinDist = i;
              }
            }

            Pose2d target =
                new Pose2d(
                    Constants.Landmarks.LEFT_OUTPOST_BLUE[sideOfMinDist],
                    Constants.Landmarks.reefFacingAngleBlue[sideOfMinDist]);

            DogLog.log("JamesHardenMovement/toClosestLeftOutpost/sideOfMinDist(m)", sideOfMinDist);
            DogLog.log("JamesHardenMovement/toClosestLeftOutpost/minDist(m)", minDist);

            return target;
          }
        };

    return new JamesHardenMovement(swerve, targetPose, isInAuto);
  }

  public static JamesHardenMovement toClosestRightOutpost(
      SwerveSubsystem swerve, BooleanSupplier redSide, boolean isInAuto) {
    Supplier<Pose2d> targetPose =
        () -> {
          Translation2d currPosition = swerve.getCurrentState().Pose.getTranslation();
          if (redSide.getAsBoolean()) {
            double minDist = currPosition.getDistance(Constants.Landmarks.RIGHT_OUTPOST_RED[0]);
            int sideOfMinDist = 0;
            for (int i = 1; i < 6; i++) {
              if (currPosition.getDistance(Constants.Landmarks.RIGHT_OUTPOST_RED[i]) < minDist) {
                minDist = currPosition.getDistance(Constants.Landmarks.RIGHT_OUTPOST_RED[i]);
                sideOfMinDist = i;
              }
            }

            Pose2d target =
                new Pose2d(
                    Constants.Landmarks.RIGHT_OUTPOST_RED[sideOfMinDist],
                    Constants.Landmarks.reefFacingAngleRed[sideOfMinDist]);
            DogLog.log("JamesHardenMovement/toClosestRightOutpost/sideOfMinDist(m)", sideOfMinDist);
            DogLog.log("JamesHardenMovement/toClosestRightOutpost/minDist(m)", minDist);

            return target;
          } else {
            double minDist = currPosition.getDistance(Constants.Landmarks.RIGHT_OUTPOST_BLUE[0]);
            int sideOfMinDist = 0;
            for (int i = 1; i < 6; i++) {
              if (currPosition.getDistance(Constants.Landmarks.RIGHT_OUTPOST_BLUE[i]) < minDist) {
                minDist = currPosition.getDistance(Constants.Landmarks.RIGHT_OUTPOST_BLUE[i]);
                sideOfMinDist = i;
              }
            }

            Pose2d target =
                new Pose2d(
                    Constants.Landmarks.RIGHT_OUTPOST_BLUE[sideOfMinDist],
                    Constants.Landmarks.reefFacingAngleBlue[sideOfMinDist]);

            DogLog.log("JamesHardenMovement/toClosestRightOutpost/sideOfMinDist(m)", sideOfMinDist);
            DogLog.log("JamesHardenMovement/toClosestRightOutpost/minDist(m)", minDist);

            return target;
          }
        };

    return new JamesHardenMovement(swerve, targetPose, isInAuto);
  }
}
