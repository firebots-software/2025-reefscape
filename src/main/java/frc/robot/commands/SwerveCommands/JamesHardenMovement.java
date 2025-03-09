package frc.robot.commands.SwerveCommands;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.BlueLandmarkPose;
import frc.robot.Constants.LandmarkPose;
import frc.robot.Constants.RedLandmarkPose;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class JamesHardenMovement extends Command {

  private final SwerveSubsystem swerve;

  private static final ArrayList<RedLandmarkPose> redBranchesL =
      new ArrayList<>(
          Arrays.asList(
              RedLandmarkPose.L0,
              RedLandmarkPose.L1,
              RedLandmarkPose.L2,
              RedLandmarkPose.L3,
              RedLandmarkPose.L4,
              RedLandmarkPose.L5));

  private static final ArrayList<RedLandmarkPose> redBranchesR =
      new ArrayList<>(
          Arrays.asList(
              RedLandmarkPose.R0,
              RedLandmarkPose.R1,
              RedLandmarkPose.R2,
              RedLandmarkPose.R3,
              RedLandmarkPose.R4,
              RedLandmarkPose.R5));

  private static final ArrayList<BlueLandmarkPose> blueBranchesL =
      new ArrayList<>(
          Arrays.asList(
              BlueLandmarkPose.L0,
              BlueLandmarkPose.L1,
              BlueLandmarkPose.L2,
              BlueLandmarkPose.L3,
              BlueLandmarkPose.L4,
              BlueLandmarkPose.L5));

  private static final ArrayList<BlueLandmarkPose> blueBranchesR =
      new ArrayList<>(
          Arrays.asList(
              BlueLandmarkPose.R0,
              BlueLandmarkPose.R1,
              BlueLandmarkPose.R2,
              BlueLandmarkPose.R3,
              BlueLandmarkPose.R4,
              BlueLandmarkPose.R5));

  private Supplier<Pose2d> targetPoseSupplier = null;
  private Pose2d targetPose = null;
  private boolean isInAuto;
  private boolean noTolerance;

  public JamesHardenMovement(
      SwerveSubsystem swerve,
      Supplier<Pose2d> targetPoseSupplier,
      boolean isInAuto,
      boolean noTolerance) {
    this.swerve = swerve;
    this.targetPoseSupplier = targetPoseSupplier;
    this.isInAuto = isInAuto;
    this.noTolerance = noTolerance;
    addRequirements(swerve);
  }

  public JamesHardenMovement(
      SwerveSubsystem swerve, Pose2d targetPose, boolean isInAuto, boolean noTolerance) {
    this.swerve = swerve;
    this.targetPose = targetPose;
    this.isInAuto = isInAuto;
    this.noTolerance = noTolerance;
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
    ChassisSpeeds speeds =
        (isInAuto)
            ? swerve.calculateRequiredComponentChassisSpeeds_AUTO_VERSION(targetPose)
            : swerve.calculateRequiredComponentChassisSpeeds(targetPose);

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
    if (noTolerance) {
      return false;
    } else {
      double currRot = swerve.getCurrentState().Pose.getRotation().getRadians();
      currRot = ((2.0 * Math.PI) + (currRot % (2.0 * Math.PI))) % (2.0 * Math.PI);
      double targetRot = targetPose.getRotation().getRadians();
      targetRot = ((2.0 * Math.PI) + (targetRot % (2.0 * Math.PI))) % (2.0 * Math.PI);

      if ((Math.abs(swerve.getCurrentState().Pose.getX() - targetPose.getX())
              < Constants.HardenConstants.RegularCommand.xyIndividualTolerance)
          && (Math.abs(swerve.getCurrentState().Pose.getY() - targetPose.getY())
              < Constants.HardenConstants.RegularCommand.xyIndividualTolerance)
          && (Math.min(Math.abs(targetRot - currRot), (Math.PI * 2) - Math.abs(targetRot - currRot))
              < Constants.HardenConstants.RegularCommand.headingTolerance)) {
        return true;
      } else return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      swerve.setRobotSpeeds(new ChassisSpeeds(0, 0, 0));
    }
  }

  public Pose2d getTargetPose2d() {
    return targetPose;
  }

  public static JamesHardenMovement toSpecificBranch(
      SwerveSubsystem swerve,
      boolean isInAuto,
      Supplier<LandmarkPose> branch,
      boolean noTolerance) {
    return new JamesHardenMovement(swerve, () -> branch.get().getPose(), isInAuto, noTolerance);
  }

  public static JamesHardenMovement toClosestLeftBranch(
      SwerveSubsystem swerve, BooleanSupplier redSide, boolean isInAuto, boolean noTolerance) {

    Supplier<LandmarkPose> targetBranch =
        () -> {
          Translation2d currPosition = swerve.getCurrentState().Pose.getTranslation();
          if (redSide.getAsBoolean()) {
            double minDist =
                currPosition.getDistance(RedLandmarkPose.L0.getPose().getTranslation());
            RedLandmarkPose branchOfMinDist = RedLandmarkPose.L0;
            for (RedLandmarkPose branch : redBranchesL) {
              if (currPosition.getDistance(branch.getPose().getTranslation()) < minDist) {
                minDist = currPosition.getDistance(branch.getPose().getTranslation());
                branchOfMinDist = branch;
              }
            }
            return branchOfMinDist;
          } else {
            double minDist =
                currPosition.getDistance(BlueLandmarkPose.L0.getPose().getTranslation());
            BlueLandmarkPose branchOfMinDist = BlueLandmarkPose.L0;
            for (BlueLandmarkPose branch : blueBranchesL) {
              if (currPosition.getDistance(branch.getPose().getTranslation()) < minDist) {
                minDist = currPosition.getDistance(branch.getPose().getTranslation());
                branchOfMinDist = branch;
              }
            }
            return branchOfMinDist;
          }
        };

    return JamesHardenMovement.toSpecificBranch(swerve, isInAuto, targetBranch, noTolerance);
  }

  public static JamesHardenMovement toClosestRightBranch(
      SwerveSubsystem swerve, BooleanSupplier redSide, boolean isInAuto, boolean noTolerance) {

    Supplier<LandmarkPose> targetBranch =
        () -> {
          Translation2d currPosition = swerve.getCurrentState().Pose.getTranslation();
          DogLog.log("currPositionX", currPosition.getX());
          DogLog.log("currPositionY", currPosition.getY());
          DogLog.log("currPositionTheta", currPosition.getAngle().getRadians());
          DogLog.log("hardenRed", redSide.getAsBoolean());
          if (redSide.getAsBoolean()) {
            double minDist =
                currPosition.getDistance(RedLandmarkPose.R0.getPose().getTranslation());
            RedLandmarkPose branchOfMinDist = RedLandmarkPose.R0;
            for (RedLandmarkPose branch : redBranchesR) {
              if (currPosition.getDistance(branch.getPose().getTranslation()) < minDist) {
                minDist = currPosition.getDistance(branch.getPose().getTranslation());
                branchOfMinDist = branch;
              }
            }
            return branchOfMinDist;
          } else {
            double minDist =
                currPosition.getDistance(BlueLandmarkPose.R0.getPose().getTranslation());
            BlueLandmarkPose branchOfMinDist = BlueLandmarkPose.R0;
            for (BlueLandmarkPose branch : blueBranchesR) {
              if (currPosition.getDistance(branch.getPose().getTranslation()) < minDist) {
                minDist = currPosition.getDistance(branch.getPose().getTranslation());
                branchOfMinDist = branch;
              }
            }
            return branchOfMinDist;
          }
        };

    return JamesHardenMovement.toSpecificBranch(swerve, isInAuto, targetBranch, noTolerance);
  }
}
