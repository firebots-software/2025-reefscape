package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class JamesHardenMovement extends Command {

  private final SwerveSubsystem swerve;

  private Supplier<Pose2d> targetPoseSupplier = null;
  private Pose2d targetPose = null;
  private int ct = 0;

  public JamesHardenMovement(SwerveSubsystem swerve, Supplier<Pose2d> targetPoseSupplier) {
    this.swerve = swerve;
    this.targetPoseSupplier = targetPoseSupplier;
    addRequirements(swerve);
  }

  public JamesHardenMovement(SwerveSubsystem swerve, Pose2d targetPose) {
    this.swerve = swerve;
    this.targetPose = targetPose;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    if (targetPoseSupplier != null) {
      targetPose = targetPoseSupplier.get();
    }
    swerve.resetPIDs();
    ct++;
    SmartDashboard.putNumber("ct", ct);
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = swerve.calculateChassisSpeeds(swerve.getState().Pose, targetPose);
    SmartDashboard.putNumber("x mps", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("y mps", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("theta radians ps", speeds.omegaRadiansPerSecond);

    SmartDashboard.putNumber("desiredHeading", targetPose.getRotation().getDegrees());
    SmartDashboard.putNumber("desired x", targetPose.getTranslation().getX());
    SmartDashboard.putNumber("desired y", targetPose.getTranslation().getY());
    swerve.setChassisSpeeds(speeds);
  }

  @Override
  public boolean isFinished() {
    double currRot = swerve.getState().Pose.getRotation().getRadians();
    currRot = (Math.PI + (currRot % Math.PI)) % Math.PI;
    double targetRot = targetPose.getRotation().getRadians();
    targetRot = (Math.PI + (targetRot % Math.PI)) % Math.PI;

    if ((Math.abs(swerve.getState().Pose.getX() - targetPose.getX()) < 0.01)
        && (Math.abs(swerve.getState().Pose.getY() - targetPose.getY()) < 0.01)
        && (Math.abs(targetRot - currRot) < 0.04)) {
      return true;
    } else return false;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  public static JamesHardenMovement toClosestLeftBranch(SwerveSubsystem swerve, Supplier<Boolean> redSide) {
    Supplier<Pose2d> targetPose = () -> {
      Translation2d currPosition = swerve.getState().Pose.getTranslation();
      if (redSide.get()) {
        double minDist = currPosition.getDistance(Constants.Landmarks.leftBranchesRed[0]);
        int sideOfMinDist = 0;
        for (int i = 1; i < 6; i++) {
          if (currPosition.getDistance(Constants.Landmarks.leftBranchesRed[i]) < minDist) {
            minDist = currPosition.getDistance(Constants.Landmarks.leftBranchesRed[i]);
            sideOfMinDist = i;
          }
        }

        Pose2d target =
            (new Pose2d(
                new Translation2d(
                    Constants.Landmarks.leftBranchesRed[sideOfMinDist].getX()
                        - (((Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length.in(Meters)+ (2.0*Constants.Swerve.WHICH_SWERVE_ROBOT.BUMPER_THICKNESS.thickness.in(Meters)))/2.0)
                            * Constants.Landmarks.reefFacingAngleRed[sideOfMinDist].getCos()),
                    Constants.Landmarks.leftBranchesRed[sideOfMinDist].getY()
                        - (((Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length.in(Meters)+ (2.0*Constants.Swerve.WHICH_SWERVE_ROBOT.BUMPER_THICKNESS.thickness.in(Meters)))/2.0)
                        * Constants.Landmarks.reefFacingAngleRed[sideOfMinDist].getSin())),
                            Constants.Landmarks.reefFacingAngleRed[sideOfMinDist]));
        return target;
      } else {
        double minDist = currPosition.getDistance(Constants.Landmarks.leftBranchesBlue[0]);
        int sideOfMinDist = 0;
        for (int i = 1; i < 6; i++) {
          if (currPosition.getDistance(Constants.Landmarks.leftBranchesBlue[i]) < minDist) {
            minDist = currPosition.getDistance(Constants.Landmarks.leftBranchesBlue[i]);
            sideOfMinDist = i;
          }
        }

        Pose2d target = 
            (new Pose2d(
                new Translation2d(
                    Constants.Landmarks.leftBranchesBlue[sideOfMinDist].getX()
                        - (((Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length.in(Meters)+ (2.0*Constants.Swerve.WHICH_SWERVE_ROBOT.BUMPER_THICKNESS.thickness.in(Meters)))/2.0)
                        * Constants.Landmarks.reefFacingAngleBlue[sideOfMinDist].getCos()),
                    Constants.Landmarks.leftBranchesBlue[sideOfMinDist].getY()
                        - (((Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length.in(Meters)+ (2.0*Constants.Swerve.WHICH_SWERVE_ROBOT.BUMPER_THICKNESS.thickness.in(Meters)))/2.0)
                        * Constants.Landmarks.reefFacingAngleBlue[sideOfMinDist].getSin())),
                            Constants.Landmarks.reefFacingAngleBlue[sideOfMinDist]));
        
        return target;
      }
    };

    return new JamesHardenMovement(swerve, targetPose);
  }

  public static JamesHardenMovement toClosestRightBranch(SwerveSubsystem swerve, Supplier<Boolean> redSide) {
    Supplier<Pose2d> targetPose = () -> {
      Translation2d currPosition = swerve.getState().Pose.getTranslation();
      if (redSide.get()) {
        double minDist = currPosition.getDistance(Constants.Landmarks.rightBranchesRed[0]);
        int sideOfMinDist = 0;
        for (int i = 1; i < 6; i++) {
          if (currPosition.getDistance(Constants.Landmarks.rightBranchesRed[i]) < minDist) {
            minDist = currPosition.getDistance(Constants.Landmarks.rightBranchesRed[i]);
            sideOfMinDist = i;
          }
        }

        Pose2d target =
            (new Pose2d(
                new Translation2d(
                    Constants.Landmarks.rightBranchesRed[sideOfMinDist].getX()
                        - (((Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length.in(Meters)+ (2.0*Constants.Swerve.WHICH_SWERVE_ROBOT.BUMPER_THICKNESS.thickness.in(Meters)))/2.0)
                            * Constants.Landmarks.reefFacingAngleRed[sideOfMinDist].getCos()),
                    Constants.Landmarks.rightBranchesRed[sideOfMinDist].getY()
                        - (((Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length.in(Meters)+ (2.0*Constants.Swerve.WHICH_SWERVE_ROBOT.BUMPER_THICKNESS.thickness.in(Meters)))/2.0)
                        * Constants.Landmarks.reefFacingAngleRed[sideOfMinDist].getSin())),
                            Constants.Landmarks.reefFacingAngleRed[sideOfMinDist]));
        return target;
      } else {
        double minDist = currPosition.getDistance(Constants.Landmarks.rightBranchesBlue[0]);
        int sideOfMinDist = 0;
        for (int i = 1; i < 6; i++) {
          if (currPosition.getDistance(Constants.Landmarks.rightBranchesBlue[i]) < minDist) {
            minDist = currPosition.getDistance(Constants.Landmarks.rightBranchesBlue[i]);
            sideOfMinDist = i;
          }
        }

        Pose2d target = 
            (new Pose2d(
                new Translation2d(
                    Constants.Landmarks.rightBranchesBlue[sideOfMinDist].getX()
                        - (((Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length.in(Meters)+ (2.0*Constants.Swerve.WHICH_SWERVE_ROBOT.BUMPER_THICKNESS.thickness.in(Meters)))/2.0)
                        * Constants.Landmarks.reefFacingAngleBlue[sideOfMinDist].getCos()),
                    Constants.Landmarks.rightBranchesBlue[sideOfMinDist].getY()
                        - (((Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length.in(Meters)+ (2.0*Constants.Swerve.WHICH_SWERVE_ROBOT.BUMPER_THICKNESS.thickness.in(Meters)))/2.0)
                        * Constants.Landmarks.reefFacingAngleBlue[sideOfMinDist].getSin())),
                            Constants.Landmarks.reefFacingAngleBlue[sideOfMinDist]));
        
        return target;
      }
    };

    return new JamesHardenMovement(swerve, targetPose);
  } 
}
