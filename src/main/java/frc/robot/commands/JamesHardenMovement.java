package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
      currRot = (Math.PI+(currRot%Math.PI))%Math.PI;
      double targetRot = targetPose.getRotation().getRadians();
      targetRot = (Math.PI+(targetRot%Math.PI))%Math.PI;

      if ((Math.abs(swerve.getState().Pose.getX()-targetPose.getX()) < 0.01) &&
          (Math.abs(swerve.getState().Pose.getY()-targetPose.getY()) < 0.01) &&
          (Math.abs(targetRot-currRot) < 0.04)) {
            return true;
          }
          else return false;
  }
  @Override
  public void end(boolean interrupted) {
    swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }
}
