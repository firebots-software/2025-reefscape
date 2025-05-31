package frc.robot.commands;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class EndWhenCloseEnough extends Command {
  SwerveSubsystem driveTrain;
  Supplier<Pose2d> poseSupplier;
  double translationalTolerance;
  double rotationalTolerance;

  public EndWhenCloseEnough(Supplier<Pose2d> targetTranslation) {
    driveTrain = SwerveSubsystem.getInstance();
    poseSupplier = targetTranslation;
    translationalTolerance =
        Constants.HardenConstants.EndWhenCloseEnough.translationalToleranceTeleop;
    rotationalTolerance = Constants.HardenConstants.EndWhenCloseEnough.headingTolerance;
  }

  public EndWhenCloseEnough(
      Supplier<Pose2d> targetTranslation,
      double translationalTolerance,
      double rotationalTolerance) {
    driveTrain = SwerveSubsystem.getInstance();
    poseSupplier = targetTranslation;
    this.translationalTolerance = translationalTolerance;
    this.rotationalTolerance = rotationalTolerance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DogLog.log("END_WHEN_CLOSE_ENOUGH", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DogLog.log("END_WHEN_CLOSE_ENOUGH", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currRot = driveTrain.getCurrentState().Pose.getRotation().getRadians();
    currRot = ((2.0 * Math.PI) + (currRot % (2.0 * Math.PI))) % (2.0 * Math.PI);
    double targetRot = poseSupplier.get().getRotation().getRadians();
    targetRot = ((2.0 * Math.PI) + (targetRot % (2.0 * Math.PI))) % (2.0 * Math.PI);

    if ((Math.abs(
                driveTrain
                    .getCurrentState()
                    .Pose
                    .getTranslation()
                    .getDistance(poseSupplier.get().getTranslation()))
            <= translationalTolerance)
        && (Math.min(Math.abs(targetRot - currRot), (Math.PI * 2) - Math.abs(targetRot - currRot))
            <= rotationalTolerance)) {
      return true;
    } else return false;
  }
}
