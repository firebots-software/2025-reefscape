package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class EndWhenCloseEnough extends Command {
  SwerveSubsystem driveTrain;
  double tolerance;
  Supplier<Translation2d> translationSupplier;
  public EndWhenCloseEnough(Supplier<Translation2d> targetTranslation, double tolerance) {
    driveTrain = SwerveSubsystem.getInstance();
    translationSupplier = targetTranslation;
    this.tolerance = tolerance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (((driveTrain.getCurrentState().Pose.getTranslation().getDistance(translationSupplier.get())) <= tolerance) ? true : false);
  }
}
