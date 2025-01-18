package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TootsieSlide;
import frc.robot.subsystems.TootsieSlideSubsystem;

/**
 * Runs the intake and preshooter until IR sensor detects note
 *
 * @param subsystem The subsystem used by this command.
 */
public class RunIntakeUntilDetection extends Command {
  private TootsieSlideSubsystem TootsieSlide;

  public RunIntakeUntilDetection(TootsieSlideSubsystem TootsieSlide) {
    this.TootsieSlide = TootsieSlide;
    addRequirements(TootsieSlide);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TootsieSlide.spinTootsie();
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    TootsieSlide.stopTootsie();;  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return TootsieSlide.coralPresent();
  }
}