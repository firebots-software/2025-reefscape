package frc.robot.commands.DebugCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TootsieSlideSubsystem;

public class DebugTootsieSlide extends Command {
  private TootsieSlideSubsystem tootsieSlideSubsystem;

  public DebugTootsieSlide(TootsieSlideSubsystem tootsieSlideSubsystem) {
    this.tootsieSlideSubsystem = tootsieSlideSubsystem;
    addRequirements(tootsieSlideSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    tootsieSlideSubsystem.shootTootsie();
  }

  @Override
  public void end(boolean interrupted) {
    tootsieSlideSubsystem.stopTootsie();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
