package frc.robot.commands.FunnelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

public class RunFunnelAndTootsieOutCommand extends Command {
  private FunnelSubsystem funnelSubsystem;
  private TootsieSlideSubsystem tootsieSlideSubsystem;

  public RunFunnelAndTootsieOutCommand(
      FunnelSubsystem funnelSubsystem, TootsieSlideSubsystem tootsieSlideSubsystem) {
    this.funnelSubsystem = funnelSubsystem;
    this.tootsieSlideSubsystem = tootsieSlideSubsystem;

    addRequirements(funnelSubsystem, tootsieSlideSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Store the position of the coral when it was first checked out.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    funnelSubsystem.debugSpinBack();
    tootsieSlideSubsystem.runTootsieBackwards();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    funnelSubsystem.stopFunnel();
    tootsieSlideSubsystem.stopTootsie();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
