package frc.robot.commands.FunnelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FunnelSubsystem;

public class StopFunnel extends Command {
  private FunnelSubsystem funnelSubsystem;

  public StopFunnel(FunnelSubsystem funnelSubsystem) {
    this.funnelSubsystem = funnelSubsystem;
    addRequirements(funnelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Store the position of the coral when it was first checked out.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    funnelSubsystem.stopFunnel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
