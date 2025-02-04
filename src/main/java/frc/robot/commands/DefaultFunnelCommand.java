package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FunnelSubsystem;

// import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the intake and preshooter until IR sensor detects note
 *
 * @param subsystem The subsystem used by this command.
 */
public class DefaultFunnelCommand extends Command {
  private FunnelSubsystem funnelSubsystem;

  public DefaultFunnelCommand(FunnelSubsystem funnelSubsystem) {
    this.funnelSubsystem = funnelSubsystem;
    addRequirements(funnelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (funnelSubsystem.getAbsolutePositionalError()
        > Constants.FunnelConstants.MAX_POSITIONAL_ERROR) {
      funnelSubsystem.maintainCurrentPosition();
    } else if (funnelSubsystem.isCoralCheckedOut()) {
      funnelSubsystem.spinBackSlowly();
    } else {
      funnelSubsystem.stopFunnel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    funnelSubsystem.maintainCurrentPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
