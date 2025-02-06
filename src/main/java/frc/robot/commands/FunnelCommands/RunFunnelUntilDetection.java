package frc.robot.commands.FunnelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FunnelSubsystem;

// import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the intake and preshooter until IR sensor detects note
 *
 * @param subsystem The subsystem used by this command.
 */
public class RunFunnelUntilDetection extends Command {
  private FunnelSubsystem funnelSubsystem;

  public RunFunnelUntilDetection(FunnelSubsystem funnelSubsystem) {
    this.funnelSubsystem = funnelSubsystem;
    addRequirements(funnelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Store the position of the coral when it was first checked out.
    funnelSubsystem.spinFunnel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    funnelSubsystem.spinFunnel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // funnelSubsystem.moveBackFlywheel();
    funnelSubsystem.setCoralInFunnel(true);
    funnelSubsystem.maintainCurrentPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return funnelSubsystem.isCoralCheckedOut();
  }
}
