//TODO: Updated to spin Funnel Backwards
package frc.robot.commands.FunnelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.FunnelSubsystem;

public class ShootSecCoral extends Command {
  private final FunnelSubsystem funnelSubsystem;

  public ShootSecCoral(FunnelSubsystem funnelSubsystem) {
    this.funnelSubsystem = funnelSubsystem;
    addRequirements(funnelSubsystem);
  }

  @Override
  public void initialize() {
    funnelSubsystem.spinFunnel();
  }

  @Override
  public void execute() {
    funnelSubsystem.spinFunnel();
  }

  @Override
  public void end(boolean interrupted) {
    funnelSubsystem.stopFunnel();
    CoralPosition.setCoralInFunnel(false);
  }

  @Override
  public boolean isFinished() {
    return !funnelSubsystem.isCoralCheckedOut();
  }
}
