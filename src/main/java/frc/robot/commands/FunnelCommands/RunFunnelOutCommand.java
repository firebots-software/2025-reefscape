package frc.robot.commands.FunnelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.FunnelSubsystem;
import java.util.function.BooleanSupplier;

public class RunFunnelOutCommand extends Command {
  private FunnelSubsystem funnelSubsystem;
  private BooleanSupplier L1;

  public RunFunnelOutCommand(FunnelSubsystem funnelSubsystem, BooleanSupplier L1) {
    this.funnelSubsystem = funnelSubsystem;
    this.L1 = L1;
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
    funnelSubsystem.debugSpinBack(L1.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CoralPosition.setCoralInFunnel(false);
    funnelSubsystem.stopFunnel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
