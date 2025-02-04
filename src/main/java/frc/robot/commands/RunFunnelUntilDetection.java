package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FunnelSubsystem;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
/**
 * Runs the intake and preshooter until IR sensor detects note
 *
 * @param subsystem The subsystem used by this command.
 */
public class RunFunnelUntilDetection extends Command {
  private FunnelSubsystem funnelSubsystem;
  private boolean coralCheckedOut;

  public RunFunnelUntilDetection(FunnelSubsystem funnelSubsystem) {
    this.funnelSubsystem = funnelSubsystem;
    addRequirements(funnelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    funnelSubsystem.spinFunnel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    funnelSubsystem.stopFunnel();

    if (coralCheckedOut) {
      // Once the coral is detected as checked in, wait for momentum to carry it a little furthe
      SequentialCommandGroup commandGroup = new SequentialCommandGroup(
      new WaitCommand(.2));
      // Then, readjust to the correct position based on the checked-out value
      funnelSubsystem.reAdjustMotor();

      commandGroup.schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return funnelSubsystem.isCoralCheckedOut();
  }
}
