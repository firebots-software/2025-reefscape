/*
This Command spins the Funnel motors until the Coral is detected by the Checkout sensor. At That
point, the funnel maintains its current position (if the Coral moves the Funnel motors due to
momentum, then this moves the funnel motors backwards)
*/
package frc.robot.commands.FunnelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;

// import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the intake and preshooter until IR sensor detects note
 *
 * @param subsystem The subsystem used by this command.
 */
public class RunFunnelUntilDetection extends Command {
  private FunnelSubsystem funnelSubsystem;
  private ElevatorSubsystem elevatorSubsystem;

  public RunFunnelUntilDetection(FunnelSubsystem funnelSubsystem, ElevatorSubsystem elevator) {
    this.funnelSubsystem = funnelSubsystem;
    this.elevatorSubsystem = elevator;
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
    if (elevatorSubsystem.isAtPosition()) {
      funnelSubsystem.spinFunnel();
    } else {
      funnelSubsystem.maintainCurrentPosition();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    funnelSubsystem.maintainCurrentPosition();
    CoralPosition.setCoralInFunnel(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return funnelSubsystem.isCoralCheckedOut();
  }
}
