package frc.robot.commands.FunnelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;

public class RampUpFunnel extends Command {
    private FunnelSubsystem funnelSubsystem;
  private ElevatorSubsystem elevatorSubsystem;

  public RampUpFunnel(FunnelSubsystem funnelSubsystem, ElevatorSubsystem elevator) {
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
      funnelSubsystem.rampUp();
    } else {
      funnelSubsystem.maintainCurrentPosition();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    funnelSubsystem.maintainCurrentPosition();
    if (!interrupted) {
      CoralPosition.setCoralInFunnel(true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return funnelSubsystem.isCoralCheckedOut();
  }
}
