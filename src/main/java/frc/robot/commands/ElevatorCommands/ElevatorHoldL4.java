package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorHoldL4 extends Command {
  private ElevatorSubsystem elevatorSubsystem;

  public ElevatorHoldL4(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  public void execute() {
    elevatorSubsystem.ElevatorTorqueMode();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !CoralPosition.isCoralInTootsieSlide();
  }
}
