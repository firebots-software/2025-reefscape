package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.ElevatorSubsystem;

public class CheckingSetElevatorLevel extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final ElevatorPositions pos;
  private boolean coralified;
  private boolean move;

  public CheckingSetElevatorLevel(ElevatorSubsystem subsystem, ElevatorPositions pos) {
    elevatorSubsystem = subsystem;
    this.pos = pos;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    coralified = CoralPosition.isCoralInTootsieSlide();
    move = coralified || (pos == ElevatorPositions.L2DALE || pos == ElevatorPositions.L3DALE);
    if (move) new SetElevatorLevel(elevatorSubsystem, pos);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    // return elevatorSubsystem.isAtPosition() || !move;
    return elevatorSubsystem.getLevel() == pos && elevatorSubsystem.isAtPosition();
  }
}
