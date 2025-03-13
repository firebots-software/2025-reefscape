package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorLevel extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final ElevatorPositions pos;
  private final boolean checkIfCoralInTootsie;

  public SetElevatorLevel(
      ElevatorSubsystem subsystem, ElevatorPositions pos, boolean checkIfCoralInTootsie) {
    elevatorSubsystem = subsystem;
    this.pos = pos;
    this.checkIfCoralInTootsie = checkIfCoralInTootsie;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (checkIfCoralInTootsie) {
      if (CoralPosition.isCoralInTootsieSlide()) {
        elevatorSubsystem.elevateTo(pos);
      }
    } else {
      elevatorSubsystem.elevateTo(pos);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.isAtPosition();
  }
}
