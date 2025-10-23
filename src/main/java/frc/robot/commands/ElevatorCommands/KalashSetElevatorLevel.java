package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.KalashElevatorSubsystem;

public class KalashSetElevatorLevel extends Command {
  private final KalashElevatorSubsystem elevatorSubsystem;
  private final ElevatorPositions pos;
  private final boolean checkIfCoralInTootsie;

  public KalashSetElevatorLevel(
      KalashElevatorSubsystem subsystem, ElevatorPositions pos, boolean checkIfCoralInTootsie) {
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
        elevatorSubsystem.setHeight(pos);
      }
    } else {
      elevatorSubsystem.setHeight(pos);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.targetReached();
  }
}
