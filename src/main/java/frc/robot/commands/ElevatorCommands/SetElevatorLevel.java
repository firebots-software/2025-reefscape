package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.ElevatorSubsystemMD2;

public class SetElevatorLevel extends Command {
  private final ElevatorSubsystemMD2 elevatorSubsystem;
  private final ElevatorPositions pos;
  private final boolean checkIfCoralInTootsie;

  public SetElevatorLevel(
      ElevatorSubsystemMD2 subsystem, ElevatorPositions pos, boolean checkIfCoralInTootsie) {
        
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
        elevatorSubsystem.setHeight(pos.height);
      }
    } else {
      elevatorSubsystem.setHeight(pos.height);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.isAtTargetHeight();
  }
}
