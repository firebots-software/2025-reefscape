package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralPosition;

public class DefaultElevator extends Command {
  private final ElevatorSubsystem elevatorSubsystem;

  public DefaultElevator(ElevatorSubsystem subsystem) {
    elevatorSubsystem = subsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (!CoralPosition.isCoralInTootsieSlide()) {
      elevatorSubsystem.elevateTo(ElevatorPositions.Intake);
    }
    else if(CoralPosition.isCoralInTootsieSlide()) {
      elevatorSubsystem.elevateTo(ElevatorPositions.L1);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.isAtPosition();
  }
}
