package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroElevatorToFFiltered extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private double ticksAtPosition;

  public ZeroElevatorToFFiltered(ElevatorSubsystem subsystem) {
    elevatorSubsystem = subsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    ticksAtPosition = 0;
    elevatorSubsystem.elevateTo(ElevatorPositions.Intake);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.resetPositionFiltered();
  }

  @Override
  public boolean isFinished() {
    if (elevatorSubsystem.isAtPosition() && elevatorSubsystem.atIntake()) {
      ticksAtPosition++;
    } else {
      ticksAtPosition = 0;
    }
    if (ticksAtPosition >= 5) {
      return true;
    }
    return false;
  }
}
