package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.ElevatorSubsystem;

public class DefaultElevator extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  boolean hasReset;
  public DefaultElevator(ElevatorSubsystem subsystem) {
    elevatorSubsystem = subsystem;
    hasReset = false;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(elevatorSubsystem.isAtPosition() && elevatorSubsystem.getLevel().equals(ElevatorPositions.Intake) && !hasReset){
      elevatorSubsystem.resetPosition();
      hasReset = true;
    } else if(!elevatorSubsystem.getLevel().equals(ElevatorPositions.Intake) && hasReset) {
      hasReset = false;
    }
    if (!CoralPosition.isCoralInTootsieSlide()) {
      elevatorSubsystem.elevateTo(ElevatorPositions.Intake);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
