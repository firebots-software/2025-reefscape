package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoRoutines;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.ElevatorSubsystem;

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
    if (!CoralPosition.isCoralInTootsieSlide() && !AutoRoutines.getIsAutoRunning()) {
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
