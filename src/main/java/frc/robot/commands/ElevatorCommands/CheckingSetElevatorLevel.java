package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralPosition;

public class CheckingSetElevatorLevel extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final ElevatorPositions pos;

  public CheckingSetElevatorLevel(ElevatorSubsystem subsystem, ElevatorPositions pos) {
    elevatorSubsystem = subsystem;
    this.pos = pos;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevatorSubsystem.elevateTo(pos);
    // if (CoralPosition.isCoralInTootsieSlide()) {
    //   elevatorSubsystem.elevateTo(pos);
    // }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.isAtPosition();
  }
}
