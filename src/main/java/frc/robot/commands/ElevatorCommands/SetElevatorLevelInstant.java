package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystemMD2;

public class SetElevatorLevelInstant extends Command {
  private final ElevatorSubsystemMD2 elevatorSubsystem;
  private final ElevatorPositions pos;

  public SetElevatorLevelInstant(ElevatorSubsystemMD2 subsystem, ElevatorPositions pos) {
    elevatorSubsystem = subsystem;
    this.pos = pos;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevatorSubsystem.setHeight(pos.height);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
