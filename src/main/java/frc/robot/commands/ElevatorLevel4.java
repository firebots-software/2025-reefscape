package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorLevel4 extends Command {
  private final ElevatorSubsystem elevatorSubsystem;

  public ElevatorLevel4(ElevatorSubsystem subsystem) {
    elevatorSubsystem = subsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevatorSubsystem.elevate(ElevatorPositions.L4);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.isAtPosition();
  }
}
