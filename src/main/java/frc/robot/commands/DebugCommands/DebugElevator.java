package frc.robot.commands.DebugCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class DebugElevator extends Command {

  private final ElevatorSubsystem elevatorSubsystem;

  public DebugElevator(ElevatorSubsystem subsystem) {
    this.elevatorSubsystem = subsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevatorSubsystem.testElevator(0.5);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.isAtPosition();
  }
}
