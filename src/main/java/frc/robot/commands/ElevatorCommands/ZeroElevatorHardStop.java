package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroElevatorHardStop extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private double timesExceededCurrent;

  public ZeroElevatorHardStop(ElevatorSubsystem subsystem) {
    elevatorSubsystem = subsystem;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timesExceededCurrent = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem
        .moveElevatorNegative(); // this works no, it just hits dale against the hardstop
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      elevatorSubsystem.resetElevatorPositionToZero();
    }
    elevatorSubsystem.resetCurrentLimits();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (elevatorSubsystem.checkCurrent()) {
      timesExceededCurrent++;
    } else {
      timesExceededCurrent = 0;
    }

    return timesExceededCurrent >= 25;
  }
}
