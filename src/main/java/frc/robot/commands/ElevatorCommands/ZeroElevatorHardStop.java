package frc.robot.commands.ElevatorCommands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroElevatorHardStop extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private double timesExceededCurrent;

  public ZeroElevatorHardStop(ElevatorSubsystem subsystem) {
    elevatorSubsystem = subsystem;
    DogLog.log("Commands/ZeroElevatorHardStop/Running", false);
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.reduceCurrentLimits();
    DogLog.log("Commands/ZeroElevatorHardStop/Running", true);
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
    DogLog.log("Commands/ZeroElevatorHardStop/Running", false);
    if (!interrupted) {
      elevatorSubsystem.resetElevatorPositionToZero();
    }
    elevatorSubsystem.reduceCurrentLimits();
    elevatorSubsystem.elevatorHasBeenZeroed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    DogLog.log("Commands/ZeroElevatorHardStop/timesExceededCurrent", timesExceededCurrent);
    boolean checkCurrent = elevatorSubsystem.checkCurrent();
    DogLog.log("Commands/ZeroElevatorHardStop/checkCurrent", checkCurrent);
    if (checkCurrent) {
      timesExceededCurrent++;
    } else {
      timesExceededCurrent = 0;
    }

    return timesExceededCurrent >= 10;
  }
}
