package frc.robot.commands.ElevatorCommands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroElevatorToFFiltered extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private double ticksAtPosition;

  public ZeroElevatorToFFiltered(ElevatorSubsystem subsystem) {
    elevatorSubsystem = subsystem;
    DogLog.log("subsystems/Elevator/ZeroElevatorToFFiltered/running", false);
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    ticksAtPosition = 0;
    DogLog.log("subsystems/Elevator/ZeroElevatorToFFiltered/running", true);
    elevatorSubsystem.elevateTo(ElevatorPositions.Intake);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    DogLog.log("subsystems/Elevator/ZeroElevatorToFFiltered/running", false);
    elevatorSubsystem.resetPositionFiltered();
  }

  @Override
  public boolean isFinished() {
    DogLog.log("subsystems/Elevator/ZeroElevatorToFFiltered/ticksAtPosition", ticksAtPosition);
    boolean inPosition = elevatorSubsystem.isAtPosition() && elevatorSubsystem.atIntake();
    DogLog.log("subsystems/Elevator/ZeroElevatorToFFiltered/inPosition", inPosition);

    if (inPosition) {
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
