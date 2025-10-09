package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystemMD2;

public class ElevatorHoldL4 extends Command {
  private ElevatorSubsystemMD2 elevatorSubsystem;

  public ElevatorHoldL4(ElevatorSubsystemMD2 elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setHeight(ElevatorPositions.LIMIT_OF_TRAVEL.height);
  }

  // Called every time the scheduler runs while the command is scheduled.

  public void execute() {
    elevatorSubsystem.ElevatorTorqueMode();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevatorSubsystem.isAtTargetHeight();
  }
}
