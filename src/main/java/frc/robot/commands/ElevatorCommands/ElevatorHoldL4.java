package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

public class ElevatorHoldL4 extends Command {
  private ElevatorSubsystem elevatorSubsystem;
  private TootsieSlideSubsystem shooter;

  public ElevatorHoldL4(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.shooter = shooter;
    addRequirements(elevatorSubsystem, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.elevateTo(ElevatorPositions.LIMIT_OF_TRAVEL);
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
    return elevatorSubsystem.isAtPosition();
  }
}
