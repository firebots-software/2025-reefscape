package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorLevel extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final ElevatorPositions pos;

  public SetElevatorLevel(ElevatorSubsystem subsystem, ElevatorPositions pos) {
    elevatorSubsystem = subsystem;
    this.pos = pos;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevatorSubsystem.elevate(pos);
    //Constants.Swerve.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = Constants.Swerve.CONST_TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND / 4; TODO: make different for diff pos
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.atTargetPosition();
  }
}
