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
    elevatorSubsystem.elevateTo(pos);

    // so depending on height of the elvator acceleration is less (im sorry code gods i made a
    // constant not actaully constant please forgive me)
    if (pos == ElevatorPositions.L4) {
      Constants.Swerve.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND =
          Constants.Swerve.CONST_TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND_ELEVATOR_L4;
    } else if (pos == ElevatorPositions.L3) {
      Constants.Swerve.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND =
          Constants.Swerve.CONST_TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND_ELEVATOR_L3;
    } else if (pos == ElevatorPositions.L2) {
      Constants.Swerve.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND =
          Constants.Swerve.CONST_TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND_ELEVATOR_L2;
    } else if (pos == ElevatorPositions.L1) {
      Constants.Swerve.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND =
          Constants.Swerve.CONST_TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND_ELEVATOR_L1;
    } else {
      Constants.Swerve.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND =
          Constants.Swerve.CONST_TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND_ELEVATOR_INTAKE;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.isAtPosition();
  }
}
