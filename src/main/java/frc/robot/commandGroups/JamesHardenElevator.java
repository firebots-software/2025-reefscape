package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.SwerveCommands.JamesHardenMovement;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;
import java.util.function.BooleanSupplier;

public class JamesHardenElevator extends SequentialCommandGroup {
  public JamesHardenElevator(
      ElevatorSubsystem elevatorSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      SwerveSubsystem swerveSubsystem,
      ElevatorPositions height,
      BooleanSupplier redSide,
      boolean moveRight,
      boolean isInAuto) {

    Command movementCommand;
    if (moveRight) {
      movementCommand =
          JamesHardenMovement.toClosestRightBranch(swerveSubsystem, redSide, isInAuto);
    } else {
      movementCommand = JamesHardenMovement.toClosestLeftBranch(swerveSubsystem, redSide, isInAuto);
    }

    Command elevatorCommand;
    if (height.equals(ElevatorPositions.L4)) {
      elevatorCommand = new ElevatorL4(elevatorSubsystem);
    } else {
      elevatorCommand = new SetElevatorLevel(elevatorSubsystem, height);
    }

    addCommands(movementCommand, elevatorCommand);
  }
}
