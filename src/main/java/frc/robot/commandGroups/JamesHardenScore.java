package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.SwerveCommands.JamesHardenMovement;
import frc.robot.commands.TootsieSlideCommands.ShootTootsieSlide;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;
import java.util.function.BooleanSupplier;

public class JamesHardenScore extends SequentialCommandGroup {
  public JamesHardenScore(
      ElevatorSubsystem elevatorSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      SwerveSubsystem swerveSubsystem,
      ElevatorPositions height,
      BooleanSupplier redSide,
      boolean moveRight) {

    Command movementCommand;
    if (moveRight) {
      movementCommand = JamesHardenMovement.toClosestRightBranch(swerveSubsystem, redSide);
    } else {
      movementCommand = JamesHardenMovement.toClosestLeftBranch(swerveSubsystem, redSide);
    }
    Command elevateCommand;
    if (height.equals(ElevatorPositions.L4)) {
      elevateCommand = new ElevatorL4(elevatorSubsystem);
    } else {
      elevateCommand = new SetElevatorLevel(elevatorSubsystem, height);
    }

    addCommands(
        movementCommand,
        elevateCommand,
        new WaitCommand(0.25),
        new ShootTootsieSlide(tootsieSlideSubsystem).withTimeout(0.5));
  }
}
