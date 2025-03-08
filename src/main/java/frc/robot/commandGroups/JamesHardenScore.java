package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.LandmarkPose;
import frc.robot.commands.EndWhenCloseEnough;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.SwerveCommands.JamesHardenMovement;
import frc.robot.commands.TootsieSlideCommands.ShootTootsieSlide;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;
import java.util.function.BooleanSupplier;

import dev.doglog.DogLog;

public class JamesHardenScore extends SequentialCommandGroup {
  public JamesHardenScore(
      ElevatorSubsystem elevatorSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      SwerveSubsystem swerveSubsystem,
      ElevatorPositions height,
      BooleanSupplier redSide,
      boolean moveRight,
      boolean isInAuto) {

    JamesHardenMovement movementCommand;
    if (moveRight) {
      movementCommand =
          JamesHardenMovement.toClosestRightBranch(swerveSubsystem, redSide, isInAuto);
    } else {
      movementCommand = JamesHardenMovement.toClosestLeftBranch(swerveSubsystem, redSide, isInAuto);
    }
    Command elevateCommand;
    if (height.equals(ElevatorPositions.L4)) {
      elevateCommand = new ElevatorL4(elevatorSubsystem);
    } else {
      elevateCommand = new SetElevatorLevel(elevatorSubsystem, height);
    }

    addCommands(
        movementCommand.alongWith((new EndWhenCloseEnough(()->movementCommand.getTargetPose2d())).andThen(elevateCommand)),
        new ShootTootsieSlide(tootsieSlideSubsystem).withTimeout(0.5));
  }

  public JamesHardenScore(
      ElevatorSubsystem elevatorSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      SwerveSubsystem swerveSubsystem,
      ElevatorPositions height,
      boolean isInAuto,
      LandmarkPose branch) {

    JamesHardenMovement movementCommand;
    if (!branch.isBranch()) {
      DogLog.log("JamesHardenScore/Errors", "called without real branch");
      return;
    }
   
    movementCommand = JamesHardenMovement.toSpecificBranch(swerveSubsystem, isInAuto, branch);

    Command elevateCommand;
    if (height.equals(ElevatorPositions.L4)) {
      elevateCommand = new ElevatorL4(elevatorSubsystem);
    } else {
      elevateCommand = new SetElevatorLevel(elevatorSubsystem, height);
    }

    addCommands(
        movementCommand.alongWith((new EndWhenCloseEnough(()->movementCommand.getTargetPose2d())).andThen(elevateCommand)),
        new ShootTootsieSlide(tootsieSlideSubsystem).withTimeout(0.5));
  }
}
