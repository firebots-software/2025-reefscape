package frc.robot.commandGroups;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.LandmarkPose;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.EndWhenCloseEnough;
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
      boolean moveRight,
      boolean edwardVersion) {

    JamesHardenMovement movementCommand, maintainCommand;

    if (moveRight) {
      movementCommand =
          JamesHardenMovement.toClosestRightBranch(swerveSubsystem, redSide, edwardVersion, false);
      maintainCommand =
          JamesHardenMovement.toClosestRightBranch(swerveSubsystem, redSide, edwardVersion, true);
    } else {
      movementCommand =
          JamesHardenMovement.toClosestLeftBranch(swerveSubsystem, redSide, edwardVersion, false);
      maintainCommand =
          JamesHardenMovement.toClosestLeftBranch(swerveSubsystem, redSide, edwardVersion, true);
    }
    Command elevateCommand;
    if (height.equals(ElevatorPositions.L4)) {
      elevateCommand = new ElevatorL4(elevatorSubsystem, true);
    } else {
      elevateCommand = new SetElevatorLevel(elevatorSubsystem, height, true);
    }

    addCommands(
        movementCommand.alongWith(
            (new EndWhenCloseEnough(() -> movementCommand.getTargetPose2d()))
                .andThen(elevateCommand)),
        new ParallelDeadlineGroup(
            new ShootTootsieSlide(tootsieSlideSubsystem).withTimeout(0.5), maintainCommand));
  }

  public JamesHardenScore(
      ElevatorSubsystem elevatorSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      SwerveSubsystem swerveSubsystem,
      ElevatorPositions height,
      boolean edwardVersion,
      LandmarkPose branch) {

    JamesHardenMovement movementCommand, maintainCommand;
    if (!branch.isBranch()) {
      DogLog.log("JamesHardenScore/Errors", "called without real branch");
      return;
    }

    movementCommand =
        JamesHardenMovement.toSpecificBranch(swerveSubsystem, edwardVersion, () -> branch, false);
    maintainCommand =
        JamesHardenMovement.toSpecificBranch(swerveSubsystem, edwardVersion, () -> branch, true);

    Command elevateCommand;
    if (height.equals(ElevatorPositions.L4)) {
      elevateCommand = new ElevatorL4(elevatorSubsystem, true);
    } else {
      elevateCommand = new SetElevatorLevel(elevatorSubsystem, height, true);
    }
    if (DriverStation.isTeleop()) {
      addCommands(
          movementCommand.alongWith(
              (new EndWhenCloseEnough(() -> movementCommand.getTargetPose2d()))
                  .andThen(elevateCommand)),
          new ParallelDeadlineGroup(
              new ShootTootsieSlide(tootsieSlideSubsystem).withTimeout(0.5), maintainCommand));
    } else {
      addCommands(
          movementCommand.alongWith(
              (new EndWhenCloseEnough(() -> movementCommand.getTargetPose2d()))
                  .andThen(elevateCommand)
                  .withTimeout(5.0)),
          new ParallelDeadlineGroup(
              new ShootTootsieSlide(tootsieSlideSubsystem).withTimeout(0.5), maintainCommand));
    }
  }
}
