package frc.robot.commandGroups;

import dev.doglog.DogLog;
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
import frc.robot.subsystems.LedSubsystem;
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
      LedSubsystem leds) {

    JamesHardenMovement movementCommand, maintainCommand;

    if (moveRight) {
      movementCommand = JamesHardenMovement.toClosestRightBranch(swerveSubsystem, redSide, false);
      maintainCommand = JamesHardenMovement.toClosestRightBranch(swerveSubsystem, redSide, true);
    } else {
      movementCommand = JamesHardenMovement.toClosestLeftBranch(swerveSubsystem, redSide, false);
      maintainCommand = JamesHardenMovement.toClosestLeftBranch(swerveSubsystem, redSide, true);
    }
    Command elevateCommand;
    if (height.equals(ElevatorPositions.L4)) {
      elevateCommand = new ElevatorL4(elevatorSubsystem, true);
    } else {
      elevateCommand = new SetElevatorLevel(elevatorSubsystem, height, true);
    }

    addCommands(
        leds.updateLedsCommand(LedSubsystem.LedState.ALIGNMENT_FLASH),
        movementCommand.alongWith(
            (new EndWhenCloseEnough(() -> movementCommand.getTargetPose2d()))
                .andThen(elevateCommand)),
        leds.updateLedsCommand(LedSubsystem.LedState.SCORING_SOLID),
        new ParallelDeadlineGroup(
            new ShootTootsieSlide(tootsieSlideSubsystem).withTimeout(0.5), maintainCommand));

    // addCommands(
    //     maintainCommand
    //         .alongWith(
    //             (new EndWhenCloseEnough(() -> maintainCommand.getTargetPose2d()))
    //                 .andThen(elevateCommand))
    //         .andThen(
    //             new EndWhenCloseEnough(() -> maintainCommand.getTargetPose2d(),
    // Constants.HardenConstants.RegularCommand.xyIndividualTolerance * Math.sqrt(2),
    // Constants.HardenConstants.RegularCommand.headingTolerance)
    //                 .andThen(new ShootTootsieSlide(tootsieSlideSubsystem).withTimeout(0.5))));
  }

  public JamesHardenScore(
      ElevatorSubsystem elevatorSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      SwerveSubsystem swerveSubsystem,
      ElevatorPositions height,
      LandmarkPose branch) {

    JamesHardenMovement movementCommand, maintainCommand;
    if (!branch.isBranch()) {
      DogLog.log("JamesHardenScore/Errors", "called without real branch");
      return;
    }

    movementCommand = JamesHardenMovement.toSpecificBranch(swerveSubsystem, () -> branch, false);
    maintainCommand = JamesHardenMovement.toSpecificBranch(swerveSubsystem, () -> branch, true);

    Command elevateCommand;
    if (height.equals(ElevatorPositions.L4)) {
      elevateCommand = new ElevatorL4(elevatorSubsystem, true);
    } else {
      elevateCommand = new SetElevatorLevel(elevatorSubsystem, height, true);
    }
    // if (DriverStation.isTeleop()) {
    DogLog.log("JamesHardenScore/Version", "teleop");

    addCommands(
        movementCommand.alongWith(
            (new EndWhenCloseEnough(() -> movementCommand.getTargetPose2d()))
                .andThen(elevateCommand)),
        new ParallelDeadlineGroup(
            new ShootTootsieSlide(tootsieSlideSubsystem).withTimeout(0.5), maintainCommand));
    // addCommands(
    //     maintainCommand
    //         .alongWith(
    //             (new EndWhenCloseEnough(() -> maintainCommand.getTargetPose2d()))
    //                 .andThen(elevateCommand))
    //         .alongWith(
    //             new EndWhenCloseEnough(() -> maintainCommand.getTargetPose2d(),
    // Constants.HardenConstants.RegularCommand.xyIndividualTolerance * Math.sqrt(2),
    // Constants.HardenConstants.RegularCommand.headingTolerance)
    //                 .andThen(new ShootTootsieSlide(tootsieSlideSubsystem).withTimeout(0.5))));
    // )
    // new ParallelDeadlineGroup(
    //     new ShootTootsieSlide(tootsieSlideSubsystem).withTimeout(0.5), maintainCommand));
    // } else {
    //   DogLog.log("JamesHardenScore/Version", "auto");
    //   addCommands(
    //       movementCommand
    //           .withTimeout(5.0)
    //           .alongWith(
    //               (new EndWhenCloseEnough(() -> movementCommand.getTargetPose2d()))
    //                   .andThen(elevateCommand)),
    //       new ParallelDeadlineGroup(
    //           new ShootTootsieSlide(tootsieSlideSubsystem).withTimeout(0.5), maintainCommand));
    //       new EndWhenCloseEnough(() -> movementCommand.getTargetPose2d(), 2.8284271247);
    // }
  }
}
