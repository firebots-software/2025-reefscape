package frc.robot.AutoRoutines;

import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.LandmarkPose;
import frc.robot.commandGroups.Intake;
import frc.robot.commandGroups.JamesHardenScore;
import frc.robot.commands.DaleCommands.ZeroArm;
import frc.robot.commands.ElevatorCommands.ElevatorHoldL4;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.ElevatorCommands.SetElevatorLevelInstant;
import frc.robot.commands.ElevatorCommands.ZeroElevatorHardStop;
import frc.robot.commands.EndWhenCloseEnough;
import frc.robot.commands.FunnelCommands.CoralCheckedIn;
import frc.robot.commands.SwerveCommands.JamesHardenMovement;
import frc.robot.commands.TootsieSlideCommands.ShootTootsieSlide;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;
import java.util.List;

public class AutoProducer extends SequentialCommandGroup {
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  public AutoProducer(
      SwerveSubsystem driveTrain,
      TootsieSlideSubsystem shooter,
      ElevatorSubsystem elevator,
      FunnelSubsystem funnel,
      ArmSubsystem arm,
      List<LandmarkPose> autoInformation) {
    // initializing commands

    addCommands(new InstantCommand(() -> driveTrain.resetPose(autoInformation.get(0).getPose())));

    // first score
    addCommands(
        new ParallelCommandGroup(
            new ZeroArm(arm).withTimeout(1.25),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new ZeroElevatorHardStop(elevator).withTimeout(1.5),
                    new Intake(elevator, funnel, shooter).withTimeout(2.0)),
                JamesHardenMovement.toSpecificBranch(
                    driveTrain, true, () -> autoInformation.get(1), false))),
        new JamesHardenScore(
                elevator, shooter, driveTrain, ElevatorPositions.L4, true, autoInformation.get(1))
            .withTimeout(5.0),
        new SetElevatorLevelInstant(elevator, ElevatorPositions.Intake));

    if (autoInformation.size() > 2) {
      settyCycle(
          elevator,
          funnel,
          shooter,
          driveTrain,
          autoInformation.get(2),
          autoInformation.get(autoInformation.size() - 1));
    }
    if (autoInformation.size() > 3) {
      settyCycle(
          elevator,
          funnel,
          shooter,
          driveTrain,
          autoInformation.get(3),
          autoInformation.get(autoInformation.size() - 1));
    }
    // first hps visit, second score
    // if (autoInformation.size() > 2) {
    //   addCommands(
    //       new ParallelDeadlineGroup(
    //           new Intake(elevator, funnel, shooter),
    //           new SequentialCommandGroup(
    //               new CoralCheckedIn(funnel)
    //                   .deadlineFor(
    //                       new JamesHardenMovement(
    //                           driveTrain,
    //                           autoInformation.get(autoInformation.size() - 1).getPose(),
    //                           true,
    //                           false)),
    //               JamesHardenMovement.toSpecificBranch(
    //                   driveTrain, true, () -> autoInformation.get(2), false))),
    //       new JamesHardenScore(
    //           elevator, shooter, driveTrain, ElevatorPositions.L4, true, autoInformation.get(2)),
    //       new SetElevatorLevelInstant(elevator, ElevatorPositions.Intake));
    // }

    // if (autoInformation.size() > 3) {
    //   addCommands(
    //       new ParallelDeadlineGroup(
    //           new Intake(elevator, funnel, shooter),
    //           new SequentialCommandGroup(
    //               new CoralCheckedIn(funnel)
    //                   .deadlineFor(
    //                       new JamesHardenMovement(
    //                           driveTrain,
    //                           autoInformation.get(autoInformation.size() - 1).getPose(),
    //                           true,
    //                           false)),
    //               JamesHardenMovement.toSpecificBranch(
    //                   driveTrain, true, () -> autoInformation.get(3), false))),
    //       new JamesHardenScore(
    //           elevator, shooter, driveTrain, ElevatorPositions.L4, true, autoInformation.get(3)),
    //       new SetElevatorLevelInstant(elevator, ElevatorPositions.Intake));
    // }

    // addCommands(
    //     new ParallelCommandGroup(
    //         new Intake(elevator, funnel, shooter),
    //         new SequentialCommandGroup(
    //             new CoralCheckedIn(funnel)
    //                 .deadlineFor(
    //                     new JamesHardenMovement(
    //                         driveTrain,
    //                         autoInformation.get(autoInformation.size() - 1).getPose(),
    //                         true,
    //                         false))
    //                 .andThen(
    //                     new JamesHardenScore(
    //                         elevator,
    //                         shooter,
    //                         driveTrain,
    //                         ElevatorPositions.L4,
    //                         true,
    //                         autoInformation.get(3))),
    //             new SetElevatorLevelInstant(elevator, ElevatorPositions.Intake))));
  }

  private void settyCycle(
      ElevatorSubsystem elevator,
      FunnelSubsystem funnel,
      TootsieSlideSubsystem shooter,
      SwerveSubsystem driveTrain,
      LandmarkPose scorePosition,
      LandmarkPose HPSPosition) {

    JamesHardenMovement movementCommand, maintainCommand;
    if (!scorePosition.isBranch()) {
      DogLog.log("JamesHardenScore/Errors", "called without real branch");
      return;
    }
    movementCommand =
        JamesHardenMovement.toSpecificBranch(driveTrain, true, () -> scorePosition, false);
    maintainCommand =
        JamesHardenMovement.toSpecificBranch(driveTrain, true, () -> scorePosition, true);

    addCommands(
        new ParallelCommandGroup(
            // Elevator related
            new Intake(elevator, funnel, shooter)
                .andThen(
                    new EndWhenCloseEnough(
                        () -> movementCommand.getTargetPose2d(),
                        Constants.HardenConstants.EndWhenCloseEnough.translationalToleranceAuto))
                .andThen(new SetElevatorLevel(elevator, ElevatorPositions.L4, true)),

            // Movement related
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new CoralCheckedIn(funnel),
                    new JamesHardenMovement(driveTrain, HPSPosition.getPose(), true, false)),
                movementCommand)),
        // When the elevator is up and when the movement command is done, then do the following
        new ElevatorHoldL4(elevator).withTimeout(0.25),
        new ParallelDeadlineGroup(new ShootTootsieSlide(shooter).withTimeout(0.5), maintainCommand),
        new SetElevatorLevelInstant(
            elevator, ElevatorPositions.Intake)); // sets elevator back to intake when finished
  }
}

// .alongWith(
//                     new EndWhenCloseEnough(() -> movementCommand.getTargetPose2d()).andThen(new
// Command().on))

// are we sure that the autoinformation.size thing works? I feel like its going to run the 3 every
// time
