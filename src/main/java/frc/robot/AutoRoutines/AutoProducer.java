package frc.robot.AutoRoutines;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.LandmarkPose;
import frc.robot.commandGroups.Intake;
import frc.robot.commandGroups.JamesHardenScore;
import frc.robot.commands.DaleCommands.ZeroArm;
import frc.robot.commands.DebugCommands.DogLogCmd;
import frc.robot.commands.ElevatorCommands.SetElevatorLevelInstant;
import frc.robot.commands.ElevatorCommands.ZeroElevatorHardStop;
import frc.robot.commands.FunnelCommands.CoralCheckedIn;
import frc.robot.commands.SwerveCommands.JamesHardenMovement;
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
            new ZeroArm(arm).withTimeout(1.25).alongWith(new DogLogCmd("AutoRunningWhich", 0)),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new ZeroElevatorHardStop(elevator)
                        .alongWith(new DogLogCmd("AutoRunningWhich", 1)),
                    new Intake(elevator, funnel, shooter)
                        .alongWith(new DogLogCmd("AutoRunningWhich", 2))),
                JamesHardenMovement.toSpecificBranch(
                        driveTrain, true, () -> autoInformation.get(1), false)
                    .alongWith(new DogLogCmd("AutoRunningWhich", 3)))),
        new JamesHardenScore(
                elevator, shooter, driveTrain, ElevatorPositions.L4, true, autoInformation.get(1))
            .alongWith(new DogLogCmd("AutoRunningWhich", 4)),
        new SetElevatorLevelInstant(elevator, ElevatorPositions.Intake)
            .alongWith(new DogLogCmd("AutoRunningWhich", 5)));

    // first hps visit, second score
    if (autoInformation.size() > 2) {
      addCommands(
          new ParallelDeadlineGroup(
              new Intake(elevator, funnel, shooter),
              new SequentialCommandGroup(
                  new CoralCheckedIn(funnel)
                      .deadlineFor(
                          new JamesHardenMovement(
                              driveTrain,
                              autoInformation.get(autoInformation.size() - 1).getPose(),
                              true,
                              false)),
                  JamesHardenMovement.toSpecificBranch(
                      driveTrain, true, () -> autoInformation.get(2), false))),
          new JamesHardenScore(
              elevator, shooter, driveTrain, ElevatorPositions.L4, true, autoInformation.get(2)),
          new SetElevatorLevelInstant(elevator, ElevatorPositions.Intake));
    }

    if (autoInformation.size() > 3) {
      addCommands(
          new ParallelDeadlineGroup(
              new Intake(elevator, funnel, shooter),
              new SequentialCommandGroup(
                  new CoralCheckedIn(funnel)
                      .deadlineFor(
                          new JamesHardenMovement(
                              driveTrain,
                              autoInformation.get(autoInformation.size() - 1).getPose(),
                              true,
                              false)),
                  JamesHardenMovement.toSpecificBranch(
                      driveTrain, true, () -> autoInformation.get(3), false))),
          new JamesHardenScore(
              elevator, shooter, driveTrain, ElevatorPositions.L4, true, autoInformation.get(3)),
          new SetElevatorLevelInstant(elevator, ElevatorPositions.Intake));

      // addCommands(
      //   new ParallelCommandGroup(
      //       new Intake(elevator, funnel, shooter),
      //       new SequentialCommandGroup(
      //           new CoralCheckedIn(funnel)
      //               .deadlineFor(
      //                   new JamesHardenMovement(
      //                       driveTrain,
      //                       autoInformation.get(autoInformation.size() - 1).getPose(),
      //                       true,
      //                       false)),
      //           JamesHardenMovement.toSpecificBranch(
      //               driveTrain, true,  () -> autoInformation.get(3), false))),
      // //   JamesHardenMovement.toSpecificBranch(
      // //       driveTrain, true, () -> autoInformation.get(3), false),
      //   new JamesHardenScore(
      //       elevator, shooter, driveTrain, ElevatorPositions.L4, true, autoInformation.get(3)),
      //   new SetElevatorLevelInstant(elevator, ElevatorPositions.Intake));

      //   addCommands(
      //       new ParallelCommandGroup(
      //           new Intake(elevator, funnel, shooter),
      //           new JamesHardenMovement(
      //               driveTrain,
      //               autoInformation.get(autoInformation.size() - 1).getPose(),
      //               true,
      //               false)),
      //       new SequentialCommandGroup(
      //           driveTrain.applyRequest(() -> brake).withTimeout(0.1),
      //           JamesHardenMovement.toSpecificBranch(
      //               driveTrain, true, () -> autoInformation.get(3), false)),
      //       new JamesHardenScore(
      //           elevator, shooter, driveTrain, ElevatorPositions.L4, true,
      // autoInformation.get(3)),
      //       new SetElevatorLevelInstant(elevator, ElevatorPositions.Intake));
    }
  }
}
// are we sure that the autoinformation.size thing works? I feel like its going to run the 3 every
// time
