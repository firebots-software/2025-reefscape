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
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.ElevatorCommands.ZeroElevatorHardStop;
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

    addCommands(
        new ParallelCommandGroup(
            new ZeroElevatorHardStop(elevator),
            new InstantCommand(() -> driveTrain.resetPose(autoInformation.get(0).getPose())),
            new ZeroArm(arm)));

    // first score
    addCommands(
        new ParallelDeadlineGroup(
            new Intake(elevator, funnel, shooter),
            JamesHardenMovement.toSpecificBranch(driveTrain, false, autoInformation.get(1))),
        new JamesHardenScore(
            elevator, shooter, driveTrain, ElevatorPositions.L4, false, autoInformation.get(1)),
        new SetElevatorLevel(elevator, ElevatorPositions.Intake));

    // first hps visit, second score
    if (autoInformation.size() > 2) {
      addCommands(
          new JamesHardenMovement(
              driveTrain, autoInformation.get(autoInformation.size() - 1).getPose(), false),
          new ParallelDeadlineGroup(
              new Intake(elevator, funnel, shooter),
              new SequentialCommandGroup(
                  driveTrain.applyRequest(() -> brake).withTimeout(0.1),
                  JamesHardenMovement.toSpecificBranch(driveTrain, false, autoInformation.get(2)))),
          new JamesHardenScore(
              elevator, shooter, driveTrain, ElevatorPositions.L4, false, autoInformation.get(2)),
          new SetElevatorLevel(elevator, ElevatorPositions.L4));
    }

    if (autoInformation.size() > 3) {
      addCommands(
          new JamesHardenMovement(
              driveTrain, autoInformation.get(autoInformation.size() - 1).getPose(), false),
          new ParallelDeadlineGroup(
              new Intake(elevator, funnel, shooter),
              new SequentialCommandGroup(
                  driveTrain.applyRequest(() -> brake).withTimeout(0.1),
                  JamesHardenMovement.toSpecificBranch(driveTrain, false, autoInformation.get(3)))),
          new JamesHardenScore(
              elevator, shooter, driveTrain, ElevatorPositions.L4, false, autoInformation.get(3)),
          new SetElevatorLevel(elevator, ElevatorPositions.L4));
    }
  }
}
