package frc.robot.AutoRoutines;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.LandmarkPose;
import frc.robot.commandGroups.Intake;
import frc.robot.commandGroups.PutUpAndShoot;
import frc.robot.commands.DaleCommands.ZeroArm;
import frc.robot.commands.DebugCommands.DogLogCmd;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.ElevatorCommands.ZeroElevatorAcrossTimeframe;
import frc.robot.commands.SwerveCommands.JamesHardenMovement;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

public class AutoProducer extends SequentialCommandGroup {
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  public AutoProducer(
      SwerveSubsystem driveTrain,
      TootsieSlideSubsystem shooter,
      ElevatorSubsystem elevator,
      FunnelSubsystem funnel,
      ArmSubsystem arm,
      List<LandmarkPose> autoInformation) {

    addCommands(
        new ZeroElevatorAcrossTimeframe(elevator),
        new InstantCommand(() -> driveTrain.resetPose(autoInformation.get(0).getPose()))
            .alongWith((new ZeroArm(arm))),
        // (new Intake(elevator, funnel, shooter))
        //     .alongWith(JamesHardenMovement.toSpecificRightBranch(driveTrain, () -> true, true, 2)),
        // new PutUpAndShoot(elevator, shooter, ElevatorPositions.L4),
        // new SetElevatorLevel(elevator, ElevatorPositions.Intake),
        // new JamesHardenMovement(driveTrain, Constants.Landmarks.redClearSideHPS, true)
        //     .withTimeout(3),
        // (new Intake(elevator, funnel, shooter))
        //     .alongWith(
        //         (driveTrain.applyRequest(() -> brake).withTimeout(0.1))
        //             .andThen(
        //                 JamesHardenMovement.toSpecificLeftBranch(driveTrain, () -> true, true, 1))),
        // new PutUpAndShoot(elevator, shooter, ElevatorPositions.L4),
        // new SetElevatorLevel(elevator, ElevatorPositions.Intake),
        // new JamesHardenMovement(driveTrain, Constants.Landmarks.redClearSideHPS, true)
        //     .withTimeout(3),
        // (new Intake(elevator, funnel, shooter))
        //     .alongWith(
        //         (driveTrain.applyRequest(() -> brake).withTimeout(0.1))
        //             .andThen(
        //                 JamesHardenMovement.toSpecificRightBranch(
        //                     driveTrain, () -> true, true, 1))),
        // new PutUpAndShoot(elevator, shooter, ElevatorPositions.L4),
        // new SetElevatorLevel(elevator, ElevatorPositions.Intake));
  }
}
