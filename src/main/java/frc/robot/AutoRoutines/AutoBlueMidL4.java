package frc.robot.AutoRoutines;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commandGroups.Intake;
import frc.robot.commandGroups.PutUpAndShoot;
import frc.robot.commands.DaleCommands.ZeroArm;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.ElevatorCommands.ZeroElevatorAcrossTimeframe;
import frc.robot.commands.SwerveCommands.JamesHardenMovement;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

public class AutoBlueMidL4 extends SequentialCommandGroup {
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  public AutoBlueMidL4(
      SwerveSubsystem driveTrain,
      TootsieSlideSubsystem shooter,
      ElevatorSubsystem elevator,
      FunnelSubsystem funnel,
      ArmSubsystem arm) {
    addCommands(
        new ZeroElevatorAcrossTimeframe(elevator),
        new InstantCommand(() -> driveTrain.resetPose(Constants.Landmarks.blueMidAutoStart))
            .alongWith((new ZeroArm(arm))),
        (new Intake(elevator, funnel, shooter)
            .alongWith(
                JamesHardenMovement.toSpecificRightBranch(driveTrain, () -> false, false, 3)).withTimeout(8.0)),
        new PutUpAndShoot(elevator, shooter, ElevatorPositions.L4),
        new SetElevatorLevel(elevator, ElevatorPositions.Intake));
  }
}
