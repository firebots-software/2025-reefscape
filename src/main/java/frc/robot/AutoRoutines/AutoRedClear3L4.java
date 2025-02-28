package frc.robot.AutoRoutines;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commandGroups.ElevatorL4;
import frc.robot.commandGroups.Intake;
import frc.robot.commands.DaleCommands.ZeroArm;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.ElevatorCommands.ZeroElevator;
import frc.robot.commands.SwerveCommands.JamesHardenMovement;
import frc.robot.commands.TootsieSlideCommands.ShootTootsieSlide;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

public class AutoRedClear3L4 extends SequentialCommandGroup {
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  public AutoRedClear3L4(
      SwerveSubsystem driveTrain,
      TootsieSlideSubsystem shooter,
      ElevatorSubsystem elevator,
      FunnelSubsystem funnel,
      ArmSubsystem arm) {
    addCommands(
        new ZeroElevator(elevator),
        new InstantCommand(
                () -> driveTrain.resetPose(Constants.Landmarks.closerRedClearSideAutoStart))
            .alongWith((new ZeroArm(arm))),
        (new Intake(elevator, funnel, shooter).andThen(new ElevatorL4(elevator)))
            .alongWith(JamesHardenMovement.toSpecificRightBranch(driveTrain, () -> true, true, 2)),
        new ShootTootsieSlide(shooter).withTimeout(0.5),
        new SetElevatorLevel(elevator, ElevatorPositions.Intake)
            .alongWith(
                new JamesHardenMovement(driveTrain, Constants.Landmarks.redClearSideHPS, true)
                    .withTimeout(3)),
        (new Intake(elevator, funnel, shooter).andThen(new ElevatorL4(elevator)))
            .alongWith(
                (driveTrain.applyRequest(() -> brake).withTimeout(0.1))
                    .andThen(
                        JamesHardenMovement.toSpecificLeftBranch(driveTrain, () -> true, true, 1))),
        new ShootTootsieSlide(shooter).withTimeout(0.5),
        new SetElevatorLevel(elevator, ElevatorPositions.Intake)
            .alongWith(
                new JamesHardenMovement(driveTrain, Constants.Landmarks.redClearSideHPS, true)
                    .withTimeout(3)),
        (new Intake(elevator, funnel, shooter).andThen(new ElevatorL4(elevator)))
            .alongWith(
                (driveTrain.applyRequest(() -> brake).withTimeout(0.1))
                    .andThen(
                        JamesHardenMovement.toSpecificRightBranch(
                            driveTrain, () -> true, true, 1))),
        new ShootTootsieSlide(shooter).withTimeout(0.5),
        new SetElevatorLevel(elevator, ElevatorPositions.Intake));
  }
}
