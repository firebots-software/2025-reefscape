package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.TootsieSlideCommands.ShootTootsieSlide;
import frc.robot.commands.TransferPieceBetweenFunnelAndElevator;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

public class D2Intake extends SequentialCommandGroup {
  public D2Intake(
      ElevatorSubsystem elevatorSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      FunnelSubsystem funnelSubsystem) {
    addCommands(
        new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L1)
            .alongWith(new ShootTootsieSlide(tootsieSlideSubsystem).withTimeout(0.5))
            .andThen(
                new TransferPieceBetweenFunnelAndElevator(
                    elevatorSubsystem, funnelSubsystem, tootsieSlideSubsystem)));
  }
}
