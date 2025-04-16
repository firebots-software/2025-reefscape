package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.FunnelCommands.ReverseFunnel;
import frc.robot.commands.TootsieSlideCommands.ReverseTootsie;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

public class ShootL1Funnel extends SequentialCommandGroup {
  public ShootL1Funnel(
      ElevatorSubsystem elevatorSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      FunnelSubsystem funnelSubsystem) {
    if (CoralPosition.isCoralInTootsieSlide()) {
      addCommands(
          new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake, false),
          new ParallelCommandGroup(
              new ReverseTootsie(tootsieSlideSubsystem), new ReverseFunnel(funnelSubsystem)));
    }
  }
}
