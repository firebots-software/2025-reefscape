package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.FunnelCommands.RunFunnelOutUntilUnstuckCommand;
import frc.robot.commands.TransferPieceBetweenFunnelAndElevator;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

public class UnjamFunnelAndIntake extends SequentialCommandGroup {
  public UnjamFunnelAndIntake(
      ElevatorSubsystem elevatorSubsystem,
      FunnelSubsystem funnelSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem) {
    addCommands(
        new RunFunnelOutUntilUnstuckCommand(funnelSubsystem)
            .andThen(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake))
            .andThen(
                new TransferPieceBetweenFunnelAndElevator(
                    elevatorSubsystem, funnelSubsystem, tootsieSlideSubsystem)));
  }
}
