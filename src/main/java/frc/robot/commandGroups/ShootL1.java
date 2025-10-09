package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.TootsieSlideCommands.ShootSlow;
import frc.robot.subsystems.ElevatorSubsystemMD2;
import frc.robot.subsystems.TootsieSlideSubsystem;

public class ShootL1 extends SequentialCommandGroup {
  public ShootL1(ElevatorSubsystemMD2 elevatorSubsystem, TootsieSlideSubsystem tootsieSlideSubsystem) {
    addCommands(
        new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L1, false)
            .andThen(new ShootSlow(tootsieSlideSubsystem))
        // .andThen(new WaitCommand(0.2))
        );
  }
}
