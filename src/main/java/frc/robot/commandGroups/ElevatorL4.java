package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.ElevatorCommands.ElevatorHoldL4;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

public class ElevatorL4 extends SequentialCommandGroup {

  public ElevatorL4(ElevatorSubsystem elevatorSubsystem,TootsieSlideSubsystem shooter) {

    addCommands(
        new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L4)
            .andThen(new ElevatorHoldL4(elevatorSubsystem)));
  }
}
