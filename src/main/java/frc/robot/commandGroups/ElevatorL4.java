package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.ElevatorCommands.ElevatorHoldL4;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.ElevatorCommands.CheckingSetElevatorLevel;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorL4 extends SequentialCommandGroup {

  public ElevatorL4(ElevatorSubsystem elevatorSubsystem) {

    addCommands(
        new CheckingSetElevatorLevel(elevatorSubsystem, ElevatorPositions.L4)
            .andThen(new ElevatorHoldL4(elevatorSubsystem).withTimeout(0.25)));
  }
}
