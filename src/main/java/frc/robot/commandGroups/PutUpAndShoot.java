package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.TootsieSlideCommands.ShootTootsieSlide;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

public class PutUpAndShoot extends SequentialCommandGroup {
  public PutUpAndShoot(
      ElevatorSubsystem elevatorSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      ElevatorPositions height) {

    addCommands(
        (height == ElevatorPositions.L4)
            ? new ElevatorL4(elevatorSubsystem, false)
            : new SetElevatorLevel(elevatorSubsystem, height, false),
        // new WaitCommand(0.25),
        new ShootTootsieSlide(tootsieSlideSubsystem).withTimeout(0.5));
  }
}
