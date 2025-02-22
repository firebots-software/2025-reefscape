package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.SwerveCommands.JamesHardenMovement;
import frc.robot.commands.TootsieSlideCommands.ShootTootsieSlide;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;
import java.util.function.BooleanSupplier;

public class PutUpAndShoot extends SequentialCommandGroup {
  public PutUpAndShoot(
      ElevatorSubsystem elevatorSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      ElevatorPositions height) {

    addCommands(
        (height == ElevatorPositions.L4) ? new ElevatorL4(elevatorSubsystem) : new SetElevatorLevel(elevatorSubsystem, height),
        // new WaitCommand(0.25),
        new ShootTootsieSlide(tootsieSlideSubsystem).withTimeout(0.5));
  }
}
