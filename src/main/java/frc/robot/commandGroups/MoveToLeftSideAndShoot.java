package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.SwerveCommands.JamesHardenMovement;
import frc.robot.commands.TootsieSlideCommands.ShootTootsieSlide;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;
import java.util.function.BooleanSupplier;

public class MoveToLeftSideAndShoot extends SequentialCommandGroup {
  public MoveToLeftSideAndShoot(
      ElevatorSubsystem elevatorSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      SwerveSubsystem swerveSubsystem,
      ElevatorPositions height,
      BooleanSupplier redSide) {  
    addCommands(
        new SetElevatorLevel(elevatorSubsystem, height)
            .alongWith(JamesHardenMovement.toClosestLeftBranch(swerveSubsystem, redSide)), 
        new ShootTootsieSlide(tootsieSlideSubsystem));
  }
}
