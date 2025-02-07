package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
import frc.robot.commands.TootsieSlideCommands.ShootTootsieSlide;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

public class ElevatorAndShooting extends SequentialCommandGroup {
  public ElevatorAndShooting(
      ElevatorSubsystem elevatorSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      SwerveSubsystem swerveSubsystem,
      ElevatorPositions height,
      boolean isLeft) { 
    addCommands(
        new SetElevatorLevel(elevatorSubsystem, height),  
        new SwerveJoystickCommand(null, null, null, null, swerveSubsystem, isLeft),       
        new ShootTootsieSlide(tootsieSlideSubsystem));   
  }
}
