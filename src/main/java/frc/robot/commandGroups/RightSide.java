package frc.robot.commandGroups;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.SwerveCommands.JamesHardenMovement;
import frc.robot.commands.TootsieSlideCommands.ShootTootsieSlide;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

public class RightSide extends SequentialCommandGroup {
  public RightSide(
      ElevatorSubsystem elevatorSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      SwerveSubsystem swerveSubsystem,
      ElevatorPositions height,
      Pose2d righPose2d, Supplier<Pose2d> rightPose2d) { 
    addCommands(
        new SetElevatorLevel(elevatorSubsystem, height).alongWith(new JamesHardenMovement(swerveSubsystem, rightPose2d)),  
        new ShootTootsieSlide(tootsieSlideSubsystem));   
  }
}