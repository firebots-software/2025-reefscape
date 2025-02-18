package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.SwerveCommands.JamesHardenMovement;
import frc.robot.commands.TootsieSlideCommands.ShootTootsieSlide;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;
import java.util.function.BooleanSupplier;

public class EdwardScore extends SequentialCommandGroup {
  public EdwardScore(
      ElevatorSubsystem elevatorSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      SwerveSubsystem swerveSubsystem,
      ElevatorPositions height,
      BooleanSupplier redSide,
      boolean moveRight) {

    Command outpostMovement;
    if (moveRight) {
      outpostMovement = JamesHardenMovement.toClosestRightOutpost(swerveSubsystem, redSide);
    } else {
      outpostMovement = JamesHardenMovement.toClosestLeftOutpost(swerveSubsystem, redSide);
    }

    Command qDirectionalMovement;
    if (moveRight) {
      qDirectionalMovement = 
    } else {
      qDirectionalMovement = 

    addCommands(
        outpostMovement,
        setElevatorLevel(elevatorSubsystem, height).alongWith(q direction command (edward mvmnt)),
        new ShootTootsieSlide(tootsieSlideSubsystem));w Se
  }
}
