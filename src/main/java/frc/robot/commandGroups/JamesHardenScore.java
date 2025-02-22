package frc.robot.commandGroups;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.Landmarks.BranchSide;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.SwerveCommands.JamesHardenMovement;
import frc.robot.commands.TootsieSlideCommands.ShootTootsieSlide;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;
import frc.robot.util.RobotPosition;

import java.util.function.BooleanSupplier;

public class JamesHardenScore extends SequentialCommandGroup {
  public JamesHardenScore(
      ElevatorSubsystem elevatorSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      SwerveSubsystem swerveSubsystem,
      BooleanSupplier redSide,
      RobotPosition position) {
    double currentTime = Timer.getFPGATimestamp();
    if(position != null && Math.abs(currentTime - position.dTime) <= 2.0){ //If the button has been pressed in the last 2 seconds
      Command movementCommand;
      if (position.side.equals(BranchSide.RIGHT)) {
        movementCommand = JamesHardenMovement.toClosestRightBranch(swerveSubsystem, redSide);
      } else {
        movementCommand = JamesHardenMovement.toClosestLeftBranch(swerveSubsystem, redSide);
      }

      Command elevateCommand;
      if (position.elevator.equals(ElevatorPositions.L4)) {
        elevateCommand = new ElevatorL4(elevatorSubsystem);
      } else {
        elevateCommand = new SetElevatorLevel(elevatorSubsystem, position.elevator);
      }
      addCommands(
          movementCommand,
          elevateCommand,
          new WaitCommand(0.25),
          new ShootTootsieSlide(tootsieSlideSubsystem).withTimeout(0.5));
    }
  }
}
