package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;
import frc.robot.subsystems.CoralPosition;

public class MoveElevatorAndShoot extends Command {
  private final TootsieSlideSubsystem tootsiesubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final ElevatorPositions pos;

  public MoveElevatorAndShoot(TootsieSlideSubsystem tootsieslidesubsystem, ElevatorSubsystem subsystem, ElevatorPositions pos) {
    tootsiesubsystem = tootsieslidesubsystem;
    elevatorSubsystem = subsystem;
    this.pos = pos;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevatorSubsystem.elevateTo(pos);
    if(elevatorSubsystem.isAtPosition()){
      tootsiesubsystem.shootTootsie();
    }
    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.isAtPosition() && !CoralPosition.isCoralInTootsieSlide();
  }
}
