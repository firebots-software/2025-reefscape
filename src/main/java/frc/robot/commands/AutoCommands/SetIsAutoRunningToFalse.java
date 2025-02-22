package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoRoutines;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetIsAutoRunningToFalse extends Command {

  public SetIsAutoRunningToFalse() {}

  @Override
  public void initialize() {
    AutoRoutines.setIsAutoRunning(false);
  }

  @Override
  public void execute() {
    AutoRoutines.setIsAutoRunning(false);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
