package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoRoutines;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetIsAutoRunning extends Command {
  private final boolean isAutoRunning;

  public SetIsAutoRunning(boolean isAutoRunning) {
    this.isAutoRunning = isAutoRunning;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    AutoRoutines.setIsAutoRunning(isAutoRunning);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
