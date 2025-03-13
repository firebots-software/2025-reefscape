package frc.robot.commands.FunnelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPosition;

public class isCoralInTootsieSlide extends Command {

  public isCoralInTootsieSlide() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Store the position of the coral when it was first checked out.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return CoralPosition.isCoralInTootsieSlide();
  }
}
