package frc.robot.commands.FunnelCommands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FunnelSubsystem;

public class CoralCheckedIn extends Command {
  private FunnelSubsystem funnelSubsystem;

  public CoralCheckedIn(FunnelSubsystem funnelSubsystem) {
    this.funnelSubsystem = funnelSubsystem;
    DogLog.log("CoralCheckedIn", false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DogLog.log("CoralCheckedIn", true);

    // Store the position of the coral when it was first checked out.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DogLog.log("CoralCheckedIn", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return funnelSubsystem.isCoralCheckedIn();
  }
}
