package frc.robot.commands.FunnelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;

public class RampUpFunnel extends Command {
  private FunnelSubsystem funnelSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private double tolerance = 2;

  public RampUpFunnel(FunnelSubsystem funnelSubsystem, ElevatorSubsystem elevator) {
    this.funnelSubsystem = funnelSubsystem;
    this.elevatorSubsystem = elevator;
    addRequirements(funnelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Store the position of the coral when it was first checked out.
    funnelSubsystem.rampUp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevatorSubsystem.isAtPosition()) {
      if(funnelSubsystem.getSpeed() > Constants.FunnelConstants.RAMP_UP_SPEED - tolerance && funnelSubsystem.getSpeed() < Constants.FunnelConstants.RAMP_UP_SPEED + tolerance){
        funnelSubsystem.runFunnelAtRPS(Constants.FunnelConstants.RAMP_UP_SPEED);
      }
      // funnelSubsystem.rampUp();
    } 
    else {
      // funnelSubsystem.maintainCurrentPosition();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    funnelSubsystem.resetFunnelMotor();
    funnelSubsystem.maintainCurrentPosition();
    if (!interrupted) {
      funnelSubsystem.maintainCurrentPosition();
      // CoralPosition.setCoralInFunnel(true);
    } else {
      funnelSubsystem.runFunnelAtRPS(Constants.FunnelConstants.RAMP_UP_SPEED);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return funnelSubsystem.isCoralCheckedOut();
  }
}
