package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

// import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the intake and preshooter until IR sensor detects note
 *
 * @param subsystem The subsystem used by this command.
 */
public class FunnelShootCoral extends Command {
  private FunnelSubsystem funnelSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private TootsieSlideSubsystem tootsieSlideSubsystem;
  
  public FunnelShootCoral(FunnelSubsystem funnelSubsystem, ElevatorSubsystem elevatorSubsystem, TootsieSlideSubsystem tootsieSlideSubsystem) {
    this.funnelSubsystem = funnelSubsystem;
    this.tootsieSlideSubsystem=tootsieSlideSubsystem;
    this.elevatorSubsystem=elevatorSubsystem;
    addRequirements(funnelSubsystem,tootsieSlideSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevatorSubsystem.canFunnelShoot()){
        funnelSubsystem.spinFunnel();
        tootsieSlideSubsystem.intakeCoral();
    } else {
        funnelSubsystem.maintainCurrentPosition();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    funnelSubsystem.setCoralInFunnel(false);
    funnelSubsystem.maintainCurrentPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return funnelSubsystem.drakeTripped();
  }
}
