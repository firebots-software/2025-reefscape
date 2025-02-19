package frc.robot.commands.DebugCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class DebugDaleSpin extends Command {

  private final ArmSubsystem arm;

  public DebugDaleSpin(ArmSubsystem arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    arm.spinFlywheel();
  }

  @Override
  public void end(boolean interrupted) {
    arm.stopFlywheel();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
