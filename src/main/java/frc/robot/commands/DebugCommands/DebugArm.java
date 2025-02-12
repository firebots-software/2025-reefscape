package frc.robot.commands.DebugCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class DebugArm extends Command {

  private final ArmSubsystem arm;
  private final int tolerance = 5;

  public DebugArm(ArmSubsystem arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    arm.setPosition(30);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return arm.atTarget(tolerance);
  }
}
