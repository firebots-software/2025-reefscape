package frc.robot.commands.DaleCommands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmDefault extends Command {
  public ArmDefault(ArmSubsystem arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  private final ArmSubsystem arm;
  private double tolerance = 5;

  @Override
  public void initialize() {
    arm.stopFlywheel();
  }

  @Override
  public void execute() {
    arm.setPosition(0);
  }

  @Override
  public boolean isFinished() {
    return arm.atTarget(tolerance);
  }

  @Override
  public void end(boolean interrupted) {}
}
