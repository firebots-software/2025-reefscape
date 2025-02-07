package frc.robot.commands.DaleCommands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToAngleCmd extends Command {
  public ArmToAngleCmd(double angle, ArmSubsystem arm) {
    this.angle = angle;
    this.arm = arm;
    addRequirements(arm);
  }

  private final ArmSubsystem arm;
  private final double angle;
  private double tolerance = 5;

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    DogLog.log("Running to target angle: ", angle);
    arm.setPosition(angle);
  }

  @Override
  public boolean isFinished() {
    return arm.atTarget(tolerance);
  }

  @Override
  public void end(boolean interrupted) {}
}
