package frc.robot.commands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.Supplier;

public class ArmToAngleCmd extends Command {
  public ArmToAngleCmd(Supplier<Double> angle, ArmSubsystem arm) {
    this.angle = angle;
    this.arm = arm;
    addRequirements(arm);
  }

  private final ArmSubsystem arm;
  private final Supplier<Double> angle;
  private double tolerance = 5;

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    DogLog.log("Running to target angle: ", angle.get());
    arm.setPosition(angle.get());
  }

  @Override
  public boolean isFinished() {
    return arm.atTarget(tolerance);
  }

  @Override
  public void end(boolean interrupted) {}
}
