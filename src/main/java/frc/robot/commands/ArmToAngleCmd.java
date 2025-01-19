package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.Supplier;

import dev.doglog.DogLog;

public class ArmToAngleCmd extends Command {
  public ArmToAngleCmd(Supplier<Double> angle, ArmSubsystem arm) {
    this.angle = angle;
    this.arm = arm;
    addRequirements(arm);
  }

  private final ArmSubsystem arm;
  private final Supplier<Double> angle;


  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    DogLog.log("Running to target angle: ", angle.get());
    arm.setPosition(angle.get());
  }

  @Override
  public boolean isFinished() {
    return arm.atTarget(5);
  }

  @Override
  public void end(boolean interrupted) {
  }

}
