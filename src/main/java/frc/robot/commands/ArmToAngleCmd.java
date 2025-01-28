package frc.robot.commands;

//import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class ArmToAngleCmd extends Command {
  public ArmToAngleCmd(DoubleSupplier angle, ArmSubsystem arm) {
    this.angle = angle;
    this.arm = arm;
    addRequirements(arm);
  }

  private final ArmSubsystem arm;
  private final DoubleSupplier angle;
  private double tolerance = 5;

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    //DogLog.log("Running to target angle: ", angle.getAsDouble());
    arm.setPosition(angle.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return arm.atTarget(tolerance);
  }

  @Override
  public void end(boolean interrupted) {}
}
