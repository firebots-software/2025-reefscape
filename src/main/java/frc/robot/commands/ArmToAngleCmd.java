package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewArmSubsystem;
import java.util.function.Supplier;

public class ArmToAngleCmd extends Command {
  public ArmToAngleCmd(Supplier<Double> angle, NewArmSubsystem arm) {
    this.angle = angle;
    this.arm = arm;
    addRequirements(arm);
  }

  private final NewArmSubsystem arm;
  private final Supplier<Double> angle;
  private double endToleranceDegrees = 5;
  private boolean returnToRestIfInterrupted = true;

  @Override
  public void initialize() {
    // SmartDashboard.putBoolean("RUNNNININGINNIG", true);
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("RUNNNINING Angle to target ", angle.get());
    arm.setPosition(angle.get());
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("Is arm at target", arm.atTarget(endToleranceDegrees));
    return arm.atTarget(endToleranceDegrees);
  }

  @Override
  public void end(boolean interrupted) {
    // SmartDashboard.putBoolean("RUNNNININGINNIG BUNDT", false);
    // if (returnToRestIfInterrupted && interrupted) {
    //   // arm.rotateToRestPosition;
    //   arm.resetPosition();
    // }

  }

  public ArmToAngleCmd withTolerance(double degrees) {
    this.endToleranceDegrees = degrees;
    return this;
  }
}
