// TODO: THIS COMMAND NEED TO BE UPDATED TO WORK PROPERLY
package frc.robot.commands.DaleCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SpinFlywheel extends Command {
  private final ArmSubsystem armPlusFlywheel;
  private final double flywheelSpeed; // Desired flywheel speed in RPM or appropriate units

  public SpinFlywheel(ArmSubsystem armSub, double speed) {
    armPlusFlywheel = armSub;
    flywheelSpeed = speed;
    addRequirements(armPlusFlywheel);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    armPlusFlywheel.spinFlywheel(flywheelSpeed); // Spin the flywheel at desired speed
  }

  @Override
  public void end(boolean interrupted) {
    armPlusFlywheel.stopFlywheel(); // Stop the flywheel
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until manually interrupted
  }
}
