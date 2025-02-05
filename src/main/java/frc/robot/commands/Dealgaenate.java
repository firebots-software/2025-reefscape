// TODO: THIS COMMAND NEED TO BE UPDATED TO WORK PROPERLY
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class Dealgaenate extends Command {
  private final ArmSubsystem armPlusFlywheel;
  private final double flywheelSpeed; // Desired flywheel speed in RPM or appropriate units

  public Dealgaenate(ArmSubsystem armSub, double speed) {
    armPlusFlywheel = armSub;
    flywheelSpeed = speed;
    addRequirements(armPlusFlywheel);
  }

  @Override
  public void initialize() {
    // armPlusFlywheel.deployArm(); // Deploy the arm when command starts
  }

  @Override
  public void execute() {
    armPlusFlywheel.spinFlywheel(flywheelSpeed); // Spin the flywheel at desired speed
  }

  @Override
  public void end(boolean interrupted) {
    armPlusFlywheel.stopFlywheel(); // Stop the flywheel
    // armPlusFlywheel.retractArm();   // Retract the arm
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until manually interrupted
  }
}
