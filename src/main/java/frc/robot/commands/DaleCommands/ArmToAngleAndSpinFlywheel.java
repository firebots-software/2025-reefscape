// TODO: THIS COMMAND NEED TO BE UPDATED TO WORK PROPERLY
package frc.robot.commands.DaleCommands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ArmToAngleAndSpinFlywheel extends Command {
  private final ArmSubsystem armPlusFlywheel;
  private double angle;
  private double tolerance = 5;
  private ElevatorSubsystem elevatorSubsystem;

  public ArmToAngleAndSpinFlywheel(double angle, ArmSubsystem armSub, ElevatorSubsystem elevatorSubsystem) {
    armPlusFlywheel = armSub;
    this.angle = angle;
    this.elevatorSubsystem = elevatorSubsystem; 
    addRequirements(armPlusFlywheel,elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    armPlusFlywheel.spinFlywheel(); // Spin the flywheel at desired speed
    DogLog.log("Running to target angle: ", angle);
    armPlusFlywheel.setPosition(angle);
  }

  @Override
  public void end(boolean interrupted) {
    armPlusFlywheel.stopFlywheel(); // Stop the flywheel
    elevatorSubsystem.elevateTo(ElevatorPositions.safePosition);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
