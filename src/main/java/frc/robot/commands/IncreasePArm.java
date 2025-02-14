package frc.robot.commands.AbstractedPID;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class IncreasePArm extends Command {



  public IncreasePArm(ArmSubsystem arm) {
        
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Constants.Arm.S0C_KP += 0.1;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
