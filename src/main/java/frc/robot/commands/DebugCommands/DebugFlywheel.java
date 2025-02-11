package frc.robot.commands.DebugCommands;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class DebugFlywheel extends Command{

  private final ArmSubsystem arm;

    public DebugFlywheel(ArmSubsystem arm){
        this.arm = arm;
        addRequirements(arm);
    }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    arm.spinFlywheel(40);
  }

  @Override
  public void end(boolean interrupted) {
    arm.stopFlywheel();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
