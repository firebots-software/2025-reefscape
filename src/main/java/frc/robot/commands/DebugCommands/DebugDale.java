package frc.robot.commands.DebugCommands;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class DebugDale extends Command{
  private ArmSubsystem dale;
  private double angle;
    public DebugDale(ArmSubsystem dale){
        this.dale = dale;
        this.angle = 90;
        addRequirements(dale);
    }

    @Override
    public void initialize() {
  
    }
  
    @Override
    public void execute() {
        dale.setPosition(angle);
    }
    
     @Override
    public void end(boolean interrupted) {
        dale.setPosition(0);
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
    
}
