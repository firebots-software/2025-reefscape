package frc.robot.commands.DebugCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FunnelSubsystem;
public class DebugFunnelIntake extends Command{
    private FunnelSubsystem funnelSubsystem;
    public DebugFunnelIntake(FunnelSubsystem funnelSubsystem){
        this.funnelSubsystem = funnelSubsystem;
        addRequirements(funnelSubsystem);
    }

    @Override
    public void initialize() {
  
    }
  
    @Override
    public void execute() {
        funnelSubsystem.spinFunnel();
    }
    
     @Override
    public void end(boolean interrupted) {
        funnelSubsystem.stopFunnel();
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
  

      
}
