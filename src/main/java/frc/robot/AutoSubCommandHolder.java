package frc.robot;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSubCommandHolder {
    public AutoSubCommandHolder() {
        
    }

    public Command subCommand(String pathName, AutoRoutine routine) {
        AutoTrajectory traj = routine.trajectory(pathName);

        //-------- your autosubcommand goes here --------
        
        return null;
    }
}
