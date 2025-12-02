package frc.robot;

import java.util.function.BooleanSupplier;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commandGroups.Intake;
import frc.robot.commandGroups.JamesHardenScore;
import frc.robot.commands.DebugCommands.DogLogCmd;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.FunnelCommands.RunFunnelUntilCheckedIn;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

public class AutoSubCommandHolder {
    public AutoSubCommandHolder() {
        
    }

    public Command subCommand(String trajName, AutoRoutine routine, ElevatorSubsystem elevatorSubsystem, FunnelSubsystem funnelSubsystem, TootsieSlideSubsystem tootsieSlideSubsystem, SwerveSubsystem driveTrain, BooleanSupplier redside) {
                            AutoTrajectory trajectory = routine.trajectory(trajName);
                    
                            //-------- your autosubcommand goes here --------
                            
                            
                            //example autosubcommand
                            BooleanSupplier pathGoesToHPS =
                                () -> !(trajName.contains("HPS-") || trajName.contains("START-"));
                            BooleanSupplier startOrLeavingHPS = () -> !pathGoesToHPS.getAsBoolean();
                            boolean goRightBranch = trajName.substring(trajName.length() - 1).equals("R");
                    
                            DogLog.log("Auto/trajName", trajName);
                            DogLog.log("Auto/pathGoesToHPS", pathGoesToHPS.getAsBoolean());
                            Command newStructure2 =
                            Commands.sequence(
                                new Intake(elevatorSubsystem, funnelSubsystem, tootsieSlideSubsystem)
                            .onlyIf(startOrLeavingHPS),
                        new SetElevatorLevel(
                            elevatorSubsystem,
                            ElevatorPositions
                                .safePosition), // using L1 as the Safe Position because not sure if the "pos"
                        // value in
                        // the Constants Enum should be 0 or 1
                        trajectory
                            .cmd()
                            .alongWith(new DogLogCmd("Auto/CurrTrajRunning", trajName))
                            .andThen(new DogLogCmd("Auto/CurrTrajRunning", "none")), // actual robot movement
                        (pathGoesToHPS.getAsBoolean()
                            ? new ParallelCommandGroup(
                                new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake),
                            new RunFunnelUntilCheckedIn(funnelSubsystem))
                        : new JamesHardenScore(
                            elevatorSubsystem,
                            tootsieSlideSubsystem,
                            driveTrain,
                        ElevatorPositions.L4,
                        redside,
                    goRightBranch)));
        
        return newStructure2
        .alongWith(new DogLogCmd("Auto/CurrTrajRunning", trajName))
        .andThen(new DogLogCmd("Auto/CurrTrajRunning", "none"));
    }
}
