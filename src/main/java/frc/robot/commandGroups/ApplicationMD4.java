package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.FunnelCommands.RunFunnelOutCommand;
import frc.robot.commands.TootsieSlideCommands.ShootTootsieSlide;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

/*
 *
 *
 * Task Card:
 * You will be creating a CommandGroup that does the following:
 * 1. Intakes the Coral
 * 2. Elevator goes to L4
 * 3. Elevator goes to L2
 * 4. Elevator goes to L3
 * 5. Elevator goes to L2
 * 6. Elevator Shoots AND Spins the Funnel at the same time.
 *    When the Shooting ENDS stop the Funnel
 * 7. Elevator goes to L4
 * 8. AS Elevator lowers down to L2 spin both the Funnel and Shooter
 *
 *
 * NOTES:
 * * Auto-intaking has been disabled
 * * Auto-eject has been disabled
 * --> This means you have to be careful with trying to feed a coral
 * --> when there is already another coral in the robot
 *
 * * INVESTIGATE ALL OF THE RELEVANT COMMANDS AND THEIR END BEHAVIORS
 * * BEFORE YOU BEGIN CREATING THE COMMAND GROUP
 *
 * Relevant Commands:
 * SetElevatorLevel <-- takes care of Elevator raising to certain hegiht
 * ShootTootsieSlide <-- Shoots the coral (spins the shooter)
 * RunFunnelOutCommand <-- spins the funnel
 *
 *
 * * STARTING HINT: This class must extend either SequentialCommandGroup or
 * * ParallelCommandGroup
 *
 * Good luck and refer to the MD.4 Resources, me or your Software
 * mentors if you have any questions or difficulties!
 *
 * After you make your CommandGroup, check RobotContainer.java
 * Lines 359-360 and you will bind your CommandGroup to the
 * custom controller board left L1 button and test that way
 *
 */

public class ApplicationMD4 extends SequentialCommandGroup {
  public ApplicationMD4(
      ElevatorSubsystem elevatorSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      FunnelSubsystem funnelSubsystem,
      LedSubsystem leds) {
    {
      addCommands(
          // intake coral
          // elevator to l4, l3, l2, l1
          // shoot and spin funnel; when shooting ends stop funnel
          // elevator to l4
          // as elevator lowers to l2 spin both funnel and shooter

          new SequentialCommandGroup(
              new Intake(elevatorSubsystem, funnelSubsystem, tootsieSlideSubsystem, leds),
              new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L4, false),
              new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L3, false),
              new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L2, false),
              new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L1, false),
              new ParallelDeadlineGroup(
                  new ShootTootsieSlide(tootsieSlideSubsystem),
                  new RunFunnelOutCommand(funnelSubsystem, () -> false)),
                  new WaitCommand(1),
              new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L4, false),
              new ParallelDeadlineGroup(
                  new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L2, false),
                  new ShootTootsieSlide(tootsieSlideSubsystem),
                  new RunFunnelOutCommand(funnelSubsystem, () -> false))));
    }
  }
}
