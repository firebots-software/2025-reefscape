package frc.robot.AutoRoutines;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;


import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.LandmarkPose;
import frc.robot.commandGroups.Intake;
import frc.robot.commandGroups.JamesHardenScore;
import frc.robot.commandGroups.PutUpAndShoot;
import frc.robot.commands.DaleCommands.ZeroArm;
import frc.robot.commands.ElevatorCommands.ElevatorHoldL4;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.ElevatorCommands.SetElevatorLevelInstant;
import frc.robot.commands.ElevatorCommands.ZeroElevatorHardStop;
import frc.robot.commands.EndWhenCloseEnough;
import frc.robot.commands.FunnelCommands.CoralCheckedIn;
import frc.robot.commands.SwerveCommands.JamesHardenMovement;
import frc.robot.commands.TootsieSlideCommands.ShootTootsieSlide;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;
import java.util.List;

public class AutoProducer {
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final SwerveSubsystem driveTrain;
  private final TootsieSlideSubsystem shooter;
  private final ElevatorSubsystem elevator;
  private final FunnelSubsystem funnel;
  private final ArmSubsystem arm;
  private final LedSubsystem leds;

  private final AutoFactory autoFactory;


  public AutoProducer(
      SwerveSubsystem driveTrain,
      TootsieSlideSubsystem shooter,
      ElevatorSubsystem elevator,
      FunnelSubsystem funnel,
      ArmSubsystem arm,
      LedSubsystem leds) { //List<LandmarkPose> autoInformation,
        this.driveTrain=driveTrain;
        this.shooter=shooter;
        this.elevator=elevator;
        this.funnel=funnel;
        this.arm=arm;
        this.leds=leds;

        autoFactory = new AutoFactory(driveTrain::getPose, driveTrain::resetPose, driveTrain::followTrajectory, true, driveTrain);

      }
    
    public AutoRoutine getRoutine(int autoValue) {
      AutoRoutine curr = null;
      switch (autoValue) {
        case 1:
          curr = topRed();
          break;
      }
      return curr;
    }

    private AutoRoutine topRed() {
      AutoRoutine routine = autoFactory.newRoutine("CR7");
      AutoTrajectory topRed = routine.trajectory("TR.traj");

      routine.active().onTrue(Commands.sequence(topRed.resetOdometry(), topRed.cmd()));

      topRed.atTime("shoot1").onTrue(new JamesHardenScore(elevator, shooter, driveTrain, ElevatorPositions.L4, () -> true, true, leds));
      topRed.atTime("intake1").onTrue(new Intake(elevator, funnel, shooter, leds));
      topRed.done().onTrue(new JamesHardenScore(elevator, shooter, driveTrain, ElevatorPositions.L4, () -> true, true, leds));
      // topRed.atTime("shoot2").onTrue(new JamesHardenScore(elevator, shooter, driveTrain, ElevatorPositions.L4, () -> true, true, leds));

      return routine;
    }
}


    // // first score
    // addCommands(
    //     new ParallelCommandGroup(
    //         new ZeroArm(arm).withTimeout(1.25),
    //         new ParallelDeadlineGroup(
    //             new SequentialCommandGroup(
    //                 new ZeroElevatorHardStop(elevator).withTimeout(1.5),
    //                 new Intake(elevator, funnel, shooter, leds).withTimeout(2.0)),
    //             JamesHardenMovement.toSpecificBranch(
    //                 driveTrain, () -> autoInformation.get(1), false))),
    //     new JamesHardenScore(
    //             elevator, shooter, driveTrain, ElevatorPositions.L4, autoInformation.get(1))
    //         .until(() -> !CoralPosition.isCoralInTootsieSlide()),
    //     new SetElevatorLevelInstant(elevator, ElevatorPositions.Intake));

    // if (autoInformation.size() > 2) {
    //   settyCycle(
    //       elevator,
    //       funnel,
    //       shooter,
    //       driveTrain,
    //       autoInformation.get(2),
    //       autoInformation.get(autoInformation.size() - 1),
    //       leds);
    // }
    // if (autoInformation.size() > 3) {
    //   settyCycle(
    //       elevator,
    //       funnel,
    //       shooter,
    //       driveTrain,
    //       autoInformation.get(3),
    //       autoInformation.get(autoInformation.size() - 1),
    //       leds);
    // }
//   }

//   private void settyCycle(
//       ElevatorSubsystem elevator,
//       FunnelSubsystem funnel,
//       TootsieSlideSubsystem shooter,
//       SwerveSubsystem driveTrain,
//       LandmarkPose scorePosition,
//       LandmarkPose HPSPosition,
//       LedSubsystem leds) {

//     JamesHardenMovement movementCommand, maintainCommand;
//     if (!scorePosition.isBranch()) {
//       DogLog.log("Commands/JamesHardenScore/Errors", "Called without a real branch");
//       return;
//     }
//     movementCommand = JamesHardenMovement.toSpecificBranch(driveTrain, () -> scorePosition, false);
//     maintainCommand = JamesHardenMovement.toSpecificBranch(driveTrain, () -> scorePosition, true);

//     addCommands(
//         new ParallelCommandGroup(
//             // Elevator related
//             new Intake(elevator, funnel, shooter, leds)
//                 .andThen(
//                     new EndWhenCloseEnough(
//                         () -> movementCommand.getTargetPose2d(),
//                         Constants.HardenConstants.EndWhenCloseEnough.translationalToleranceAuto,
//                         Constants.HardenConstants.EndWhenCloseEnough.headingTolerance))
//                 .andThen(new SetElevatorLevel(elevator, ElevatorPositions.L4, true)),

//             // Movement related
//             new SequentialCommandGroup(
//                 new ParallelDeadlineGroup(
//                     new CoralCheckedIn(funnel),
//                     new JamesHardenMovement(driveTrain, HPSPosition.getPose(), true)),
//                 new WaitCommand(0.2),
//                 movementCommand.withTimeout(5.0))), // Added timeout to movement command
//         // When the elevator is up and when the movement command is done, then do the following
//         new ElevatorHoldL4(elevator).withTimeout(0.25),
//         new ParallelDeadlineGroup(new ShootTootsieSlide(shooter).withTimeout(0.5), maintainCommand),
//         new SetElevatorLevelInstant(
//             elevator, ElevatorPositions.Intake)); // sets elevator back to intake when finished
//   }
// }

// .alongWith(
//                     new EndWhenCloseEnough(() -> movementCommand.getTargetPose2d()).andThen(new
// Command().on))

// are we sure that the autoinformation.size thing works? I feel like its going to run the 3 every
// time
