package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commandGroups.AutoLiftAndShoot;
import frc.robot.commandGroups.LoadAndPutUp;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.FunnelCommands.RunFunnelUntilCheckedIn;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class AutoRoutines {
  private final AutoFactory autoFactory;
  private final SwerveSubsystem driveTrain;
  private final ElevatorSubsystem elevatorSubsystem;
  private final TootsieSlideSubsystem tootsieSlideSubsystem;
  private final FunnelSubsystem funnelSubsystem;
  private final BooleanSupplier redside;

  private AutoRoutine routine;
  private ArrayList<String> topNames;
  private ArrayList<String> middleNames;
  private ArrayList<String> bottomNames;
  private ArrayList<AutoTrajectory> topTraj;
  private ArrayList<AutoTrajectory> middleTraj;
  private ArrayList<AutoTrajectory> bottomTraj;

  public AutoRoutines(
      AutoFactory factory,
      SwerveSubsystem driveTrain,
      ElevatorSubsystem elevatorSubsystem,
      TootsieSlideSubsystem tootsieSlideSubsystem,
      FunnelSubsystem funnelSubsystem,
      BooleanSupplier redside) {
    autoFactory = factory;
    this.driveTrain = driveTrain;
    this.elevatorSubsystem = elevatorSubsystem;
    this.tootsieSlideSubsystem = tootsieSlideSubsystem;
    this.funnelSubsystem = funnelSubsystem;
    this.redside = redside;

    this.routine = autoFactory.newRoutine("routine");

    this.topNames = new ArrayList<>();
    this.topNames.add("TSTART-4R");
    this.topNames.add("4R-THPS");
    this.topNames.add("THPS-5L");
    this.topNames.add("5L-THPS");
    this.topNames.add("THPS-5R");
    this.topNames.add("5R-THPS");
    this.topNames.add("THPS-0L");

    this.middleNames = new ArrayList<>();
    this.middleNames.add("MSTART-3R");

    this.bottomNames = new ArrayList<>();
    this.bottomNames.add("BSTART-2L");
    this.bottomNames.add("2L-BHPS");
    this.bottomNames.add("BHPS-1R");
    this.bottomNames.add("1R-BHPS");
    this.bottomNames.add("BHPS-1L");
    this.bottomNames.add("1L-BHPS");
    this.bottomNames.add("BHPS-0R");

    this.topTraj = new ArrayList<>();
    this.middleTraj = new ArrayList<>();
    this.bottomTraj = new ArrayList<>();

    for(String n : topNames){
      topTraj.add(routine.trajectory(n));
    }
    for(String n : middleNames){
      middleTraj.add(routine.trajectory(n));
    }
    for(String n : bottomNames){
      bottomTraj.add(routine.trajectory(n));
    }

  }

  public AutoRoutine autoRoutine(String chosenAuto) {
    switch (chosenAuto) {
      case "top":
        routine
            .active()
            .onTrue(
              new SequentialCommandGroup(
                topTraj.get(0)
                .resetOdometry(),
                autoSubCommand(routine, chosenAuto, 0 ),
                autoSubCommand(routine, chosenAuto, 1 ),
                autoSubCommand(routine, chosenAuto, 2 ),
                autoSubCommand(routine, chosenAuto, 3 ),
                autoSubCommand(routine, chosenAuto, 4 ),
                autoSubCommand(routine, chosenAuto, 5 ),
                autoSubCommand(routine, chosenAuto, 6 )
                ));
              
            
                // routine
                //     .trajectory(topTraj.get(0).toString())
                //     .resetOdometry()
                //     .andThen(autoSubCommand(routine, "TSTART-4R"))
                //     .andThen(autoSubCommand(routine, "4R-THPS"))
                //     .andThen(autoSubCommand(routine, "THPS-5L"))
                //     .andThen(autoSubCommand(routine, "5L-THPS"))
                //     .andThen(autoSubCommand(routine, "THPS-5R"))
                //     .andThen(autoSubCommand(routine, "5R-THPS"))
                //     .andThen(autoSubCommand(routine, "THPS-0L")));
        break;

      case "middle":
        routine
            .active()
            .onTrue(
                routine
                    .trajectory("MSTART-3R")
                    .resetOdometry()
                    .andThen(autoSubCommand(routine, "MSTART-3R")));
        break;

      case "bottom":
        routine
            .active()
            .onTrue(
                routine
                    .trajectory("BSTART-2L")//
                    .resetOdometry()
                    .andThen(autoSubCommand(routine, "BSTART-2L"))//
                    .andThen(autoSubCommand(routine, "2L-BHPS"))//
                    .andThen(autoSubCommand(routine, "BHPS-1R"))//
                    .andThen(autoSubCommand(routine, "1R-BHPS"))//
                    .andThen(autoSubCommand(routine, "BHPS-1L"))//
                    .andThen(autoSubCommand(routine, "1L-BHPS"))//
                    .andThen(autoSubCommand(routine, "BHPS-0R")));
        break;
    }

    return routine;
  }

  /**
   * @param routine
   * @param baseCommandName
   * @return
   */
  public Command autoSubCommand(AutoRoutine routine, String chosenAuto, int index) {
    String trajName;
    AutoTrajectory trajectory;
    boolean pathGoesToHPS;

    switch (chosenAuto) {
      
      case "top":
        trajName = topNames.get(index);
        trajectory = topTraj.get(index);
        pathGoesToHPS = !(trajName.contains("HPS-") || trajName.contains("START-"));
        
        break;
        
      case "middle":
        trajName = middleNames.get(index);
        trajectory = middleTraj.get(index);
        pathGoesToHPS = !(trajName.contains("HPS-") || trajName.contains("START-"));
        
        break;

      case "bottom":
        trajName = bottomNames.get(index);
        trajectory = bottomTraj.get(index);
        pathGoesToHPS = !(trajName.contains("HPS-") || trajName.contains("START-"));
        break;
    }

    return new SequentialCommandGroup(
      
    )

    // return Commands.parallel(
    //         routine.trajectory(baseCommandName).cmd(),
    //         Commands.sequence(
    //             new LoadAndPutUp(
    //                     elevatorSubsystem,
    //                     funnelSubsystem,
    //                     tootsieSlideSubsystem,
    //                     ElevatorPositions.Intake) // TEST TEST TEST
    //                 .onlyIf(() -> !pathGoesToHPS.getAsBoolean()),
    //             ((pathGoesToHPS.getAsBoolean())
    //                 ? new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake) // TEST THIS
    //                 : new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L4)))) // TEST THIS
    //     .andThen(
    //         (pathGoesToHPS.getAsBoolean())
    //             ? new RunFunnelUntilCheckedIn(funnelSubsystem) // TEST
    //             : new AutoLiftAndShoot(elevatorSubsystem, tootsieSlideSubsystem)); // TEST!!
  }
}
// new SetElevatorLevel // Intake
// new RunFunnelUntilDetectionSafe
// new TransferPieceBetweenFunnelAndElevator
// new SetElevatorLevel // L4
// new ShootTootsieSlide
// new SetElevatorLevel // Intake
