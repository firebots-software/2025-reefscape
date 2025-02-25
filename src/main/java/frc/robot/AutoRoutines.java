package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commandGroups.PutUpAndShoot;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;

public class AutoRoutines {
  private final AutoFactory autoFactory;
  private final SwerveSubsystem driveTrain;
  private final TootsieSlideSubsystem shooter;
  private final ElevatorSubsystem elevator;

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  public AutoRoutines(
      AutoFactory factory,
      SwerveSubsystem driveTrain,
      TootsieSlideSubsystem shooter,
      ElevatorSubsystem elevator) {
    autoFactory = factory;
    this.driveTrain = driveTrain;
    this.shooter = shooter;
    this.elevator = elevator;
  }

  public AutoRoutine basicFourCoralAuto() {
    final AutoRoutine routine = autoFactory.newRoutine("BasicFourCoralAuto");
    final AutoTrajectory blueStartToL2 = routine.trajectory("BSTART-L2");
    final AutoTrajectory l2ToBlueHumanPlayerStation = routine.trajectory("L2-BHPS");
    final AutoTrajectory blueHumanPlayerStationToR1 = routine.trajectory("BHPS-R1");
    final AutoTrajectory r1ToBlueHumanPlayerStation = routine.trajectory("R1-BHPS");
    final AutoTrajectory blueHumanPlayerStationToL1 = routine.trajectory("BHPS-L1");
    final AutoTrajectory l1ToBlueHumanPlayerStation = routine.trajectory("L1-BHPS");
    final AutoTrajectory blueHumanPlayerStationToR0 = routine.trajectory("BHPS-R0");

    routine
        .active()
        .onTrue(
            new SequentialCommandGroup(
                blueStartToL2.resetOdometry(),
                blueStartToL2.cmd(),
                new PutUpAndShoot(elevator, shooter, ElevatorPositions.L3),
                driveTrain.applyRequest(() -> brake).withTimeout(0.5),
                l2ToBlueHumanPlayerStation.cmd(),
                driveTrain.applyRequest(() -> brake).withTimeout(0.5),
                blueHumanPlayerStationToR1.cmd(),
                new PutUpAndShoot(elevator, shooter, ElevatorPositions.L3),
                driveTrain.applyRequest(() -> brake).withTimeout(0.5),
                r1ToBlueHumanPlayerStation.cmd(),
                driveTrain.applyRequest(() -> brake).withTimeout(0.5),
                blueHumanPlayerStationToL1.cmd(),
                new PutUpAndShoot(elevator, shooter, ElevatorPositions.L3),
                driveTrain.applyRequest(() -> brake).withTimeout(0.5),
                l1ToBlueHumanPlayerStation.cmd(),
                driveTrain.applyRequest(() -> brake).withTimeout(0.5),
                blueHumanPlayerStationToR0.cmd(),
                new PutUpAndShoot(elevator, shooter, ElevatorPositions.L3)));

    return routine;
  }
}
