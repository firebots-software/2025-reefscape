// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commandGroups.Dealgaenate;
import frc.robot.commands.DaleCommands.ArmToAngleCmd;
import frc.robot.commands.ElevatorCommands.*;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.FunnelCommands.DefaultFunnelCommand;
import frc.robot.commands.FunnelCommands.RunFunnelUntilDetection;
import frc.robot.commands.SwerveCommands.JamesHardenMovement;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
import frc.robot.commands.TootsieSlideCommands.ShootTootsieSlide;
import frc.robot.commands.TransferPieceBetweenFunnelAndElevator;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;
import frc.robot.subsystems.VisionSystem;
import frc.robot.util.GyroStabilizer;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RobotContainer {
  private static Matrix<N3, N1> visionMatrix = VecBuilder.fill(0.01, 0.03d, 100d);
  private static Matrix<N3, N1> odometryMatrix = VecBuilder.fill(0.1, 0.1, 0.1);

  TootsieSlideSubsystem tootsieSlideSubsystem = TootsieSlideSubsystem.getInstance();
  FunnelSubsystem funnelSubsystem = FunnelSubsystem.getInstance();
  ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
  ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
  // Alliance color
  Boolean coralInFunnel = Boolean.valueOf(false);
  Boolean coralInElevator = Boolean.valueOf(false);

  private BooleanSupplier redside = () -> redAlliance;
  private static boolean redAlliance;

  // SmartDashboard Auto Chooser: Returns "B", "T", or "M"
  private final SendableChooser<String> startPosChooser = new SendableChooser<String>();

  private final SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();

  private final Telemetry logger =
      new Telemetry(Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
  private VisionSystem visionBack = VisionSystem.getInstance(Constants.Vision.Cameras.BACK_CAM);
  private VisionSystem visionFront = VisionSystem.getInstance(Constants.Vision.Cameras.FRONT_CAM);
  private final CommandXboxController joystick = new CommandXboxController(0);

  private final AutoFactory autoFactory;

  // Starts telemetry operations (essentially logging -> look on SmartDashboard, AdvantageScope)

  public RobotContainer() {
    // SmartDashboard Auto Chooser: Returns "B", "T", or "M"
    startPosChooser.setDefaultOption("Top (Next to Blue Barge Zone)", "top");
    startPosChooser.addOption("Middle (In between to Barge Zones)", "middle");
    startPosChooser.addOption("Bottom (Next to Red Barge Zone)", "bottom");
    SmartDashboard.putData("Auto Start Position", startPosChooser);

    configureBindings();
    autoFactory =
        new AutoFactory(
            driveTrain::getPose, // A function that returns the current robot pose
            driveTrain
                ::resetPose, // A function that resets the current robot pose to the provided Pose2d
            driveTrain::followTrajectory, // The drive subsystem trajectory follower
            true, // If alliance flipping should be enabled
            driveTrain);
  }

  // Starts telemetry operations (essentially logging -> look on SmartDashboard, AdvantageScope)
  public void doTelemetry() {
    // logger.telemeterize(driveTrain.getState()); idk
    // logger.telemeterize(driveTrain.getCurrentState());
    Pose2d camPose = visionFront.getPose2d();
    Pose2d camPose2 = visionBack.getPose2d();
    if (camPose != null || camPose2 != null) {
      logger.logVisionPose(VisionSystem.getAverageForOffBotTesting(camPose, camPose2));
    }
  }

  private void configureBindings() {
    funnelSubsystem.setDefaultCommand(new DefaultFunnelCommand(funnelSubsystem));
    Trigger funnelCheckin = new Trigger(() -> funnelSubsystem.isCoralCheckedIn());
    funnelCheckin.onTrue(new RunFunnelUntilDetection(funnelSubsystem, elevatorSubsystem));

    Trigger leftShoulderTrigger = joystick.leftBumper();
    Supplier<Double>
        frontBackFunction = () -> ((redAlliance) ? joystick.getLeftY() : -joystick.getLeftY()),
        leftRightFunction = () -> ((redAlliance) ? joystick.getLeftX() : -joystick.getLeftX()),
        rotationFunction = () -> -joystick.getRightX(),
        speedFunction =
            () ->
                leftShoulderTrigger.getAsBoolean()
                    ? 0d
                    : 1d; // slowmode when left shoulder is pressed, otherwise fast
    SwerveJoystickCommand swerveJoystickCommand =
        new SwerveJoystickCommand(
            frontBackFunction,
            leftRightFunction,
            rotationFunction,
            speedFunction, // slowmode when left shoulder is pressed, otherwise fast
            () -> joystick.leftTrigger().getAsBoolean(),
            driveTrain);
    driveTrain.setDefaultCommand(swerveJoystickCommand);
    Trigger tipping = new Trigger(() -> GyroStabilizer.magnitudeTipVector(GyroStabilizer.getTipVectorXY(driveTrain.getPigeon2())) > GyroStabilizer.TIP_THRESHOLD);
    tipping.onTrue(new GyroStabilizer(driveTrain));

    // joystick.rightBumper().whileTrue(new
    // TootsieSlideShooting(TootsieSlideSubsystem.getInstance()));

    /*

    Sysid button commands, commented out (I like keeping this commented because
    every branch will have access to the necessary commands to run SysID immediately)

       joystick.povUp().onTrue(Commands.runOnce(SignalLogger::start));
       joystick.povDown().onTrue(Commands.runOnce(SignalLogger::stop));

    * Joystick Y = quasistatic forward
    * Joystick A = quasistatic reverse
    * Joystick B = dynamic forward
    * Joystick X = dyanmic reverse
    *
       joystick.y().whileTrue(driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
       joystick.a().whileTrue(driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
       joystick.b().whileTrue(driveTrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
       joystick.x().whileTrue(driveTrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    */

    joystick
        .a()
        .onTrue(
            driveTrain.runOnce(
                () ->
                    driveTrain.resetPose(
                        new Pose2d(
                            new Translation2d(
                                Constants.Landmarks.leftBranchesRed[5].getX()
                                    - (((Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length
                                                    .in(Meters)
                                                / 2.0)
                                            + Constants.Swerve.WHICH_SWERVE_ROBOT.BUMPER_THICKNESS
                                                .thickness.in(Meters)))
                                        * Constants.Landmarks.reefFacingAngleRed[5].getCos(),
                                Constants.Landmarks.leftBranchesRed[5].getY()
                                    - (((Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length
                                                    .in(Meters)
                                                / 2.0)
                                            + Constants.Swerve.WHICH_SWERVE_ROBOT.BUMPER_THICKNESS
                                                .thickness.in(Meters)))
                                        * Constants.Landmarks.reefFacingAngleRed[5].getSin()),
                            new Rotation2d(
                                Constants.Landmarks.reefFacingAngleRed[5].getRadians())))));

    joystick.y().whileTrue(JamesHardenMovement.toClosestRightBranch(driveTrain, redside));
    Trigger rightBumper = joystick.rightBumper();

    rightBumper.onTrue(new Dealgaenate(ArmSubsystem.getInstance()));
    rightBumper.onFalse(
        new ArmToAngleCmd(Constants.Arm.RETRACTED_ANGLE, ArmSubsystem.getInstance()));
    joystick.y().whileTrue(JamesHardenMovement.toClosestRightBranch(driveTrain, redside));

    joystick.povUp().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L1));
    joystick.povRight().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L2));
    joystick.povDown().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L3));
    joystick.povLeft().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L4));

    joystick
        .a()
        .whileTrue(
            new SetElevatorLevel(
                ElevatorSubsystem.getInstance(),
                ElevatorPositions.safePosition)); // change safepos in constants

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  public static void setAlliance() {
    redAlliance =
        (DriverStation.getAlliance().isEmpty())
            ? false
            : (DriverStation.getAlliance().get() == Alliance.Red);
  }

  public Command getAutonomousCommand() {
    // SmartDashboard Auto Chooser: Returns "bottom", "top", or "middle"

    /*
    Field Diagram

              BLUE                        RED
    ___________________________________________________
    |THPS                 TSTART                  THPS|
    |                       | |                       |
    |        5    4         | |         4    5        |
    |     0          3    MSTART     3          0     |
    |        1    2         | |         2    1        |
    |                       | |                       |
    |BHPS_________________BSTART__________________BHPS|

    L and R branches are on the left and right of the robot when it is at the reef in between the two branches

    a path name consists of a start and a destination
    a start and a destinantion can be on any one of those marked areas
    the format for a path is start-destination
    ex. 2L-BHPS
    this path starts from the left branch on the second part of the reef (2L), and goes to the bottom human player station (BHPS)
    */
    String chosenPath = startPosChooser.getSelected();

    AutoRoutine routine = autoFactory.newRoutine("routine");
    switch (chosenPath) {
      case "top":
        routine
            .active()
            .onTrue(
                routine
                    .trajectory("TSTART-4R")
                    .resetOdometry()
                    .andThen(autoSubCommand(routine, "TSTART-4R"))
                    .andThen(autoSubCommand(routine, "4R-THPS"))
                    .andThen(autoSubCommand(routine, "THPS-5L"))
                    .andThen(autoSubCommand(routine, "5L-THPS"))
                    .andThen(autoSubCommand(routine, "THPS-5R"))
                    .andThen(autoSubCommand(routine, "5R-THPS"))
                    .andThen(autoSubCommand(routine, "THPS-0L")));
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
                    .trajectory("BSTART-2L")
                    .resetOdometry()
                    .andThen(autoSubCommand(routine, "BSTART-2L"))
                    .andThen(autoSubCommand(routine, "2L-BHPS"))
                    .andThen(autoSubCommand(routine, "BHPS-1R"))
                    .andThen(autoSubCommand(routine, "1R-BHPS"))
                    .andThen(autoSubCommand(routine, "BHPS-1L"))
                    .andThen(autoSubCommand(routine, "1L-BHPS"))
                    .andThen(autoSubCommand(routine, "BHPS-0R")));
        break;
    }

    return routine.cmd();
  }

  public Command autoSubCommand(AutoRoutine routine, String baseCommandName) { // for actual robot
    BooleanSupplier pathGoesToHPS =
        () -> !(baseCommandName.contains("HPS-") || baseCommandName.contains("START-"));
    // if it has HPS- or START- the path ends at the reef and thus we will want to raise elevator
    // and shoot, else lower elevator and intake
    return Commands.parallel(
            routine.trajectory(baseCommandName).cmd(),
            ((pathGoesToHPS.getAsBoolean())
                ? new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake)
                : new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L4)))
        .andThen(
            (pathGoesToHPS.getAsBoolean())
                ? new RunFunnelUntilDetection(funnelSubsystem, elevatorSubsystem)
                : new ShootTootsieSlide(tootsieSlideSubsystem))
        .andThen(
            new TransferPieceBetweenFunnelAndElevator(
                elevatorSubsystem, funnelSubsystem, tootsieSlideSubsystem))
        .onlyIf(pathGoesToHPS); // transfers piece to elevator only if path doesnt go to hps
  }
}
