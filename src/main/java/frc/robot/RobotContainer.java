// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.Landmarks.BranchSide;
import frc.robot.commandGroups.D2Intake;
import frc.robot.commandGroups.Dealgaenate;
import frc.robot.commandGroups.EjectCoralFR;
import frc.robot.commandGroups.ElevatorL4;
import frc.robot.commandGroups.Intake;
import frc.robot.commandGroups.JamesHardenScore;
import frc.robot.commands.CoralInTootsie;
import frc.robot.commands.RobotPositionCommand;
import frc.robot.commandGroups.PutUpAndShoot;
import frc.robot.commands.DaleCommands.ArmToAngleCmd;
import frc.robot.commands.DaleCommands.ZeroArm;
import frc.robot.commands.DebugCommands.DogLogCmd;
import frc.robot.commands.ElevatorCommands.DefaultElevator;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.FunnelCommands.RunFunnelAndTootsieInCommand;
import frc.robot.commands.FunnelCommands.RunFunnelAndTootsieOutCommand;
import frc.robot.commands.FunnelCommands.RunFunnelUntilDetectionSafe;
import frc.robot.commands.SwerveCommands.JamesHardenMovement;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
import frc.robot.commands.TootsieSlideCommands.ShootTootsieSlide;
import frc.robot.commands.TransferPieceBetweenFunnelAndElevator;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;
import frc.robot.util.CustomController;
import frc.robot.util.RobotPosition;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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

  private final SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();

  private final Telemetry logger =
      new Telemetry(Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController joystick2 = new CommandXboxController(1);
  private final CommandXboxController debugJoystick = new CommandXboxController(3);
  private final CustomController customController = new CustomController(4);

  // Starts telemetry operations (essentially logging -> look on SmartDashboard, AdvantageScope)
  public void doTelemetry() {
    logger.telemeterize(driveTrain.getCurrentState());
  }

  private final AutoFactory autoFactory;
  private final AutoChooser autoChooser;

  public RobotContainer() {
    autoFactory =
        new AutoFactory(
            driveTrain::getPose, // A function that returns the current robot pose
            driveTrain
                ::resetPose, // A function that resets the current robot pose to the provided Pose2d
            driveTrain::followTrajectory, // The drive subsystem trajectory follower
            true, // If alliance flipping should be enabled
            driveTrain);
    autoChooser = new AutoChooser();
    AutoRoutines autoRoutines =
        new AutoRoutines(autoFactory, driveTrain, tootsieSlideSubsystem, elevatorSubsystem);
    // Add options to the chooser
    autoChooser.addRoutine("Basic Four Coral Auto", autoRoutines::basicFourCoralAuto);
    RobotPosition.Instance = new RobotPosition(ElevatorPositions.L1, BranchSide.RIGHT, 0);
    // Put the auto chooser on the dashboard
    SmartDashboard.putData("autochooser", autoChooser);
    configureBindings();
  }

  private void configureBindings() {

    armSubsystem.setDefaultCommand(new ArmToAngleCmd(0.0, armSubsystem));
    elevatorSubsystem.setDefaultCommand(new DefaultElevator(elevatorSubsystem));
    DoubleSupplier frontBackFunction = () -> -joystick.getLeftY(),
        leftRightFunction = () -> -joystick.getLeftX(),
        rotationFunction = () -> -joystick.getRightX(),
        speedFunction =
            () ->
                joystick.leftTrigger().getAsBoolean()
                    ? 0d
                    : 1d; // slowmode when left shoulder is pressed, otherwise fast
    SwerveJoystickCommand swerveJoystickCommand =
        new SwerveJoystickCommand(
            frontBackFunction,
            leftRightFunction,
            rotationFunction,
            speedFunction, // slowmode when left shoulder is pressed, otherwise fast
            () -> true, // always field centric
            driveTrain);
    driveTrain.setDefaultCommand(swerveJoystickCommand);
    // Custom Controller:

    // Left Elevator Levels
    customController
        .LeftL1()
        .whileTrue(new RobotPositionCommand(ElevatorPositions.L1,BranchSide.LEFT));

    customController
        .LeftL2()
        .whileTrue(new RobotPositionCommand(ElevatorPositions.L2,BranchSide.LEFT));
    customController
        .LeftL3()
        .whileTrue(new RobotPositionCommand(ElevatorPositions.L3,BranchSide.LEFT));
    customController
        .LeftL4()
        .whileTrue(new RobotPositionCommand(ElevatorPositions.L4,BranchSide.LEFT));

    // Right Elevator Levels
    customController
        .RightL1()
        .whileTrue(new RobotPositionCommand(ElevatorPositions.L1,BranchSide.RIGHT));
    customController
        .RightL2()
        .whileTrue(new RobotPositionCommand(ElevatorPositions.L2,BranchSide.RIGHT));

    customController
        .RightL3()
        .whileTrue(new RobotPositionCommand(ElevatorPositions.L3,BranchSide.RIGHT));

    customController
        .RightL4()
        .whileTrue(new RobotPositionCommand(ElevatorPositions.L4,BranchSide.RIGHT));

    // Bottom Three Buttons
    customController.Eject().onTrue(new EjectCoralFR(elevatorSubsystem, tootsieSlideSubsystem));
    customController
        .In()
        .whileTrue(new RunFunnelAndTootsieInCommand(funnelSubsystem, tootsieSlideSubsystem));
    customController
        .Out()
        .whileTrue(new RunFunnelAndTootsieOutCommand(funnelSubsystem, tootsieSlideSubsystem));

    // Joystick 1:
    // Dale
    joystick
        .rightBumper()
        .whileTrue(new Dealgaenate(armSubsystem, elevatorSubsystem, ElevatorPositions.L3DALE));
    joystick
        .leftBumper()
        .whileTrue(new Dealgaenate(armSubsystem, elevatorSubsystem, ElevatorPositions.L2DALE));
    // Zero Rotations
    joystick
        .y()
        .onTrue(
            driveTrain.runOnce(
                () ->
                    driveTrain.resetPose(
                        new Pose2d(driveTrain.getPose().getTranslation(), new Rotation2d(0)))));
    joystick.rightTrigger().whileTrue(
        new JamesHardenScore(
        elevatorSubsystem,
        tootsieSlideSubsystem,
        driveTrain,
        redside,
        () -> RobotPosition.Instance.elevator,
        () -> RobotPosition.Instance.side
        ).onlyIf(() -> Math.abs(RobotPosition.Instance.getCurrentTime() - RobotPosition.Instance.time) <= 3.0));
    // Joystick 2:
    // Elevator
    joystick2.x().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L3));
    joystick2.a().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L2));
    joystick2.y().onTrue(new ElevatorL4(elevatorSubsystem));
    joystick2.b().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L3));
    joystick2
        .rightBumper()
        .onTrue(
            new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.safePosition)); // reset mode

    // Shoot Tootsie Slide
    joystick2.rightTrigger().whileTrue(new ShootTootsieSlide(tootsieSlideSubsystem));

    // Intake
    joystick2
        .leftTrigger()
        .onTrue(new D2Intake(elevatorSubsystem, tootsieSlideSubsystem, funnelSubsystem));

    // Auto Intake and Eject
    Trigger funnelCheckin =
        new Trigger(
            () -> funnelSubsystem.isCoralCheckedIn() && !CoralPosition.isCoralInTootsieSlide());
    Trigger ejectTime =
        new Trigger(
            () -> (funnelSubsystem.isCoralCheckedIn() && CoralPosition.isCoralInTootsieSlide()));
    ejectTime.onTrue(new EjectCoralFR(elevatorSubsystem, tootsieSlideSubsystem));
    funnelCheckin.onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake));
    funnelCheckin.onTrue(new RunFunnelUntilDetectionSafe(funnelSubsystem, elevatorSubsystem));
    Trigger funnelCheckout =
        new Trigger(
            () ->
                CoralPosition.isCoralInFunnel()
                    && elevatorSubsystem.atIntake()
                    && elevatorSubsystem.isAtPosition());
    funnelCheckout.onTrue(
        new TransferPieceBetweenFunnelAndElevator(
            elevatorSubsystem, funnelSubsystem, tootsieSlideSubsystem));
    Trigger coralInElevator = new Trigger(() -> CoralPosition.isCoralInTootsieSlide());
    coralInElevator.onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.safePosition));

    // If we start with Coral in Tootsie Slide
    Trigger drake = new Trigger(() -> funnelSubsystem.drakeTripped());
    drake.whileTrue(new CoralInTootsie());

    // Swerve
    joystick
        .a()
        .onTrue(
            driveTrain.runOnce(
                () ->
                    driveTrain.resetPose(
                        new Pose2d(
                            Constants.Landmarks.LEFT_LINEUP_RED[5],
                            Constants.Landmarks.reefFacingAngleRed[5]))));

    joystick
        .b()
        .whileTrue(
            new PutUpAndShoot(elevatorSubsystem, tootsieSlideSubsystem, ElevatorPositions.L3));
    joystick
        .povUp()
        .whileTrue(
            new PutUpAndShoot(elevatorSubsystem, tootsieSlideSubsystem, ElevatorPositions.L2));
    joystick
        .povDown()
        .whileTrue(
            new PutUpAndShoot(elevatorSubsystem, tootsieSlideSubsystem, ElevatorPositions.L4));

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

    // new Translation2d(
    //     Constants.Landmarks.leftBranchesRed[5].getX()
    //         - (((Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length
    //                         .in(Meters)
    //                     / 2.0)
    //                 + Constants.Swerve.WHICH_SWERVE_ROBOT.BUMPER_THICKNESS
    //                     .thickness.in(Meters)))
    //             * Constants.Landmarks.reefFacingAngleRed[5].getCos(),
    //     Constants.Landmarks.leftBranchesRed[5].getY()
    //         - (((Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length
    //                         .in(Meters)
    //                     / 2.0)
    //                 + Constants.Swerve.WHICH_SWERVE_ROBOT.BUMPER_THICKNESS
    //                     .thickness.in(Meters)))
    //             * Constants.Landmarks.reefFacingAngleRed[5].getSin()),
    // new Rotation2d(
    //     Constants.Landmarks.reefFacingAngleRed[5].getRadians());

    // Debugging
    debugJoystick.leftTrigger().whileTrue(new ShootTootsieSlide(tootsieSlideSubsystem));

    debugJoystick
        .y()
        .whileTrue(
            new Dealgaenate(
                armSubsystem,
                elevatorSubsystem,
                Constants.ElevatorConstants.ElevatorPositions.L2DALE));
    debugJoystick
        .x()
        .onTrue(new SetElevatorLevel(ElevatorSubsystem.getInstance(), ElevatorPositions.Intake));
    debugJoystick.a().onTrue(new ZeroArm(armSubsystem));
    debugJoystick
        .b()
        .whileTrue(
            new Dealgaenate(
                armSubsystem,
                elevatorSubsystem,
                Constants.ElevatorConstants.ElevatorPositions.L3DALE));

    debugJoystick.leftTrigger().whileTrue(new ShootTootsieSlide(tootsieSlideSubsystem));

    // debugJoystick
    //     .y()
    //     .whileTrue(
    //         new Dealgaenate(
    //             armSubsystem,
    //             elevatorSubsystem,
    //             Constants.ElevatorConstants.ElevatorPositions.L2DALE));

    debugJoystick.y().whileTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L4));
    debugJoystick
        .x()
        .onTrue(new SetElevatorLevel(ElevatorSubsystem.getInstance(), ElevatorPositions.Intake));
    debugJoystick
        .b()
        .whileTrue(
            new Dealgaenate(
                armSubsystem,
                elevatorSubsystem,
                Constants.ElevatorConstants.ElevatorPositions.L3DALE));

    debugJoystick
        .rightTrigger()
        .onTrue(new Intake(elevatorSubsystem, funnelSubsystem, tootsieSlideSubsystem));

    // debugJoystick
    //     .rightTrigger()
    //     .onTrue(
    //         new LoadAndPutUp(
    //             elevatorSubsystem, funnelSubsystem, tootsieSlideSubsystem,
    // ElevatorPositions.L3));
  }

  public static void setAlliance() {
    redAlliance =
        (DriverStation.getAlliance().isEmpty())
            ? false
            : (DriverStation.getAlliance().get() == Alliance.Red);
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    if (redside.getAsBoolean()) {
      return new SequentialCommandGroup(
          new InstantCommand(
              () ->
                  driveTrain.resetPose(
                      new Pose2d(
                          new Translation2d(10.463430404663086, 7.600519180297852),
                          new Rotation2d()))),
          new DogLogCmd("CURRENT COMMAND", "RESET POSE"),
          new JamesHardenScore(
              elevatorSubsystem,
              tootsieSlideSubsystem,
              driveTrain,
              ElevatorPositions.L3,
              redside,
              () -> BranchSide.LEFT),
          new DogLogCmd("CURRENT COMMAND", "1ST JH SCORE"),
          new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake),
          new DogLogCmd("CURRENT COMMAND", "1ST ELEVATOR DOWN"),
          new JamesHardenMovement(
              driveTrain,
              new Pose2d(
                  new Translation2d(16.70710563659668, 6.779853343963623),
                  new Rotation2d(0.9429051116124475 + Math.PI))),
          new DogLogCmd("CURRENT COMMAND", "1ST HPS VISIT"),
          driveTrain.applyRequest(() -> brake).withTimeout(0.5),
          new DogLogCmd("CURRENT COMMAND", "1ST BRAKE"),
          new JamesHardenScore(
              elevatorSubsystem,
              tootsieSlideSubsystem,
              driveTrain,
              ElevatorPositions.L3,
              redside,
              () -> BranchSide.LEFT),
          new DogLogCmd("CURRENT COMMAND", "2ND JH SCORE"),
          new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake),
          new DogLogCmd("CURRENT COMMAND", "2ND ELEVATOR DOWN"),
          new JamesHardenMovement(
              driveTrain,
              new Pose2d(
                  new Translation2d(16.70710563659668, 6.779853343963623),
                  new Rotation2d(0.9429051116124475 + Math.PI))),
          new DogLogCmd("CURRENT COMMAND", "2ND HPS VISIT"),
          driveTrain.applyRequest(() -> brake).withTimeout(0.5),
          new DogLogCmd("CURRENT COMMAND", "2ND BRAKE"),
          new JamesHardenScore(
              elevatorSubsystem,
              tootsieSlideSubsystem,
              driveTrain,
              ElevatorPositions.L3,
              redside,
              () -> BranchSide.RIGHT),
          new DogLogCmd("CURRENT COMMAND", "3RD JH SCORE"),
          new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake),
          new DogLogCmd("CURRENT COMMAND", "3RD ELEVATOR DOWN"),
          new JamesHardenMovement(
              driveTrain,
              new Pose2d(
                  new Translation2d(16.70710563659668, 6.779853343963623),
                  new Rotation2d(0.9429051116124475 + Math.PI))),
          new DogLogCmd("CURRENT COMMAND", "3RD HPS VISIT"),
          driveTrain.applyRequest(() -> brake).withTimeout(0.5));
    } else {
      return new SequentialCommandGroup(
          new InstantCommand(
              () ->
                  driveTrain.resetPose(
                      new Pose2d(
                          new Translation2d(7.117019176483154, 0.44347667694091797),
                          new Rotation2d(Math.PI)))),
          new DogLogCmd("CURRENT COMMAND", "RESET POSE"),
          new JamesHardenScore(
              elevatorSubsystem,
              tootsieSlideSubsystem,
              driveTrain,
              ElevatorPositions.L3,
              redside,
              () -> BranchSide.LEFT),
          new DogLogCmd("CURRENT COMMAND", "1ST JH SCORE"),
          new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake),
          new DogLogCmd("CURRENT COMMAND", "1ST ELEVATOR DOWN"),
          new JamesHardenMovement(
              driveTrain,
              new Pose2d(
                  new Translation2d(1.102602243423462, 1.0577188730239868),
                  new Rotation2d(0.9481256208435748))),
          new DogLogCmd("CURRENT COMMAND", "1ST HPS VISIT"),
          driveTrain.applyRequest(() -> brake).withTimeout(0.5),
          new DogLogCmd("CURRENT COMMAND", "1ST BRAKE"),
          new JamesHardenScore(
              elevatorSubsystem,
              tootsieSlideSubsystem,
              driveTrain,
              ElevatorPositions.L3,
              redside,
              () -> BranchSide.LEFT),
          new DogLogCmd("CURRENT COMMAND", "2ND JH SCORE"),
          new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake),
          new DogLogCmd("CURRENT COMMAND", "2ND ELEVATOR DOWN"),
          new JamesHardenMovement(
              driveTrain,
              new Pose2d(
                  new Translation2d(1.102602243423462, 1.0577188730239868),
                  new Rotation2d(0.9481256208435748))),
          new DogLogCmd("CURRENT COMMAND", "2ND HPS VISIT"),
          driveTrain.applyRequest(() -> brake).withTimeout(0.5),
          new DogLogCmd("CURRENT COMMAND", "2ND BRAKE"),
          new JamesHardenScore(
              elevatorSubsystem,
              tootsieSlideSubsystem,
              driveTrain,
              ElevatorPositions.L3,
              redside,
              () -> BranchSide.RIGHT),
          new DogLogCmd("CURRENT COMMAND", "3RD JH SCORE"),
          new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake),
          new DogLogCmd("CURRENT COMMAND", "3RD ELEVATOR DOWN"),
          new JamesHardenMovement(
              driveTrain,
              new Pose2d(
                  new Translation2d(1.102602243423462, 1.0577188730239868),
                  new Rotation2d(0.9481256208435748))),
          new DogLogCmd("CURRENT COMMAND", "3RD HPS VISIT"),
          driveTrain.applyRequest(() -> brake).withTimeout(0.5));
    }
  }
}
