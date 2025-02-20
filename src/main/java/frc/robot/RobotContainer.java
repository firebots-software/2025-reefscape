// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commandGroups.Dealgaenate;
import frc.robot.commandGroups.EdwardScore;
import frc.robot.commandGroups.Intake;
import frc.robot.commandGroups.JamesHardenScore;
import frc.robot.commandGroups.EjectCoralFR;
import frc.robot.commands.DaleCommands.ArmToAngleCmd;
import frc.robot.commands.DaleCommands.ZeroArm;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
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
  private final CommandXboxController debugJoystick = new CommandXboxController(3);

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
    AutoRoutines autoRoutines = new AutoRoutines(autoFactory, driveTrain);
    // Add options to the chooser
    autoChooser.addRoutine("Basic Four Coral Auto", autoRoutines::basicFourCoralAuto);

    // Put the auto chooser on the dashboard
    SmartDashboard.putData("autochooser", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    // Automatic
    // funnelSubsystem.setDefaultCommand(new DefaultFunnelCommand(funnelSubsystem));
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
        .onTrue(new SetElevatorLevel(ElevatorSubsystem.getInstance(), ElevatorPositions.L4));
    debugJoystick.a().onTrue(new ZeroArm(armSubsystem));
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

    // Swerve
    Trigger leftShoulderTrigger = joystick.leftBumper();
    DoubleSupplier frontBackFunction = () -> -joystick.getLeftY(),
        leftRightFunction = () -> -joystick.getLeftX(),
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
    armSubsystem.setDefaultCommand(new ArmToAngleCmd(0.0, armSubsystem));
    // elevatorSubsystem.setDefaultCommand(
    //     new SetElevatorLevel(
    //         elevatorSubsystem, Constants.ElevatorConstants.ElevatorPositions.Intake));

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
        .rightBumper()
        .onTrue(new Dealgaenate(armSubsystem, elevatorSubsystem, ElevatorPositions.L2DALE));
    joystick.rightBumper().onFalse(new ArmToAngleCmd(Constants.Arm.RETRACTED_ANGLE, armSubsystem));

    joystick
        .b()
        .onTrue(new Intake(elevatorSubsystem, funnelSubsystem, tootsieSlideSubsystem));

    joystick.x().whileTrue(new JamesHardenScore(elevatorSubsystem, tootsieSlideSubsystem, driveTrain, ElevatorPositions.L3, redside, true));
    joystick.y().whileTrue(new EdwardScore(elevatorSubsystem, tootsieSlideSubsystem, driveTrain, ElevatorPositions.L3, redside, true));

    // joystick.povUp().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L1));
    // joystick.povRight().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L2));
    // joystick.povDown().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L3));
    // joystick.povLeft().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L4));

    // joystick
    //     .a()
    //     .whileTrue(
    //         new SetElevatorLevel(
    //             elevatorSubsystem, ElevatorPositions.safePosition)); // change safepos in
    // constants

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
  }

  public static void setAlliance() {
    redAlliance =
        (DriverStation.getAlliance().isEmpty())
            ? false
            : (DriverStation.getAlliance().get() == Alliance.Red);
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.selectedCommandScheduler();
  }
}
