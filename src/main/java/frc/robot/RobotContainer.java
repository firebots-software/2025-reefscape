// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commandGroups.AutoLiftAndShoot;
import frc.robot.commandGroups.Dealgaenate;
import frc.robot.commandGroups.EjectCoralFR;
import frc.robot.commandGroups.LoadAndPutUp;
import frc.robot.commands.DaleCommands.ArmToAngleCmd;
import frc.robot.commands.DaleCommands.ZeroArm;
import frc.robot.commands.ElevatorCommands.DefaultElevator;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.FunnelCommands.RunFunnelUntilCheckedIn;
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

  private static SendableChooser<String> startPosChooser = new SendableChooser<>();

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
  private final AutoRoutines autoRoutines;

  public RobotContainer() {
    autoFactory =
        new AutoFactory(
            driveTrain::getPose, // A function that returns the current robot pose
            driveTrain
                ::resetPose, // A function that resets the current robot pose to the provided Pose2d
            driveTrain::followTrajectory, // The drive subsystem trajectory follower
            true, // If alliance flipping should be enabled
            driveTrain);

    autoRoutines =
        new AutoRoutines(
            autoFactory,
            driveTrain,
            elevatorSubsystem,
            tootsieSlideSubsystem,
            funnelSubsystem,
            redside);

    // Set up the Auto chooser in SmartDashboard, which allows you to choose between the Top,
    // Middle, and Bottom auto paths
    // (Mirroring for Blue or Red side happens automatically with Choreo)
    startPosChooser = new SendableChooser<String>();
    startPosChooser.setDefaultOption("Top (next to blue barge zone)", "top");
    startPosChooser.addOption("Middle (between blue and red barge zones)", "middle");
    startPosChooser.addOption("Bottom (next to red barge zone)", "bottom");
    SmartDashboard.putData(startPosChooser);

    configureBindings();
  }

  public void configureBindings() {
    // Automatic Command Bindings
    // funnelSubsystem.setDefaultCommand(new DefaultFunnelCommand(funnelSubsystem));

    // This triggers each time a new Coral enters the funnel.
    // (When the Check-In sensor detects a Coral AND there is NO Coral in the Tootsie Slide.)
    Trigger funnelCheckin =
        new Trigger(
            () -> funnelSubsystem.isCoralCheckedIn() && !CoralPosition.isCoralInTootsieSlide());
    
    // This triggers when we need to eject a Coral from the funnel.
    // (When the Check-In sensor detects a Coral AND there IS a Coral in the Tootsie Slide.)
    Trigger ejectTime =
        new Trigger(
            () -> (funnelSubsystem.isCoralCheckedIn() && CoralPosition.isCoralInTootsieSlide()));

    //  !!!  When we need to eject a Coral from the funnel, we run EjectCoralFR().
    ejectTime.onTrue(new EjectCoralFR(elevatorSubsystem, tootsieSlideSubsystem));

    //  !!!  Each time a new Coral enters the funnel, we move elevator to Intake and intake the Coral.
    // (RunFunnelUntilDetectionSafe() runs the intake motors until the Check-Out sensor gets triggered.)
    funnelCheckin.onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake));
    funnelCheckin.onTrue(new RunFunnelUntilDetectionSafe(funnelSubsystem, elevatorSubsystem));

    // This triggers when there is a Coral in the funnel ready to be transferred to Tootsie Slide.
    // (When a Coral is in the funnel AND the elevator is definitely at the Intake position.)
    Trigger funnelCheckout =
        new Trigger(
            () ->
                CoralPosition.isCoralInFunnel()
                    && elevatorSubsystem.atIntake()
                    && elevatorSubsystem.isAtPosition());

    //  !!!  Each time a Coral is ready to be transferred to the Tootsie Slide, we transfer it to the Tootsie Slide.
    funnelCheckout.onTrue(
        new TransferPieceBetweenFunnelAndElevator(
            elevatorSubsystem, funnelSubsystem, tootsieSlideSubsystem));

    // Triggers when there is a Coral in the Tootsie Slide.
    Trigger coralInElevator = new Trigger(() -> CoralPosition.isCoralInTootsieSlide());

    //  !!!  Each time a new Coral goes into the Tootsie Slide, we move the elevator to the Safe Position (L1).
    coralInElevator.onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.safePosition));

    //  !!!  By default (when no other commands are using the Elevator Subsystem), we move the elevator to Intake unless it
    // is carrying a Coral in the Tootsie Slide.
    elevatorSubsystem.setDefaultCommand(new DefaultElevator(elevatorSubsystem));

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

    debugJoystick
        .rightTrigger()
        .onTrue(
            new LoadAndPutUp(
                elevatorSubsystem, funnelSubsystem, tootsieSlideSubsystem,
    ElevatorPositions.L3));

    // Auto debugging
    // debugJoystick
    //     .a()
    //     .onTrue(
    //         new LoadAndPutUp(
    //             elevatorSubsystem,
    //             funnelSubsystem,
    //             tootsieSlideSubsystem,
    //             ElevatorPositions.Intake));

    // debugJoystick.povUp().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake));
    // debugJoystick.povDown().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L4));
    // debugJoystick.b().onTrue(new RunFunnelUntilCheckedIn(funnelSubsystem));
    // debugJoystick.x().onTrue(new AutoLiftAndShoot(elevatorSubsystem, tootsieSlideSubsystem));

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

    // Mechanisms:

    joystick
        .rightBumper()
        .onTrue(new Dealgaenate(armSubsystem, elevatorSubsystem, ElevatorPositions.L2DALE));
    joystick.rightBumper().onFalse(new ArmToAngleCmd(Constants.Arm.RETRACTED_ANGLE, armSubsystem));
    joystick.y().whileTrue(JamesHardenMovement.toClosestRightBranch(driveTrain, redside));

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
    // SmartDashboard Auto Chooser: Returns "bottom", "top", or "middle"
    String chosenPath = startPosChooser.getSelected();

    return autoRoutines.autoRoutine(chosenPath).cmd();
  }

  //   public void testAutoCommands() {}
}
