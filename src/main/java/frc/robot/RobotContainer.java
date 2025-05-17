// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.AutoRoutines.AutoProducer;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commandGroups.Dealgaenate;
import frc.robot.commandGroups.EjectCoralFR;
import frc.robot.commandGroups.ElevatorL4;
import frc.robot.commandGroups.JamesHardenScore;
import frc.robot.commandGroups.PutUpAndShoot;
import frc.robot.commandGroups.RunFunnelUntilDetectionSafeSmooth;
import frc.robot.commands.DaleCommands.ArmToAngleCmd;
import frc.robot.commands.ElevatorCommands.DefaultElevator;
import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
import frc.robot.commands.ElevatorCommands.ZeroElevatorHardStop;
import frc.robot.commands.FunnelCommands.RunFunnelAndTootsieInCommand;
import frc.robot.commands.FunnelCommands.RunFunnelOutCommand;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
import frc.robot.commands.TootsieSlideCommands.ShootTootsieSlide;
import frc.robot.commands.TransferPieceBetweenFunnelAndElevator;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;
import frc.robot.util.CustomController;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class RobotContainer {
  private static Matrix<N3, N1> visionMatrix = VecBuilder.fill(0.01, 0.03d, 100d);
  private static Matrix<N3, N1> odometryMatrix = VecBuilder.fill(0.1, 0.1, 0.1);

  TootsieSlideSubsystem tootsieSlideSubsystem = TootsieSlideSubsystem.getInstance();
  FunnelSubsystem funnelSubsystem = FunnelSubsystem.getInstance();
  ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
  ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
  LedSubsystem leds = new LedSubsystem();
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

  private static SendableChooser<Integer> autoChooser = new SendableChooser<>();

  // Starts telemetry operations (essentially logging -> look on SmartDashboard, AdvantageScope)
  public void doTelemetry() {
    logger.telemeterize(driveTrain.getCurrentState());

    String commandName = "nah";

    if (driveTrain.getCurrentCommand() != null) {
      commandName = driveTrain.getCurrentCommand().getName();
    }
    DogLog.log("Robot/SwerveDriveCommand", commandName);
  }

  public RobotContainer() {
    autoChooser.setDefaultOption("Nothing", 0);
    autoChooser.addOption("Processor 3", 1);
    autoChooser.addOption("Processor 2", 2);
    autoChooser.addOption("Processor 1", 3);
    autoChooser.addOption("Clear 3", 4);
    autoChooser.addOption("Clear 2", 5);
    autoChooser.addOption("Clear 1", 6);
    autoChooser.addOption("Mid 1", 7);
    SmartDashboard.putData("Auto Side Choices", autoChooser);
    configureBindings();
  }

  public void teleopInit() {
    // // CoralPosition.setCoralInTootsieSlide(funnelSubsystem.drakeTripped());
    // CoralPosition.setCoralInFunnel(
    //     funnelSubsystem.isCoralCheckedIn() || funnelSubsystem.isCoralCheckedOut());
  }

  private void configureBindings() {
    leds.setDefaultCommand(new InstantCommand(() -> leds.updateLedsCommand(LedSubsystem.LedState.IDLE)));
    armSubsystem.setDefaultCommand(new ArmToAngleCmd(0.0, armSubsystem));
    elevatorSubsystem.setDefaultCommand(new DefaultElevator(elevatorSubsystem));

    // Custom Controller:

    // Left Elevator Levels
    // customController
    //     .LeftL1()
    //     .whileTrue(
    //         new JamesHardenScore(
    //             elevatorSubsystem,
    //             tootsieSlideSubsystem,
    //             driveTrain,
    //             ElevatorPositions.L1,
    //             redside,
    //             false));
    // customController.LeftL1().whileTrue(new ApplySwerveVoltage(driveTrain, 1.0));
    // customController.RightL1().whileTrue(new ApplySwerveVoltage(driveTrain, 2));

    // customController.LeftL1().whileTrue(new SwerveJoystickCommand(null, null,  null, driveTrain,
    // redAlliance, ))
    // customController.LeftL1().whileTrue(new JamesHardenMovement(driveTrain, null, redAlliance));
    // customController
    //     .RightL1()
    //     .whileTrue(JamesHardenMovement.toProcessorHPS(driveTrain, redside, false));

    customController
        .LeftL2()
        .whileTrue(
            new JamesHardenScore(
                elevatorSubsystem,
                tootsieSlideSubsystem,
                driveTrain,
                ElevatorPositions.L2,
                redside,
                false,
                leds));
    customController
        .LeftL3()
        .whileTrue(
            new JamesHardenScore(
                elevatorSubsystem,
                tootsieSlideSubsystem,
                driveTrain,
                ElevatorPositions.L3,
                redside,
                false,
                leds));
    customController
        .LeftL4()
        .whileTrue(
            new JamesHardenScore(
                elevatorSubsystem,
                tootsieSlideSubsystem,
                driveTrain,
                ElevatorPositions.L4,
                redside,
                false,
                leds));

    // // Right Elevator Levels
    // customController
    //     .RightL1()
    //     .whileTrue(
    //         new JamesHardenScore(
    //             elevatorSubsystem,
    //             tootsieSlideSubsystem,
    //             driveTrain,
    //             ElevatorPositions.L1,
    //             redside,
    //             true));
    customController
        .RightL2()
        .whileTrue(
            new JamesHardenScore(
                elevatorSubsystem,
                tootsieSlideSubsystem,
                driveTrain,
                ElevatorPositions.L2,
                redside,
                true,
                leds));
    customController
        .RightL3()
        .whileTrue(
            new JamesHardenScore(
                elevatorSubsystem,
                tootsieSlideSubsystem,
                driveTrain,
                ElevatorPositions.L3,
                redside,
                true,
                leds));
    customController
        .RightL4()
        .whileTrue(
            new JamesHardenScore(
                elevatorSubsystem,
                tootsieSlideSubsystem,
                driveTrain,
                ElevatorPositions.L4,
                redside,
                true,
                leds));

    // Bottom Three Buttons
    customController.Eject().onTrue(new EjectCoralFR(elevatorSubsystem, tootsieSlideSubsystem));
    customController
        .In()
        .whileTrue(
            new RunFunnelAndTootsieInCommand(funnelSubsystem, tootsieSlideSubsystem)
            // new UnjamFunnelAndIntake(
            //     elevatorSubsystem,
            //     funnelSubsystem,
            //     tootsieSlideSubsystem)
            ); // RunFunnelAndTootsieInCommand(funnelSubsystem,
    // tootsieSlideSubsystem));
    customController
        .Out()
        .whileTrue(
            new RunFunnelOutCommand(funnelSubsystem, () -> joystick.rightTrigger().getAsBoolean()));

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

    // Joystick 2:
    // Elevator
    joystick2.x().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L1, false));
    joystick2.a().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L2, false));
    joystick2.y().onTrue(new ElevatorL4(elevatorSubsystem, false));
    joystick2.b().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L3, false));
    joystick2
        .rightBumper()
        .onTrue(
            new SetElevatorLevel(
                elevatorSubsystem, ElevatorPositions.safePosition, false)); // reset mode

    // Shoot Tootsie Slide
    joystick2.rightTrigger().whileTrue(new ShootTootsieSlide(tootsieSlideSubsystem));

    // Intake
    joystick2.leftTrigger();
    //     .onTrue(new D2Intake(elevatorSubsystem, tootsieSlideSubsystem, funnelSubsystem));

    // Auto Intake and Eject
    Trigger funnelCheckin =
        new Trigger(
                () -> funnelSubsystem.isCoralCheckedIn() && !CoralPosition.isCoralInTootsieSlide())
            .and(RobotModeTriggers.teleop());
    Trigger ejectTime =
        new Trigger(
                () -> (funnelSubsystem.isCoralCheckedIn() && CoralPosition.isCoralInTootsieSlide()))
            .and(RobotModeTriggers.teleop());
    ejectTime.onTrue(new EjectCoralFR(elevatorSubsystem, tootsieSlideSubsystem));
    funnelCheckin.onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake, false));
    funnelCheckin.onTrue(new RunFunnelUntilDetectionSafeSmooth(elevatorSubsystem, funnelSubsystem, leds));
    Trigger funnelCheckout =
        new Trigger(
                () ->
                    CoralPosition.isCoralInFunnel()
                        && elevatorSubsystem.atIntake()
                        && elevatorSubsystem.isAtPosition())
            .and(RobotModeTriggers.teleop());

    funnelCheckout
        .and(joystick.rightTrigger().negate())
        .onTrue(
            new TransferPieceBetweenFunnelAndElevator(
                elevatorSubsystem, funnelSubsystem, tootsieSlideSubsystem));
    Trigger coralInElevator =
        new Trigger(() -> CoralPosition.isCoralInTootsieSlide()).and(RobotModeTriggers.teleop());
    coralInElevator.onTrue(
        new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.safePosition, false));

    // // Debugging
    // debugJoystick.leftTrigger().whileTrue(new ShootTootsieSlide(tootsieSlideSubsystem));

    // // debugJoystick
    // //     .y()
    // //     .whileTrue(
    // //         new Dealgaenate(
    // //             armSubsystem,
    // //             elevatorSubsystem,
    // //             Constants.ElevatorConstants.ElevatorPositions.L2DALE));

    // debugJoystick.y().whileTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L4));
    // debugJoystick
    //     .x()
    //     .onTrue(new SetElevatorLevel(ElevatorSubsystem.getInstance(), ElevatorPositions.Intake));
    // debugJoystick.a().onTrue(new ZeroArm(armSubsystem));
    // debugJoystick
    //     .b()
    //     .whileTrue(
    //         new Dealgaenate(
    //             armSubsystem,
    //             elevatorSubsystem,
    //             Constants.ElevatorConstants.ElevatorPositions.L3DALE));

    // debugJoystick
    //     .rightTrigger()
    //     .onTrue(new Intake(elevatorSubsystem, funnelSubsystem, tootsieSlideSubsystem));

    // Swerve
    // spark commit
    Trigger leftTrigger = joystick.leftTrigger();
    DoubleSupplier frontBackFunction = () -> -joystick.getLeftY(),
        leftRightFunction = () -> -joystick.getLeftX(),
        rotationFunction = () -> -joystick.getRightX(),
        speedFunction =
            () ->
                leftTrigger.getAsBoolean()
                    ? 0d
                    : 1d; // slowmode when left shoulder is pressed, otherwise fast
    SwerveJoystickCommand swerveJoystickCommand =
        new SwerveJoystickCommand(
            frontBackFunction,
            leftRightFunction,
            rotationFunction,
            speedFunction, // slowmode when left shoulder is pressed, otherwise fast
            () -> joystick.leftTrigger().getAsBoolean(),
            // () -> joystick.rightTrigger().getAsBoolean(),
            redside,
            () -> joystick.a().getAsBoolean(),
            () -> customController.LeftL1().getAsBoolean(),
            () -> customController.RightL1().getAsBoolean(),
            driveTrain);
    driveTrain.setDefaultCommand(swerveJoystickCommand);

    // IMPORTANT
    // joystick.a().whileTrue(new ShootL1(elevatorSubsystem, tootsieSlideSubsystem));

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

    // joystick
    //     .a()
    //     .whileTrue(new ShootL1Funnel(elevatorSubsystem, tootsieSlideSubsystem, funnelSubsystem));
    // joystick.povRight().whileTrue(new ShootL1Funnel(elevatorSubsystem, tootsieSlideSubsystem,
    // funnelSubsystem));

    // joystick
    //     .a()
    //     .onTrue(
    //         driveTrain.runOnce(
    //             () -> driveTrain.resetPose(new Pose2d(new Translation2d(0, 0), new
    // Rotation2d()))));

    joystick.x().onTrue(new ZeroElevatorHardStop(elevatorSubsystem));

    // new InstantCommand()

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
    // debugJoystick.leftTrigger().whileTrue(new ShootTootsieSlide(tootsieSlideSubsystem));

    // debugJoystick
    //     .y()
    //     .whileTrue(
    //         new Dealgaenate(
    //             armSubsystem,
    //             elevatorSubsystem,
    //             Constants.ElevatorConstants.ElevatorPositions.L2DALE));
    // debugJoystick
    //     .x()
    //     .onTrue(new SetElevatorLevel(ElevatorSubsystem.getInstance(), ElevatorPositions.Intake));
    // debugJoystick.a().onTrue(new ZeroArm(armSubsystem));
    // debugJoystick
    //     .b()
    //     .whileTrue(
    //         new Dealgaenate(
    //             armSubsystem,
    //             elevatorSubsystem,
    //             Constants.ElevatorConstants.ElevatorPositions.L3DALE));

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

  public BooleanSupplier getRedSide(){
    DogLog.log("get alliance", redside.getAsBoolean());
    return redside;
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    int autoValue = autoChooser.getSelected();
    Command autoCommand;
    DogLog.log("auto/selected", autoValue);
    switch (autoValue) {
      case 1:
        autoCommand =
            redside.getAsBoolean()
                ? new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.RED_PROCESSOR_3,leds)
                : new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.BLUE_PROCESSOR_3,leds);
        break;
      case 2:
        autoCommand =
            redside.getAsBoolean()
                ? new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.RED_PROCESSOR_2,leds)
                : new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.BLUE_PROCESSOR_2,leds);
        break;
      case 3:
        autoCommand =
            redside.getAsBoolean()
                ? new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.RED_PROCESSOR_1,leds)
                : new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.BLUE_PROCESSOR_1,leds);
        break;
      case 4:
        autoCommand =
            redside.getAsBoolean()
                ? new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.RED_CLEAR_3,leds)
                : new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.BLUE_CLEAR_3,leds);
        break;
      case 5:
        autoCommand =
            redside.getAsBoolean()
                ? new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.RED_CLEAR_2,leds)
                : new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.BLUE_CLEAR_2,leds);
        break;
      case 6:
        autoCommand =
            redside.getAsBoolean()
                ? new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.RED_CLEAR_1,leds)
                : new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.BLUE_CLEAR_1,leds);
        break;
      case 7:
        autoCommand =
            redside.getAsBoolean()
                ? new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.RED_MID_1,leds)
                : new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.BLUE_MID_1,leds);
        break;
      default:
        autoCommand = null;
        break;
    }
    return autoCommand;
  }
}
