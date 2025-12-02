// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import choreo.auto.AutoFactory;
// import choreo.auto.AutoRoutine;
// import choreo.auto.AutoTrajectory;
// import dev.doglog.DogLog;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
// import frc.robot.commandGroups.Dealgaenate;
// import frc.robot.commandGroups.EjectCoralFR;
// import frc.robot.commandGroups.ElevatorL4;
// import frc.robot.commandGroups.JamesHardenScore;
// import frc.robot.commandGroups.PutUpAndShoot;
// import frc.robot.commandGroups.RunFunnelUntilDetectionSafeSmooth;
// import frc.robot.commands.DaleCommands.ArmToAngleCmd;
// import frc.robot.commands.ElevatorCommands.DefaultElevator;
// import frc.robot.commands.ElevatorCommands.SetElevatorLevel;
// import frc.robot.commands.ElevatorCommands.ZeroElevatorHardStop;
// import frc.robot.commands.FunnelCommands.RunFunnelAndTootsieInCommand;
// import frc.robot.commands.FunnelCommands.RunFunnelOutCommand;
// import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
// import frc.robot.commands.TootsieSlideCommands.ShootTootsieSlide;
// import frc.robot.commands.TransferPieceBetweenFunnelAndElevator;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.CoralPosition;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.FunnelSubsystem;
// import frc.robot.subsystems.LedSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.TootsieSlideSubsystem;
// import frc.robot.util.CustomController;
// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// public class RobotContainer {
//   private static Matrix<N3, N1> visionMatrix = VecBuilder.fill(0.01, 0.03d, 100d);
//   private static Matrix<N3, N1> odometryMatrix = VecBuilder.fill(0.1, 0.1, 0.1);

//   TootsieSlideSubsystem tootsieSlideSubsystem = TootsieSlideSubsystem.getInstance();
//   FunnelSubsystem funnelSubsystem = FunnelSubsystem.getInstance();
//   ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
//   ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
//   LedSubsystem leds = new LedSubsystem();
//   // Alliance color
//   Boolean coralInFunnel = Boolean.valueOf(false);
//   Boolean coralInElevator = Boolean.valueOf(false);

//   private BooleanSupplier redside = () -> redAlliance;
//   private static boolean redAlliance;

//   private final SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();

//   private final Telemetry logger =
//       new Telemetry(Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
//   private final CommandXboxController joystick = new CommandXboxController(0);
//   private final CommandXboxController joystick2 = new CommandXboxController(1);
//   private final CommandXboxController debugJoystick = new CommandXboxController(3);
//   private final CustomController customController = new CustomController(4);

//   private static SendableChooser<Integer> autoChooser = new SendableChooser<>();

//   private final AutoFactory autoFactory;

//   // Starts telemetry operations (essentially logging -> look on SmartDashboard, AdvantageScope)
//   public void doTelemetry() {
//     logger.telemeterize(driveTrain.getCurrentState());

//     String commandName = "nah";

//     if (driveTrain.getCurrentCommand() != null) {
//       commandName = driveTrain.getCurrentCommand().getName();
//     }
//   }

//   public RobotContainer() {
//     autoFactory =
//         new AutoFactory(
//             driveTrain::getPose, // A function that returns the current robot pose
//             driveTrain
//                 ::resetPose, // A function that resets the current robot pose to the provided Pose2d
//             driveTrain::followTrajectory, // The drive subsystem trajectory follower
//             // controller
//             false, // If alliance flipping should be enabled
//             driveTrain);

//     autoChooser.setDefaultOption("cr7", 0);
//     SmartDashboard.putData("Auto Side Choices", autoChooser);
//     configureBindings();
//   }

//   public void teleopInit() {
//     // // CoralPosition.setCoralInTootsieSlide(funnelSubsystem.drakeTripped());
//     // CoralPosition.setCoralInFunnel(
//     //     funnelSubsystem.isCoralCheckedIn() || funnelSubsystem.isCoralCheckedOut());
//   }

//   private void configureBindings() {
//     // leds.setDefaultCommand(
//     //     new InstantCommand(() -> leds.updateLedsCommand(LedSubsystem.LedState.IDLE)));
//     armSubsystem.setDefaultCommand(new ArmToAngleCmd(0.0, armSubsystem));
//     elevatorSubsystem.setDefaultCommand(new DefaultElevator(elevatorSubsystem));

//     // Custom Controller:

//     // Left Elevator Levels
//     // customController
//     //     .LeftL1()
//     //     .whileTrue(
//     //         new JamesHardenScore(
//     //             elevatorSubsystem,
//     //             tootsieSlideSubsystem,
//     //             driveTrain,
//     //             ElevatorPositions.L1,
//     //             redside,
//     //             false));
//     // customController.LeftL1().whileTrue(new ApplySwerveVoltage(driveTrain, 1.0));
//     // customController.RightL1().whileTrue(new ApplySwerveVoltage(driveTrain, 2));

//     // customController.LeftL1().whileTrue(new SwerveJoystickCommand(null, null,  null, driveTrain,
//     // redAlliance, ))
//     // customController.LeftL1().whileTrue(new JamesHardenMovement(driveTrain, null, redAlliance));
//     // customController
//     //     .RightL1()
//     //     .whileTrue(JamesHardenMovement.toProcessorHPS(driveTrain, redside, false));

//     customController
//         .LeftL2()
//         .whileTrue(
//             new JamesHardenScore(
//                 elevatorSubsystem,
//                 tootsieSlideSubsystem,
//                 driveTrain,
//                 ElevatorPositions.L2,
//                 redside,
//                 false,
//                 leds));
//     customController
//         .LeftL3()
//         .whileTrue(
//             new JamesHardenScore(
//                 elevatorSubsystem,
//                 tootsieSlideSubsystem,
//                 driveTrain,
//                 ElevatorPositions.L3,
//                 redside,
//                 false,
//                 leds));
//     customController
//         .LeftL4()
//         .whileTrue(
//             new JamesHardenScore(
//                 elevatorSubsystem,
//                 tootsieSlideSubsystem,
//                 driveTrain,
//                 ElevatorPositions.L4,
//                 redside,
//                 false,
//                 leds));

//     // // Right Elevator Levels
//     // customController
//     //     .RightL1()
//     //     .whileTrue(
//     //         new JamesHardenScore(
//     //             elevatorSubsystem,
//     //             tootsieSlideSubsystem,
//     //             driveTrain,
//     //             ElevatorPositions.L1,
//     //             redside,
//     //             true));
//     customController
//         .RightL2()
//         .whileTrue(
//             new JamesHardenScore(
//                 elevatorSubsystem,
//                 tootsieSlideSubsystem,
//                 driveTrain,
//                 ElevatorPositions.L2,
//                 redside,
//                 true,
//                 leds));
//     customController
//         .RightL3()
//         .whileTrue(
//             new JamesHardenScore(
//                 elevatorSubsystem,
//                 tootsieSlideSubsystem,
//                 driveTrain,
//                 ElevatorPositions.L3,
//                 redside,
//                 true,
//                 leds));
//     customController
//         .RightL4()
//         .whileTrue(
//             new JamesHardenScore(
//                 elevatorSubsystem,
//                 tootsieSlideSubsystem,
//                 driveTrain,
//                 ElevatorPositions.L4,
//                 redside,
//                 true,
//                 leds));

//     // Bottom Three Buttons
//     customController.Eject().onTrue(new EjectCoralFR(elevatorSubsystem, tootsieSlideSubsystem));
//     customController
//         .In()
//         .whileTrue(
//             new RunFunnelAndTootsieInCommand(funnelSubsystem, tootsieSlideSubsystem)
//             // new UnjamFunnelAndIntake(
//             //     elevatorSubsystem,
//             //     funnelSubsystem,
//             //     tootsieSlideSubsystem)
//             ); // RunFunnelAndTootsieInCommand(funnelSubsystem,
//     // tootsieSlideSubsystem));
//     customController
//         .Out()
//         .whileTrue(
//             new RunFunnelOutCommand(funnelSubsystem, () -> joystick.rightTrigger().getAsBoolean()));

//     // Joystick 1:

//     // Dale
//     joystick
//         .rightBumper()
//         .whileTrue(new Dealgaenate(armSubsystem, elevatorSubsystem, ElevatorPositions.L3DALE));
//     joystick
//         .leftBumper()
//         .whileTrue(new Dealgaenate(armSubsystem, elevatorSubsystem, ElevatorPositions.L2DALE));

//     // Zero Rotations
//     joystick
//         .y()
//         .onTrue(
//             driveTrain.runOnce(
//                 () ->
//                     driveTrain.resetPose(
//                         new Pose2d(driveTrain.getPose().getTranslation(), new Rotation2d(0)))));

//     // Joystick 2:
//     // Elevator
//     joystick2.x().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L1, false));
//     joystick2.a().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L2, false));
//     joystick2.y().onTrue(new ElevatorL4(elevatorSubsystem, false));
//     joystick2.b().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L3, false));
//     joystick2
//         .rightBumper()
//         .onTrue(
//             new SetElevatorLevel(
//                 elevatorSubsystem, ElevatorPositions.safePosition, false)); // reset mode

//     // Shoot Tootsie Slide
//     joystick2.rightTrigger().whileTrue(new ShootTootsieSlide(tootsieSlideSubsystem));

//     // Intake
//     joystick2.leftTrigger();
//     //     .onTrue(new D2Intake(elevatorSubsystem, tootsieSlideSubsystem, funnelSubsystem));

//     // Auto Intake and Eject
//     Trigger funnelCheckin =
//         new Trigger(
//                 () -> funnelSubsystem.isCoralCheckedIn() && !CoralPosition.isCoralInTootsieSlide())
//             .and(RobotModeTriggers.teleop());
//     Trigger ejectTime =
//         new Trigger(
//                 () -> (funnelSubsystem.isCoralCheckedIn() && CoralPosition.isCoralInTootsieSlide()))
//             .and(RobotModeTriggers.teleop());
//     ejectTime.onTrue(new EjectCoralFR(elevatorSubsystem, tootsieSlideSubsystem));
//     funnelCheckin.onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake, false));
//     funnelCheckin.onTrue(
//         new RunFunnelUntilDetectionSafeSmooth(elevatorSubsystem, funnelSubsystem, leds));
//     Trigger funnelCheckout =
//         new Trigger(
//                 () ->
//                     CoralPosition.isCoralInFunnel()
//                         && elevatorSubsystem.atIntake()
//                         && elevatorSubsystem.isAtPosition())
//             .and(RobotModeTriggers.teleop());

//     funnelCheckout
//         .and(joystick.rightTrigger().negate())
//         .onTrue(
//             new TransferPieceBetweenFunnelAndElevator(
//                 elevatorSubsystem, funnelSubsystem, tootsieSlideSubsystem));
//     Trigger coralInElevator =
//         new Trigger(() -> CoralPosition.isCoralInTootsieSlide()).and(RobotModeTriggers.teleop());
//     coralInElevator.onTrue(
//         new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.safePosition, false));

//     // // Debugging
//     // debugJoystick.leftTrigger().whileTrue(new ShootTootsieSlide(tootsieSlideSubsystem));

//     // // debugJoystick
//     // //     .y()
//     // //     .whileTrue(
//     // //         new Dealgaenate(
//     // //             armSubsystem,
//     // //             elevatorSubsystem,
//     // //             Constants.ElevatorConstants.ElevatorPositions.L2DALE));

//     // debugJoystick.y().whileTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L4));
//     // debugJoystick
//     //     .x()
//     //     .onTrue(new SetElevatorLevel(ElevatorSubsystem.getInstance(), ElevatorPositions.Intake));
//     // debugJoystick.a().onTrue(new ZeroArm(armSubsystem));
//     // debugJoystick
//     //     .b()
//     //     .whileTrue(
//     //         new Dealgaenate(
//     //             armSubsystem,
//     //             elevatorSubsystem,
//     //             Constants.ElevatorConstants.ElevatorPositions.L3DALE));

//     // debugJoystick
//     //     .rightTrigger()
//     //     .onTrue(new Intake(elevatorSubsystem, funnelSubsystem, tootsieSlideSubsystem));

//     // Swerve
//     // spark commit
//     Trigger leftTrigger = joystick.leftTrigger();
//     DoubleSupplier frontBackFunction = () -> -joystick.getLeftY(),
//         leftRightFunction = () -> -joystick.getLeftX(),
//         rotationFunction = () -> -joystick.getRightX(),
//         speedFunction =
//             () ->
//                 leftTrigger.getAsBoolean()
//                     ? 0d
//                     : 1d; // slowmode when left shoulder is pressed, otherwise fast
//     SwerveJoystickCommand swerveJoystickCommand =
//         new SwerveJoystickCommand(
//             frontBackFunction,
//             leftRightFunction,
//             rotationFunction,
//             speedFunction, // slowmode when left shoulder is pressed, otherwise fast
//             () -> joystick.leftTrigger().getAsBoolean(),
//             // () -> joystick.rightTrigger().getAsBoolean(),
//             redside,
//             () -> joystick.a().getAsBoolean(),
//             () -> customController.LeftL1().getAsBoolean(),
//             () -> customController.RightL1().getAsBoolean(),
//             driveTrain);
//     driveTrain.setDefaultCommand(swerveJoystickCommand);

//     // IMPORTANT
//     // joystick.a().whileTrue(new ShootL1(elevatorSubsystem, tootsieSlideSubsystem));

//     joystick
//         .b()
//         .whileTrue(
//             new PutUpAndShoot(elevatorSubsystem, tootsieSlideSubsystem, ElevatorPositions.L3));
//     joystick
//         .povUp()
//         .whileTrue(
//             new PutUpAndShoot(elevatorSubsystem, tootsieSlideSubsystem, ElevatorPositions.L2));
//     joystick
//         .povDown()
//         .whileTrue(
//             new PutUpAndShoot(elevatorSubsystem, tootsieSlideSubsystem, ElevatorPositions.L4));

//     // joystick
//     //     .a()
//     //     .whileTrue(new ShootL1Funnel(elevatorSubsystem, tootsieSlideSubsystem, funnelSubsystem));
//     // joystick.povRight().whileTrue(new ShootL1Funnel(elevatorSubsystem, tootsieSlideSubsystem,
//     // funnelSubsystem));

//     // joystick
//     //     .a()
//     //     .onTrue(
//     //         driveTrain.runOnce(
//     //             () -> driveTrain.resetPose(new Pose2d(new Translation2d(0, 0), new
//     // Rotation2d()))));

//     joystick.x().onTrue(new ZeroElevatorHardStop(elevatorSubsystem));

//     // new InstantCommand()

//     // joystick.povUp().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L1));
//     // joystick.povRight().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L2));
//     // joystick.povDown().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L3));
//     // joystick.povLeft().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L4));

//     // joystick
//     //     .a()
//     //     .whileTrue(
//     //         new SetElevatorLevel(
//     //             elevatorSubsystem, ElevatorPositions.safePosition)); // change safepos in
//     // constants

//     /*
//     Sysid button commands, commented out (I like keeping this commented because
//     every branch will have access to the necessary commands to run SysID immediately)

//        joystick.povUp().onTrue(Commands.runOnce(SignalLogger::start));
//        joystick.povDown().onTrue(Commands.runOnce(SignalLogger::stop));

//     * Joystick Y = quasistatic forward
//     * Joystick A = quasistatic reverse
//     * Joystick B = dynamic forward
//     * Joystick X = dyanmic reverse
//     *
//        joystick.y().whileTrue(driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
//        joystick.a().whileTrue(driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
//        joystick.b().whileTrue(driveTrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
//        joystick.x().whileTrue(driveTrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
//     */
//     // new Translation2d(
//     //     Constants.Landmarks.leftBranchesRed[5].getX()
//     //         - (((Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length
//     //                         .in(Meters)
//     //                     / 2.0)
//     //                 + Constants.Swerve.WHICH_SWERVE_ROBOT.BUMPER_THICKNESS
//     //                     .thickness.in(Meters)))
//     //             * Constants.Landmarks.reefFacingAngleRed[5].getCos(),
//     //     Constants.Landmarks.leftBranchesRed[5].getY()
//     //         - (((Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length
//     //                         .in(Meters)
//     //                     / 2.0)
//     //                 + Constants.Swerve.WHICH_SWERVE_ROBOT.BUMPER_THICKNESS
//     //                     .thickness.in(Meters)))
//     //             * Constants.Landmarks.reefFacingAngleRed[5].getSin()),
//     // new Rotation2d(
//     //     Constants.Landmarks.reefFacingAngleRed[5].getRadians());

//     // Debugging
//     // debugJoystick.leftTrigger().whileTrue(new ShootTootsieSlide(tootsieSlideSubsystem));

//     // debugJoystick
//     //     .y()
//     //     .whileTrue(
//     //         new Dealgaenate(
//     //             armSubsystem,
//     //             elevatorSubsystem,
//     //             Constants.ElevatorConstants.ElevatorPositions.L2DALE));
//     // debugJoystick
//     //     .x()
//     //     .onTrue(new SetElevatorLevel(ElevatorSubsystem.getInstance(), ElevatorPositions.Intake));
//     // debugJoystick.a().onTrue(new ZeroArm(armSubsystem));
//     // debugJoystick
//     //     .b()
//     //     .whileTrue(
//     //         new Dealgaenate(
//     //             armSubsystem,
//     //             elevatorSubsystem,
//     //             Constants.ElevatorConstants.ElevatorPositions.L3DALE));

//     // debugJoystick
//     //     .rightTrigger()
//     //     .onTrue(
//     //         new LoadAndPutUp(
//     //             elevatorSubsystem, funnelSubsystem, tootsieSlideSubsystem,
//     // ElevatorPositions.L3));
//   }

//   public static void setAlliance() {
//     redAlliance =
//         (DriverStation.getAlliance().isEmpty())
//             ? false
//             : (DriverStation.getAlliance().get() == Alliance.Red);
//   }

//   public BooleanSupplier getRedSide() {
//     DogLog.log("Info/Alliance", redside.getAsBoolean());
//     return redside;
//   }

//   public Command getAutonomousCommand() {
//     /* Run the path selected from the auto chooser */
//     // int autoValue = autoChooser.getSelected();
//     // Creates a new routine with the name "exampleRoutine"

//     AutoRoutine routine = autoFactory.newRoutine("GoodPath.chor");
//     // Load the routine's trajectories
//     AutoTrajectory moveForward = routine.trajectory("Path8.traj");

//     // When the routine begins, reset odometry and start the first trajectory (1)
//     routine
//         .active()
//         .onTrue(
//             Commands.sequence(
//                 new InstantCommand(
//                     () -> DogLog.log("Auto/resetOdometry", "completed reset odometry first")),
//                 moveForward.resetOdometry(),
//                 new InstantCommand(
//                     () -> DogLog.log("Auto/resetOdometry", "completed reset odometry")),
//                 moveForward.cmd(),
//                 new InstantCommand(
//                     () -> DogLog.log("Auto/run entire command", "completed reset odometry")),
//                 new JamesHardenScore(
//                     elevatorSubsystem,
//                     tootsieSlideSubsystem,
//                     driveTrain,
//                     Constants.ElevatorConstants.ElevatorPositions.L3,
//                     Constants.RedLandmarkPose.L4)));

//     return routine.cmd();
//   }
//   //   /* Run the path selected from the auto chooser */
//   //   int autoValue = autoChooser.getSelected();
//   //   Command autoCommand;
//   //   DogLog.log("Info/AutoSelected", autoValue);
//   //   switch (autoValue) {
//   //     case 1:
//   //       autoCommand =
//   //           redside.getAsBoolean()
//   //               ? new AutoProducer(
//   //                   driveTrain,
//   //                   tootsieSlideSubsystem,
//   //                   elevatorSubsystem,
//   //                   funnelSubsystem,
//   //                   armSubsystem,
//   //                   Constants.AutoRoutines.RED_PROCESSOR_3,
//   //                   leds)
//   //               : new AutoProducer(
//   //                   driveTrain,
//   //                   tootsieSlideSubsystem,
//   //                   elevatorSubsystem,
//   //                   funnelSubsystem,
//   //                   armSubsystem,
//   //                   Constants.AutoRoutines.BLUE_PROCESSOR_3,
//   //                   leds);
//   //       break;
//   //     case 2:
//   //       autoCommand =
//   //           redside.getAsBoolean()
//   //               ? new AutoProducer(
//   //                   driveTrain,
//   //                   tootsieSlideSubsystem,
//   //                   elevatorSubsystem,
//   //                   funnelSubsystem,
//   //                   armSubsystem,
//   //                   Constants.AutoRoutines.RED_PROCESSOR_2,
//   //                   leds)
//   //               : new AutoProducer(
//   //                   driveTrain,
//   //                   tootsieSlideSubsystem,
//   //                   elevatorSubsystem,
//   //                   funnelSubsystem,
//   //                   armSubsystem,
//   //                   Constants.AutoRoutines.BLUE_PROCESSOR_2,
//   //                   leds);
//   //       break;
//   //     case 3:
//   //       autoCommand =
//   //           redside.getAsBoolean()
//   //               ? new AutoProducer(
//   //                   driveTrain,
//   //                   tootsieSlideSubsystem,
//   //                   elevatorSubsystem,
//   //                   funnelSubsystem,
//   //                   armSubsystem,
//   //                   Constants.AutoRoutines.RED_PROCESSOR_1,
//   //                   leds)
//   //               : new AutoProducer(
//   //                   driveTrain,
//   //                   tootsieSlideSubsystem,
//   //                   elevatorSubsystem,
//   //                   funnelSubsystem,
//   //                   armSubsystem,
//   //                   Constants.AutoRoutines.BLUE_PROCESSOR_1,
//   //                   leds);
//   //       break;
//   //     case 4:
//   //       autoCommand =
//   //           redside.getAsBoolean()
//   //               ? new AutoProducer(
//   //                   driveTrain,
//   //                   tootsieSlideSubsystem,
//   //                   elevatorSubsystem,
//   //                   funnelSubsystem,
//   //                   armSubsystem,
//   //                   Constants.AutoRoutines.RED_CLEAR_3,
//   //                   leds)
//   //               : new AutoProducer(
//   //                   driveTrain,
//   //                   tootsieSlideSubsystem,
//   //                   elevatorSubsystem,
//   //                   funnelSubsystem,
//   //                   armSubsystem,
//   //                   Constants.AutoRoutines.BLUE_CLEAR_3,
//   //                   leds);
//   //       break;
//   //     case 5:
//   //       autoCommand =
//   //           redside.getAsBoolean()
//   //               ? new AutoProducer(
//   //                   driveTrain,
//   //                   tootsieSlideSubsystem,
//   //                   elevatorSubsystem,
//   //                   funnelSubsystem,
//   //                   armSubsystem,
//   //                   Constants.AutoRoutines.RED_CLEAR_2,
//   //                   leds)
//   //               : new AutoProducer(
//   //                   driveTrain,
//   //                   tootsieSlideSubsystem,
//   //                   elevatorSubsystem,
//   //                   funnelSubsystem,
//   //                   armSubsystem,
//   //                   Constants.AutoRoutines.BLUE_CLEAR_2,
//   //                   leds);
//   //       break;
//   //     case 6:
//   //       autoCommand =
//   //           redside.getAsBoolean()
//   //               ? new AutoProducer(
//   //                   driveTrain,
//   //                   tootsieSlideSubsystem,
//   //                   elevatorSubsystem,
//   //                   funnelSubsystem,
//   //                   armSubsystem,
//   //                   Constants.AutoRoutines.RED_CLEAR_1,
//   //                   leds)
//   //               : new AutoProducer(
//   //                   driveTrain,
//   //                   tootsieSlideSubsystem,
//   //                   elevatorSubsystem,
//   //                   funnelSubsystem,
//   //                   armSubsystem,
//   //                   Constants.AutoRoutines.BLUE_CLEAR_1,
//   //                   leds);
//   //       break;
//   //     case 7:
//   //       autoCommand =
//   //           redside.getAsBoolean()
//   //               ? new AutoProducer(
//   //                   driveTrain,
//   //                   tootsieSlideSubsystem,
//   //                   elevatorSubsystem,
//   //                   funnelSubsystem,
//   //                   armSubsystem,
//   //                   Constants.AutoRoutines.RED_MID_1,
//   //                   leds)
//   //               : new AutoProducer(
//   //                   driveTrain,
//   //                   tootsieSlideSubsystem,
//   //                   elevatorSubsystem,
//   //                   funnelSubsystem,
//   //                   armSubsystem,
//   //                   Constants.AutoRoutines.BLUE_MID_1,
//   //                   leds);
//   //       break;
//   //     default:
//   //       autoCommand = null;
//   //       break;
//   //   }
//   //   return autoCommand;
//   // }
// }































package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import java.util.function.Supplier;

// import choreo.trajectory.SwerveSample;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class SwerveSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements Subsystem {
  private static SwerveSubsystem instance;

  private ProfiledPIDController qProfiledPIDController, headingProfiledPIDController;

  private SwerveDriveState currentState;

  public SwerveSubsystem(
      SwerveDrivetrainConstants drivetrainConstants,
      double OdometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        TalonFX::new,
        TalonFX::new,
        CANcoder::new,
        drivetrainConstants,
        OdometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }

    currentState = getState(); // getCurrentState
    // 1.7, 0.345, 0.0015
    qProfiledPIDController =
        new ProfiledPIDController(
            Constants.HardenConstants.QKP, // 3.4 not bad // [3.4 good for 0.2-1.2, 0.425 I]
            Constants.HardenConstants.QKI, // 345
            Constants.HardenConstants.QKD, // 0.0015
            new TrapezoidProfile.Constraints(
                Constants.HardenConstants.QCRUISE,
                Constants.HardenConstants.QACCEL)); // 8.25 // 5 accel and 0.75 p was good

    headingProfiledPIDController =
        new ProfiledPIDController(
            Constants.HardenConstants.HKP, // 4 was good
            Constants.HardenConstants.HKI, //
            Constants.HardenConstants.HKD,
            new TrapezoidProfile.Constraints(
                Constants.HardenConstants.HCRUISE, // -1 was good
                Constants.HardenConstants.HACCEL)); // -13 was good
    // headingProfiledPIDController =
    //     new ProfiledPIDController(
    //         1, // 4 was good
    //         0.2, //
    //         0,
    //         new TrapezoidProfile.Constraints(
    //             Constants.Swerve.TELE_DRIVE_MAX_ANGULAR_RATE - 1.5, // -1 was good
    //             Constants.Swerve.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND
    //                 - 16)); // -13 was good

    qProfiledPIDController.setIZone(Constants.HardenConstants.QIZONE);
    headingProfiledPIDController.setIZone(Constants.HardenConstants.HIZONE);

    qProfiledPIDController.setIntegratorRange(
        Constants.HardenConstants.QIRANGE_LOWER, Constants.HardenConstants.QIRANGE_UPPER);
    headingProfiledPIDController.setIntegratorRange(
        Constants.HardenConstants.HIRANGE_LOWER,
        Constants.HardenConstants.HIRANGE_UPPER); // 0.3 before

    headingProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);
    // configureAutoBuilder();
  }

  // Values relevant for the simulation
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  // starts the simulator thread
  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  /** Swerve request to apply during field-centric path following */
  private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds =
      new SwerveRequest.ApplyFieldSpeeds();

  // private void configureAutoBuilder() {
  //   try {
  //     var config = RobotConfig.fromGUISettings();
  //     AutoBuilder.configure(
  //         () -> currentState.Pose, // Supplier of current robot pose
  //         this::resetPose, // Consumer for seeding pose against auto
  //         () -> currentState.Speeds, // Supplier of current robot speeds
  //         // Consumer of ChassisSpeeds and feedforwards to drive the robot
  //         (speeds, feedforwards) ->
  //             setControl(
  //                 m_pathApplyRobotSpeeds
  //                     .withSpeeds(speeds)
  //                     .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
  //                     .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
  //         new PPHolonomicDriveController(
  //             // PID constants for translation
  //             new PIDConstants(10, 0, 0),
  //             // PID constants for rotation
  //             new PIDConstants(7, 0, 0)),
  //         config,
  //         // Assume the path needs to be flipped for Red vs Blue, this is normally the case
  //         () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
  //         this // Subsystem for requirements
  //         );
  //   } catch (Exception ex) {
  //     DriverStation.reportError(
  //         "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
  //   }
  // }

  public static SwerveSubsystem getInstance() {
    if (instance == null) {
      instance =
          new SwerveSubsystem(
              Constants.Swerve.DrivetrainConstants,
              250,
              Constants.Kalman.odometryMatrix,
              Constants.Kalman.visionMatrix,
              Constants.Swerve.FrontLeft,
              Constants.Swerve.FrontRight,
              Constants.Swerve.BackLeft,
              Constants.Swerve.BackRight);
    }
    return instance;
  }

  public double getDirectionalChassisSpeeds(Rotation2d qDirection) {
    return (qDirection.getCos() * getFieldSpeeds().vxMetersPerSecond)
        + (qDirection.getSin() * getFieldSpeeds().vyMetersPerSecond);
  }

  public void resetRotationPID() {
    headingProfiledPIDController.reset(
        currentState.Pose.getRotation().getRadians(), getFieldSpeeds().omegaRadiansPerSecond);
  }

  // Resets PID controllers
  public void resetProfiledPIDs() {
    headingProfiledPIDController.reset(
        currentState.Pose.getRotation().getRadians(), getFieldSpeeds().omegaRadiansPerSecond);
  }

  public void resetProfiledPIDs(Rotation2d qDirection) {
    headingProfiledPIDController.reset(
        currentState.Pose.getRotation().getRadians(), getFieldSpeeds().omegaRadiansPerSecond);
    qProfiledPIDController.reset(0, getDirectionalChassisSpeeds(qDirection));
  }

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4.0), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  /**
   * Getter for Robot's SwerveDriveState
   *
   * @return Robot's current SwerveDriveState
   */
  public SwerveDriveState getCurrentState() {
    return currentState;
  }

  /**
   * @return Robot's current Robot Chassis Speeds
   */
  public ChassisSpeeds getRobotSpeeds() {
    return currentState.Speeds;
  }

  /**
   * @return Robot's current Field-Centric Chassis Speeds
   */
  public ChassisSpeeds getFieldSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotSpeeds(), currentState.Pose.getRotation());
  }

  public void setRobotSpeeds(ChassisSpeeds speeds) {
    setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds));
  }

  public void setFieldSpeeds(ChassisSpeeds speeds) {
    setControl(
        m_pathApplyFieldSpeeds.withSpeeds(speeds).withDriveRequestType(DriveRequestType.Velocity));
  }

  public Pose2d getPose() {
    return currentState.Pose;
  }

  public Rotation2d travelAngleTo(Pose2d targetPose) {
    double deltaX = targetPose.getX() - getCurrentState().Pose.getX();
    double deltaY = targetPose.getY() - getCurrentState().Pose.getY();
    return new Rotation2d(Math.atan2(deltaY, deltaX));
  }

  public double calculateRequiredRotationalRate(Rotation2d targetRotation) {
    double omega =
        // headingProfiledPIDController.getSetpoint().velocity+
        headingProfiledPIDController.calculate(
            currentState.Pose.getRotation().getRadians(), targetRotation.getRadians());
    return omega;
  }

  public ChassisSpeeds calculateRequiredEdwardChassisSpeeds(
      Pose2d targetPose, double completePathDistance) {
    double distanceToTarget =
        getCurrentState().Pose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScaler =
        MathUtil.clamp(
            (distanceToTarget - Constants.HardenConstants.ffMinRadius)
                / (Constants.HardenConstants.ffMaxRadius - Constants.HardenConstants.ffMinRadius),
            0.0,
            1.0);

    double qSpeed =
        (qProfiledPIDController.getSetpoint().velocity * ffScaler)
            + qProfiledPIDController.calculate(
                completePathDistance - distanceToTarget, completePathDistance);
    double omega =
        // headingProfiledPIDController.getSetpoint().velocity+
        headingProfiledPIDController.calculate(
            currentState.Pose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    Rotation2d travelAngle = travelAngleTo(targetPose);

    DogLog.log(
        "Commands/JamesHarden/Edward/qPositionMeasurement",
        completePathDistance - distanceToTarget);
    DogLog.log(
        "Commands/JamesHarden/Edward/qPositionSetpoint",
        qProfiledPIDController.getSetpoint().position);
    DogLog.log(
        "Commands/JamesHarden/Edward/qVelocityMeasurement",
        getDirectionalChassisSpeeds(travelAngle));
    DogLog.log(
        "Commands/JamesHarden/Edward/qVelocitySetpoint",
        qProfiledPIDController.getSetpoint().velocity);
    DogLog.log(
        "Commands/JamesHarden/Edward/qPositionError", qProfiledPIDController.getPositionError());
    DogLog.log(
        "Commands/JamesHarden/Edward/qVelocityError", qProfiledPIDController.getVelocityError());

    DogLog.log(
        "Commands/JamesHarden/Rotational/PositionMeasurement",
        currentState.Pose.getRotation().getRadians());
    DogLog.log(
        "Commands/JamesHarden/Rotational/PositionSetpoint",
        headingProfiledPIDController.getSetpoint().position);
    DogLog.log(
        "Commands/JamesHarden/Rotational/VelocityMeasurement",
        currentState.Speeds.omegaRadiansPerSecond);
    DogLog.log(
        "Commands/JamesHarden/Rotational/VelocitySetpoint",
        headingProfiledPIDController.getSetpoint().velocity);
    DogLog.log(
        "Commands/JamesHarden/Rotational/PositionError",
        headingProfiledPIDController.getPositionError());
    DogLog.log(
        "Commands/JamesHarden/Rotational/VelocityError",
        headingProfiledPIDController.getVelocityError());

    return new ChassisSpeeds(
        qSpeed * Math.cos(travelAngle.getRadians())
            + (0 * 0.075 * Math.cos(getCurrentState().Pose.getRotation().getRadians())),
        qSpeed * Math.sin(travelAngle.getRadians())
            + (0 * 0.075 * Math.sin(getCurrentState().Pose.getRotation().getRadians())),
        omega);
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to {@link
   * #setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement in the form
   *     [x, y, theta]ᵀ, with units in meters and radians.
   */
  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }

  @Override
  public void periodic() {
    currentState = getState();

    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }

    DogLog.log("PoseEstimation/DrivetrainPose", currentState.Pose);

    DogLog.log(
        "Subsystems/SwerveSubsystem/CurrentCommand",
        this.getCurrentCommand() == null ? "NOTHING" : this.getCurrentCommand().getName());
  }
}