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
import frc.robot.commandGroups.Dealgaenate;
import frc.robot.commands.DaleCommands.ArmToAngleCmd;
import frc.robot.commands.DebugCommands.DebugArm;
import frc.robot.commands.DebugCommands.DebugDaleSpin;
import frc.robot.commands.DebugCommands.DebugElevator;
import frc.robot.commands.DebugCommands.DebugFunnelIntake;
import frc.robot.commands.DebugCommands.DebugFunnelOuttake;
import frc.robot.commands.DebugCommands.DebugTootsieSlide;
import frc.robot.commands.SwerveCommands.JamesHardenMovement;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.commands.DebugCommands.DebugTootsieSlide;
import frc.robot.commands.DebugCommands.DebugElevator;
import frc.robot.commands.DebugCommands.DebugFunnelIntake;
import frc.robot.commands.DebugCommands.DebugFunnelOuttake;
import frc.robot.subsystems.FunnelSubsystem;


public class RobotContainer {
  private static Matrix<N3, N1> visionMatrix = VecBuilder.fill(0.01, 0.03d, 100d);
  private static Matrix<N3, N1> odometryMatrix = VecBuilder.fill(0.1, 0.1, 0.1);

  // TODO: Uncomment when mechanisms arrive on the robot:
  //   TootsieSlideSubsystem tootsieSlideSubsystem = TootsieSlideSubsystem.getInstance();
  //   FunnelSubsystem funnelSubsystem = FunnelSubsystem.getInstance();
  //   ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
  //   ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
  // Alliance color
  Boolean coralInFunnel = Boolean.valueOf(false);
  Boolean coralInElevator = Boolean.valueOf(false);

  private BooleanSupplier redside = () -> redAlliance;
  private static boolean redAlliance;

  private final SwerveSubsystem driveTrain =
      new SwerveSubsystem(
          Constants.Swerve.DrivetrainConstants,
          250.0, // TODO: CHANGE ODOMETRY UPDATE FREQUENCY TO CONSTANT,
          odometryMatrix,
          visionMatrix,
          Constants.Swerve.FrontLeft,
          Constants.Swerve.FrontRight,
          Constants.Swerve.BackLeft,
          Constants.Swerve.BackRight);

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
    // TODO: Uncomment when mechanisms arrive on the robot:
    // Joystick suppliers,
    // funnelSubsystem.setDefaultCommand(new DefaultFunnelCommand(funnelSubsystem));
    // Trigger funnelCheckin = new Trigger(() -> funnelSubsystem.isCoralCheckedIn());
    // funnelCheckin.onTrue(new RunFunnelUntilDetection(funnelSubsystem, elevatorSubsystem));

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

    BooleanSupplier increaseFunction = () -> debugJoystick.rightBumper().getAsBoolean(),
        decreaseFunction = () -> debugJoystick.leftBumper().getAsBoolean(),
        pidChangeFunction = () -> debugJoystick.a().getAsBoolean(),
        mechChangeFunction = () -> debugJoystick.b().getAsBoolean();
    
    


    // TODO: Uncomment when mechanisms arrive on the robot:
    // joystick.rightBumper().whileTrue(new
    // TootsieSlideShooting(TootsieSlideSubsystem.getInstance()));

    // Debugging
    debugJoystick
        .rightTrigger()
        .whileTrue(new DebugTootsieSlide(TootsieSlideSubsystem.getInstance()));

    debugJoystick.rightBumper().whileTrue(new DebugFunnelIntake(FunnelSubsystem.getInstance()));

    debugJoystick.leftBumper().whileFalse(new DebugFunnelOuttake(FunnelSubsystem.getInstance()));

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
    Trigger rightBumper = joystick.rightBumper();

    // TODO: Uncomment when mechanisms arrive on the robot:
    // Currently this code uses commands that we can't call or else it will throw errors
    // rightBumper.onTrue(new Dealgaenate(ArmSubsystem.getInstance()));
    // rightBumper.onFalse(
    //     new ArmToAngleCmd(Constants.Arm.RETRACTED_ANGLE, ArmSubsystem.getInstance()));
    // joystick.y().whileTrue(JamesHardenMovement.toClosestRightBranch(driveTrain, redside));

    // Debugging
    debugJoystick.rightTrigger().whileTrue(new DebugTootsieSlide(TootsieSlideSubsystem.getInstance()));
    debugJoystick.y().onTrue(new DebugElevator(ElevatorSubsystem.getInstance()));

    // Debugging
    debugJoystick.leftTrigger().whileTrue(new DebugDaleSpin(ArmSubsystem.getInstance()));
    debugJoystick.b().onTrue(new DebugArm(ArmSubsystem.getInstance()));

    debugJoystick.leftStick().whileTrue(new DebugFunnelIntake(FunnelSubsystem.getInstance()));
    debugJoystick.rightStick().whileTrue(new DebugFunnelOuttake(FunnelSubsystem.getInstance()));


    // TODO: Uncomment when mechanisms arrive on the robot:
    // joystick.povUp().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L1));
    // joystick.povRight().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L2));
    // joystick.povDown().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L3));
    // joystick.povLeft().onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.L4));



    // joystick
    //     .a()
    //     .whileTrue(
    //         new SetElevatorLevel(
    //             elevatorSubsystem,
    //             ElevatorPositions.safePosition)); // change safepos in constants
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
