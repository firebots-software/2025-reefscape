// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.AutoRoutines.AutoProducer;
import frc.robot.commandGroups.RunFunnelUntilDetectionSafeSmooth;
import frc.robot.commands.DaleCommands.ArmToAngleCmd;
import frc.robot.commands.ElevatorCommands.DefaultElevator;
import frc.robot.commands.FunnelCommands.RunFunnelOutCommand;
import frc.robot.commands.FunnelCommands.stopFunnel;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;
import frc.robot.util.CustomController;
import java.util.function.BooleanSupplier;

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
    armSubsystem.setDefaultCommand(new ArmToAngleCmd(0.0, armSubsystem));
    elevatorSubsystem.setDefaultCommand(new DefaultElevator(elevatorSubsystem));

    // customController.Eject().onTrue(new EjectCoralFR(elevatorSubsystem, tootsieSlideSubsystem));

    // Trigger funnelCheckin =
    //     new Trigger(
    //             () -> funnelSubsystem.isCoralCheckedIn() &&
    // !CoralPosition.isCoralInTootsieSlide())
    //         .and(RobotModeTriggers.teleop());
    // Trigger ejectTime =
    //     new Trigger(
    //             () -> (funnelSubsystem.isCoralCheckedIn() &&
    // CoralPosition.isCoralInTootsieSlide()))
    //         .and(RobotModeTriggers.teleop());
    // ejectTime.onTrue(new EjectCoralFR(elevatorSubsystem, tootsieSlideSubsystem));
    // funnelCheckin.onTrue(new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.Intake,
    // false));
    // funnelCheckin.onTrue(
    //     new RunFunnelUntilDetectionSafeSmooth(elevatorSubsystem, funnelSubsystem, leds));
    // Trigger funnelCheckout =
    //     new Trigger(
    //             () ->
    //                 CoralPosition.isCoralInFunnel()
    //                     && elevatorSubsystem.atIntake()
    //                     && elevatorSubsystem.isAtPosition())
    //         .and(RobotModeTriggers.teleop());

    // funnelCheckout
    //     .and(joystick.rightTrigger().negate())
    //     .onTrue(
    //         new TransferPieceBetweenFunnelAndElevator(
    //             elevatorSubsystem, funnelSubsystem, tootsieSlideSubsystem));
    // Trigger coralInElevator =
    //     new Trigger(() -> CoralPosition.isCoralInTootsieSlide()).and(RobotModeTriggers.teleop());
    // coralInElevator.onTrue(
    //     new SetElevatorLevel(elevatorSubsystem, ElevatorPositions.safePosition, false));

    // Trigger leftTrigger = joystick.leftTrigger();
    // DoubleSupplier frontBackFunction = () -> -joystick.getLeftY(),
    //     leftRightFunction = () -> -joystick.getLeftX(),
    //     rotationFunction = () -> -joystick.getRightX(),
    //     speedFunction = () -> leftTrigger.getAsBoolean() ? 0d : 1d;
    // SwerveJoystickCommand swerveJoystickCommand =
    //     new SwerveJoystickCommand(
    //         frontBackFunction,
    //         leftRightFunction,
    //         rotationFunction,
    //         speedFunction,
    //         () -> joystick.leftTrigger().getAsBoolean(),
    //         redside,
    //         () -> joystick.a().getAsBoolean(),
    //         () -> customController.LeftL1().getAsBoolean(),
    //         () -> customController.RightL1().getAsBoolean(),
    //         driveTrain);
    // driveTrain.setDefaultCommand(swerveJoystickCommand);

    // joystick.x().onTrue(new ZeroElevatorHardStop(elevatorSubsystem));

    Trigger funnelCheckin =
        new Trigger(
                () -> funnelSubsystem.isCoralCheckedIn() && !CoralPosition.isCoralInTootsieSlide())
            .and(RobotModeTriggers.teleop());
    Trigger funnelCheckout =
        new Trigger(
                () ->
                    CoralPosition.isCoralInFunnel()
                        && elevatorSubsystem.atIntake()
                        && elevatorSubsystem.isAtPosition())
            .and(RobotModeTriggers.teleop());

    funnelCheckin.onTrue(
        new RunFunnelUntilDetectionSafeSmooth(elevatorSubsystem, funnelSubsystem, leds));

    funnelCheckout.onTrue(new stopFunnel(funnelSubsystem));

    customController.LeftL1().onTrue(new RunFunnelOutCommand(funnelSubsystem, () -> true));

    // Trigger coralInElevator =
    //     new Trigger(() -> CoralPosition.isCoralInTootsieSlide()).and(RobotModeTriggers.teleop());
    // coralInElevator.onTrue(
    //     new RunFunnelOutCommand(funnelSubsystem, () ->
    // elevatorSubsystem.getLevel()==ElevatorPositions.L1));
  }

  public static void setAlliance() {
    redAlliance =
        (DriverStation.getAlliance().isEmpty())
            ? false
            : (DriverStation.getAlliance().get() == Alliance.Red);
  }

  public BooleanSupplier getRedSide() {
    DogLog.log("Info/Alliance", redside.getAsBoolean());
    return redside;
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    int autoValue = autoChooser.getSelected();
    Command autoCommand;
    DogLog.log("Info/AutoSelected", autoValue);
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
                    Constants.AutoRoutines.RED_PROCESSOR_3,
                    leds)
                : new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.BLUE_PROCESSOR_3,
                    leds);
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
                    Constants.AutoRoutines.RED_PROCESSOR_2,
                    leds)
                : new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.BLUE_PROCESSOR_2,
                    leds);
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
                    Constants.AutoRoutines.RED_PROCESSOR_1,
                    leds)
                : new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.BLUE_PROCESSOR_1,
                    leds);
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
                    Constants.AutoRoutines.RED_CLEAR_3,
                    leds)
                : new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.BLUE_CLEAR_3,
                    leds);
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
                    Constants.AutoRoutines.RED_CLEAR_2,
                    leds)
                : new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.BLUE_CLEAR_2,
                    leds);
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
                    Constants.AutoRoutines.RED_CLEAR_1,
                    leds)
                : new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.BLUE_CLEAR_1,
                    leds);
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
                    Constants.AutoRoutines.RED_MID_1,
                    leds)
                : new AutoProducer(
                    driveTrain,
                    tootsieSlideSubsystem,
                    elevatorSubsystem,
                    funnelSubsystem,
                    armSubsystem,
                    Constants.AutoRoutines.BLUE_MID_1,
                    leds);
        break;
      default:
        autoCommand = null;
        break;
    }
    return autoCommand;
  }
}
