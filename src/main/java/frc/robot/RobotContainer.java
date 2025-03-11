// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.DoubleSupplier;

public class RobotContainer {

  // Alliance color
  Boolean coralInFunnel = Boolean.valueOf(false);
  Boolean coralInElevator = Boolean.valueOf(false);

  private static boolean redAlliance;

  private final SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();

  private final Telemetry logger =
      new Telemetry(Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
  private final CommandXboxController joystick = new CommandXboxController(0);
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
    autoChooser.addOption("Processor Side Start", 1);
    autoChooser.addOption("Clear Side Start (No Processor)", 2);
    autoChooser.addOption("Leave Only", 3);
    autoChooser.addOption("Mid 1 L4", 4);
    SmartDashboard.putData("Auto Side Choices", autoChooser);
    configureBindings();
  }

  public void teleopInit() {
    // CoralPosition.setCoralInTootsieSlide(funnelSubsystem.drakeTripped());

  }

  private void configureBindings() {
    joystick
        .y()
        .onTrue(
            driveTrain.runOnce(
                () ->
                    driveTrain.resetPose(
                        new Pose2d(driveTrain.getPose().getTranslation(), new Rotation2d(0)))));



    // Swerve
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
            driveTrain);
    driveTrain.setDefaultCommand(swerveJoystickCommand);



  }

  public static void setAlliance() {
    redAlliance =
        (DriverStation.getAlliance().isEmpty())
            ? false
            : (DriverStation.getAlliance().get() == Alliance.Red);
  }
    
    
}
