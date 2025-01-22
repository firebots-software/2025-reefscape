// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.VisionSystem;

import java.util.function.Supplier;

public class RobotContainer {
  private static Matrix<N3, N1> visionMatrix = VecBuilder.fill(0.01, 0.03d, 100d);
  private static Matrix<N3, N1> odometryMatrix = VecBuilder.fill(0.1, 0.1, 0.1);

  // Alliance color
  private Supplier<Boolean> redside = () -> redAlliance;
  private static boolean redAlliance;

  // private final SwerveSubsystem driveTrain = new SwerveSubsystem(
  //     Constants.Swerve.DrivetrainConstants,
  //     250.0, // TODO: CHANGE ODOMETRY UPDATE FREQUENCY TO CONSTANT,
  //     odometryMatrix,
  //     visionMatrix,
  //     Constants.Swerve.FrontLeft,
  //     Constants.Swerve.FrontRight,
  //     Constants.Swerve.BackLeft,
  //     Constants.Swerve.BackRight
  // );

  private final Telemetry logger =
      new Telemetry(Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
  private VisionSystem frontCamera = VisionSystem.getInstance("front-camera");
  private final CommandXboxController joystick = new CommandXboxController(0);

  // Starts telemetry operations (essentially logging -> look on SmartDashboard, AdvantageScope)
  public void doTelemetry() {
    // logger.telemeterize(driveTrain.getState());
    logger.logVisionPose(frontCamera.getMultiTagPose3d(new Pose2d()));
    SmartDashboard.putBoolean("Im loggin here", true);
  }

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Joystick suppliers,
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
    // SwerveJoystickCommand swerveJoystickCommand =
    //     new SwerveJoystickCommand(
    //         frontBackFunction,
    //         leftRightFunction,
    //         rotationFunction,
    //         speedFunction, // slowmode when left shoulder is pressed, otherwise fast
    //         () -> joystick.leftTrigger().getAsBoolean(),
    //         driveTrain);
    // driveTrain.setDefaultCommand(swerveJoystickCommand);

    joystick.povUp().onTrue(Commands.runOnce(SignalLogger::start));
    joystick.povDown().onTrue(Commands.runOnce(SignalLogger::stop));

    /*
     * Joystick Y = quasistatic forward
     * Joystick A = quasistatic reverse
     * Joystick B = dynamic forward
     * Joystick X = dyanmic reverse
     */

    // joystick.y().whileTrue(driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // joystick.a().whileTrue(driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // joystick.b().whileTrue(driveTrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // joystick.x().whileTrue(driveTrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public static void setAlliance() {
    redAlliance =
        (DriverStation.getAlliance().isEmpty())
            ? false
            : (DriverStation.getAlliance().get() == Alliance.Red);
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    SmartDashboard.putNumber("arjun IQ", 2.0);
    return new WaitCommand(10);
  }
}
