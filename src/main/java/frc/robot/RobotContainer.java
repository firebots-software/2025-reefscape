// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmToAngleCmd;
import frc.robot.commands.ElevatorLevel1;
import frc.robot.commands.ElevatorLevel2;
import frc.robot.commands.ElevatorLevel3;
import frc.robot.commands.ElevatorLevel4;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TootsieSlideSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class RobotContainer {

  private static Matrix<N3, N1> visionMatrix = VecBuilder.fill(0.01, 0.03d, 100d);
  private static Matrix<N3, N1> odometryMatrix = VecBuilder.fill(0.1, 0.1, 0.1);

  TootsieSlideSubsystem testerTootsie = new TootsieSlideSubsystem();

  private final ElevatorSubsystem m_ElevatorSubsystem = ElevatorSubsystem.getInstance();

  // Alliance color
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

  // Starts telemetry operations (essentially logging -> look on SmartDashboard, AdvantageScope)
  public void doTelemetry() {
    logger.telemeterize(driveTrain.getState());
  }

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Joystick suppliers,
    Trigger leftShoulderTrigger = joystick.leftBumper();
    DoubleSupplier
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

    // joystick.rightBumper().whileTrue(new
    // TootsieSlideShooting(TootsieSlideSubsystem.getInstance()));

    joystick
        .x()
        .onTrue(
            driveTrain.runOnce(
                () ->
                    driveTrain.resetPose(
                        new Pose2d(new Translation2d(0.48, 4), Rotation2d.fromDegrees(0)))));

    Trigger rightBumper = joystick.rightBumper();
    rightBumper.onTrue(new ArmToAngleCmd(() -> 90d, ArmSubsystem.getInstance()));
    rightBumper.onFalse(new ArmToAngleCmd(() -> 45d, ArmSubsystem.getInstance()));

    joystick.povUp().onTrue(new ElevatorLevel1(m_ElevatorSubsystem));
    joystick.povRight().onTrue(new ElevatorLevel2(m_ElevatorSubsystem));
    joystick.povDown().onTrue(new ElevatorLevel3(m_ElevatorSubsystem));
    joystick.povLeft().onTrue(new ElevatorLevel4(m_ElevatorSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  public static void setAlliance() {
    redAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    SmartDashboard.putNumber("arjun IQ", 2.0);
    SmartDashboard.putNumber("Nikash's IQ which is muy mucho high", 2.0);
    return new WaitCommand(10);
  }
}
