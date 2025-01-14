// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ElevatorLevel1;
import frc.robot.commands.ElevatorLevel2;
import frc.robot.commands.ElevatorLevel3;
import frc.robot.commands.ElevatorLevel4;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static Matrix<N3, N1> visionMatrix = VecBuilder.fill(0.01, 0.03d, 100d);
  private static Matrix<N3, N1> odometryMatrix = VecBuilder.fill(0.1, 0.1, 0.1);

  // Alliance color
  private Supplier<Boolean> redside = () -> redAlliance;
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
  // The robot's subsystems and commands are defined here...
  private final ElevatorSubsystem m_ElevatorSubsystem = ElevatorSubsystem.getInstance();
  private final Telemetry logger =
      new Telemetry(Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
  private final CommandXboxController joystick = new CommandXboxController(0);

  // Starts telemetry operations (essentially logging -> look on SmartDashboard, AdvantageScope)
  public void doTelemetry() {
    logger.telemeterize(driveTrain.getState());
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
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
    SwerveJoystickCommand swerveJoystickCommand =
        new SwerveJoystickCommand(
            frontBackFunction,
            leftRightFunction,
            rotationFunction,
            speedFunction, // slowmode when left shoulder is pressed, otherwise fast
            () -> joystick.leftTrigger().getAsBoolean(),
            driveTrain);
    driveTrain.setDefaultCommand(swerveJoystickCommand);

   joystick.povUp().onTrue(new ElevatorLevel1(m_ElevatorSubsystem));
   joystick.povRight().onTrue(new ElevatorLevel2(m_ElevatorSubsystem));
   joystick.povDown().onTrue(new ElevatorLevel3(m_ElevatorSubsystem));
   joystick.povLeft().onTrue(new ElevatorLevel4(m_ElevatorSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }
  
  public static void setAlliance() {
    redAlliance =
        (DriverStation.getAlliance().isEmpty())
            ? false
            : (DriverStation.getAlliance().get() == Alliance.Red);
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return new Command() {};
  }
}
