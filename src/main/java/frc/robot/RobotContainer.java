// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.OtherXBoxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;

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
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final OtherXBoxController m_driverController =
      new OtherXBoxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final SwerveSubsystem driveTrain = new SwerveSubsystem(
        Constants.Swerve.DrivetrainConstants,
        250.0, // TODO: CHANGE ODOMETRY UPDATE FREQUENCY TO CONSTANT,
        odometryMatrix,
        visionMatrix,
        Constants.Swerve.FrontLeft,
        Constants.Swerve.FrontRight,
        Constants.Swerve.BackLeft,
        Constants.Swerve.BackRight
    );

    private final Telemetry logger = new Telemetry(Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
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
