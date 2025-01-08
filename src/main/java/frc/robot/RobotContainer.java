// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
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

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.ReefLocation;

public class RobotContainer {
    private static Matrix<N3, N1> visionMatrix = VecBuilder.fill(0.01, 0.03d, 100d);
    private static Matrix<N3, N1> odometryMatrix = VecBuilder.fill(0.1, 0.1, 0.1);

    // Alliance color
    private Supplier<Boolean> redside = () -> redAlliance;
    private static boolean redAlliance;

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
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Modular Auto Selection */
    private final SendableChooser<String> startPosChooser;
    private final SendableChooser<String> reef1Chooser;
    private final SendableChooser<String> reef2Chooser;
    private final SendableChooser<String> reef3Chooser;
    private final SendableChooser<String> endPosChooser;

    private String m_startPosSelected;
    private String m_reef1Selected;
    private String m_reef2Selected;
    private String m_reef3Selected;
    private String m_endPosSelected;

    public RobotContainer() {
        configureBindings();
    }

    private static SendableChooser<Optional<ReefLocation>>
      pickup1choice = new SendableChooser<Optional<ReefLocation>>(),
      pickup2choice = new SendableChooser<Optional<ReefLocation>>(),
      pickup3choice = new SendableChooser<Optional<ReefLocation>>();
    SendableChooser<String> startchoice = new SendableChooser<String>();

    private void setupChooser() {
        pickup1choice.setDefaultOption("SECOND SHOT: DO NOTHING", Optional.empty());
        pickup1choice.addOption("AMPSIDE", Optional.of(NoteLocation.AMPSIDE));
        pickup1choice.addOption("MIDDLE", Optional.of(NoteLocation.MIDDLE));
        pickup1choice.addOption("STAGESIDE NOTE", Optional.of(NoteLocation.STAGESIDE));
        pickup2choice.setDefaultOption("THIRD SHOT: DO NOTHING", Optional.empty());
        pickup2choice.addOption("AMPSIDE NOTE", Optional.of(NoteLocation.AMPSIDE));
        pickup2choice.addOption("MIDDLE NOTE", Optional.of(NoteLocation.MIDDLE));
        pickup2choice.addOption("STAGESIDE NOTE", Optional.of(NoteLocation.STAGESIDE));
        pickup3choice.setDefaultOption("FOURTH SHOT: DO NOTHING", Optional.empty());
        pickup3choice.addOption("AMPSIDE", Optional.of(NoteLocation.AMPSIDE));
        pickup3choice.addOption("MIDDLE", Optional.of(NoteLocation.MIDDLE));
        pickup3choice.addOption("STAGESIDE NOTE", Optional.of(NoteLocation.STAGESIDE));
        startchoice.setDefaultOption("STARTING POSITION: MIDDLE START", "Mid");
        startchoice.addOption("AMPSIDE START", "Amp");
        startchoice.addOption("STAGESIDE START", "Stage");
        SmartDashboard.putData(pickup1choice);
        SmartDashboard.putData(pickup2choice);
        SmartDashboard.putData(pickup3choice);
        SmartDashboard.putData(startchoice);
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

        joystick.povUp().onTrue(Commands.runOnce(SignalLogger::start));
        joystick.povDown().onTrue(Commands.runOnce(SignalLogger::stop));

        /*
         * Joystick Y = quasistatic forward
         * Joystick A = quasistatic reverse
         * Joystick B = dynamic forward
         * Joystick X = dyanmic reverse
         */

        joystick.y().whileTrue(driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        joystick.a().whileTrue(driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        joystick.b().whileTrue(driveTrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        joystick.x().whileTrue(driveTrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
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