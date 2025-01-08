// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.ReefLocation;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Modular Auto Selection */
    private final SendableChooser<String> allianceChooser;
    private final SendableChooser<String> startPosChooser;
    private final SendableChooser<String> reef1Chooser;
    private final SendableChooser<String> reef2Chooser;
    private final SendableChooser<String> reef3Chooser;
    private final SendableChooser<String> endPosChooser;

    private String m_allianceSelected;
    private String m_startPosSelected;
    private String m_reef1Selected;
    private String m_reef2Selected;
    private String m_reef3Selected;
    private String m_endPosSelected;

    public RobotContainer() {
        // autoChooser = AutoBuilder.buildAutoChooser("Tests");
        // SmartDashboard.putData("Auto Mode", autoChooser);

        startPosChooser.setDefaultOption("Position 1", );
        startPosChooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto Start Position", startPosChooser);

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
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}