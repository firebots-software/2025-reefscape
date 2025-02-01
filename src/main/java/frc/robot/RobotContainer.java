// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.JamesHardenMovement;
import frc.robot.commands.SwerveJoystickCommand;
// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private static Matrix<N3, N1> visionMatrix = VecBuilder.fill(0.01, 0.03d, 100d);
  private static Matrix<N3, N1> odometryMatrix = VecBuilder.fill(0.1, 0.1, 0.1);

  private AutoFactory autoFactory;
    // Alliance color
    //private BooleanSupplier redside = () -> redAlliance;
    private static boolean redAlliance;
  
    private final SwerveSubsystem driveTrain =
        new SwerveSubsystem(
            Constants.Swerve.DrivetrainConstants,
            Constants.Swerve.Odometry.ODOMETRY_UPDATE_CONSTANT,
            odometryMatrix,
            visionMatrix,
            Constants.Swerve.FrontLeft,
            Constants.Swerve.FrontRight,
            Constants.Swerve.BackLeft,
            Constants.Swerve.BackRight);
  
    private final Telemetry logger =
        new Telemetry(Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
    private final CommandXboxController joystick = new CommandXboxController(0);
  
    // Starts telemetry operations (essentially logging -> look on SmartDashboard,
    // AdvantageScope)
    public void doTelemetry() {
      logger.telemeterize(driveTrain.getCurrentState());
    }
  
    // public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  
    /* Modular Auto Selection */
    private final SendableChooser<String> startPosChooser = new SendableChooser<String>();
  
    /**
   * 
   */
  public RobotContainer() {
      SmartDashboard.putString("follow_traj_running", "no");
  
      startPosChooser.setDefaultOption("Top (Next to Blue Barge Zone)", "top");
      startPosChooser.addOption("Middle (In between to Barge Zones)", "Middle");
      startPosChooser.addOption("Bottom (Next to Red Barge Zone)", "bottom");
      SmartDashboard.putData("Auto Start Position", startPosChooser);
  
      configureBindings();
      setAlliance();
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
  
    public void setAlliance() {
      redAlliance =
          (DriverStation.getAlliance().isEmpty())
              ? false
              : (DriverStation.getAlliance().get() == Alliance.Red);
    }
  
    public Command getAutonomousCommand() {
      /* Run the path selected from the auto chooser */
      //return new PathPlannerAuto("top", redAlliance ^ startPosChooser.getSelected() == "bottom"); // flips when red or bottom, doesn't flip if both
      boolean autoAlliance = redAlliance;
      autoFactory =
        new AutoFactory(
            () -> driveTrain.getState().Pose,
            driveTrain::resetPose,
            driveTrain::followTrajectory,
            true,
            driveTrain);
    Command auto = Commands.sequence(autoFactory.resetOdometry(startPosChooser.getSelected() + " start to reef1").andThen());
    return auto;
  }
}
