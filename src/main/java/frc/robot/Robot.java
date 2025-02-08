// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSystem;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private static Matrix<N3, N1> visionMatrix = VecBuilder.fill(0.01, 0.03d, 100d);
  private static Matrix<N3, N1> odometryMatrix = VecBuilder.fill(0.1, 0.1, 0.1);
  private VisionSystem visionBack = VisionSystem.getInstance(Constants.Vision.Cameras.BACK_CAM);
  private VisionSystem visionFront = VisionSystem.getInstance(Constants.Vision.Cameras.FRONT_CAM);
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
  private double leastDist;

  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
      m_robotContainer.doTelemetry();
    CommandScheduler.getInstance().run();
    Optional<EstimatedRobotPose> frontRobotPose =
        visionFront.getMultiTagPose3d(driveTrain.getState().Pose);
    Optional<EstimatedRobotPose> backRobotPose =
    visionBack.getMultiTagPose3d(driveTrain.getState().Pose);
    if (frontRobotPose.isPresent() || backRobotPose.isPresent() ) {

      double frontdistToAprilTag =
          visionFront.gAprilTagFieldLayout().getTagPose(visionFront.gPipelineResult().getBestTarget().getFiducialId())
              .get()
              .getTranslation()
              .getDistance(
                  new Translation3d(
                      driveTrain.getState().Pose.getX(), driveTrain.getState().Pose.getY(), 0.0));

    double backdistToAprilTag =
    visionBack.gAprilTagFieldLayout().getTagPose(visionFront.gPipelineResult().getBestTarget().getFiducialId())
        .get()
        .getTranslation() 
        .getDistance(
            new Translation3d(
                driveTrain.getState().Pose.getX(), driveTrain.getState().Pose.getY(), 0.0));

    if (frontdistToAprilTag <= backdistToAprilTag){
       leastDist = frontdistToAprilTag;
    }
    else {
       leastDist = backdistToAprilTag;
    }

      double xKalman = 0.01 * Math.pow(1.15, leastDist);

      double yKalman = 0.01 * Math.pow(1.4, leastDist);

      visionMatrix.set(0, 0, xKalman);
      visionMatrix.set(1, 0, yKalman);

      driveTrain.addVisionMeasurement(
          frontRobotPose.get().estimatedPose.toPose2d(),
          Timer.getFPGATimestamp() - 0.02,
          visionMatrix);
    }
  }
  

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    m_robotContainer.doTelemetry();
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void robotInit() {
    // DogLog.setOptions(
    // new DogLogOptions().withNtPublish(true).withCaptureDs(true).withLogExtras(true));
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    RobotContainer.setAlliance();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    RobotContainer.setAlliance();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    RobotContainer.setAlliance();
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
