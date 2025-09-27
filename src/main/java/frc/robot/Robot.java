// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ElevatorCommands.ZeroElevatorHardStop;
import frc.robot.subsystems.AnthonyVision;
import frc.robot.subsystems.CoralPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LoggedTalonFX;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  // Commented this out because arm is not on bot and this is activiating
  // something that doesn't physically exist

  // TODO: uncomment when arm is on real bot
  // private ZeroArm zeroArm = new ZeroArm(ArmSubsystem.getInstance());

  // private VisionSystem visionRight =
  // VisionSystem.getInstance(Constants.Vision.Cameras.RIGHT_CAM);
  // private VisionSystem visionLeft = VisionSystem.getInstance(Constants.Vision.Cameras.LEFT_CAM);
  private SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();
  private final RobotContainer m_robotContainer;
  private AnthonyVision visionRight;
  private AnthonyVision visionLeft;

  // standard deviation for x (meters), y (meters) and rotation (radians) camera data

  double rightDistToAprilTag, leftDistToAprilTag, leastPoseAmbDist;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    visionRight =
        AnthonyVision.getInstance(
            Constants.Vision.Cameras.RIGHT_CAM, m_robotContainer.getRedSide());
    visionLeft =
        AnthonyVision.getInstance(Constants.Vision.Cameras.LEFT_CAM, m_robotContainer.getRedSide());
    absoluteInit();
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
    LoggedTalonFX.periodic_static();
    CommandScheduler.getInstance().run();
    m_robotContainer.doTelemetry();

    visionRight.addFilteredPose();
    visionLeft.addFilteredPose();


    DogLog.log("CoralPosition/isCoralInFunnel", CoralPosition.isCoralInFunnel());
    DogLog.log("CoralPosition/isCoralInTootsieSlide", CoralPosition.isCoralInTootsieSlide());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    absoluteInit();
  }

  @Override
  public void robotInit() {
    DogLog.setOptions(
        new DogLogOptions().withNtPublish(true).withCaptureDs(true).withLogExtras(true));
    DogLog.log("PIDValues/ArmKP", Constants.Arm.S0C_KP);
    DogLog.log("PIDValues/ArmKI", Constants.Arm.S0C_KI);
    DogLog.log("PIDValues/ArmKD", Constants.Arm.S0C_KD);
    DogLog.log("PIDValues/ArmKS", Constants.Arm.S0C_KS);
    DogLog.log("PIDValues/ArmKG", Constants.Arm.S0C_KG);

    DogLog.log("PIDValues/ElevatorKP", Constants.ElevatorConstants.S0C_KP);
    DogLog.log("PIDValues/ElevatorKI", Constants.ElevatorConstants.S0C_KI);
    DogLog.log("PIDValues/ElevatorKD", Constants.ElevatorConstants.S0C_KD);
    DogLog.log("PIDValues/ElevatorKS", Constants.ElevatorConstants.S0C_KS);
    DogLog.log("PIDValues/ElevatorKG", Constants.ElevatorConstants.S0C_KG);

    DogLog.log("PIDValues/TootsieKP", Constants.TootsieSlide.S0C_KP);
    DogLog.log("PIDValues/TootsieKI", Constants.TootsieSlide.S0C_KI);
    DogLog.log("PIDValues/TootsieKD", Constants.TootsieSlide.S0C_KD);
    DogLog.log("PIDValues/TootsieKS", Constants.TootsieSlide.S0C_KS);
    DogLog.log("PIDValues/TootsieKG", Constants.TootsieSlide.S0C_KG);

    // Q set
    DogLog.log("PIDValues/QKP", Constants.HardenConstants.QKP);
    DogLog.log("PIDValues/QKI", Constants.HardenConstants.QKI);
    DogLog.log("PIDValues/QKD", Constants.HardenConstants.QKD);
    DogLog.log("PIDValues/QCRUISE", Constants.HardenConstants.QCRUISE);
    DogLog.log("PIDValues/QACCEL", Constants.HardenConstants.QACCEL);
    DogLog.log("PIDValues/QIZONE", Constants.HardenConstants.QIZONE);
    DogLog.log("PIDValues/QIRANGE_LOWER", Constants.HardenConstants.QIRANGE_LOWER);
    DogLog.log("PIDValues/QIRANGE_UPPER", Constants.HardenConstants.QIRANGE_UPPER);

    // H set
    DogLog.log("PIDValues/HKP", Constants.HardenConstants.HKP);
    DogLog.log("PIDValues/HKI", Constants.HardenConstants.HKI);
    DogLog.log("PIDValues/HKD", Constants.HardenConstants.HKD);
    DogLog.log("PIDValues/HCRUISE", Constants.HardenConstants.HCRUISE);
    DogLog.log("PIDValues/HACCEL", Constants.HardenConstants.HACCEL);
    DogLog.log("PIDValues/HIZONE", Constants.HardenConstants.HIZONE);
    DogLog.log("PIDValues/HIRANGE_LOWER", Constants.HardenConstants.HIRANGE_LOWER);
    DogLog.log("PIDValues/HIRANGE_UPPER", Constants.HardenConstants.HIRANGE_UPPER);
    // Commented this code that logs the electric data because it crashed the robot code
    // there is an error related to the usage of this
    // DogLog.setPdh(new PowerDistribution());
    absoluteInit();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    RobotContainer.setAlliance();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    CoralPosition.preLoadedCoral();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  // @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_robotContainer.teleopInit();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    absoluteInit();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if (ElevatorSubsystem.getInstance().isElevatorZeroed() == false) {
      CommandScheduler.getInstance()
          .schedule(new ZeroElevatorHardStop(ElevatorSubsystem.getInstance()));
    }

    // CommandScheduler.getInstance();
    // .schedule(zeroArm); // TODO: Fix this to not expose the CommandScheduler
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // RobotContainer.setAlliance();
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    absoluteInit();
  }

  public void absoluteInit() {
    RobotContainer.setAlliance();
  }
}
