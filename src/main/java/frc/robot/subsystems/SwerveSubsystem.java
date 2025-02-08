package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.GyroStabilizer;
import java.util.function.Supplier;

public class SwerveSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements Subsystem {

  private static SwerveSubsystem instance;
  private ProfiledPIDController xPidController, yPidController, driverRotationPidController;
  private PIDController choreoX_pid, choreoY_pid, choreoRotation_pid;
  private SwerveDriveState currentState;
  private GyroStabilizer stabilizer;
  

  public SwerveSubsystem(
      SwerveDrivetrainConstants drivetrainConstants,
      double OdometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        TalonFX::new,
        TalonFX::new,
        CANcoder::new,
        drivetrainConstants,
        OdometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);

    instance = this;
    stabilizer = new GyroStabilizer();

    if (Utils.isSimulation()) {
      startSimThread();
    }
    currentState = getCurrentState();

    choreoX_pid = new PIDController(8, 0, 0);
    choreoY_pid = new PIDController(8, 0, 0);
    choreoRotation_pid = new PIDController(2.5, 0, 0);
    choreoRotation_pid.enableContinuousInput(-Math.PI, Math.PI);

    xPidController =
        new ProfiledPIDController(
            5.5,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND, 6));
    yPidController =
        new ProfiledPIDController(
            5.5,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND, 6));

    driverRotationPidController =
        new ProfiledPIDController(
            2.5,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Constants.Swerve.TELE_DRIVE_MAX_ANGULAR_RATE,
                Constants.Swerve.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND));
    driverRotationPidController.enableContinuousInput(-Math.PI, Math.PI);
    configureAutoBuilder();
  }

  public static SwerveSubsystem getInstance() {
    if (instance == null) {
      throw new Error("Please create one instance of SwerveSubsystem first.");
    }
    return instance;
  }

  // Values relevant for the simulation
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  // starts the simulator thread
  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  /** Swerve request to apply during field-centric path following */
  private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds =
      new SwerveRequest.ApplyFieldSpeeds();

  private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> currentState.Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> currentState.Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) ->
              setControl(
                  m_pathApplyRobotSpeeds
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(10, 0, 0),
              // PID constants for rotation
              new PIDConstants(7, 0, 0)),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  // Resets PID controllers
  public void resetPIDs() {
    xPidController.reset(currentState.Pose.getX());
    yPidController.reset(currentState.Pose.getY());
    driverRotationPidController.reset(currentState.Pose.getRotation().getRadians());
  }

  public Pose2d getPose() {
    return currentState.Pose;
  }

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4.0), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  /**
   * Getter for Robot's SwerveDriveState
   *
   * @return Robot's current SwerveDriveState
   */
  public SwerveDriveState getCurrentState() {
    return currentState;
  }

  /**
   * @return Robot's current Chassis Speeds
   */
  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return currentState.Speeds;
  }

  public ChassisSpeeds getCurrentFieldChassisSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        getCurrentRobotChassisSpeeds(), currentState.Pose.getRotation());
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    setControl(m_pathApplyFieldSpeeds.withSpeeds(speeds));
  }

  public ChassisSpeeds calculateRequiredChassisSpeeds(Pose2d targetPose) {
    double xFeedback = xPidController.calculate(currentState.Pose.getX(), targetPose.getX());
    double yFeedback = yPidController.calculate(currentState.Pose.getY(), targetPose.getY());
    double thetaFeedback =
        driverRotationPidController.calculate(
            currentState.Pose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    return new ChassisSpeeds(xFeedback, yFeedback, thetaFeedback);
  }

  public void followTrajectory(SwerveSample sample) {
    // Get the current pose of the robot
    Pose2d pose = getCurrentState().Pose;
    
    // Generate the next speeds for the robot
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx + choreoX_pid.calculate(pose.getX(), sample.x),
            sample.vy + choreoY_pid.calculate(pose.getY(), sample.y),
            sample.omega
                + choreoRotation_pid.calculate(pose.getRotation().getRadians(), sample.heading));

    DogLog.log("followTrajectory/sample.x", sample.x);
    DogLog.log("followTrajectory/sample.y", sample.y);
    DogLog.log("followTrajectory/sample.heading", sample.heading);

    DogLog.log("followTrajectory/sample.vx", sample.vx);
    DogLog.log("followTrajectory/sample.vy", sample.vy);
    DogLog.log("followTrajectory/sample.omega", sample.omega);

    DogLog.log("followTrajectory/pidOutputX", choreoX_pid.calculate(pose.getX(), sample.x));
    DogLog.log("followTrajectory/pidOutputY", choreoY_pid.calculate(pose.getY(), sample.y));
    DogLog.log(
        "followTrajectory/pidOutputX",
        choreoRotation_pid.calculate(pose.getRotation().getRadians(), sample.heading));

    DogLog.log("followTrajectory/speeds.vx", speeds.vxMetersPerSecond);
    DogLog.log("followTrajectory/speeds.vy", speeds.vyMetersPerSecond);
    DogLog.log("followTrajectory/speeds.omega", speeds.omegaRadiansPerSecond);

    // Apply the generated speed
    setChassisSpeeds(speeds);
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  @Override
  public void periodic() {
    currentState = getState();

    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }

    currentState = getState();

    DogLog.log("Swerve/RobotChassisSpeedsX(mps)", getCurrentRobotChassisSpeeds().vxMetersPerSecond);
    DogLog.log("Swerve/RobotChassisSpeedsY(mps)", getCurrentRobotChassisSpeeds().vyMetersPerSecond);
    DogLog.log(
        "Swerve/RobotChassisSpeedsTurning(radps)",
        getCurrentRobotChassisSpeeds().omegaRadiansPerSecond);
    DogLog.log("Swerve/FieldChassisSpeedsX(mps)", getCurrentFieldChassisSpeeds().vxMetersPerSecond);
    DogLog.log("Swerve/FieldChassisSpeedsY(mps)", getCurrentFieldChassisSpeeds().vyMetersPerSecond);
    DogLog.log(
        "Swerve/FieldChassisSpeedsTurning(radps)",
        getCurrentFieldChassisSpeeds().omegaRadiansPerSecond);
    DogLog.log(
        "Swerve/CurrentCommand",
        (getCurrentCommand() == null) ? "nothing" : getCurrentCommand().getName());

    stabilizer.getTipVector();
  }
}
