// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import java.util.Arrays;
import java.util.List;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static class Kalman {
    public static final Matrix<N3, N1> visionMatrix = VecBuilder.fill(0.01, 0.03d, 100d);
    public static final Matrix<N3, N1> odometryMatrix = VecBuilder.fill(0.1, 0.1, 0.1);
  }

  public static class HardenConstants {
    public static class EndWhenCloseEnough {
      public static final double translationalTolerance = 0.43105229381;
      // public static final double translationalTolerance = 0.6;
      public static final double headingTolerance = 0.7853975; // Math.PI/4
    }

    public static class RegularCommand {
      public static final double xyIndividualTolerance = 0.02;
      public static final double headingTolerance = 0.0075;
    }
  }

  public static class Vision {

    public static enum Cameras {
      RIGHT_CAM,
      LEFT_CAM;
    }

    // TODO: CHANGE FOR NEW ROBOT
    public static final double RIGHT_CAM_TO_ROBOT_TRANSLATION_X = Units.inchesToMeters(8.867);
    public static final double RIGHT_CAM_TO_ROBOT_TRANSLATION_Y = Units.inchesToMeters(-12.478);
    public static final double RIGHT_CAM_TO_ROBOT_TRANSLATION_Z = Units.inchesToMeters(6.158);
    public static final double RIGHT_CAM_TO_ROBOT_ROTATION_ROLL = 0;
    public static final double RIGHT_CAM_TO_ROBOT_ROTATION_PITCH = Units.degreesToRadians(-12.5);
    public static final double RIGHT_CAM_TO_ROBOT_ROTATION_YAW = Units.degreesToRadians(40);

    public static final double LEFT_CAM_TO_ROBOT_TRANSLATION_X = Units.inchesToMeters(8.867);
    public static final double LEFT_CAM_TO_ROBOT_TRANSLATION_Y = Units.inchesToMeters(12.478);
    public static final double LEFT_CAM_TO_ROBOT_TRANSLATION_Z = Units.inchesToMeters(6.158);
    public static final double LEFT_CAM_TO_ROBOT_ROTATION_ROLL = 0;
    public static final double LEFT_CAM_TO_ROBOT_ROTATION_PITCH = Units.degreesToRadians(-12.5);
    public static final double LEFT_CAM_TO_ROBOT_ROTATION_YAW = Units.degreesToRadians(-40);
  }

  public static class AutoRoutines {
    public static List<LandmarkPose>
        BLUE_PROCESSOR_3 =
            Arrays.asList(
                BlueLandmarkPose.PROCESSOR_AUTO_START,
                BlueLandmarkPose.L2,
                BlueLandmarkPose.R1,
                BlueLandmarkPose.L1,
                BlueLandmarkPose.PROCESSOR_HPS),
        BLUE_PROCESSOR_2 =
            Arrays.asList(
                BlueLandmarkPose.PROCESSOR_AUTO_START,
                BlueLandmarkPose.L2,
                BlueLandmarkPose.R1,
                BlueLandmarkPose.PROCESSOR_HPS),
        BLUE_PROCESSOR_1 =
            Arrays.asList(BlueLandmarkPose.PROCESSOR_AUTO_START, BlueLandmarkPose.L2),
        BLUE_CLEAR_3 =
            Arrays.asList(
                BlueLandmarkPose.CLEAR_AUTO_START,
                BlueLandmarkPose.R4,
                BlueLandmarkPose.R5,
                BlueLandmarkPose.L5,
                BlueLandmarkPose.CLEAR_HPS),
        BLUE_CLEAR_2 =
            Arrays.asList(
                BlueLandmarkPose.CLEAR_AUTO_START,
                BlueLandmarkPose.R4,
                BlueLandmarkPose.R5,
                BlueLandmarkPose.CLEAR_HPS),
        BLUE_CLEAR_1 = Arrays.asList(BlueLandmarkPose.CLEAR_AUTO_START, BlueLandmarkPose.R4),
        BLUE_MID_1 = Arrays.asList(BlueLandmarkPose.MID_AUTO_START, BlueLandmarkPose.L3),
        RED_PROCESSOR_3 =
            Arrays.asList(
                RedLandmarkPose.PROCESSOR_AUTO_START,
                RedLandmarkPose.L4,
                RedLandmarkPose.R5,
                RedLandmarkPose.L5,
                RedLandmarkPose.PROCESSOR_HPS),
        RED_PROCESSOR_2 =
            Arrays.asList(
                RedLandmarkPose.PROCESSOR_AUTO_START, RedLandmarkPose.L4, RedLandmarkPose.R5),
        RED_PROCESSOR_1 =
            Arrays.asList(
                RedLandmarkPose.PROCESSOR_AUTO_START,
                RedLandmarkPose.L4,
                RedLandmarkPose.PROCESSOR_HPS),
        RED_CLEAR_3 =
            Arrays.asList(
                RedLandmarkPose.CLEAR_AUTO_START,
                RedLandmarkPose.R2,
                RedLandmarkPose.L1,
                RedLandmarkPose.R1,
                RedLandmarkPose.CLEAR_HPS),
        RED_CLEAR_2 =
            Arrays.asList(
                RedLandmarkPose.CLEAR_AUTO_START,
                RedLandmarkPose.R2,
                RedLandmarkPose.L1,
                RedLandmarkPose.CLEAR_HPS),
        RED_CLEAR_1 = Arrays.asList(RedLandmarkPose.CLEAR_AUTO_START, RedLandmarkPose.R2),
        RED_MID_1 = Arrays.asList(RedLandmarkPose.MID_AUTO_START, RedLandmarkPose.L3);
  }

  public static class Landmarks {
    public static final double MIDLINE_X = 8.7741125;
  }

  public interface LandmarkPose {
    Pose2d getPose();

    boolean isRed();

    default boolean isBranch() {
      if (this instanceof Enum<?>) {
        String name = ((Enum<?>) this).name();
        return ((name.charAt(0) == 'L' || name.charAt(0) == 'R')
            && ((name.charAt(1) >= '0') && (name.charAt(1)) <= '5'));
      }
      return false;
    }

    default boolean endsWithHPS() {
      if (this instanceof Enum<?>) {
        String name = ((Enum<?>) this).name();
        if (name.length() >= 3) {
          return name.endsWith("HPS");
        }
      }
      return false;
    }
  }

  public static enum BlueLandmarkPose implements LandmarkPose {
    PROCESSOR_AUTO_START(
        new Pose2d(
            new Translation2d(7.11305570602417, 0.4940316081047058), new Rotation2d(Math.PI))),
    CLEAR_AUTO_START(
        new Pose2d(
            new Translation2d(7.113824844360352, 7.60764217376709), new Rotation2d(Math.PI))),
    MID_AUTO_START(
        new Pose2d(new Translation2d(7.11305570602417, 4.19448), new Rotation2d(Math.PI))),
    CLOSER_PROCESSOR_AUTO_START(
        new Pose2d(
            new Translation2d(7.11305570602417, 1.898607850074768), new Rotation2d(Math.PI))),
    CLOSER_CLEAR_AUTO_START(
        new Pose2d(
            new Translation2d(7.113824844360352, 6.167776107788086), new Rotation2d(Math.PI))),
    PROCESSOR_HPS(
        new Pose2d(
            new Translation2d(1.118087887763977, 1.0306631326675415),
            new Rotation2d(0.9334126223560425))),
    CLEAR_HPS(
        new Pose2d(
            new Translation2d(1.1465998888015747, 7.014684677124023),
            new Rotation2d(-0.9419997588093272))),
    L0(new Pose2d(new Translation2d(3.14058, 4.19448), new Rotation2d(Degrees.of(0)))),
    L1(new Pose2d(new Translation2d(3.66895, 2.94212), new Rotation2d(Degrees.of(60)))),
    L2(new Pose2d(new Translation2d(5.0177, 2.77352), new Rotation2d(Degrees.of(120)))),
    L3(new Pose2d(new Translation2d(5.83809, 3.85728), new Rotation2d(Degrees.of(180)))),
    L4(new Pose2d(new Translation2d(5.30973, 5.10963), new Rotation2d(Degrees.of(-120)))),
    L5(new Pose2d(new Translation2d(3.96097, 5.27823), new Rotation2d(Degrees.of(-60)))),
    R0(new Pose2d(new Translation2d(3.14058, 3.85728), new Rotation2d(Degrees.of(0)))),
    R1(new Pose2d(new Translation2d(3.96097, 2.77352), new Rotation2d(Degrees.of(60)))),
    R2(new Pose2d(new Translation2d(5.30973, 2.94212), new Rotation2d(Degrees.of(120)))),
    R3(new Pose2d(new Translation2d(5.83809, 4.19448), new Rotation2d(Degrees.of(180)))),
    R4(new Pose2d(new Translation2d(5.0177, 5.27823), new Rotation2d(Degrees.of(-120)))),
    R5(new Pose2d(new Translation2d(3.66895, 5.10963), new Rotation2d(Degrees.of(-60))));
    public final Pose2d pose;

    BlueLandmarkPose(Pose2d pose) {
      this.pose = pose;
    }

    @Override
    public Pose2d getPose() {
      return this.pose;
    }

    @Override
    public boolean isRed() {
      return false;
    }
  }

  public static enum RedLandmarkPose implements LandmarkPose {
    PROCESSOR_AUTO_START(
        new Pose2d(new Translation2d(10.441716194152832, 7.607936382293701), new Rotation2d())),
    CLEAR_AUTO_START(
        new Pose2d(new Translation2d(10.441716194152832, 0.47134917974472046), new Rotation2d())),
    MID_AUTO_START(new Pose2d(new Translation2d(10.441716194152832, 3.85728), new Rotation2d())),
    CLOSER_PROCESSOR_AUTO_START(
        new Pose2d(new Translation2d(10.441716194152832, 6.167776107788086), new Rotation2d())),
    CLOSER_CLEAR_AUTO_START(
        new Pose2d(new Translation2d(10.441716194152832, 1.898607850074768), new Rotation2d())),
    PROCESSOR_HPS(
        new Pose2d(
            new Translation2d(16.341829299926758, 7.0662689208984375),
            new Rotation2d(-2.1939969266716175))),
    CLEAR_HPS(
        new Pose2d(
            new Translation2d(16.39461326599121, 1.0060197114944458),
            new Rotation2d(2.1932607985206625))),
    R0(new Pose2d(new Translation2d(14.4076704, 4.19448), new Rotation2d(Degrees.of(180)))),
    R1(new Pose2d(new Translation2d(13.8793004, 2.94212), new Rotation2d(Degrees.of(120)))),
    R2(new Pose2d(new Translation2d(12.5305504, 2.77352), new Rotation2d(Degrees.of(60)))),
    R3(new Pose2d(new Translation2d(11.7101604, 3.85728), new Rotation2d(Degrees.of(0)))),
    R4(new Pose2d(new Translation2d(12.2385204, 5.10963), new Rotation2d(Degrees.of(-60)))),
    R5(new Pose2d(new Translation2d(13.5872804, 5.27823), new Rotation2d(Degrees.of(-120)))),
    L0(new Pose2d(new Translation2d(14.4076704, 3.85728), new Rotation2d(Degrees.of(180)))),
    L1(new Pose2d(new Translation2d(13.5872804, 2.77352), new Rotation2d(Degrees.of(120)))),
    L2(new Pose2d(new Translation2d(12.2385204, 2.94212), new Rotation2d(Degrees.of(60)))),
    L3(new Pose2d(new Translation2d(11.7101604, 4.19448), new Rotation2d(Degrees.of(0)))),
    L4(new Pose2d(new Translation2d(12.5305504, 5.27823), new Rotation2d(Degrees.of(-60)))),
    L5(new Pose2d(new Translation2d(13.8793004, 5.10963), new Rotation2d(Degrees.of(-120))));

    public final Pose2d pose;

    RedLandmarkPose(Pose2d pose) {
      this.pose = pose;
    }

    @Override
    public Pose2d getPose() {
      return this.pose;
    }

    @Override
    public boolean isRed() {
      return true;
    }
  }

  public static final class Arm {
    // this is new code
    public static final double PIVOT_GEAR_RATIO = 1 / 36d; // TODO

    // end of new code
    public static final double STATOR_CURRENT_LIMIT_AMPS = 30.0;
    public static final double SUPPLY_CURRENT_LIMIT_AMPS = 30.0;

    public static double DEGREES_TO_ROTATIONS(double degrees) {
      double conversionFactor = 360;
      return (degrees / conversionFactor) / PIVOT_GEAR_RATIO;
    }

    public static double ROTATIONS_TO_DEGEREES(double rotations) {
      double conversionFactor = 360;
      return (rotations * conversionFactor * PIVOT_GEAR_RATIO);
    }

    public static final int PIVOT_MOTOR_PORT = 16;

    public static final double CURRENT_LIMIT = 8.0;
    public static double S0C_KP = 0.75;
    public static double S0C_KI = 0.0;
    public static double S0C_KD = 0.0;
    public static double S0C_KS = 0.0;
    public static double S0C_KG = 0.0;

    public static final double MOTIONMAGIC_MAX_VELOCITY =
        200; // MotionMagic Cruise Velocity in RPS of the arm
    public static final double MOTIONMAGIC_MAX_ACCELERATION =
        5 * MOTIONMAGIC_MAX_VELOCITY; // MotionMagic Acceleration in RPS^2 of the arm
    public static final double ZERO_CURRENT = 5;
    public static final double DEALGAENATE_SPEED_ZOOM_ZOOM = 60;
    public static final double EXTENDED_ANGLE = 90;
    public static final double RETRACTED_ANGLE = 0;
  }

  public static class Flywheel {
    public static final int FLYWHEEL_PORT = 17;

    public static double S0C_KP = 1.0;
    public static double S0C_KI = 0.0;
    public static double S0C_KD = 0.0;
    public static double S0C_KS = 0.0;
    public static double S0C_KG = 0.0;

    public static final double MOTIONMAGIC_MAX_VELOCITY = 100;
    public static final double MOTIONMAGIC_MAX_ACCELERATION = 200;

    public static final double SUPPLY_CURRENT_LIMIT_AMPS = 15.0;
    public static final double STATOR_CURRENT_LIMIT_AMPS = 30.0;
    public static final double SPEED_RPS = 100.0;
    public static final double GEAR_RATIO = 1 / 6.7556;

    public static double ANGLE_TO_ENCODER_ROTATIONS(double angle) {
      double conversionFactor =
          0.159344d; // TODO: Find for actual bot. Will change with gear ratios.
      double zeroOffset =
          0.088; // TODO: For some reason when zeroing arm, zeros to 0.088. Fix on actual bot
      return (conversionFactor * angle) + zeroOffset;
    }
  }

  public static class OI {
    public static final double LEFT_JOYSTICK_DEADBAND = 0.07;
    public static final double RIGHT_JOYSTICK_DEADBAND = 0.07;
    public static final int JOYSTICK_A_PORT = 0;
  }

  public static class Swerve {
    public static final SwerveType WHICH_SWERVE_ROBOT = SwerveType.JAMES_HARDEN;

    public static enum SwerveLevel {
      L2(6.75, 21.428571428571427),
      L3(6.12, 21.428571428571427);
      public final double DRIVE_GEAR_RATIO, STEER_GEAR_RATIO;

      SwerveLevel(double drive, double steer) {
        DRIVE_GEAR_RATIO = drive;
        STEER_GEAR_RATIO = steer;
      }
    }

    public static enum SwerveDrivePIDValues {
      SERRANO(0.18014, 0d, 0d, -0.023265, 0.12681, 0.058864),
      PROTO(0.053218, 0d, 0d, 0.19977, 0.11198, 0.0048619),
      JAMES_HARDEN(0.041539, 0d, 0d, 0.14856, 0.12301, 0.0053369);
      public final double KP, KI, KD, KS, KV, KA;

      SwerveDrivePIDValues(double KP, double KI, double KD, double KS, double KV, double KA) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.KS = KS;
        this.KV = KV;
        this.KA = KA;
      }
    }

    public static enum SwerveSteerPIDValues {
      SERRANO(50d, 0d, 0.2, 0d, 1.5, 0d),
      PROTO(20d, 0d, 0d, 0d, 0d, 0d),
      JAMES_HARDEN(50d, 0d, 0d, 0d, 0d, 0d);
      public final double KP, KI, KD, KS, KV, KA;

      SwerveSteerPIDValues(double KP, double KI, double KD, double KS, double KV, double KA) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.KS = KS;
        this.KV = KV;
        this.KA = KA;
      }
    }

    public static enum RobotDimensions {
      SERRANO(Inches.of(22.52), Inches.of(22.834)), // length, width
      PROTO(Inches.of(22.52), Inches.of(22.834)), // length, width
      JAMES_HARDEN(Inches.of(26.75), Inches.of(22.75)); // length, width
      public final Distance length, width;

      RobotDimensions(Distance length, Distance width) {
        this.length = length;
        this.width = width;
      }
    }

    public static enum BumperThickness {
      SERRANO(Inches.of(2.625)), // thickness
      PROTO(Inches.of(2.625)), // thickness
      JAMES_HARDEN(Inches.of(3.313)); // thickness
      public final Distance thickness;

      BumperThickness(Distance thickness) {
        this.thickness = thickness;
      }
    }

    public static enum SwerveType {
      SERRANO(
          Rotations.of(-0.466552734375), // front left
          Rotations.of(-0.436767578125), // front right
          Rotations.of(-0.165283203125), // back left
          Rotations.of(-0.336181640625), // back right
          SwerveLevel.L3, // what level the swerve drive is
          SwerveDrivePIDValues.SERRANO,
          SwerveSteerPIDValues.SERRANO,
          RobotDimensions.SERRANO,
          "Patrice the Pineapple",
          BumperThickness.SERRANO),
      PROTO(
          Rotations.of(0.3876953125), // front left
          Rotations.of(0.159912109375), // front right
          Rotations.of(0.213134765625), // back left
          Rotations.of(-0.3818359375), // back right
          SwerveLevel.L2, // what level the swerve drive is
          SwerveDrivePIDValues.PROTO,
          SwerveSteerPIDValues.PROTO,
          RobotDimensions.PROTO,
          "rio",
          BumperThickness.PROTO),
      JAMES_HARDEN(
          Rotations.of(-0.076171875), // front left
          Rotations.of(-0.493896484375), // front right
          Rotations.of(0.18798828125), // back left
          Rotations.of(-0.1591796875), // back right
          SwerveLevel.L3,
          SwerveDrivePIDValues.JAMES_HARDEN,
          SwerveSteerPIDValues.JAMES_HARDEN,
          RobotDimensions.JAMES_HARDEN,
          "JamesHarden",
          BumperThickness.JAMES_HARDEN);
      public final Angle FRONT_LEFT_ENCODER_OFFSET,
          FRONT_RIGHT_ENCODER_OFFSET,
          BACK_LEFT_ENCODER_OFFSET,
          BACK_RIGHT_ENCODER_OFFSET;
      public final SwerveLevel SWERVE_LEVEL;
      public final SwerveDrivePIDValues SWERVE_DRIVE_PID_VALUES;
      public final SwerveSteerPIDValues SWERVE_STEER_PID_VALUES;
      public final RobotDimensions ROBOT_DIMENSIONS;
      public final String CANBUS_NAME;
      public final BumperThickness BUMPER_THICKNESS;

      SwerveType(
          Angle fl,
          Angle fr,
          Angle bl,
          Angle br,
          SwerveLevel swerveLevel,
          SwerveDrivePIDValues swerveDrivePIDValues,
          SwerveSteerPIDValues swerveSteerPIDValues,
          RobotDimensions robotDimensions,
          String canbus_name,
          BumperThickness thickness) {
        FRONT_LEFT_ENCODER_OFFSET = fl;
        FRONT_RIGHT_ENCODER_OFFSET = fr;
        BACK_LEFT_ENCODER_OFFSET = bl;
        BACK_RIGHT_ENCODER_OFFSET = br;
        SWERVE_LEVEL = swerveLevel;
        SWERVE_DRIVE_PID_VALUES = swerveDrivePIDValues;
        SWERVE_STEER_PID_VALUES = swerveSteerPIDValues;
        ROBOT_DIMENSIONS = robotDimensions;
        CANBUS_NAME = canbus_name;
        BUMPER_THICKNESS = thickness;
      }
    }

    public static class Simulation {
      // These are only used for simulation
      private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
      private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.01);
      // Simulated voltage necessary to overcome friction
      private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
      private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);
    }

    // TODO: Tune the Steer and Drive gains using SysID
    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs STEER_GAINS =
        new Slot0Configs()
            .withKP(WHICH_SWERVE_ROBOT.SWERVE_STEER_PID_VALUES.KP)
            .withKI(WHICH_SWERVE_ROBOT.SWERVE_STEER_PID_VALUES.KI)
            .withKD(WHICH_SWERVE_ROBOT.SWERVE_STEER_PID_VALUES.KD)
            .withKS(WHICH_SWERVE_ROBOT.SWERVE_STEER_PID_VALUES.KS)
            .withKV(WHICH_SWERVE_ROBOT.SWERVE_STEER_PID_VALUES.KV)
            .withKA(WHICH_SWERVE_ROBOT.SWERVE_STEER_PID_VALUES.KA);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs DRIVE_GAINS =
        new Slot0Configs()
            .withKP(WHICH_SWERVE_ROBOT.SWERVE_DRIVE_PID_VALUES.KP)
            .withKI(WHICH_SWERVE_ROBOT.SWERVE_DRIVE_PID_VALUES.KI)
            .withKD(WHICH_SWERVE_ROBOT.SWERVE_DRIVE_PID_VALUES.KD)
            .withKS(WHICH_SWERVE_ROBOT.SWERVE_DRIVE_PID_VALUES.KS)
            .withKV(WHICH_SWERVE_ROBOT.SWERVE_DRIVE_PID_VALUES.KV)
            .withKA(WHICH_SWERVE_ROBOT.SWERVE_DRIVE_PID_VALUES.KA);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT =
        ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT =
        ClosedLoopOutputType.Voltage;

    // The type of motor used for the drive motor
    private static final DriveMotorArrangement DRIVE_MOTOR_TYPE =
        DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SteerMotorArrangement STEER_MOTOR_TYPE =
        SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to
    //  RemoteCANcoder
    private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // This is where we apply Current Limits for swerve
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration DRIVE_INITIAL_CONFIGS =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(90.0))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40.0))
                    .withSupplyCurrentLimitEnable(true));
    private static final TalonFXConfiguration STEER_INITIAL_CONFIGS =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(40)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(30)));

    private static final CANcoderConfiguration ENCODER_INITIAL_CONFIGS =
        new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    // TODO: investigate Pigeon2Configuration and how it's relevant
    private static final Pigeon2Configuration PIGEON2_CONFIGS = null;

    // TODO: CHANGE FOR NEW ROBOT
    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus CANBUS_NAME = new CANBus(WHICH_SWERVE_ROBOT.CANBUS_NAME);

    // TODO: VERIFY FOR NEW ROBOT
    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current SLIP_CURRENT_AMPS = Amps.of(100.0);

    public static final Current DRIVE_STATOR_CURRENT_LIMIT_AMPS = Amps.of(90.0);
    public static final Current STEER_STATOR_CURRENT_LIMIT_AMPS = Amps.of(40.0);

    public static final Current DRIVE_SUPPLY_CURRENT_LIMIT_AMPS = Amps.of(40.0);
    public static final Current TURNING_SUPPLY_CURRENT_LIMIT_AMPS = Amps.of(30.0);

    public static final Current DUTY_CYCLE_VELOCITY = Current.ofBaseUnits(30.0, Amp);
    public static final Current ACCELERATION = Current.ofBaseUnits(50.0, Amp);

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity SPEED_AT_12V_METERS_PER_SECOND =
        MetersPerSecond.of(4.73); // TODO: VERIFY FOR NEW ROBOT

    private static final double COUPLE_RATIO = 3.5714285714285716;

    private static final double DRIVE_GEAR_RATIO =
        WHICH_SWERVE_ROBOT.SWERVE_LEVEL.DRIVE_GEAR_RATIO; // TODO: VERIFY FOR NEW ROBOT
    private static final double STEER_GEAR_RATIO =
        WHICH_SWERVE_ROBOT.SWERVE_LEVEL.STEER_GEAR_RATIO; // TODO: VERIFY FOR NEW ROBOT
    private static final Distance WHEEL_RADIUS_INCHES = Inches.of(2); // TODO: VERIFY FOR NEW ROBOT

    private static final boolean STEER_MOTOR_REVERSED = true; // TODO: CHANGE FOR NEW ROBOT
    private static final boolean INVERT_LEFT_SIDE = true; // TODO: CHANGE FOR NEW ROBOT
    private static final boolean INVERT_RIGHT_SIDE = true; // TODO: CHANGE FOR NEW ROBOT

    private static final int kPigeonId = 40; // TODO: CHANGE FOR NEW ROBOT

    public static final SwerveDrivetrainConstants DrivetrainConstants =
        new SwerveDrivetrainConstants()
            .withCANBusName(CANBUS_NAME.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(PIGEON2_CONFIGS);

    // Uses SwerveModuleConstantsFactory to organize all the previously mentioned configurations
    // related to the Swerve Drive
    private static final SwerveModuleConstantsFactory<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        ConstantCreator =
            new SwerveModuleConstantsFactory<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLE_RATIO)
                .withWheelRadius(WHEEL_RADIUS_INCHES)
                .withSteerMotorGains(STEER_GAINS)
                .withDriveMotorGains(DRIVE_GAINS)
                .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
                .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
                .withSlipCurrent(SLIP_CURRENT_AMPS)
                .withSpeedAt12Volts(SPEED_AT_12V_METERS_PER_SECOND)
                .withDriveMotorType(DRIVE_MOTOR_TYPE)
                .withSteerMotorType(STEER_MOTOR_TYPE)
                .withFeedbackSource(STEER_FEEDBACK_TYPE)
                .withDriveMotorInitialConfigs(DRIVE_INITIAL_CONFIGS)
                .withSteerMotorInitialConfigs(STEER_INITIAL_CONFIGS)
                .withEncoderInitialConfigs(ENCODER_INITIAL_CONFIGS)
                .withSteerInertia(Simulation.STEER_INERTIA)
                .withDriveInertia(Simulation.DRIVE_INERTIA)
                .withSteerFrictionVoltage(Simulation.STEER_FRICTION_VOLTAGE)
                .withDriveFrictionVoltage(Simulation.DRIVE_FRICTION_VOLTAGE);

    // Front Left
    // TODO: CHANGE FOR NEW ROBOT
    private static final int FRONT_LEFT_STEER_MOTOR_ID = 3;
    private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 4;
    private static final int FRONT_LEFT_ENCODER_ID = 21;
    private static final Angle FRONT_LEFT_ENCODER_OFFSET_ROT =
        WHICH_SWERVE_ROBOT.FRONT_LEFT_ENCODER_OFFSET;

    // TODO: CHANGE FOR NEW ROBOT
    private static final Distance FRONT_LEFT_X_POS =
        WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length.div(2);
    private static final Distance FRONT_LEFT_Y_POS =
        WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.width.div(2);

    // Front Right
    // TODO: CHANGE FOR NEW ROBOT
    private static final int FRONT_RIGHT_STEER_MOTOR_ID = 5;
    private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 6;
    private static final int FRONT_RIGHT_ENCODER_ID = 22;
    private static final Angle FRONT_RIGHT_ENCODER_OFFSET_ROT =
        WHICH_SWERVE_ROBOT.FRONT_RIGHT_ENCODER_OFFSET;

    // TODO: CHANGE FOR NEW ROBOT
    private static final Distance FRONT_RIGHT_X_POS =
        WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length.div(2);
    private static final Distance FRONT_RIGHT_Y_POS =
        WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.width.div(-2);

    // Back Left
    // TODO: CHANGE FOR NEW ROBOT
    private static final int BACK_LEFT_STEER_MOTOR_ID = 1;
    private static final int BACK_LEFT_DRIVE_MOTOR_ID = 2;
    private static final int BACK_LEFT_ENCODER_ID = 20;
    private static final Angle BACK_LEFT_ENCODER_OFFSET_ROT =
        WHICH_SWERVE_ROBOT.BACK_LEFT_ENCODER_OFFSET;

    // TODO: CHANGE FOR NEW ROBOT
    private static final Distance BACK_LEFT_X_POS =
        WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length.div(-2);
    private static final Distance BACK_LEFT_Y_POS =
        WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.width.div(2);

    // Back Right
    // TODO: CHANGE FOR NEW ROBOT
    private static final int BACK_RIGHT_STEER_MOTOR_ID = 7;
    private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 8;
    private static final int BACK_RIGHT_ENCODER_ID = 23;
    private static final Angle BACK_RIGHT_ENCODER_OFFSET_ROT =
        WHICH_SWERVE_ROBOT.BACK_RIGHT_ENCODER_OFFSET;

    // TODO: CHANGE FOR NEW ROBOT
    private static final Distance BACK_RIGHT_X_POS =
        WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length.div(-2);
    private static final Distance BACK_RIGHT_Y_POS =
        WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.width.div(-2);

    // Set the constants per module (constants defined above)
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontLeft =
            ConstantCreator.createModuleConstants(
                FRONT_LEFT_STEER_MOTOR_ID,
                FRONT_LEFT_DRIVE_MOTOR_ID,
                FRONT_LEFT_ENCODER_ID,
                FRONT_LEFT_ENCODER_OFFSET_ROT,
                FRONT_LEFT_X_POS,
                FRONT_LEFT_Y_POS,
                INVERT_LEFT_SIDE,
                STEER_MOTOR_REVERSED,
                false);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontRight =
            ConstantCreator.createModuleConstants(
                FRONT_RIGHT_STEER_MOTOR_ID,
                FRONT_RIGHT_DRIVE_MOTOR_ID,
                FRONT_RIGHT_ENCODER_ID,
                FRONT_RIGHT_ENCODER_OFFSET_ROT,
                FRONT_RIGHT_X_POS,
                FRONT_RIGHT_Y_POS,
                INVERT_RIGHT_SIDE,
                STEER_MOTOR_REVERSED,
                false);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackLeft =
            ConstantCreator.createModuleConstants(
                BACK_LEFT_STEER_MOTOR_ID,
                BACK_LEFT_DRIVE_MOTOR_ID,
                BACK_LEFT_ENCODER_ID,
                BACK_LEFT_ENCODER_OFFSET_ROT,
                BACK_LEFT_X_POS,
                BACK_LEFT_Y_POS,
                INVERT_LEFT_SIDE,
                STEER_MOTOR_REVERSED,
                false);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackRight =
            ConstantCreator.createModuleConstants(
                BACK_RIGHT_STEER_MOTOR_ID,
                BACK_RIGHT_DRIVE_MOTOR_ID,
                BACK_RIGHT_ENCODER_ID,
                BACK_RIGHT_ENCODER_OFFSET_ROT,
                BACK_RIGHT_X_POS,
                BACK_RIGHT_Y_POS,
                INVERT_RIGHT_SIDE,
                STEER_MOTOR_REVERSED,
                false);

    // These constants are necessary for new Telemetry with swerve
    // TODO: CHANGE FOR NEW ROBOT
    private double MAX_SPEED_MPS =
        SPEED_AT_12V_METERS_PER_SECOND.magnitude(); // kSpeedAt12Volts desired top speed
    private double MAX_ANGULAR_RATE_RPS =
        RotationsPerSecond.of(0.75)
            .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // TODO: CHANGE FOR NEW ROBOT
    // these outline the speed calculations
    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 4.868;
    // 5.944; // before: 4.8768;// 18ft/s = 5.486, 19m/s = 5.791ft/s, 19.5m/s = 5.944 ft/s,
    public static final double PHYSICAL_MAX_ANGLUAR_SPEED_RADIANS_PER_SECOND = 10.917;
    public static final double TELE_DRIVE_FAST_MODE_SPEED_PERCENT = 0.75;
    public static final double TELE_DRIVE_SLOW_MODE_SPEED_PERCENT = 0.3;
    public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 8;
    public static final double TELE_DRIVE_PERCENT_SPEED_RANGE =
        (TELE_DRIVE_FAST_MODE_SPEED_PERCENT - TELE_DRIVE_SLOW_MODE_SPEED_PERCENT);
    public static final double TELE_DRIVE_MAX_ANGULAR_RATE = 10.917;
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 26.971;
  }

  public static class TootsieSlide {
    public static final int MOTOR_PORT = 15;
    public static final int CHECKOUT_PORT = 1;
    public static final double SUPPLY_CURRENT_LIMIT = 90.0; // TODO
    public static final double STATOR_CURRENT_LIMIT = 90.0; // TODO

    public static double S0C_KP = 1.0; // TODO
    public static double S0C_KI = 0.0; // TODO
    public static double S0C_KD = 0.0; // TODO
    public static double S0C_KS = 0.0; // TODO
    public static double S0C_KG = 0.0; // TODO
    public static final double CRUISE_VELOCITY = 10; // TODO
    public static final double ACCELERATION = 10; // TODO
    public static final double GEAR_RATIO = 1d / 6d;
    public static final double INTAKE_SPEED_RPS = 5d; // TODO
    public static final double SHOOTING_SPEED_RPS =
        11.5d; // 12.5, 12 good for L3 and L2  worked for L2
  }

  public static class FunnelConstants {
    public static final int RIGHT_MOTOR_PORT = 14; // TODO
    public static final int LEFT_MOTOR_PORT = 13; // TODO
    public static final double SUPPLY_CURRENT_LIMIT = 20.0; // TODO
    public static final double STATOR_CURRENT_LIMIT = 15.0; // TODO
    public static final double S0C_KP = 1.0; // TODO
    public static final double S0C_KI = 0.0; // TODO
    public static final double S0C_KD = 0.0; // TODO
    public static final double CRUISE_VELOCITY = 10.0; // TODO
    public static final double ACCELERATION = 10.0; // TODO

    public static final double SLOW_BACKWARDS_VELOCITY = -0.1;
    public static final double SPEED_RPS = 15.0;
    public static final double GEAR_RATIO = 1d / 5d;

    public static final int CHECK_IN_PORT = 1;
    public static final int CHECK_OUT_PORT = 0;
    public static final int DRAKE_PORT = 2;
    public static final double MAX_POSITIONAL_ERROR = 0.05; // TODO
  }

  public static class ElevatorConstants {
    public static final int MOTOR1_PORT = 11; // TODO: change port
    public static final int MOTOR2_PORT = 12; // TODO: change port
    public static final int CANRANGE_PORT = 41; // TODO: change port
    public static final int kDriverControllerPort = 0; // todo: change port
    public static final double STATOR_CURRENT_LIMIT = 50.0; // TODO: change for actual match
    public static final double SUPPLY_CURRENT_LIMIT = 30.0; // TODO: change for actual match

    public static double S0C_KP = 2.0;
    public static double S0C_KI = 0.0;
    public static double S0C_KD = 0.005;

    public static double S1C_KP = 0.3501;
    public static double S1C_KI = 0.0;
    public static double S1C_KD = 0.0;

    public static double S0C_KS = 0.0;
    public static double S0C_KG = 0.29;
    public static double S0C_KA = 0.0004657452997; // 0.04
    public static double S0C_KV = 0.124; // 10.66

    public static final double MOTIONMAGIC_MAX_VELOCITY = 60;
    public static final double MOTIONMAGIC_MAX_ACCELERATION = 125;

    public static double SENSOR_OFFSET = 0.11;
    // public static final double MOTIONMAGIC_KG = 0.28;
    public static final double CRUISE_VELOCITY = 6.0; // To-do
    public static final double ACCELERATION = 6.0; // To-do
    public static final double SETPOINT_TOLERANCE = 0.2; // To-do
    public static final double MAX_POSITIONAL_ERROR = 0.02;
    public static final double SPROCKET_CIRCUM_INCHES =
        1.751 * Math.PI; // TODO: change 0 to radius/diameter
    public static final double GEAR_RATIO = 1d / 12d;
    public static final double SPROCKET_GEAR_RATIO = 12d / 1d; // TODO
    public static final double CARRAIGE_UPDUCTION = 3d / 1d; // TODO
    // public static final double CONVERSION_FACTOR = SPROCKET_GEAR_RATIO/(SPROCKET_CIRCUM_INCHES *
    // 0.0254); //This is converted to meters
    public static final double CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS =
        (SPROCKET_GEAR_RATIO) / (SPROCKET_CIRCUM_INCHES * 0.0254); // This is converted to meters
    public static final double CONVERSION_FACTOR_UP_ROTATIONS_TO_DISTANCE =
        1d / CONVERSION_FACTOR_UP_DISTANCE_TO_ROTATIONS;
    public static final double ELEVATOR_TORQUE = 32;
    public static final double ELEVATOR_DUTY_CYCLE = 0.4;
    public static final double elevatorRecalibration = 0.04;

    public static enum ElevatorPositions {
      // TODO: Change the height values based on heights needed to score/intake coral on
      Intake(
          0, 0.057 + elevatorRecalibration), // 0.71 really high but we were using before // 0.0685
      // //0.065 still too high
      safePosition(0, 0.3 + elevatorRecalibration),
      L1(1, 0.657 + elevatorRecalibration + 0.05),
      L2DALE(0, 0.493 + elevatorRecalibration), // 0.8636 - 0.379
      L2(2, 0.9036 - 0.02 + elevatorRecalibration),
      L3DALE(0, 0.91 + elevatorRecalibration), // 1.27 - 0.379
      L3(
          3,
          1.285 - 0.02 + elevatorRecalibration), // 1.27 // KALASH wants 1cm lower //old value 1.32
      L4(4, 1.835), // 1.81
      LIMIT_OF_TRAVEL(1, 1.825 + 0.025); // 1.825

      public final int position;
      public final double height;

      ElevatorPositions(int pos, double height) {
        this.position = pos;
        this.height = height;
      }

      public double getPosition() {
        return this.position;
      }

      public double getHeight() {
        return this.height;
      }
    }
  }

  public static final class MotorConstants {
    public final int PORT;
    public final boolean REVERSED;
    public final double GEAR_RATIO;
    public final double STATOR_CURRENT_LIMIT_AMPS;
    public final double SPEED_RPS;
    public final double AMP_SPEED_RPS;
    public final double SPEED_VOLTAGE;

    private MotorConstants(
        int port,
        boolean reversed,
        double gearRatio,
        double statorCurrent,
        double speed,
        double ampSpeed,
        double voltage) {
      PORT = port;
      REVERSED = reversed;
      GEAR_RATIO = gearRatio;
      STATOR_CURRENT_LIMIT_AMPS = statorCurrent;
      SPEED_RPS = speed;
      AMP_SPEED_RPS = ampSpeed;
      SPEED_VOLTAGE = voltage;
    }
  }
}
