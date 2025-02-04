// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.*;

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

  public static class Landmarks {
    // midline constant
    public static final double MIDLINE_X = 8.774;

    // blude side
    public static final Translation2d[] leftBranchesBlue = {
      new Translation2d(3.7084, 4.191),
      new Translation2d(3.95732, 3.4328608),
      new Translation2d(4.736846, 3.2685736),
      new Translation2d(5.2689506, 3.861562),
      new Translation2d(5.0214276, 4.6188884),
      new Translation2d(4.2418254, 4.783201)
    };

    // blue side
    public static final Translation2d[] rightBranchesBlue = {
      new Translation2d(3.7085778, 3.861562),
      new Translation2d(4.2412666, 3.267583),
      new Translation2d(5.0220118, 3.4318702),
      new Translation2d(5.2700936, 4.1901872),
      new Translation2d(4.7374048, 4.7841916),
      new Translation2d(3.9566596, 4.619879)
    };

    public static final Translation2d[] rightBranchesRed = {
      new Translation2d(13.839825, 4.191),
      new Translation2d(13.590905, 3.4328608),
      new Translation2d(12.811379, 3.2685736),
      new Translation2d(12.2792744, 3.861562),
      new Translation2d(12.5267974, 4.6188884),
      new Translation2d(13.3063996, 4.783201)
    };

    // blue side
    public static final Translation2d[] leftBranchesRed = {
      new Translation2d(13.8396472, 3.861562),
      new Translation2d(13.3069584, 3.267583),
      new Translation2d(12.5262132, 3.4318702),
      new Translation2d(12.2781314, 4.1901872),
      new Translation2d(12.8108202, 4.7841916),
      new Translation2d(13.5915654, 4.619879)
    };

    public static final Rotation2d[] reefFacingAngleBlue = {
      new Rotation2d(Degrees.of(0)),
      new Rotation2d(Degrees.of(60)),
      new Rotation2d(Degrees.of(120)),
      new Rotation2d(Degrees.of(180)),
      new Rotation2d(Degrees.of(-120)),
      new Rotation2d(Degrees.of(-60))
    };

    public static final Rotation2d[] reefFacingAngleRed = {
      new Rotation2d(Degrees.of(180)),
      new Rotation2d(Degrees.of(120)),
      new Rotation2d(Degrees.of(60)),
      new Rotation2d(Degrees.of(0)),
      new Rotation2d(Degrees.of(-60)),
      new Rotation2d(Degrees.of(-120))
    };
  }

  public static final class Arm {
    public static final double ARM_STATOR_CURRENT_LIMIT_AMPS = 5.0;
    public static final double ARM_SUPPLY_CURRENT_LIMIT_AMPS = 5.0;
    public static final double DEFAULT_ARM_ANGLE = 250.0;
    public static final double INTAKE_ANGLE = 3; // subject to change
    public static final double AMP_ANGLE = 95; // subject to change

    public static double ANGLE_TO_ENCODER_ROTATIONS(double angle) {
      double conversionFactor =
          0.159344d; // TODO: Find for actual bot. Will change with gear ratios.
      double zeroOffset =
          0.088; // TODO: For some reason when zeroing arm, zeros to 0.088. Fix on actual bot
      return (conversionFactor * angle) + zeroOffset;
    }

    public static final int RT_PORT = 14; // Right Top motor
    public static final int RB_PORT = 13; // Right Bottom motor
    public static final int LT_PORT = 1; // Left Top motor
    public static final int LB_PORT = 11; // Left Bottom motor
    public static final int ENCODER_PORT =
        0; // subject to changepublic static final int ENCODER_PORT = 0; // subject to change

    public static final double CURRENT_LIMIT = 8.0;
    public static final double S0C_KP = 1.0;
    public static final double ARMFF_KS = 0.16969;
    public static final double ARMFF_KG = 0.34;
    public static final double ARMFF_KV = 2.49;
    public static final double MOTIONMAGIC_KV = 36; // MotionMagic Cruise Velocity in RPS of the arm
    public static final double MOTIONMAGIC_KA =
        2.2 * 36; // MotionMagic Acceleration in RPS^2 of the arm

    // Lmao this is useless, but it has my name on it
    public static double GET_YAJWINS_EQUATION(double distance) {
      double a = -6.02207;
      double b = -8.6529 * Math.pow(10, 15);
      double c = 252.816;
      double d = 35.7582;
      return b * Math.pow((distance + c), a) + d;
    }
  }

  public static class Flywheel {
    public static final int FLYWHEEL_PORT = 0;
    public static final double MOTIONMAGIC_KV = 0;
    public static final double MOTIONMAGIC_KA = 0;
    public static final double FLYWHEEL_S0C_KP = 0;
    public static final double FLYWHEEL_SUPPLY_CURRENT_LIMIT_AMPS = 5.0;
    public static final double FLYWHEEL_STATOR_CURRENT_LIMIT_AMPS = 5.0;

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
      JAMES_HARDEN(0.034816, 0d, 0d, 0.15396, 0.12145, 0.0029718);
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
      JAMES_HARDEN(Inches.of(26.749), Inches.of(22.74)); // length, width
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
          Rotations.of(-0.158447265625), // front left
          Rotations.of(-0.310791015625), // front right
          Rotations.of(-0.48681640625), // back left
          Rotations.of(0.4248046875), // back right
          SwerveLevel.L3,
          SwerveDrivePIDValues.JAMES_HARDEN,
          SwerveSteerPIDValues.JAMES_HARDEN,
          RobotDimensions.JAMES_HARDEN,
          "FireBot",
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
    private static final boolean INVERT_LEFT_SIDE = false; // TODO: CHANGE FOR NEW ROBOT
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
    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND =
        5.944; // before: 4.8768;// 18ft/s = 5.486, 19m/s = 5.791ft/s, 19.5m/s = 5.944 ft/s,
    public static final double PHYSICAL_MAX_ANGLUAR_SPEED_RADIANS_PER_SECOND = 2 * 2 * Math.PI;
    public static final double TELE_DRIVE_FAST_MODE_SPEED_PERCENT = 0.75;
    public static final double TELE_DRIVE_SLOW_MODE_SPEED_PERCENT = 0.3;
    public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 6.01420;
    public static final double TELE_DRIVE_PERCENT_SPEED_RANGE =
        (TELE_DRIVE_FAST_MODE_SPEED_PERCENT - TELE_DRIVE_SLOW_MODE_SPEED_PERCENT);
    public static final double TELE_DRIVE_MAX_ANGULAR_RATE = Math.PI * 1.5;
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND =
        TELE_DRIVE_MAX_ANGULAR_RATE * 8;
  }

  public static class TootsieSlide {
    public static final int MOTOR_PORT = 1; // TODO
    public static final int CHECKOUT_PORT = 1; // TODO
    public static final double SUPPLY_CURRENT_LIMIT = 90.0; // TODO
    public static final double STATOR_CURRENT_LIMIT = 90.0; // TODO
    public static final double S0C_KP = 1; // TODO
    public static final double S0C_KI = 0; // TODO
    public static final double S0C_KD = 0; // TODO
    public static final double CRUISE_VELOCITY = 1000; // TODO
    public static final double ACCELERATION = 1000; // TODO

    public static final int GEAR_RATIO = 12 / 15; // TODO
    public static final double SPEED_RPS = 2000; // TODO
  }

  public static class FunnelConstants {
    public static final int RIGHT_MOTOR_PORT = 0; // TODO
    public static final int LEFT_MOTOR_PORT = 0; // TODO
    public static final double SUPPLY_CURRENT_LIMIT = 5.0; // TODO
    public static final double STATOR_CURRENT_LIMIT = 5.0; // TODO
    public static final double S0C_KP = 0.0; // TODO
    public static final double S0C_KI = 0.0; // TODO
    public static final double S0C_KD = 0.0; // TODO
    public static final double CRUISE_VELOCITY = 0.0; // TODO
    public static final double ACCELERATION = 0.0; // TODO

    public static final double SLOW_BACKWARDS_VELOCITY = -0.1;
    public static final double SPEED_RPS = 0.0; // TODO
    public static final int GEAR_RATIO = 0; // TODO

    public static final int CHECK_IN_PORT = 0;
    public static final int CHECK_OUT_PORT = 0;
    public static final int DRAKE_PORT = 0;
    public static final double MAX_POSITIONAL_ERROR = 0.05; // TODO
  }

  public static class ElevatorConstants {
    public static final int MOTOR1_PORT = 0; // TODO: change port
    public static final int MOTOR2_PORT = 0; // TODO: change port
    public static final int kDriverControllerPort = 0; // todo: change port
    public static final double STATOR_CURRENT_LIMIT = 5.0; // TODO: change for actual match
    public static final double SUPPLY_CURRENT_LIMIT = 5.0; // TODO: change for actual match
    public static final int S0C_KP = 0;
    public static final int S0C_KI = 0;
    public static final int S0C_KD = 0;
    public static final int MOTIONMAGIC_KV = 0;
    public static final int MOTIONMAGIC_KA = 0;
    public static final double currentLimit = 0;
    public static final double CRUISE_VELOCITY = 0.0; // To-do
    public static final double ACCELERATION = 0.0; // To-do
    public static final double SETPOINT_TOLERANCE = 0; // To-do
    public static final double MAX_POSITIONAL_ERROR = 0.05;
    public static final double PULLEY_CIRCUM = 2 * Math.PI * 0; // TODO: change 0 to radius/diameter
    public static final double PULLEY_GEAR_RATIO = 1 / 5; // TODO
    public static final double CONVERSION_FACTOR = PULLEY_CIRCUM * PULLEY_GEAR_RATIO;

    public static enum ElevatorPositions {
      // TODO: Change the height values based on heights needed to score/intake coral on
      Intake(0, 0.0),
      L1(1, 0.0),
      L2(2, 0.0),
      L3(3, 0.0),
      L4(4, 0.0);

      public final int position;
      public final double height;

      ElevatorPositions(int pos, double height) {
        this.position = pos;
        this.height = height;
      }

    public double getPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
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
