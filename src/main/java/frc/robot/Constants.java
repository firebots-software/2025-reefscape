// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
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

  public static final class Arm {
    public static final double ARM_STATOR_CURRENT_LIMIT_AMPS = 5.0;
    public static final double ARM_SUPPLY_CURRENT_LIMIT_AMPS = 5.0;
    public static final double DEFAULT_ARM_ANGLE = 250.0;
    public static final double INTAKE_ANGLE = 3; // subject to change
    public static final double AMP_ANGLE = 95; // subject to change
    // public static final double ARM_ENCODER_OFFSET = 0; // TODO: Change the offset so that the 0
    // position is when the arm is at its resting
    // position.
    public static final String CANBUS_NAME = "Patrice the Pineapple";

    public static final int RT_PORT = 14; // Right Top motor
    public static final int RB_PORT = 13; // Right Bottom motor
    public static final int LT_PORT = 1; // Left Top motor
    public static final int LB_PORT = 11; // Left Bottom motor
    public static final int ENCODER_PORT = 0; // subject to change

    public static final double CURRENT_LIMIT = 8.0;
    public static final double S0C_KP = 1.0;
    public static final double ARMFF_KS = 0.16969;
    public static final double ARMFF_KG = 0.34;
    public static final double ARMFF_KV = 2.49;
    public static final double MOTIONMAGIC_KV = 1; // MotionMagic Cruise Velocity in RPS of the arm
    public static final double MOTIONMAGIC_KA = 2.2; // MotionMagic Acceleration in RPS^2 of the arm

    public static final double FEET_TO_METERS_CONVERSION_FACTOR = 0.3048;
    public static final double ABSOLUTE_ARM_CONVERSION_FACTOR = 0.5;
    public static final double INTEGRATED_ABSOLUTE_CONVERSION_FACTOR = 55.9867;
    public static final double INTEGRATED_ARM_CONVERSION_FACTOR =
        ABSOLUTE_ARM_CONVERSION_FACTOR
            * (INTEGRATED_ABSOLUTE_CONVERSION_FACTOR); // 130.63563333333335;
    public static final double ABSOLUTE_ENCODER_HORIZONTAL = 0.6655; // 0.6547
    public static final double ABSOLUTE_HORIZONTAL_OFFSET = 0.05; // 0.05
    public static double ARM_INTERMAP_OFFSET = 0;
    // public static double ZERO_SPEAKER_OFFSET_METERS = 0.6;
    public static final InterpolatingDoubleTreeMap INTERMAP = new InterpolatingDoubleTreeMap();

    static {
      UPDATE_INTERMAP_PETER();
    }

    public static void UPDATE_INTERMAP_PETER() {
      INTERMAP.clear();
      INTERMAP.put(
          1.34,
          6.5 + ARM_INTERMAP_OFFSET); // measurements of distance are from front of robot bumper to
      // wall
      INTERMAP.put(2.1, 17d + ARM_INTERMAP_OFFSET);
      INTERMAP.put(Units.feetToMeters(9) + Units.inchesToMeters(17), 23.5d + ARM_INTERMAP_OFFSET);
    }

    public static void UPDATE_INTERMAP_PIPER() {
      INTERMAP.clear();
      INTERMAP.put(1.34, 6.46 + ARM_INTERMAP_OFFSET);
      INTERMAP.put(1.34 + Units.inchesToMeters(30), 20.6 + ARM_INTERMAP_OFFSET);
      INTERMAP.put(1.34 + Units.inchesToMeters(60), 27.8 + ARM_INTERMAP_OFFSET);
      INTERMAP.put(1.34 + Units.inchesToMeters(90), 31.339 + ARM_INTERMAP_OFFSET);
      INTERMAP.put(1.34 + Units.inchesToMeters(120), 32.67 + ARM_INTERMAP_OFFSET);
    }

    public static double GET_YAJWINS_EQUATION(double distance) {
      double a = -6.02207;
      double b = -8.6529 * Math.pow(10, 15);
      double c = 252.816;
      double d = 35.7582;
      return b * Math.pow((distance + c), a) + d;
    }
  }

  public static class OI {
    public static final double LEFT_JOYSTICK_DEADBAND = 0.07;
    public static final double RIGHT_JOYSTICK_DEADBAND = 0.07;
    public static final int JOYSTICK_A_PORT = 0;

    public enum XBoxButtonID {
      /** A. */
      A(1),
      /** B. */
      B(2),
      /** X. */
      X(3),
      /** Y. */
      Y(4),
      /** Left bumper. */
      LeftBumper(5),
      /** Right bumper. */
      RightBumper(6),
      /** Left stick. */
      LeftStick(9),
      /** Right stick. */
      RightStick(10),
      /** Back. */
      Back(7),
      /** Start. */
      Start(8);
      public final int value;

      XBoxButtonID(int value) {
        this.value = value;
      }
    }

    public enum AxisID {
      /** Left X. */
      LeftX(0),
      /** Right X. */
      RightX(4),
      /** Left Y. */
      LeftY(1),
      /** Right Y. */
      RightY(5),
      /** Left trigger. */
      LeftTrigger(2),
      /** Right trigger. */
      RightTrigger(3);

      /** Axis value. */
      public final int value;

      AxisID(int value) {
        this.value = value;
      }
    }
  }

  //  public static class Swerve {
  //   public static class Simulation {
  //     // These are only used for simulation
  //     private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
  //     private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.01);
  //     // Simulated voltage necessary to overcome friction
  //     private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
  //     private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);
  //   }

  //   // TODO: Tune the Steer and Drive gains using SysID
  //   // The steer motor uses any SwerveModule.SteerRequestType control request with the
  //   // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
  //   private static final Slot0Configs STEER_GAINS =
  //       new Slot0Configs().withKP(25).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
  //   // When using closed-loop control, the drive motor uses the control
  //   // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
  //   private static final Slot0Configs DRIVE_GAINS =
  //       new Slot0Configs()
  //           .withKP(0.18)
  //           .withKI(0)
  //           .withKD(0)
  //           .withKS(-0.023265)
  //           .withKV(0.12681)
  //           .withKA(0.058864);

  //   // The closed-loop output type to use for the steer motors;
  //   // This affects the PID/FF gains for the steer motors
  //   private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT =
  //       ClosedLoopOutputType.Voltage;
  //   // The closed-loop output type to use for the drive motors;
  //   // This affects the PID/FF gains for the drive motors
  //   private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT =
  //       ClosedLoopOutputType.Voltage;

  //   // The type of motor used for the drive motor
  //   private static final DriveMotorArrangement DRIVE_MOTOR_TYPE =
  //       DriveMotorArrangement.TalonFX_Integrated;
  //   // The type of motor used for the drive motor
  //   private static final SteerMotorArrangement STEER_MOTOR_TYPE =
  //       SteerMotorArrangement.TalonFX_Integrated;

  //   // The remote sensor feedback type to use for the steer motors;
  //   // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to
  // RemoteCANcoder
  //   private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

  //   // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be
  // null.
  //   // This is where we apply Current Limits for swerve
  //   // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
  //   private static final TalonFXConfiguration DRIVE_INITIAL_CONFIGS =
  //       new TalonFXConfiguration()
  //           .withCurrentLimits(
  //               new CurrentLimitsConfigs()
  //                   .withStatorCurrentLimit(Amps.of(90.0))
  //                   .withStatorCurrentLimitEnable(true)
  //                   .withSupplyCurrentLimit(Amps.of(40.0))
  //                   .withSupplyCurrentLimitEnable(true));
  //   private static final TalonFXConfiguration STEER_INITIAL_CONFIGS =
  //       new TalonFXConfiguration()
  //           .withCurrentLimits(
  //               new CurrentLimitsConfigs()
  //                   .withSupplyCurrentLimitEnable(true)
  //                   .withSupplyCurrentLimit(Amps.of(30)));

  //   private static final CANcoderConfiguration ENCODER_INITIAL_CONFIGS =
  //       new CANcoderConfiguration();
  //   // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
  //   // TODO: investigate Pigeon2Configuration and how it's relevant
  //   private static final Pigeon2Configuration PIGEON2_CONFIGS = null;

  //   // TODO: CHANGE FOR NEW ROBOT
  //   // CAN bus that the devices are located on;
  //   // All swerve devices must share the same CAN bus
  //   public static final CANBus CANBUS_NAME = new CANBus("");

  //   // TODO: VERIFY FOR NEW ROBOT
  //   // The stator current at which the wheels start to slip;
  //   // This needs to be tuned to your individual robot
  //   private static final Current SLIP_CURRENT_AMPS = Amps.of(100.0);

  //   public static final Current DRIVE_STATOR_CURRENT_LIMIT_AMPS = Amps.of(90.0);
  //   public static final Current STEER_STATOR_CURRENT_LIMIT_AMPS = Amps.of(40.0);

  //   public static final Current DRIVE_SUPPLY_CURRENT_LIMIT_AMPS = Amps.of(40.0);
  //   public static final Current TURNING_SUPPLY_CURRENT_LIMIT_AMPS = Amps.of(30.0);

  //   // Theoretical free speed (m/s) at 12v applied output;
  //   // This needs to be tuned to your individual robot
  //   public static final LinearVelocity SPEED_AT_12V_METERS_PER_SECOND =
  //       MetersPerSecond.of(4.73); // TODO: VERIFY FOR NEW ROBOT

  //   private static final double COUPLE_RATIO = 3.5714285714285716;

  //   private static final double DRIVE_GEAR_RATIO = 6.75; // TODO: VERIFY FOR NEW ROBOT
  //   private static final double STEER_GEAR_RATIO = 21.428571428571427; // TODO: VERIFY FOR NEW
  // ROBOT
  //   private static final Distance WHEEL_RADIUS_INCHES = Inches.of(2); // TODO: VERIFY FOR NEW
  // ROBOT

  //   private static final boolean STEER_MOTOR_REVERSED = true; // TODO: CHANGE FOR NEW ROBOT
  //   private static final boolean INVERT_LEFT_SIDE = false; // TODO: CHANGE FOR NEW ROBOT
  //   private static final boolean INVERT_RIGHT_SIDE = true; // TODO: CHANGE FOR NEW ROBOT

  //   private static final int kPigeonId = 40; // TODO: CHANGE FOR NEW ROBOT

  //   public static final SwerveDrivetrainConstants DrivetrainConstants =
  //       new SwerveDrivetrainConstants()
  //           // .withCANBusName(CANBUS_NAME.getName())
  //           .withPigeon2Id(kPigeonId)
  //           .withPigeon2Configs(PIGEON2_CONFIGS);

  //   // Uses SwerveModuleConstantsFactory to organize all the previously mentioned configurations
  //   // related to the Swerve Drive
  //   private static final SwerveModuleConstantsFactory<
  //           TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
  //       ConstantCreator =
  //           new SwerveModuleConstantsFactory<
  //                   TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
  //               .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
  //               .withSteerMotorGearRatio(STEER_GEAR_RATIO)
  //               .withCouplingGearRatio(COUPLE_RATIO)
  //               .withWheelRadius(WHEEL_RADIUS_INCHES)
  //               .withSteerMotorGains(STEER_GAINS)
  //               .withDriveMotorGains(DRIVE_GAINS)
  //               .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
  //               .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
  //               .withSlipCurrent(SLIP_CURRENT_AMPS)
  //               .withSpeedAt12Volts(SPEED_AT_12V_METERS_PER_SECOND)
  //               .withDriveMotorType(DRIVE_MOTOR_TYPE)
  //               .withSteerMotorType(STEER_MOTOR_TYPE)
  //               .withFeedbackSource(STEER_FEEDBACK_TYPE)
  //               .withDriveMotorInitialConfigs(DRIVE_INITIAL_CONFIGS)
  //               .withSteerMotorInitialConfigs(STEER_INITIAL_CONFIGS)
  //               .withEncoderInitialConfigs(ENCODER_INITIAL_CONFIGS)
  //               .withSteerInertia(Simulation.STEER_INERTIA)
  //               .withDriveInertia(Simulation.DRIVE_INERTIA)
  //               .withSteerFrictionVoltage(Simulation.STEER_FRICTION_VOLTAGE)
  //               .withDriveFrictionVoltage(Simulation.DRIVE_FRICTION_VOLTAGE);

  //   // Front Left
  //   // TODO: CHANGE FOR NEW ROBOT
  //   private static final int FRONT_LEFT_STEER_MOTOR_ID = 3;
  //   private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 4;
  //   private static final int FRONT_LEFT_ENCODER_ID = 21;
  //   private static final Angle FRONT_LEFT_ENCODER_OFFSET_ROT = Rotations.of(0.3876953125);

  //   // TODO: CHANGE FOR NEW ROBOT
  //   private static final Distance FRONT_LEFT_X_POS = Inches.of(11.26);
  //   private static final Distance FRONT_LEFT_Y_POS = Inches.of(11.417);

  //   // Front Right
  //   // TODO: CHANGE FOR NEW ROBOT
  //   private static final int FRONT_RIGHT_STEER_MOTOR_ID = 5;
  //   private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 6;
  //   private static final int FRONT_RIGHT_ENCODER_ID = 22;
  //   private static final Angle FRONT_RIGHT_ENCODER_OFFSET_ROT = Rotations.of(0.159912109375);

  //   // TODO: CHANGE FOR NEW ROBOT
  //   private static final Distance FRONT_RIGHT_X_POS = Inches.of(11.26);
  //   private static final Distance FRONT_RIGHT_Y_POS = Inches.of(-11.417);

  //   // Back Left
  //   // TODO: CHANGE FOR NEW ROBOT
  //   private static final int BACK_LEFT_STEER_MOTOR_ID = 1;
  //   private static final int BACK_LEFT_DRIVE_MOTOR_ID = 2;
  //   private static final int BACK_LEFT_ENCODER_ID = 20;
  //   private static final Angle BACK_LEFT_ENCODER_OFFSET_ROT = Rotations.of(0.213134765625);

  //   // TODO: CHANGE FOR NEW ROBOT
  //   private static final Distance BACK_LEFT_X_POS = Inches.of(-11.26);
  //   private static final Distance BACK_LEFT_Y_POS = Inches.of(11.417);

  //   // Back Right
  //   // TODO: CHANGE FOR NEW ROBOT
  //   private static final int BACK_RIGHT_STEER_MOTOR_ID = 7;
  //   private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 8;
  //   private static final int BACK_RIGHT_ENCODER_ID = 23;
  //   private static final Angle BACK_RIGHT_ENCODER_OFFSET_ROT = Rotations.of(-0.3818359375);

  //   // TODO: CHANGE FOR NEW ROBOT
  //   private static final Distance BACK_RIGHT_X_POS = Inches.of(-11.26);
  //   private static final Distance BACK_RIGHT_Y_POS = Inches.of(-11.417);

  //   // Set the constants per module (constants defined above)
  //   public static final SwerveModuleConstants<
  //           TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
  //       FrontLeft =
  //           ConstantCreator.createModuleConstants(
  //               FRONT_LEFT_STEER_MOTOR_ID,
  //               FRONT_LEFT_DRIVE_MOTOR_ID,
  //               FRONT_LEFT_ENCODER_ID,
  //               FRONT_LEFT_ENCODER_OFFSET_ROT,
  //               FRONT_LEFT_X_POS,
  //               FRONT_LEFT_Y_POS,
  //               INVERT_LEFT_SIDE,
  //               STEER_MOTOR_REVERSED,
  //               false);
  //   public static final SwerveModuleConstants<
  //           TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
  //       FrontRight =
  //           ConstantCreator.createModuleConstants(
  //               FRONT_RIGHT_STEER_MOTOR_ID,
  //               FRONT_RIGHT_DRIVE_MOTOR_ID,
  //               FRONT_RIGHT_ENCODER_ID,
  //               FRONT_RIGHT_ENCODER_OFFSET_ROT,
  //               FRONT_RIGHT_X_POS,
  //               FRONT_RIGHT_Y_POS,
  //               INVERT_RIGHT_SIDE,
  //               STEER_MOTOR_REVERSED,
  //               false);
  //   public static final SwerveModuleConstants<
  //           TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
  //       BackLeft =
  //           ConstantCreator.createModuleConstants(
  //               BACK_LEFT_STEER_MOTOR_ID,
  //               BACK_LEFT_DRIVE_MOTOR_ID,
  //               BACK_LEFT_ENCODER_ID,
  //               BACK_LEFT_ENCODER_OFFSET_ROT,
  //               BACK_LEFT_X_POS,
  //               BACK_LEFT_Y_POS,
  //               INVERT_LEFT_SIDE,
  //               STEER_MOTOR_REVERSED,
  //               false);
  //   public static final SwerveModuleConstants<
  //           TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
  //       BackRight =
  //           ConstantCreator.createModuleConstants(
  //               BACK_RIGHT_STEER_MOTOR_ID,
  //               BACK_RIGHT_DRIVE_MOTOR_ID,
  //               BACK_RIGHT_ENCODER_ID,
  //               BACK_RIGHT_ENCODER_OFFSET_ROT,
  //               BACK_RIGHT_X_POS,
  //               BACK_RIGHT_Y_POS,
  //               INVERT_RIGHT_SIDE,
  //               STEER_MOTOR_REVERSED,
  //               false);

  //   // These constants are necessary for new Telemetry with swerve
  //   // TODO: CHANGE FOR NEW ROBOT
  //   private double MAX_SPEED_MPS =
  //       SPEED_AT_12V_METERS_PER_SECOND.magnitude(); // kSpeedAt12Volts desired top speed
  //   private double MAX_ANGULAR_RATE_RPS =
  //       RotationsPerSecond.of(0.75)
  //           .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  //   // TODO: CHANGE FOR NEW ROBOT
  //   // these outline the speed calculations
  //   public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND =
  //       5.944; // before: 4.8768;// 18ft/s = 5.486, 19m/s = 5.791ft/s, 19.5m/s = 5.944 ft/s,
  //   public static final double PHYSICAL_MAX_ANGLUAR_SPEED_RADIANS_PER_SECOND = 2 * 2 * Math.PI;

  //   public static final double TELE_DRIVE_FAST_MODE_SPEED_PERCENT = 0.75;
  //   public static final double TELE_DRIVE_SLOW_MODE_SPEED_PERCENT = 0.3;
  //   public static final double TELE_DRIVE_PERCENT_SPEED_RANGE =
  //       (TELE_DRIVE_FAST_MODE_SPEED_PERCENT - TELE_DRIVE_SLOW_MODE_SPEED_PERCENT);
  //   public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 6.01420;
  //   public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 3;
  // }

  // public static final String Arm = null;
}
