// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
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

  public static class OI {
    public static final double LEFT_JOYSTICK_DEADBAND = 0.07;
    public static final double RIGHT_JOYSTICK_DEADBAND = 0.07;
    public static final int JOYSTICK_A_PORT = 0;
  }

  public static class Swerve {
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
        new Slot0Configs().withKP(25).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs DRIVE_GAINS =
        new Slot0Configs()
            .withKP(0.18)
            .withKI(0)
            .withKD(0)
            .withKS(-0.023265)
            .withKV(0.12681)
            .withKA(0.058864);

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
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
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
    public static final CANBus CANBUS_NAME = new CANBus("");

    // TODO: VERIFY FOR NEW ROBOT
    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current SLIP_CURRENT_AMPS = Amps.of(100.0);

    public static final Current DRIVE_STATOR_CURRENT_LIMIT_AMPS = Amps.of(90.0);
    public static final Current STEER_STATOR_CURRENT_LIMIT_AMPS = Amps.of(40.0);

    public static final Current DRIVE_SUPPLY_CURRENT_LIMIT_AMPS = Amps.of(40.0);
    public static final Current TURNING_SUPPLY_CURRENT_LIMIT_AMPS = Amps.of(30.0);

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity SPEED_AT_12V_METERS_PER_SECOND =
        MetersPerSecond.of(4.73); // TODO: VERIFY FOR NEW ROBOT

    private static final double COUPLE_RATIO = 3.5714285714285716;

    private static final double DRIVE_GEAR_RATIO = 6.75; // TODO: VERIFY FOR NEW ROBOT
    private static final double STEER_GEAR_RATIO = 21.428571428571427; // TODO: VERIFY FOR NEW ROBOT
    private static final Distance WHEEL_RADIUS_INCHES = Inches.of(2); // TODO: VERIFY FOR NEW ROBOT

    private static final boolean STEER_MOTOR_REVERSED = true; // TODO: CHANGE FOR NEW ROBOT
    private static final boolean INVERT_LEFT_SIDE = false; // TODO: CHANGE FOR NEW ROBOT
    private static final boolean INVERT_RIGHT_SIDE = true; // TODO: CHANGE FOR NEW ROBOT

    private static final int kPigeonId = 40; // TODO: CHANGE FOR NEW ROBOT

    public static final SwerveDrivetrainConstants DrivetrainConstants =
        new SwerveDrivetrainConstants()
            // .withCANBusName(CANBUS_NAME.getName())
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
    private static final Angle FRONT_LEFT_ENCODER_OFFSET_ROT = Rotations.of(0.3876953125);

    // TODO: CHANGE FOR NEW ROBOT
    private static final Distance FRONT_LEFT_X_POS = Inches.of(11.26);
    private static final Distance FRONT_LEFT_Y_POS = Inches.of(11.417);

    // Front Right
    // TODO: CHANGE FOR NEW ROBOT
    private static final int FRONT_RIGHT_STEER_MOTOR_ID = 5;
    private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 6;
    private static final int FRONT_RIGHT_ENCODER_ID = 22;
    private static final Angle FRONT_RIGHT_ENCODER_OFFSET_ROT = Rotations.of(0.159912109375);

    // TODO: CHANGE FOR NEW ROBOT
    private static final Distance FRONT_RIGHT_X_POS = Inches.of(11.26);
    private static final Distance FRONT_RIGHT_Y_POS = Inches.of(-11.417);

    // Back Left
    // TODO: CHANGE FOR NEW ROBOT
    private static final int BACK_LEFT_STEER_MOTOR_ID = 1;
    private static final int BACK_LEFT_DRIVE_MOTOR_ID = 2;
    private static final int BACK_LEFT_ENCODER_ID = 20;
    private static final Angle BACK_LEFT_ENCODER_OFFSET_ROT = Rotations.of(0.213134765625);

    // TODO: CHANGE FOR NEW ROBOT
    private static final Distance BACK_LEFT_X_POS = Inches.of(-11.26);
    private static final Distance BACK_LEFT_Y_POS = Inches.of(11.417);

    // Back Right
    // TODO: CHANGE FOR NEW ROBOT
    private static final int BACK_RIGHT_STEER_MOTOR_ID = 7;
    private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 8;
    private static final int BACK_RIGHT_ENCODER_ID = 23;
    private static final Angle BACK_RIGHT_ENCODER_OFFSET_ROT = Rotations.of(-0.3818359375);

    // TODO: CHANGE FOR NEW ROBOT
    private static final Distance BACK_RIGHT_X_POS = Inches.of(-11.26);
    private static final Distance BACK_RIGHT_Y_POS = Inches.of(-11.417);

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
    public static final double TELE_DRIVE_PERCENT_SPEED_RANGE =
        (TELE_DRIVE_FAST_MODE_SPEED_PERCENT - TELE_DRIVE_SLOW_MODE_SPEED_PERCENT);
    public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 6.01420;
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 3;
  }

  public static class ElevatorConstants {
    public static final int MOTOR1_PORT = 0; // TODO: change port
    public static final int MOTOR2_PORT = 0; // TODO: change port
    public static final int kDriverControllerPort = 0; // todo: change port
    public static final double STATOR_CURRENT_LIMIT = 5.0; // TODO: change for actual match
    public static final double SUPPYLY_CURRENT_LIMIT = 5.0; // TODO: change for actual match
    public static final int S0C_KP = 0;
    public static final int S0C_KI = 0;
    public static final int S0C_KD = 0;
    public static final int MOTIONMAGIC_KV = 0;
    public static final int MOTIONMAGIC_KA = 0;
    public static final double level1 = 0;
    public static final double level2 = 0;
    public static final double level3 = 0;
    public static final double level4 = 0;
    public static final double currentLimit = 0;

    public static enum ElevatorPositions {
        //TODO: Change the height values based on heights needed to score/intake coral on
        Intake(0,0.0),
        L1(1,0.0),
        L2(2,0.0),
        L3(3,0.0),
        L4(4,0.0);
    
        public final int position;
        public final double height;
    
        ElevatorPositions(int pos, double height) {
          this.position = pos;
          this.height = height;
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
