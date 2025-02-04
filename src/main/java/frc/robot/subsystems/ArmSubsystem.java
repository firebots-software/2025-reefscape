// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import frc.robot.subsystems.ArmSubsystem;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;


public class ArmSubsystem extends SubsystemBase {
  private static ArmSubsystem instance;

  private LoggedTalonFX armMotor;
  private LoggedTalonFX flywheelMotor;

  private DutyCycleEncoder revEncoder;
  private MotionMagicConfigs motionMagicConfigsArm;
  private MotionMagicConfigs motionMagicConfigsFlywheel;

  private final MotionMagicVoltage controlRequestArm = new MotionMagicVoltage(0);
  private final MotionMagicVoltage controlRequestFlywheel = new MotionMagicVoltage(0);
  private double encoderDegrees;

  private double targetDegrees;

  

  private final VoltageOut voltRequestArm = new VoltageOut(0.0);

  private final SysIdRoutine sysIdRoutineArm =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Seconds), // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> armMotor.setControl(voltRequestArm.withOutput(volts.in(Volts))),
              null,
              this));

  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }

  private ArmSubsystem() {
    CurrentLimitsConfigs clcArm =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Constants.Arm.ARM_STATOR_CURRENT_LIMIT_AMPS)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.Arm.ARM_SUPPLY_CURRENT_LIMIT_AMPS);
    CurrentLimitsConfigs clcFlywheel =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Constants.Flywheel.FLYWHEEL_STATOR_CURRENT_LIMIT_AMPS)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.Flywheel.FLYWHEEL_SUPPLY_CURRENT_LIMIT_AMPS);
    FeedbackConfigs fcArm = new FeedbackConfigs();
    MotorOutputConfigs mocArm = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    MotorOutputConfigs mocFlywheel =
        new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast);
    mocArm.withInverted(InvertedValue.CounterClockwise_Positive);
    mocFlywheel.withInverted(InvertedValue.Clockwise_Positive);
    ClosedLoopGeneralConfigs clgcArm = new ClosedLoopGeneralConfigs();
    ClosedLoopGeneralConfigs clgcFlywheel = new ClosedLoopGeneralConfigs();
    Slot0Configs s0cArm =
        new Slot0Configs().withKP(Constants.Arm.S0C_KP * 0.75).withKI(0).withKD(0);
    Slot0Configs s0cFlywheel =
        new Slot0Configs().withKP(Constants.Flywheel.FLYWHEEL_S0C_KP).withKI(0).withKD(0);

    // Initialize master motor only
    armMotor = new LoggedTalonFX(Constants.Arm.LT_PORT);
    flywheelMotor = new LoggedTalonFX(Constants.Flywheel.FLYWHEEL_PORT);

    TalonFXConfigurator masterConfiguratorArm = armMotor.getConfigurator();
    TalonFXConfigurator masterConfiguratorFlywheel = flywheelMotor.getConfigurator();

    masterConfiguratorArm.apply(mocArm);
    masterConfiguratorArm.apply(clcArm); // Apply current limits to the master motor
    masterConfiguratorArm.apply(s0cArm); // Apply PID settings to the master motor
    masterConfiguratorArm.apply(clgcArm);
    masterConfiguratorArm.apply(fcArm);

    masterConfiguratorFlywheel.apply(mocFlywheel);
    masterConfiguratorFlywheel.apply(clcFlywheel); // Apply current limits to the master motor
    masterConfiguratorFlywheel.apply(s0cFlywheel); // Apply PID settings to the master motor
    masterConfiguratorFlywheel.apply(clgcFlywheel);

    // Apply MotionMagicConfigs to master motor
    motionMagicConfigsArm = new MotionMagicConfigs();
    motionMagicConfigsArm.MotionMagicCruiseVelocity = Constants.Arm.MOTIONMAGIC_KV;
    motionMagicConfigsArm.MotionMagicAcceleration = Constants.Arm.MOTIONMAGIC_KA;
    masterConfiguratorArm.apply(motionMagicConfigsArm);

    motionMagicConfigsFlywheel = new MotionMagicConfigs();
    motionMagicConfigsFlywheel.MotionMagicCruiseVelocity = Constants.Flywheel.MOTIONMAGIC_KV;
    motionMagicConfigsFlywheel.MotionMagicAcceleration = Constants.Flywheel.MOTIONMAGIC_KA;
    masterConfiguratorFlywheel.apply(motionMagicConfigsFlywheel);

    // Initialize absolute encoder
    revEncoder = new DutyCycleEncoder(Constants.Arm.ENCODER_PORT);
    
  }

  public void setPosition(double angleDegrees) {
    targetDegrees = angleDegrees;
    armMotor.setControl(
        controlRequestArm.withPosition(Constants.Arm.ANGLE_TO_ENCODER_ROTATIONS(angleDegrees)));
  }

  public void startFlywheel(double angleDegrees) {
    flywheelMotor.setControl(

        new VelocityVoltage(30));


  }

  public void stopEveryingONG() {
    flywheelMotor.stopMotor();
  }

  private double calculateDegrees() {
    return revEncoder.get() * 360d;
  }

  public boolean atTarget(double endToleranceDegrees) {
    if (encoderDegrees < targetDegrees + endToleranceDegrees
        && encoderDegrees > targetDegrees - endToleranceDegrees) {
      return true;
    } else return false;
  }

  public void moveMuyNegative() {
    double veryNegativeNumberToTurnTo = -1000d;
    armMotor.setControl(
        controlRequestArm
            .withPosition(Constants.Arm.ANGLE_TO_ENCODER_ROTATIONS(veryNegativeNumberToTurnTo))
            .withSlot(0));
  }

  public boolean checkCurrent() {
    double current = armMotor.getTorqueCurrent().getValue().magnitude();

    if (current < -10) {
      armMotor.disable();
      return true;
    }

    return false;
  }

  public void zeroSensor() {
    armMotor.setPosition(0);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutineArm.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutineArm.dynamic(direction);
  }

  @Override
  public void periodic() {
    encoderDegrees = calculateDegrees();
    // This method will be called once per scheduler run
    DogLog.log("Arm at target", atTarget(5));
    DogLog.log("Arm Degrees", encoderDegrees);
    DogLog.log("Arm Target Degrees", targetDegrees);
  }

public void spinFlywheel(double flywheelSpeed) {
    flywheelMotor.set(flywheelSpeed);
  }

  // Stops the flywheel
  public void stopFlywheel() {
    flywheelMotor.set(0);
  }
}
