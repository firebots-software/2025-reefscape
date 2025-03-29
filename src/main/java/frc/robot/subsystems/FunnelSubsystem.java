// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FunnelConstants;
import frc.robot.util.LoggedTalonFX;

public class FunnelSubsystem extends SubsystemBase {
  private static FunnelSubsystem instance;

  private LoggedTalonFX rightMotor;
  private LoggedTalonFX leftMotor;
  private DigitalInput checkOutSensor;
  private DigitalInput checkInSensor;
  private DigitalInput drake;
  private double coralCheckedOutPosition;
  private final MotionMagicVoltage controlRequest = new MotionMagicVoltage(0);
  private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0);
  private final boolean m;

  private FunnelSubsystem() {
    m = false;
    rightMotor =
        new LoggedTalonFX("subsystems/Funnel/rightMotor", FunnelConstants.RIGHT_MOTOR_PORT);
    leftMotor = new LoggedTalonFX("subsystems/Funnel/leftMotor", FunnelConstants.LEFT_MOTOR_PORT);

    drake = new DigitalInput(Constants.FunnelConstants.DRAKE_PORT);

    checkOutSensor = new DigitalInput(Constants.FunnelConstants.CHECK_OUT_PORT);
    checkInSensor = new DigitalInput(Constants.FunnelConstants.CHECK_IN_PORT);

    Follower invertedfollower = new Follower(FunnelConstants.RIGHT_MOTOR_PORT, true);
    leftMotor.setControl(invertedfollower);

    TalonFXConfigurator m1Config = rightMotor.getConfigurator();
    TalonFXConfigurator m2Config = leftMotor.getConfigurator();

    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Constants.FunnelConstants.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.FunnelConstants.SUPPLY_CURRENT_LIMIT);

    MotorOutputConfigs moc =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(Constants.FunnelConstants.S0C_KP)
            .withKI(Constants.FunnelConstants.S0C_KI)
            .withKD(Constants.FunnelConstants.S0C_KD);

    m1Config.apply(moc);
    m2Config.apply(moc);

    m1Config.apply(clc);
    m2Config.apply(clc);

    m1Config.apply(s0c);
    m2Config.apply(s0c);

    MotionMagicConfigs mmc =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.FunnelConstants.CRUISE_VELOCITY)
            .withMotionMagicAcceleration(Constants.FunnelConstants.ACCELERATION);

    // mmc.MotionMagicAcceleration =
    // MotionMagicConfigs mmcR =
    //     new MotionMagicConfigs()
    //     .withMotionMagicCruiseVelocity(15)
    //     .withMotionMagicAcceleration(60);

    m2Config.apply(mmc);

    coralCheckedOutPosition = rightMotor.getPosition().getValueAsDouble();
  }

  public static FunnelSubsystem getInstance() {
    if (instance == null) {
      instance = new FunnelSubsystem();
    }
    return instance;
  }

  public double getAbsolutePositionalError() {
    if (rightMotor.getControlMode().getValue().equals(ControlModeValue.MotionMagicVoltage)) {
      return Math.abs(rightMotor.getPosition().getValueAsDouble() - coralCheckedOutPosition);
    } else {
      return 0.0;
    }
  }

  private void runFunnelAtRPS(double speed) {
    VelocityVoltage m_velocityControlTop =
        new VelocityVoltage(speed / Constants.FunnelConstants.GEAR_RATIO);
    rightMotor.setControl(m_velocityControlTop);
  }

  public void rampUp() {
    // velocityRequest.Acceleration = 60;
    // velocityRequest.Velocity = 15;
    rightMotor.setControl(velocityRequest.withVelocity(15).withAcceleration(60));
  }

  public void spinFunnel() {
    runFunnelAtRPS(Constants.FunnelConstants.SPEED_RPS);
  }

  public void stopFunnel() {
    rightMotor.stopMotor();
  }

  public void reAdjustMotor() {
    rightMotor.setControl(controlRequest.withPosition(coralCheckedOutPosition).withSlot(0));
  }

  public void updateCoralCheckedOutPosition() {
    coralCheckedOutPosition =
        rightMotor.getPosition().getValueAsDouble(); // Store the current encoder position broom
  }

  public void maintainCurrentPosition() {
    // TODO: This code should maintain the current position of the elevator
    rightMotor.setControl(
        controlRequest.withPosition(rightMotor.getPosition().getValueAsDouble()).withSlot(0));
    // rightMotor.setPosition(0);
    // rightMotor.setControl(controlRequest.withPosition(0).withSlot(0));
    // rightMotor.setPosition(0);
    // coralCheckedOutPosition =
    //     rightMotor.getPosition().getValueAsDouble(); // Store the current encoder position broom
    // rightMotor.setControl(controlRequest.withPosition(coralCheckedOutPosition).withSlot(0));
  }

  public void spinBackSlowly() {
    rightMotor.setControl(new VelocityVoltage(Constants.FunnelConstants.SLOW_BACKWARDS_VELOCITY));
  }

  public void debugSpinBack() {
    runFunnelAtRPS(-Constants.FunnelConstants.SPEED_RPS);
  }

  public boolean isCoralCheckedIn() {
    return checkInSensor.get();
  }

  public boolean isCoralCheckedOut() {
    return checkOutSensor.get();
  }

  public boolean drakeTripped() {
    return drake.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    DogLog.log("subsystems/Funnel/CheckedInStatus", isCoralCheckedIn());
    DogLog.log("subsystems/Funnel/CheckedOutStatus", isCoralCheckedOut());
    DogLog.log("subsystems/Funnel/DrakeStatus", drakeTripped());
    // DogLog.log("subsystems/Funnel/FunnelVelocity", rightMotor.getVelocity().getValueAsDouble());
    DogLog.log("subsystems/Funnel/AbsPositionalError", getAbsolutePositionalError());
    DogLog.log(
        "subsystems/Funnel/command",
        this.getCurrentCommand() == null ? "NOTHING" : this.getCurrentCommand().getName());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
