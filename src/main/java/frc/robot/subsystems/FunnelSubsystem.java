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
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class FunnelSubsystem extends SubsystemBase {
  private static FunnelSubsystem instance;

  private LoggedTalonFX topMotor;
  private LoggedTalonFX bottomMotor;
  private DigitalInput checkOutSensor;
  private DigitalInput checkInSensor;
  private double coralCheckedOutPosition;
  private boolean coralInFunnel;
  private final MotionMagicVoltage controlRequest = new MotionMagicVoltage(0);

  public FunnelSubsystem() {
    topMotor = new LoggedTalonFX(Constants.FunnelConstants.UP_MOTOR_PORT); // Unique ID for motor1
    bottomMotor =
        new LoggedTalonFX(Constants.FunnelConstants.DOWN_MOTOR_PORT); // Unique ID for motor2
    Follower invertedfollower = new Follower(Constants.FunnelConstants.UP_MOTOR_PORT, true);
    bottomMotor.setControl(invertedfollower);

    checkOutSensor = new DigitalInput(Constants.FunnelConstants.CHECK_OUT_PORT);
    checkInSensor = new DigitalInput(Constants.FunnelConstants.CHECK_IN_PORT);

    TalonFXConfigurator m1Config = topMotor.getConfigurator();
    TalonFXConfigurator m2Config = bottomMotor.getConfigurator();

    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Constants.FunnelConstants.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.FunnelConstants.SUPPLY_CURRENT_LIMIT);

    MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(Constants.FunnelConstants.S0C_KP)
            .withKI(Constants.FunnelConstants.S0C_KI)
            .withKD(Constants.FunnelConstants.S0C_KD);

    m1Config.apply(moc);
    m2Config.apply(moc);
    m1Config.apply(clc);
    m2Config.apply(clc);

    topMotor.getConfigurator().apply(s0c);
    bottomMotor.getConfigurator().apply(s0c);

    MotionMagicConfigs mmc =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.FunnelConstants.CRUISE_VELOCITY)
            .withMotionMagicAcceleration(Constants.FunnelConstants.ACCELERATION);
    topMotor.getConfigurator().apply(mmc);
    bottomMotor.getConfigurator().apply(mmc);

    coralCheckedOutPosition = topMotor.getPosition().getValueAsDouble();
  }

  public static FunnelSubsystem getInstance() {
    if (instance == null) {
      instance = new FunnelSubsystem();
    }
    return instance;
  }

  public double getAbsolutePositionalError() {
    if (topMotor.getControlMode().getValue().equals(ControlModeValue.PositionVoltage)) {
      return Math.abs(topMotor.getPosition().getValueAsDouble() - coralCheckedOutPosition);
    } else {
      return 0.0;
    }
  }

  private void runFunnelAtRPS(double speed) {
    VelocityVoltage m_velocityControlTop =
        new VelocityVoltage(speed * Constants.FunnelConstants.TOP_GEAR_RATIO);
    topMotor.setControl(m_velocityControlTop);
  }

  public void spinFunnel() {
    runFunnelAtRPS(Constants.FunnelConstants.TOP_MOTOR_SPEED_RPS);
  }

  public void stopFunnel() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  public void maintainCurrentPosition() {
    coralCheckedOutPosition =
        topMotor.getPosition().getValueAsDouble(); // Store the current encoder position broom
    topMotor.setControl(controlRequest.withPosition(coralCheckedOutPosition).withSlot(0));
  }

  public void spinBackSlowly() {
    topMotor.setControl(new VelocityVoltage(Constants.FunnelConstants.SLOW_BACKWARDS_VELOCITY));
  }

  public boolean isCoralCheckedIn() {
    return checkInSensor.get();
  }

  public boolean isCoralCheckedOut() {
    return checkOutSensor.get();
  }

  public void setCoralInFunnel(boolean coralInFunnel) {
    this.coralInFunnel = coralInFunnel;
  }

  public boolean isCoralInFunnel() {
    return coralInFunnel;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

//
