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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

  public FunnelSubsystem() {
    rightMotor = new LoggedTalonFX(FunnelConstants.RIGHT_MOTOR_PORT); // Unique ID for motor1
    leftMotor = new LoggedTalonFX(2); // Unique ID for motor2

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

    m1Config.apply(s0c);
    m2Config.apply(s0c);

    MotionMagicConfigs mmc =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.FunnelConstants.CRUISE_VELOCITY)
            .withMotionMagicAcceleration(Constants.FunnelConstants.ACCELERATION);
            
    m1Config.apply(mmc);
    m2Config.apply(mmc);
  }

  public static FunnelSubsystem getInstance() {
    if (instance == null) {
      instance = new FunnelSubsystem();
    }
    return instance;
  }

  private void runFunnelAtRPS(double speed) {
    VelocityVoltage m_velocityControlTop =
        new VelocityVoltage(speed * Constants.FunnelConstants.GEAR_RATIO);
    rightMotor.setControl(m_velocityControlTop);
  }

  public void spinFunnel() {
    runFunnelAtRPS(Constants.FunnelConstants.SPEED_RPS);
  }

  public void stopFunnel() {
    rightMotor.stopMotor();
  }
  public void moveBackFlywheel(){
    //TODO: Fix this bro lmao l bozo lol rofl haha hee hee hee haw
    double randomAngleTBD = 0.0d;
      rightMotor.setControl(
          new PositionVoltage(randomAngleTBD)); 
  }
  public boolean isCoralCheckedOut() {
    return checkOutSensor.get();
  }

  public boolean isCoralCheckedIn() {
    return checkInSensor.get();
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
