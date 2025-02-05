// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class FunnelSubsystem extends SubsystemBase {
  private static FunnelSubsystem instance;

  // The left and right motor of the funnel intake (should be renamed accordingly)
  private LoggedTalonFX topMotor;
  private LoggedTalonFX bottomMotor;

  // The Check-out sensor is closer to the elevator, and is in charge of detecting a coral and
  // stopping the motors to prevent it
  // from going into the elevator.
  private DigitalInput checkOutSensor;

  // The Check-in sensor is closer to the back of the robot, and detects the coral before the
  // Check-out sensor. It is in charge
  // of making sure the coral makes it into the intake funnel, before letting the robot go on from
  // the Coral Station.
  private DigitalInput checkInSensor;

  public FunnelSubsystem() {
    topMotor = new LoggedTalonFX(1); // Unique ID for motor1
    bottomMotor = new LoggedTalonFX(2); // Unique ID for motor2

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
  }

  public static FunnelSubsystem getInstance() {
    if (instance == null) {
      instance = new FunnelSubsystem();
    }
    return instance;
  }

  private void runFunnelAtRPS(double speed) {
    VelocityVoltage m_velocityControlTop =
        new VelocityVoltage(speed * Constants.FunnelConstants.TOP_GEAR_RATIO);
    VelocityVoltage m_velocityControlBottom =
        new VelocityVoltage(speed * Constants.FunnelConstants.BOTTOM_GEAR_RATIO);
    topMotor.setControl(m_velocityControlTop);
    bottomMotor.setControl(m_velocityControlBottom);
  }

  /** Spin the funnel intake motors at the default speeds */
  public void spinFunnel() {
    runFunnelAtRPS(Constants.FunnelConstants.TOP_MOTOR_SPEED_RPS);
    runFunnelAtRPS(Constants.FunnelConstants.BOTTOM_MOTOR_SPEED_RPS);
  }

  /** Stop both funnel intake motors */
  public void stopFunnel() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  /**
   * Returns whether or not the Check-out sensor is detecting an object
   *
   * @return Boolean - whether the Check-out sensor is detecting an object
   */
  public boolean isCoralCheckedOut() {
    return checkOutSensor.get();
  }

  /**
   * Returns whether or not the Check-in sensor is detecting an object
   *
   * @return Boolean - whether the Check-in sensor is detecting an object
   */
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
