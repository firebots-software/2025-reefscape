// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
// Simulation imports
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class TootsieSlideSubsystem extends SubsystemBase {
  private static TootsieSlideSubsystem instance;

  public LoggedTalonFX master;

  private final VelocityVoltage m_velocity = new VelocityVoltage(0);

  private final MotionMagicVoltage voltage = new MotionMagicVoltage(0);

  // FOR SIMULATION
  private final DCMotor m_tootsieSlideGearbox = DCMotor.getKrakenX60(1);
  public static LinearSystem<N1, N1, N1> tootsieSystem =
      LinearSystemId.identifyVelocitySystem(0.05, 0.1);
  private final FlywheelSim m_flywheelSim =
      new FlywheelSim(tootsieSystem, m_tootsieSlideGearbox, 0);

  private TootsieSlideSubsystem() {

    master =
        new LoggedTalonFX(
            "subsystems/tootsieslide/motor",
            Constants.TootsieSlide.MOTOR_PORT); // Unique ID for motor
    TalonFXConfigurator m1Config = master.getConfigurator();
    MotorOutputConfigs moc =
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);

    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Constants.TootsieSlide.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.TootsieSlide.SUPPLY_CURRENT_LIMIT);

    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(Constants.TootsieSlide.S0C_KP)
            .withKI(Constants.TootsieSlide.S0C_KI)
            .withKD(Constants.TootsieSlide.S0C_KD);

    m_velocity.Slot = 0;
    m1Config.apply(clc);
    master.getConfigurator().apply(s0c);
    master.getConfigurator().apply(clc);
    m1Config.apply(moc);
  }

  public static TootsieSlideSubsystem getInstance() {
    if (instance == null) {
      instance = new TootsieSlideSubsystem();
    }
    return instance;
  }

  private void runTootsieAtRPS(double flywheelSpeed) {
    double motor_speed = flywheelSpeed / Constants.TootsieSlide.GEAR_RATIO;
    DogLog.log("subsystems/tootsieslide/Target Motor Speed", motor_speed);
    master.setControl(m_velocity.withVelocity(motor_speed));

    m_flywheelSim.setInputVoltage(master.getSupplyVoltage().getValueAsDouble());
  }

  public void intakeCoral() {
    runTootsieAtRPS(
        Constants.TootsieSlide.INTAKE_SPEED_RPS); // TODO: Change based on speed to intake Coral at
  }

  public void shootTootsie() {
    runTootsieAtRPS(Constants.TootsieSlide.SHOOTING_SPEED_RPS);
  }

  public void stopTootsie() {
    master.setPosition(0);
    master.setControl(voltage.withPosition(0));
    master.setPosition(0);

    m_flywheelSim.setInputVoltage(0);
  }

  @Override
  public void periodic() {
    DogLog.log("subsystems/tootsieslide/tootsieVelocity", master.getVelocity().getValueAsDouble());
    DogLog.log(
        "subsystems/tootsieslide/command",
        this.getCurrentCommand() == null ? "NOTHING" : this.getCurrentCommand().getName());
  }

  @Override
  public void simulationPeriodic() {
    m_flywheelSim.update(0.02);
    DogLog.log("SIMvoltage", m_flywheelSim.getInputVoltage());
    DogLog.log("Shooter Speed at RPM", m_flywheelSim.getAngularVelocityRPM());
    DogLog.log("Flywheel Current", m_flywheelSim.getCurrentDrawAmps());
  }
}
