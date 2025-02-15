// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import dev.doglog.DogLog;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
// Simulation imports
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TootsieSlide;
import frc.robot.util.LoggedTalonFX;

public class TootsieSlideSubsystem extends SubsystemBase {
  private static TootsieSlideSubsystem instance;

  //private DigitalInput drakeSensor;
  private LoggedTalonFX master;
  private final VelocityVoltage m_velocity = new VelocityVoltage(0);

  // FOR SIMULATION
  private final DCMotor m_tootsieSlideGearbox = DCMotor.getKrakenX60(1);
  public static LinearSystem<N1, N1, N1> tootsieSystem =
      LinearSystemId.identifyVelocitySystem(0.05, 0.1);
  private final FlywheelSim m_flywheelSim =
      new FlywheelSim(tootsieSystem, m_tootsieSlideGearbox, 0);

  private TootsieSlideSubsystem() {
    master = new LoggedTalonFX(1); // Unique ID for motor
    TalonFXConfigurator m1Config = master.getConfigurator();
    //drakeSensor = new DigitalInput(TootsieSlide.CHECKOUT_PORT);
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
  }

  public static TootsieSlideSubsystem getInstance() {
    if (instance == null) {
      instance = new TootsieSlideSubsystem();
    }
    return instance;
  }

  public void setPosition(double pos) {
    master.setControl(new MotionMagicVoltage(pos));
  }

  private void runTootsieAtRPS(double speed) {
    DogLog.log("is it running", true);
    master.setControl(m_velocity.withVelocity(speed));
    DogLog.log("VelocityVoltage", master.getMotorVoltage().getValueAsDouble());
    m_flywheelSim.setInputVoltage(master.getSupplyVoltage().getValueAsDouble());
  }

  public void intakeCoral() {
    runTootsieAtRPS(1); // TODO: Change based on speed to intake Coral at
  }

  public void spinTootsie(boolean thing) {
    if (!thing) {
      runTootsieAtRPS(30);
    }
  }

  public void shootTootsie() {
    runTootsieAtRPS(30);
  }

  public void stopTootsie() {
    master.setControl(m_velocity.withVelocity(0));
    master.stopMotor();
    m_flywheelSim.setInputVoltage(0);
  }

  // Simulation

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean atTarget() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    DogLog.log("subsystems/tootsieslide/tootsieVelocity", master.getVelocity().getValueAsDouble());
    //DogLog.log("subsystems/tootsieslide/coralPresent", coralPresent());
  }

  @Override
  public void simulationPeriodic() {
    m_flywheelSim.update(0.02);
    DogLog.log("SIMvoltage", m_flywheelSim.getInputVoltage());
    DogLog.log("Shooter Speed at RPM", m_flywheelSim.getAngularVelocityRPM());
    DogLog.log("Flywheel Current", m_flywheelSim.getCurrentDrawAmps());
  }

  // public boolean coralPresent() {
  //   return drakeSensor.get();
  // }
}
