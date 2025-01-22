// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
// Simulation imports
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.LoggedTalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.compound.Diff_DutyCycleOut_Velocity;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import frc.robot.Constants;


public class TootsieSlideSubsystem extends SubsystemBase{
    private static TootsieSlideSubsystem instance;
    
    private DigitalInput coralSensor;
    private LoggedTalonFX upMotor;
    private LoggedTalonFX downMotor;
    private LoggedTalonFX master;

    private final DCMotor m_tootsieSlideGearbox = DCMotor.getKrakenX60(1);

    public static LinearSystem<N1,N1,N1> tootsieSystem = LinearSystemId.identifyVelocitySystem(1.0 , 1.0);

    private final FlywheelSim m_flywheelSim = new FlywheelSim(tootsieSystem, m_tootsieSlideGearbox, 1.0);


  public TootsieSlideSubsystem() {

    
    upMotor = new LoggedTalonFX(1); // Unique ID for motor1
    downMotor = new LoggedTalonFX(2); // Unique ID for motor2

    downMotor.setControl(new Follower(1, false)); // motor1 ID as master

    TalonFXConfigurator m1Config = upMotor.getConfigurator();
    TalonFXConfigurator m2Config = downMotor.getConfigurator();

    CurrentLimitsConfigs clc = new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Constants.TootsieSlide.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.TootsieSlide.SUPPLY_CURRENT_LIMIT);

    MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    Slot0Configs s0c = new Slot0Configs()
            .withKP(Constants.TootsieSlide.S0C_KP)
            .withKI(Constants.TootsieSlide.S0C_KI)
            .withKD(Constants.TootsieSlide.S0C_KD);

    m1Config.apply(moc);
    m2Config.apply(moc);
    m1Config.apply(clc);
    m2Config.apply(clc);

    master = upMotor;
    master.getConfigurator().apply(s0c);

    MotionMagicConfigs mmc = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.TootsieSlide.CRUISE_VELOCITY)
            .withMotionMagicAcceleration(Constants.TootsieSlide.ACCELERATION);
    master.getConfigurator().apply(mmc);
}

public static TootsieSlideSubsystem getInstance() {
    if (instance == null) {
        instance = new TootsieSlideSubsystem();
    }
    return instance;

  }


  public void setPosition(double pos) {
    master.setControl(new MotionMagicVoltage(pos));
    m_flywheelSim.setInputVoltage(master.getMotorVoltage().getValue().magnitude());
    }

  private void runTootsieAtRPS(double speed) {
    VelocityVoltage m_velocityControl =
        new VelocityVoltage(speed * Constants.TootsieSlide.GEAR_RATIO);
    master.setControl(m_velocityControl);

    m_flywheelSim.setInputVoltage(master.getMotorVoltage().getValue().magnitude());
  }

  public void spinTootsie() {
    runTootsieAtRPS(Constants.TootsieSlide.SPEED_RPS);
  }

   public void stopTootsie() {
    master.stopMotor();
  }

    // Simulation



  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    // m_flywheelSim.setInputVolatge()

    // // Next, we update it. The standard loop time is 20ms.
    // m_flywheelSim.update(0.020);

    // // Finally, we set our simulated encoder's readings and simulated battery voltage
    // m_flywheelSim.setDistance(m_flywheelSim.getPositionMeters());
    // // SimBattery estimates loaded battery voltages
    // RoboRioSim.setVInVoltage(
    //     BatterySim.calculateDefaultBatteryLoadedVoltage(m_flywheelSim.getCurrentDrawAmps()));
    SmartDashboard.putNumber("Flywheel Velocity", m_flywheelSim.getAngularVelocityRadPerSec());
  }

public boolean coralPresent() {
  return !coralSensor.get();
}
}