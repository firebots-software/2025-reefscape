// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.LoggedTalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

public class TootsieSlideSubsystem extends SubsystemBase {
    private static TootsieSlideSubsystem instance;
    
    private DigitalInput coralSensor;
    private LoggedTalonFX upMotor;
    private LoggedTalonFX downMotor;
    private LoggedTalonFX master;

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
  }

  private void runTootsieAtRPS(double speed) {
    VelocityVoltage m_velocityControl =
        new VelocityVoltage(speed * Constants.TootsieSlide.GEAR_RATIO);
    master.setControl(m_velocityControl);
  }

  public void spinTootsie() {
    runTootsieAtRPS(Constants.TootsieSlide.SPEED_RPS);
  }

   public void stopTootsie() {
    master.stopMotor();
  }


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
    // This method will be called once per scheduler run during simulation
  }

public boolean coralPresent() {
  return !coralSensor.get();
}
}