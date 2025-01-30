// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
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
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.compound.Diff_DutyCycleOut_Velocity;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import frc.robot.Constants;


public class ElevatorSubsystem extends SubsystemBase{
    private static ElevatorSubsystem instance;
    
    private DigitalInput drakeSensor;
    private LoggedTalonFX upMotor;
    private LoggedTalonFX downMotor;
    private LoggedTalonFX master;
    private boolean coralDetected = false;


    public static LinearSystem<N1,N1,N1> ElevatorSystem = LinearSystemId.identifyVelocitySystem(0.05, 0.1);

    private final VelocityVoltage m_velocity = new VelocityVoltage(0);

  public ElevatorSubsystem() {

    upMotor = new LoggedTalonFX(1); // Unique ID for motor1
    downMotor = new LoggedTalonFX(2); // Unique ID for motor2

    downMotor.setControl(new Follower(1, false)); // motor1 ID as master



    TalonFXConfigurator m1Config = upMotor.getConfigurator();
    TalonFXConfigurator m2Config = downMotor.getConfigurator();

    CurrentLimitsConfigs clc = new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Constants.Elevator.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.Elevator.SUPPLY_CURRENT_LIMIT);

    Slot0Configs s0c = new Slot0Configs()
            .withKP(Constants.Elevator.S0C_KP)
            .withKI(Constants.Elevator.S0C_KI)
            .withKD(Constants.Elevator.S0C_KD);

    m_velocity.Slot = 0;
    m1Config.apply(clc);
    m2Config.apply(clc);

    master = upMotor;
    master.getConfigurator().apply(s0c);
    master.getConfigurator().apply(clc);



    // MotionMagicConfigs mmc = new MotionMagicConfigs()
    //          .withMotionMagicCruiseVelocity(Constants.Elevator.CRUISE_VELOCITY)
    //          .withMotionMagicAcceleration(Constants.Elevator.ACCELERATION);
    //  master.getConfigurator().apply(mmc);
}

public static ElevatorSubsystem getInstance() {
    if (instance == null) {
        instance = new ElevatorSubsystem();
    }
    return instance;

  }


  public void setPosition(double pos) {
    master.setControl(new MotionMagicVoltage(pos));
  }

  private void runElevatorAtRPS(double speed) {
    SmartDashboard.putBoolean("is it running", true);
    master.setControl(m_velocity.withVelocity(speed));
    SmartDashboard.putNumber("VelocityVoltage", master.getMotorVoltage().getValueAsDouble());
  }

  public void spinElevator(boolean checkOutSensor) {
    if(!checkOutSensor && coralDetected){
      runElevatorAtRPS(30);
    }
  }

   public void stopElevator() {
    master.stopMotor();
    master.setControl(m_velocity.withVelocity(0));

  }

    // Simulation



  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean atTarget(boolean checkingIfAtTop) {
    boolean thingToReturn = false;
    // Query some boolean state, such as a digital sensor.
    if(thingToReturn){
      coralDetected = false;
    }
    return false;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(coralPresent()){
      coralDetected = true;
    }

  }

  @Override
  public void simulationPeriodic() {

  }

public boolean coralPresent() {
  return drakeSensor.get();
}
}