package frc.robot.util;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;

public class LoggedTalonFX extends TalonFX {

  private static ArrayList<LoggedTalonFX> motors = new ArrayList<>();
  private String name;
  private String temperature,
      closedLoopError,
      closedLoopReference,
      position,
      velocity,
      acceleration,
      supplycurrent,
      statorcurrent,
      torquecurrent,
      motorvoltage,
      supplyvoltage;

  public LoggedTalonFX(String deviceName, int deviceId, String canbus) {
    super(deviceId, canbus);
    init();
    name = deviceName;
  }

  public LoggedTalonFX(String deviceName, int deviceId) {
    super(deviceId);
    init();
    name = deviceName;
  }

  public LoggedTalonFX(int deviceId, String canbus) {
    super(deviceId, canbus);
    init();
    name = "motor " + deviceId;
  }

  public LoggedTalonFX(int deviceId) {
    super(deviceId);
    init();
    name = "motor " + deviceId;
  }

  public void init() {
    motors.add(this);
    this.temperature = name + "/temperature(degC)";
    this.closedLoopError = name + "/closedLoopError";
    this.closedLoopReference = name + "/closedLoopReference";
    this.position = name + "/position(rotations)";
    this.velocity = name + "/velocity(rps)";
    this.acceleration = name + "/acceleration(rps2)";
    this.supplycurrent = name + "/current/supply(A)";
    this.statorcurrent = name + "/current/stator(A)";
    this.torquecurrent = name + "/current/torque(A)";
    this.motorvoltage = name + "/voltage/motor(V)";
    this.supplyvoltage = name + "/voltage/supply(V)";

    //Applying current limits
    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40); 
      // WITH A HIGH POWER MECHANISM, MAKE SURE TO INCREASE THE CURRENT LIMITS
    this.getConfigurator().apply(clc);
  }

  public void updateCurrentLimits(double statorCurrentLimit, double supplyCurrentLimit) {
    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(statorCurrentLimit)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(supplyCurrentLimit); 
    
    this.getConfigurator().apply(clc);
  }

  // For some reason Robot.java doesn't recognize the static method here
  // when there is another method with the same name
  public static void periodic_static() {
    for (LoggedTalonFX l : motors) {
      l.periodic();
    }
  }

  public void periodic() {
    // TODO: LABEL WHAT UNITS THESE ARE IN
    DogLog.log(temperature, this.getDeviceTemp().getValue().magnitude());
    DogLog.log(closedLoopError, this.getClosedLoopError().getValue());
    DogLog.log(closedLoopReference, this.getClosedLoopReference().getValue());

    // TODO: LABEL WHAT UNITS THESE ARE IN
    DogLog.log(position, this.getPosition().getValue().magnitude());
    DogLog.log(velocity, this.getVelocity().getValue().magnitude());
    DogLog.log(acceleration, this.getAcceleration().getValue().magnitude());

    // Current
    // TODO: LABEL WHAT UNITS THESE ARE IN
    DogLog.log(supplycurrent, this.getSupplyCurrent().getValue().magnitude());
    DogLog.log(statorcurrent, this.getStatorCurrent().getValue().magnitude());
    DogLog.log(torquecurrent, this.getTorqueCurrent().getValue().magnitude());

    // Voltage
    // TODO: LABEL WHAT UNITS THESE ARE IN
    DogLog.log(motorvoltage, this.getMotorVoltage().getValue().magnitude());
    DogLog.log(supplyvoltage, this.getSupplyVoltage().getValue().magnitude());
  }
}
