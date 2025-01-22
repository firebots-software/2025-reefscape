package frc.robot.util;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import java.util.ArrayList;

public class LoggedTalonFX extends TalonFX {

  private static ArrayList<LoggedTalonFX> motors = new ArrayList<>();
  private String name;
  private boolean refresh;
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
    this.refresh = false;

    // Applying current limits
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
    DogLog.log(temperature, this.getDeviceTemp(refresh).getValue().magnitude());
    DogLog.log(closedLoopError, this.getClosedLoopError(refresh).getValue());
    DogLog.log(closedLoopReference, this.getClosedLoopReference(refresh).getValue());

    DogLog.log(position, this.getPosition(refresh).getValue().magnitude());
    DogLog.log(velocity, this.getVelocity(refresh).getValue().magnitude());
    DogLog.log(acceleration, this.getAcceleration(refresh).getValue().magnitude());

    // Current
    DogLog.log(supplycurrent, this.getSupplyCurrent(refresh).getValue().magnitude());
    DogLog.log(statorcurrent, this.getStatorCurrent(refresh).getValue().magnitude());
    DogLog.log(torquecurrent, this.getTorqueCurrent(refresh).getValue().magnitude());

    // Voltage
    DogLog.log(motorvoltage, this.getMotorVoltage(refresh).getValue().magnitude());
    DogLog.log(supplyvoltage, this.getSupplyVoltage(refresh).getValue().magnitude());
  }
}
