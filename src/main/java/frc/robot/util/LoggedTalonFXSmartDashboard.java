package frc.robot.util;

import java.util.ArrayList;

import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LoggedTalonFXSmartDashboard extends TalonFX {

    private static ArrayList<LoggedTalonFXSmartDashboard> motors = new ArrayList<>();
    private String name;
    private String temperature,closedLoopError,closedLoopReference,position,velocity,acceleration,supplycurrent,statorcurrent,torquecurrent,motorvoltage,supplyvoltage;

    public LoggedTalonFXSmartDashboard(String deviceName,int deviceId, String canbus) {
        super(deviceId, canbus);
        init();
        name = deviceName;
    }

        public LoggedTalonFXSmartDashboard(String deviceName,int deviceId) {
        super(deviceId);
        init();
        name = deviceName;
    }

    public LoggedTalonFXSmartDashboard(int deviceId, String canbus) {
        super(deviceId, canbus);
        init();
        name = "motor "+deviceId;
    }

    public LoggedTalonFXSmartDashboard(int deviceId) {
        super(deviceId);
        init();
        name = "motor "+deviceId;
    }

    public void init(){
        motors.add(this);
        this.temperature=name + "/temperature(degC)";
        this.closedLoopError = name + "/closedLoopError";
        this.closedLoopReference = name + "/closedLoopReference";
        this.position=name + "/position(rotations)";
        this.velocity=name + "/velocity(rps)";
        this.acceleration=name + "/acceleration(rps2)";
        this.supplycurrent=name + "/current/supply(A)";
        this.statorcurrent=name + "/current/stator(A)";
        this.torquecurrent=name + "/current/torque(A)";
        this.motorvoltage=name + "/voltage/motor(V)";
        this.supplyvoltage=name + "/voltage/supply(V)";
    }

    // For some reason Robot.java doesn't recognize the static method here
    // when there is another method with the same name
    public static void periodic_static (){
        for(LoggedTalonFXSmartDashboard l: motors){
            l.periodic();
        }
    }
    
    public void periodic (){
        // TODO: LABEL WHAT UNITS THESE ARE IN
        SmartDashboard.putNumber(temperature,this.getDeviceTemp().getValue().magnitude());
        SmartDashboard.putNumber(closedLoopError,this.getClosedLoopError().getValue());
        SmartDashboard.putNumber(closedLoopReference,this.getClosedLoopReference().getValue());

        // TODO: LABEL WHAT UNITS THESE ARE IN
        SmartDashboard.putNumber(position,this.getPosition().getValue().magnitude());
        SmartDashboard.putNumber(velocity,this.getVelocity().getValue().magnitude());
        SmartDashboard.putNumber(acceleration,this.getAcceleration().getValue().magnitude());

        //Current
        // TODO: LABEL WHAT UNITS THESE ARE IN
        SmartDashboard.putNumber(supplycurrent,this.getSupplyCurrent().getValue().magnitude());
        SmartDashboard.putNumber(statorcurrent,this.getStatorCurrent().getValue().magnitude());
        SmartDashboard.putNumber(torquecurrent,this.getTorqueCurrent().getValue().magnitude());

        //Voltage
        // TODO: LABEL WHAT UNITS THESE ARE IN
        SmartDashboard.putNumber(motorvoltage,this.getMotorVoltage().getValue().magnitude());
        SmartDashboard.putNumber(supplyvoltage,this.getSupplyVoltage().getValue().magnitude());
    }

}


