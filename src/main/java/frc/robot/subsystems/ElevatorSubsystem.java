// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.util.LoggedTalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.mechanisms.MechanismState;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import frc.robot.Constants;
import frc.robot.commands.ElevatorLevel1;
import frc.robot.commands.ElevatorLevel2; 
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSubsystem extends SubsystemBase {
    private static ElevatorSubsystem instance;


    private LoggedTalonFX motor1;
    private LoggedTalonFX motor2;
    private LoggedTalonFX master;
    private Double holdPosValue = 0.0;

    // Mechanism visualization variables
    private Mechanism2d mech;
    private MechanismRoot2d root;
    private MechanismLigament2d m_elevator1;
    private MechanismLigament2d m_elevator2;
    private MechanismLigament2d m_wrist;

    //Elevator lengths
    public static double elevator1Length=0;
    public static double elevator2Length=0;

    public ElevatorSubsystem() {
        motor1 = new LoggedTalonFX(51); // Unique ID for motor1
        motor2 = new LoggedTalonFX(52); // Unique ID for motor2

        motor2.setControl(new Follower(51, false)); // motor1 ID as master

        TalonFXConfigurator m1Config = motor1.getConfigurator();
        TalonFXConfigurator m2Config = motor2.getConfigurator();

        CurrentLimitsConfigs clc = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Constants.ElevatorConstants.STATOR_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Constants.ElevatorConstants.SUPPLY_CURRENT_LIMIT);

        MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
        Slot0Configs s0c = new Slot0Configs()
                .withKP(Constants.ElevatorConstants.S0C_KP)
                .withKI(Constants.ElevatorConstants.S0C_KI)
                .withKD(Constants.ElevatorConstants.S0C_KD);

        m1Config.apply(moc);
        m2Config.apply(moc);
        m1Config.apply(clc);
        m2Config.apply(clc);

        master = motor1;
        master.getConfigurator().apply(s0c);

        MotionMagicConfigs mmc = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(Constants.ElevatorConstants.CRUISE_VELOCITY)
                .withMotionMagicAcceleration(Constants.ElevatorConstants.ACCELERATION);
        master.getConfigurator().apply(mmc);

        // Initialize Mechanism2d visualization
        mech = new Mechanism2d(3, 3);
        root = mech.getRoot("climber", 2, 250);
        m_elevator1 = root.append(new MechanismLigament2d("elevator1", Constants.ElevatorConstants.kElevatorMinimumLength, 90, 6, new Color8Bit(Color.kRed)));
        m_elevator2 = root.append(new MechanismLigament2d("elevator2", Constants.ElevatorConstants.kElevatorMinimumLength2, 90, 6, new Color8Bit(Color.kBlue)));
        m_wrist = m_elevator2.append(
                new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));
        SmartDashboard.putData("Elevator Mechanism", mech);
    }

    public static ElevatorSubsystem getInstance() {
        if (instance == null) {
            instance = new ElevatorSubsystem();
        }
        return instance;
    }

    public void elevate(double speed) {
        master.setControl(new MotionMagicVoltage(speed));
    }

    public double getSpeed() {
        return master.getVelocity().getValueAsDouble();
    }

    public boolean getStopped() {
        return getSpeed() == 0;
    }

    public void holdPosition() {
        holdPosition(holdPosValue);
    }

    public void holdPosition(double pos) {
        master.setControl(new MotionMagicVoltage(pos));
    }

    public double getPIDError() {
        return master.getClosedLoopError().getValueAsDouble();
    }

    public void setHoldPos() {
        holdPosValue = master.getPosition().getValueAsDouble();
    }

    public void resetEncoderPos() {
        master.setPosition(0);
    }

    public boolean isAtSetpoint() {
        return Math.abs(getPIDError()) < Constants.ElevatorConstants.SETPOINT_TOLERANCE;
    }

    public boolean getBottomLimits() {
        return master.getSupplyCurrent().getValueAsDouble() > Constants.ElevatorConstants.STATOR_CURRENT_LIMIT;
    }

    public double getEncoderPos() {
        return master.getPosition().getValueAsDouble();
    }

    public void setLevelOfElevator(int level) {
        double targetPosition = switch (level) {
            case 1 -> Constants.ElevatorConstants.level1;
            case 2 -> Constants.ElevatorConstants.level2;
            case 3 -> Constants.ElevatorConstants.level3;
            case 4 -> Constants.ElevatorConstants.level4;
            default -> throw new IllegalArgumentException("Invalid level: " + level);
        };

        holdPosition(targetPosition);
        SmartDashboard.putString("Elevator Error", "Level: " + level + ", Error: " + getPIDError());
    }

    // Methods to control the elevators
    public void moveElevator1(double length) {
        elevator1Length = length;
        m_elevator1.setLength(elevator1Length);
    }

    public void moveElevator2(double length) {
        elevator2Length = length;
        m_elevator2.setLength(elevator2Length);
    }  

    @Override
    public void periodic() {
        if (getBottomLimits()) {
            resetEncoderPos();
            holdPosValue = 0.5;
            master.setControl(new MotionMagicVoltage(0));
        }
        DogLog.log("HoldPosValue", holdPosValue);

        elevator1Length = master.getPosition().getValueAsDouble();
        elevator2Length = master.getPosition().getValueAsDouble();

        m_elevator1.setLength(elevator1Length);
        m_elevator2.setLength(elevator2Length);
        
        double wristAngle = Constants.ElevatorConstants.m_wristPot;
        m_wrist.setAngle(Constants.ElevatorConstants.m_wristPot);

        SmartDashboard.putData("Elevator Mechanism", mech);

        SmartDashboard.putNumber("Elevator 1 Length", elevator1Length);
        SmartDashboard.putNumber("Elevator 2 Length", elevator1Length);
        SmartDashboard.putNumber("Wrist Angle", wristAngle);
    }

    @Override
    public void simulationPeriodic() {
        double simulatedSpeed = master.getVelocity().getValueAsDouble();
        double currentPosition = master.getPosition().getValueAsDouble();

        double newPosition = currentPosition + simulatedSpeed * 0.02; // Assuming a 20ms loop
        master.setPosition(newPosition);

        SmartDashboard.putNumber("Simulated Position", newPosition);
        SmartDashboard.putNumber("Simulated Speed", simulatedSpeed);
    }



    // public void handleKeyPress(String key) {
    //   switch (key) {
    //       case "1": // Move Elevator1 to Level 1
    //           moveElevator2(Constants.ElevatorConstants.level1);
    //           break;
    //       case "2": // Move Elevator1 to Level 2
    //           moveElevator2(Constants.ElevatorConstants.level2);
    //           break;
    //       case "3": // Move Elevator2 up
    //           moveElevator2(Constants.ElevatorConstants.level3);
    //           break;
    //       case "4": // Move Elevator2 down
    //           moveElevator2(Constants.ElevatorConstants.level4);
    //           break;
    //   }
      
    // }
}