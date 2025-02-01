// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;


import frc.robot.util.LoggedTalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
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
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import dev.doglog.DogLog;

public class ElevatorSubsystem extends SubsystemBase {
   private static ElevatorSubsystem instance;

   private final LoggedTalonFX motor1;
   private final LoggedTalonFX motor2;
   private final LoggedTalonFX master;
   private double holdPosValue = 0.0;

   // Mechanism visualization variables
   private final Mechanism2d mech;
   private final MechanismRoot2d root;
   private final MechanismLigament2d m_elevator1;
   private final MechanismLigament2d m_elevator2;
   private final MechanismLigament2d m_wrist;

   // Elevator lengths  
   private static double elevator1Length = 5;
   private static double elevator2Length = 10;
   private static double elevator3Length = 15;
   private static double elevator4Length = 20;
   private double targetPosition = 0.0;

   //sensors
     Ultrasonic m_rangeFinder = new Ultrasonic(1, 2);
    // private static double distanceInches = m_rangeFinder.getRangeInches();




   private ElevatorSubsystem() {
       // Motor initialization
       motor1 = new LoggedTalonFX(51);
       motor2 = new LoggedTalonFX(52);
       master = motor1;


       motor2.setControl(new Follower(51, false)); // motor1 as master


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
               .withKD(Constants.ElevatorConstants.S0C_KD); // tune all these based on testing 


       m1Config.apply(moc);
       m2Config.apply(moc);
       m1Config.apply(clc);
       m2Config.apply(clc);
       master.getConfigurator().apply(s0c);

       MotionMagicConfigs mmc = new MotionMagicConfigs()
               .withMotionMagicCruiseVelocity(Constants.ElevatorConstants.CRUISE_VELOCITY)
               .withMotionMagicAcceleration(Constants.ElevatorConstants.ACCELERATION);
       master.getConfigurator().apply(mmc);


       // Initialize Mechanism2d visualization
       mech = new Mechanism2d(4, 4);
       root = mech.getRoot("elevator", 2, 2);
       m_elevator1 = root.append(new MechanismLigament2d("Elevator1", 0, 90, 6, new Color8Bit(Color.kRed)));
       m_elevator2 = root.append(new MechanismLigament2d("Elevator2", 0, 90, 6, new Color8Bit(Color.kBlue)));
       m_wrist = m_elevator2.append(new MechanismLigament2d("Wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));


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

   public boolean isStopped() {
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

   public boolean isAtBottomLimit() {
       return master.getSupplyCurrent().getValueAsDouble() > Constants.ElevatorConstants.STATOR_CURRENT_LIMIT;
   }

   public double getEncoderPos() {
       return master.getPosition().getValueAsDouble();
   }

   public void setLevelOfElevator(int level) {
    targetPosition = switch (level) {
        case 1 -> Constants.ElevatorConstants.level1;
        case 2 -> Constants.ElevatorConstants.level2;
        case 3 -> Constants.ElevatorConstants.level3;
        case 4 -> Constants.ElevatorConstants.level4;
        default -> throw new IllegalArgumentException("Invalid level: " + level);
    };

    holdPosition(targetPosition);
    SmartDashboard.putString("Elevator Error", "Level: " + level + ", Error: " + getPIDError());
}


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
    elevator1Length = master.getPosition().getValueAsDouble();
    elevator2Length = master.getPosition().getValueAsDouble();

    m_elevator1.setLength(elevator1Length);
    m_elevator2.setLength(elevator2Length);

    SmartDashboard.putNumber("Elevator1 Length", elevator1Length);
    SmartDashboard.putNumber("Elevator2 Length", elevator2Length);
    SmartDashboard.putBoolean("At Target", isAtSetpoint());
    DogLog.log("HoldPosValue", holdPosValue);

    // Check if at the target and allow for next command
    if (isAtSetpoint()) {
        // Optionally trigger actions when the target is reached
        SmartDashboard.putString("Elevator Status", "Target Reached");
    }
}


    private void keyPressed() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'keyPressed'");
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
}