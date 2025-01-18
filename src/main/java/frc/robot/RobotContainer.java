// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
// import frc.robot.subsystems.SwerveSubsystemlliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.ArmToAngleCmd;
import frc.robot.subsystems.NewArmSubsystem;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;

// import frc.robot.subsystems.SwerveSubsystem;
public class RobotContainer {
  private static Matrix<N3, N1> visionMatrix = VecBuilder.fill(0.01, 0.03d, 100d);
  private static Matrix<N3, N1> odometryMatrix = VecBuilder.fill(0.1, 0.1, 0.1);

  // Alliance color
  private Supplier<Boolean> redside = () -> redAlliance;
  private static boolean redAlliance;

  // private final SwerveSubsystem driveTrain = new SwerveSubsystem(
  //     Constants.Swerve.DrivetrainConstants,
  //     250.0, // TODO: CHANGE ODOMETRY UPDATE FREQUENCY TO CONSTANT,
  //     odometryMatrix,
  //     visionMatrix,
  //     Constants.Swerve.FrontLeft,
  //     Constants.Swerve.FrontRight,
  //     Constants.Swerve.BackLeft,
  //     Constants.Swerve.BackRight
  // );

  // private final Telemetry logger = new
  // Telemetry(Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
  private final CommandXboxController joystick = new CommandXboxController(0);

  // Starts telemetry operations (essentially logging -> look on SmartDashboard, AdvantageScope)
  //
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Joystick suppliers,

    // Trigger leftShoulderTrigger = joystick.leftBumper();
    // leftShoulderTrigger.onTrue(new ArmToAngleCmd(() -> 90d, NewArmSubsystem.getInstance()));
    // leftShoulderTrigger.onFalse(new ArmToAngleCmd(() -> 45d, NewArmSubsystem.getInstance()));
    // NewArmSubsystem.getInstance()));

    joystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    joystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    /*
    * Joystick Y = quasistatic forward
    * Joystick A = quasistatic reverse
    * Joystick B = dynamic forward
    * Joystick X = dyanmic reverse
    */
    joystick.y().whileTrue(NewArmSubsystem.getInstance().sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    joystick.a().whileTrue(NewArmSubsystem.getInstance().sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    joystick.b().whileTrue(NewArmSubsystem.getInstance().sysIdDynamic(SysIdRoutine.Direction.kForward));
    joystick.x().whileTrue(NewArmSubsystem.getInstance().sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // joystick.y().whileTrue(driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // joystick.a().whileTrue(driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // joystick.b().whileTrue(driveTrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // joystick.x().whileTrue(driveTrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  // public static void setAlliance() {
  //     redAlliance =
  //         (DriverStation.getAlliance().isEmpty())
  //             ? false
  //             : (DriverStation.getAlliance().get() == Alliance.Red);
  // }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    SmartDashboard.putNumber("Nikash's IQ which is muy mucho high", 2.0);
    return new WaitCommand(10);
  }
}
