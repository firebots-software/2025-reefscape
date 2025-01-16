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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.ArmToAngleCmd;
import frc.robot.subsystems.NewArmSubsystem;
import java.util.function.Supplier;

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

    Trigger leftShoulderTrigger = joystick.leftBumper();
    leftShoulderTrigger.whileTrue(new ArmToAngleCmd(() -> 270.0, NewArmSubsystem.getInstance()));
    // leftShoulderTrigger.whileFalse(new ArmToAngleCmd(() -> 220.0,
    // NewArmSubsystem.getInstance()));

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
