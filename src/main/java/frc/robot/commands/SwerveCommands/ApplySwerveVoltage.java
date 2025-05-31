package frc.robot.commands.SwerveCommands;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ApplySwerveVoltage extends Command {
  protected final SwerveSubsystem swerveDrivetrain;
  double applyVoltage;
  static int counter = 0;

  public ApplySwerveVoltage(SwerveSubsystem swerveSubsystem, double applyVoltage) {
    this.swerveDrivetrain = swerveSubsystem;
    this.applyVoltage = applyVoltage;
    // Adds the subsystem as a requirement (prevents two commands from acting on subsystem at once)
    addRequirements(swerveDrivetrain);
  }

  @Override
  public void initialize() {
    // swerveDrivetrain.applyRequest(null);
    counter++;
  }

  @Override
  public void execute() {
    DogLog.log("restarted", counter);
    swerveDrivetrain.getModule(0).getDriveMotor().setVoltage(0.2425 + applyVoltage);
    swerveDrivetrain.getModule(1).getDriveMotor().setVoltage(0.2425 + applyVoltage);
    swerveDrivetrain.getModule(2).getDriveMotor().setVoltage(0.2425 + applyVoltage);
    swerveDrivetrain.getModule(3).getDriveMotor().setVoltage(0.2425 + applyVoltage);

    swerveDrivetrain.getModule(0).getSteerMotor().setControl(new MotionMagicVoltage(0).withSlot(0));
    swerveDrivetrain.getModule(1).getSteerMotor().setControl(new MotionMagicVoltage(0).withSlot(0));
    swerveDrivetrain.getModule(2).getSteerMotor().setControl(new MotionMagicVoltage(0).withSlot(0));
    swerveDrivetrain.getModule(3).getSteerMotor().setControl(new MotionMagicVoltage(0).withSlot(0));
  } // Drive counterclockwise with negative X (left))

  @Override
  public void end(boolean interrupted) {
    // Applies SwerveDriveBrake (brakes the robot by turning wheels)
    // this.swerveDrivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
