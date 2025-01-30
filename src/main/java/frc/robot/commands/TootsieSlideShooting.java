// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.subsystems.TootsieSlideSubsystem;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;

// /** An example command that uses an example subsystem. */
// public class TootsieSlideShooting extends Command {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final TootsieSlideSubsystem m_subsystem;
//   private final
//   public TootsieSlideShooting(TootsieSlideSubsystem subsystem) {
//     m_subsystem = subsystem;
//     addRequirements(m_subsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     SmartDashboard.putString("TootsieSlideSHootingisRunning", "true");
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_subsystem.spinTootsie();
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     SmartDashboard.putString("TootsieSlideSHootingisRunning", "false");
//     m_subsystem.stopTootsie();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
