// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.FunnelConstants;
// import frc.robot.Constants.TootsieSlide;
// import frc.robot.subsystems.FunnelSubsystem;
// import frc.robot.subsystems.TootsieSlideSubsystem;
// import frc.robot.subsystems.TootsieSlideSubsystem;

// /**
//  * Runs the intake and preshooter until IR sensor detects note
//  *
//  * @param subsystem The subsystem used by this command.
//  */
// public class RunTootsieUntilDetection extends Command {
//   private TootsieSlideSubsystem tootsieSubsystem;
//   private FunnelSubsystem funnelSubsystem;
//   public RunTootsieUntilDetection(TootsieSlideSubsystem tootsieSubsystem, FunnelSubsystem funnelSubsystem) {
//     this.tootsieSubsystem = tootsieSubsystem;
//     this.funnelSubsystem = funnelSubsystem;

//     addRequirements(tootsieSubsystem);
//     addRequirements(funnelSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     tootsieSubsystem.spinTootsie(funnelSubsystem.isCoralCheckedOut());
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     tootsieSubsystem.stopTootsie();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return tootsieSubsystem.atTarget();
//   }
// }