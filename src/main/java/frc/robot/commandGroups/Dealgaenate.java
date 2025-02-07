// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DaleCommands.ArmToAngleCmd;
import frc.robot.commands.DaleCommands.SpinFlywheel;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Dealgaenate extends ParallelCommandGroup {
  /** Creates a new Dealgaenate. */
  public Dealgaenate(ArmSubsystem arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // TODO: To whomever reads this, have fun finding actual numbers;
    // Fe fi fo fum, i proclaim that ritvik's a bum
    addCommands(new ArmToAngleCmd(Constants.Arm.EXTENDED_ANGLE, arm));
    addCommands(new SpinFlywheel(arm, Constants.Arm.DEALGAENATE_SPEED_ZOOM_ZOOM));
    addCommands(new ArmToAngleCmd(Constants.Arm.RETRACTED_ANGLE, arm));
  }
}
