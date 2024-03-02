// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Lets_freakin_DRIVEEEE;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class babyAuto extends SequentialCommandGroup {
  /** Creates a new babyAuto. */
  public babyAuto(Lets_freakin_DRIVEEEE s_Lets_freakin_DRIVEEEE) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new autDrive(s_Lets_freakin_DRIVEEEE)
    );
  }
}
