// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shield;
import edu.wpi.first.wpilibj.Joystick;


public class ShieldCommand extends Command {

Shield s_Shield;
Joystick bam;

  /** Creates a new ShieldCommand. */
  public ShieldCommand(Shield s_Shield, Joystick bam) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Shield);

    this.s_Shield = s_Shield;
    this.bam = bam;
 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    s_Shield.moveShield(bam.getRawAxis(3));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
