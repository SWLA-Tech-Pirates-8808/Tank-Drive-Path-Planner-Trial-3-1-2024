// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterMan;



public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */
ShooterMan s_ShooterMan;

  public ShootCommand(ShooterMan s_ShooterMan) {
    // Use addRequirements() here to declare subsystem dependencies.
  this.s_ShooterMan = s_ShooterMan;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    s_ShooterMan.shootThing(1);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    s_ShooterMan.shootThing(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
