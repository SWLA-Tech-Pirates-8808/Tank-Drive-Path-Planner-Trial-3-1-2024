// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class OuttakeCommand extends Command {
  /** Creates a new OuttakeCommand. */
  Intake s_Intake;
  double speed;

  public OuttakeCommand(Intake s_Intake, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Intake = s_Intake;
    this.speed = speed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    s_Intake.nom(-speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    s_Intake.nom(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
