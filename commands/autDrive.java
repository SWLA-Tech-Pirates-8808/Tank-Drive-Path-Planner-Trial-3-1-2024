// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lets_freakin_DRIVEEEE;

public class autDrive extends Command {

  Lets_freakin_DRIVEEEE s_Lets_freakin_DRIVEEEE;
 // double turnSped;
  double moveSped;
  PIDController speedPID;
  double distance;

  /** Creates a new autDrive. */
  public autDrive(Lets_freakin_DRIVEEEE s_Lets_freakin_DRIVEEEE) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Lets_freakin_DRIVEEEE = s_Lets_freakin_DRIVEEEE;
    //this.distance = distance;


    speedPID = new PIDController(0.05, 0.01, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speedPID.reset();
    s_Lets_freakin_DRIVEEEE.resetEnc();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   moveSped = speedPID.calculate(s_Lets_freakin_DRIVEEEE.avgEncDistance(), 20);

    s_Lets_freakin_DRIVEEEE.goOnAnGetBOY(0, -moveSped);
   // s_Lets_freakin_DRIVEEEE.goOnAnGetBOY(0.5, 0);
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
