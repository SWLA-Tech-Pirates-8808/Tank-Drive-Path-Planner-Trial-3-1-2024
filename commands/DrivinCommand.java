// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Lets_freakin_DRIVEEEE;
import edu.wpi.first.wpilibj.Joystick;

public class DrivinCommand extends Command {
  /** Creates a new DrivinCommand. */
  Lets_freakin_DRIVEEEE s_Lets_freakin_DRIVEEEE;
  Joystick Driver;

  public DrivinCommand(Lets_freakin_DRIVEEEE s_Lets_freakin_DRIVEEEE, Joystick Driver) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Lets_freakin_DRIVEEEE);

    this.s_Lets_freakin_DRIVEEEE = s_Lets_freakin_DRIVEEEE;
    this.Driver = Driver;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Lets_freakin_DRIVEEEE.goOnAnGetBOY(Driver.getRawAxis(Constants.OperatorConstants.RotateAxis)
    , Driver.getRawAxis(Constants.OperatorConstants.MoveAxis));
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
