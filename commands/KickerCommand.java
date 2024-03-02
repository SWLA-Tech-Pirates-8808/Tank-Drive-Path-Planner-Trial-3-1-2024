// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Kicker;

public class KickerCommand extends Command {
  /** Creates a new KickerCommand. */

  Kicker s_Kicker;
  double speed;
  boolean isReal;


  public KickerCommand(Kicker s_Kicker, double speed, boolean isReal) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Kicker = s_Kicker;
    this.speed = speed;
    this.isReal = isReal;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

 
    //if(/*s_Kicker.EpicLaz() == true && isReal ==*/ false){
    //  s_Kicker.twoAndFour(0);

    //} else {
      s_Kicker.twoAndFour(-speed);

    //}
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

     s_Kicker.twoAndFour(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
