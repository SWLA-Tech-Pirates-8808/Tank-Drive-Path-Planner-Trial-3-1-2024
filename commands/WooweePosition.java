// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.woowee;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;

public class WooweePosition extends Command {

  woowee s_woowee;
  PIDController InPauseController;
  double sped;
  double position;
  NetworkTableEntry ty;


  /** Creates a new PositionIntake. */
  public WooweePosition(woowee s_woowee, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    InPauseController = new  PIDController(0.003, 0.0001, 0);

    this.s_woowee = s_woowee;
    this.position = position;
  
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    InPauseController.reset();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sped = InPauseController.calculate(s_woowee.getEnc(), position);


    s_woowee.woo(-sped);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_woowee.woo(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
