// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Lets_freakin_DRIVEEEE;
import frc.robot.subsystems.ShooterMan;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick; 


public class autoMove extends Command {
  /** Creates a new DrivinCommand. */
  Lets_freakin_DRIVEEEE s_Lets_freakin_DRIVEEEE;
  ShooterMan s_ShooterMan;
  Joystick Driver;

  double turnSpeed;
  double moveSpeed;
  double shootMove;

  NetworkTable table;
  double targetOffsetAngle_Distance;
  double swoopswoop;
  double getLime;
 // double yBro;
  NetworkTableEntry tlong;
  NetworkTableEntry tx;
  NetworkTableEntry tid;
 // NetworkTableEntry ty;


  PIDController frickPidController;
  PIDController frickPidControllerx;
 // PIDController frickPidControllery;
  

  public autoMove(Lets_freakin_DRIVEEEE s_Lets_freakin_DRIVEEEE, double turnSpeed
  , double moveSpeed, double shootMove, Joystick Driver) {
    // Use addRequirements() here to declare subsystem dependencies.

    frickPidController = new  PIDController(0.025, 0.000001, 0);
    frickPidControllerx = new  PIDController(0.04, 0.005, 0);
    //frickPidControllery = new  PIDController(0.04, 0.005, 0);
  
    this.s_Lets_freakin_DRIVEEEE = s_Lets_freakin_DRIVEEEE;
    this.turnSpeed = turnSpeed;
    this.moveSpeed = moveSpeed;
    this.shootMove = shootMove;
    this.Driver = Driver;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    frickPidController.reset();
    frickPidControllerx.reset();
    //frickPidControllery.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tid = table.getEntry("tid");
    tlong = table.getEntry("tlong");
    tx = table.getEntry("tx");
    //ty = table.getEntry("ty");
    
    targetOffsetAngle_Distance = tlong.getDouble(0.0); 
    swoopswoop = tx.getDouble(0.0);
    getLime = tid.getDouble(-1);
    //yBro = ty.getDouble(0.0);
    
    moveSpeed = frickPidController.calculate(targetOffsetAngle_Distance, 69);
    turnSpeed = frickPidControllerx.calculate(swoopswoop, 0);
    //shootMove = frickPidControllery.calculate(yBro, 0);

    if (getLime == -1) {
      s_Lets_freakin_DRIVEEEE.goOnAnGetBOY(Driver.getRawAxis(Constants.OperatorConstants.RotateAxis), Driver.getRawAxis(Constants.OperatorConstants.MoveAxis));

    } else if(getLime == 7){
    s_Lets_freakin_DRIVEEEE.goOnAnGetBOY(-turnSpeed, -moveSpeed);
    }
    

  
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
