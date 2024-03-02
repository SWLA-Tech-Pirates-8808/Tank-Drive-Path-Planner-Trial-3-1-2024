// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class Shield extends SubsystemBase {

  CANSparkMax Shield;

  /** Creates a new Shield. */
  public Shield() {
  Shield = new CANSparkMax(9, MotorType.kBrushed);
  }

public void moveShield(double speed){

Shield.set(speed);
}

  @Override
  public void periodic() {

    

    // This method will be called once per scheduler run
  }
}
