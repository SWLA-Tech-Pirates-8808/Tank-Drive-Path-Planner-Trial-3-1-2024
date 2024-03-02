// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput; 

public class Kicker extends SubsystemBase {
  /** Creates a new Kicker. */

  CANSparkMax bassPedal;
  DigitalInput EpiclazerBeam;

  public Kicker() {

    bassPedal = new CANSparkMax(7, MotorType.kBrushless);
    bassPedal.setInverted(true);

    EpiclazerBeam = new DigitalInput(0);

  }

  public void twoAndFour(double speed){
    bassPedal.set(-speed);
  }

  public boolean EpicLaz(){
    return EpiclazerBeam.get();
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
