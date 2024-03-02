// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;



public class ShooterMan extends SubsystemBase {
  /** Creates a new ShooterMan. */

  CANSparkMax shootTop;
  CANSparkMax shootBottom;

  public ShooterMan() {

    shootTop = new CANSparkMax(5, MotorType.kBrushless);
    shootBottom = new CANSparkMax(6, MotorType.kBrushless);

    


  }

public void shootThing(double shootSpeed){

    shootTop.set(shootSpeed);
    shootBottom.set(-shootSpeed);

}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
