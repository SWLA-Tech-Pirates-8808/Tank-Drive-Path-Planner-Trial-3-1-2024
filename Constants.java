// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  public static final int MoveAxis = 1;
  public static final int RotateAxis = 4;

  public static final double kGearRatio = 7.31;
  public static final double kWheelRadiusInches = 3;

  public static final double kLinearDistanceConversionFactor = (Units.inchesToMeters((kGearRatio*2*Math.PI*Units.inchesToMeters(kWheelRadiusInches))*3));
  }

  public final static class DriveConstants{
    /**
     * The DriveConstants class is used to define all of the constants we use for driving. 
     * These include the IDs of the motors, conversion rates from tics to feet, etc.
     * To call this, call from the constants file as you normally would, but instead of doing
     * Constants.whateveryouwanttocall say DriveConstants.whateveryouwanttocall
     * IDS are the ports the motors are connected to
     */
    public static final double Drive_Ks = 0.6057;
    public static final double Drive_Kv = 2.4263;
    public static final double Drive_Ka = 0.37369;
    public static final double Drive_Kp = 1.4948;
    public static final double Drive_KpTest = 0;
    public static final double Drive_Kd = 0.0;
    

    public static final double DRIVE_P = 4;
    public static final double DRIVE_I = 1;

    public static final double TIC_FT = ((Math.PI)/2048)/7.31;
    public static final double kTrackwidthMeters = Units.inchesToMeters(27);
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double GearRatio = 7.31;
    public static final int EncoderTPR = 2048;
    public static final double WheelDiameterMeters = Units.inchesToMeters(6.0);
    public static final double WheelRadiousMeters  = Units.inchesToMeters(3.0);
    public static final double kWheelRadiusInches = 3.00;
    public static final double EncoderDistancePerPulse = (WheelDiameterMeters * Math.PI)/ GearRatio / (double) EncoderTPR;
    public static final double WheelCircumferenceMeters = 2*Math.PI*WheelRadiousMeters;
    public static final double Conversion = WheelCircumferenceMeters/(GearRatio*EncoderTPR);
    public static final int k100msPerSecond = 10;
    public static boolean kGyroReversed = true;
}
  
  
}