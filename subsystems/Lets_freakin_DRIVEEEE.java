// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.*;

public class Lets_freakin_DRIVEEEE extends SubsystemBase {
  /** Creates a new Lets_freakin_DRIVEEEE. */

  DifferentialDrive UnderBoy;
  AHRS gyro;
  RamseteController controller1;
  DifferentialDriveOdometry odometry;
  DifferentialDriveKinematics kinematics;
  double speeds;
  TalonFX frontLeftLeader;
  TalonFX frontRightLeader;
  TalonFX backLeftFollower;
  TalonFX backRightFollower;
  StatusSignal<Double> enc;
  Joystick Driver;
  JoystickButton ky;
  JoystickButton kx;
  JoystickButton a;
  JoystickButton b;
  JoystickButton left;
  JoystickButton right;
  public double leftEnc;
  double rightEnc;
  double error;
  PIDController leftController;
  PIDController rightController;


  
//gor giga goof pluh 87 !@#$%^&*() pluh orpleeeeeeeeeeeeeeeee oap ur mom
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry id = table.getEntry("tid");


  public Lets_freakin_DRIVEEEE() {

    frontLeftLeader = new TalonFX(1);
    backLeftFollower = new TalonFX(2);
    frontRightLeader = new TalonFX(3);
    backRightFollower = new TalonFX(4);

    backLeftFollower.setControl(new Follower(frontLeftLeader.getDeviceID(), false));
    backRightFollower.setControl(new Follower(frontRightLeader.getDeviceID(), false));

    enc = frontLeftLeader.getPosition();

    Driver = new Joystick(0);

    gyro = new AHRS(I2C.Port.kOnboard);
    controller1 = new RamseteController();
    leftController = new PIDController(DriveConstants.Drive_Kp, 0, 0);
    rightController = new PIDController(DriveConstants.Drive_Kp, 0, 0);

    leftEnc = frontLeftLeader.getPosition().getValueAsDouble() * Constants.OperatorConstants.kLinearDistanceConversionFactor;
    rightEnc = frontRightLeader.getPosition().getValueAsDouble() * Constants.OperatorConstants.kLinearDistanceConversionFactor;

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), error, error);
    
    

    //odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftE.getPosition(), rightE.getPosition());
    //kinematics = new DifferentialDriveKinematics(0.5842);


    UnderBoy = new DifferentialDrive(frontLeftLeader, frontRightLeader);

    AutoBuilder.configureRamsete(
                
                this::getPose,
                this::resetPose, // Robot pose supplier
                this::getChassisSpeeds, // Current ChassisSpeeds supplier
                this::drive, // Method that will drive the robot given ChassisSpeeds
                new ReplanningConfig(), // Default path replanning config. See the API for the options here
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    
  } 

  // func for drive train
   public void goOnAnGetBOY(double turnspeed, double moveSpeed){
    UnderBoy.arcadeDrive(turnspeed, -moveSpeed);
   }

   public double avgEncDistance(){
    return (((-frontLeftLeader.getPosition().getValueAsDouble())
    + frontRightLeader.getPosition().getValueAsDouble()
    + (-backLeftFollower.getPosition().getValueAsDouble())
    + backRightFollower.getPosition().getValueAsDouble())/ 4);
   }


   public void resetEnc(){
    frontLeftLeader.setPosition(0);
    frontRightLeader.setPosition(0);
    backLeftFollower.setPosition(0);
    backRightFollower.setPosition(0);
   }

   // start stuff for path planner
   private double driveTrainP() {
    double error = frontLeftLeader.getPosition().getValueAsDouble() - frontRightLeader.getPosition().getValueAsDouble();
    return DriveConstants.DRIVE_P * error;
   }

   private int distanceToNativeUnits(double positionMeters) {
    double wheelRotations = positionMeters / (2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadiusInches));
    double motorRotations = wheelRotations * DriveConstants.GearRatio;
    int sensorCounts = (int)(motorRotations * DriveConstants.EncoderTPR);
    return sensorCounts;
   }

   private int velocityToNativeUnits(double velocityMetersPerSecond) {
    double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadiusInches));
    double motorRotationsPer100ms = wheelRotationsPerSecond * DriveConstants.k100msPerSecond;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * DriveConstants.EncoderTPR);
    return sensorCountsPer100ms;
  }

  private double nativeUnitsToDistanceMeters(double sensorCounts) {
    double motorRotations = (double)sensorCounts / DriveConstants.EncoderTPR;
    double wheelRotations = motorRotations / DriveConstants.GearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadiusInches));
    return positionMeters;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    double leftWheelSpeed = 10 * nativeUnitsToDistanceMeters(frontLeftLeader.getVelocity().getValueAsDouble());
    double rightWheelSpeed = 10 * nativeUnitsToDistanceMeters(frontRightLeader.getVelocity().getValueAsDouble());
    return new DifferentialDriveWheelSpeeds(leftWheelSpeed, rightWheelSpeed);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  public void drive(){
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(ChassisSpeeds);
    //wheelSpeeds.desaturate(0.5);

    UnderBoy.tankDrive(set.frontLeftLeader(leftWheelSpeed), rightController);

    SmartDashboard.putNumber("wheel sped left", wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("wheel sped right", wheelSpeeds.rightMetersPerSecond);

   }


  //public void resetOdometry(Pose2d pose) {
    //resetEnc();
    //odometry.resetPosition(gyro.getRotation2d(), DifferentialDrive.wheel, pose);;
  //}


   /*public void resetPose(Pose2d pose){
  
    odometry.resetPosition(gyro.getRotation2d(), leftE.getPosition(), rightE.getPosition(), pose);
  
    
   }

   public Pose2d getPose(){
    return odometry.getPoseMeters();
   }

   public ChassisSpeeds getCurrentSpeeds(){
     DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(leftE.getVelocity(), rightE.getVelocity());
    //need to pass values not set static speed. grab encoder data to setup actual speed
    return kinematics.toChassisSpeeds(speeds);
   }

   public void drive(ChassisSpeeds ChassisSpeeds){

    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(ChassisSpeeds);
    wheelSpeeds.desaturate(0.5);

    UnderBoy.tankDrive( wheelSpeeds.leftMetersPerSecond, -wheelSpeeds.rightMetersPerSecond);

    SmartDashboard.putNumber("wheel sped left", wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("wheel sped right", wheelSpeeds.rightMetersPerSecond);

   }

   public double readPosition(RelativeEncoder encoder, double wheelDiameter){
    double wheelCircumference = wheelDiameter * 3.14;
    return encoder.getPosition() * wheelCircumference;
   }
   */
   // end stuff for path planner

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    odometry.update(gyro.getRotation2d(), 
                    nativeUnitsToDistanceMeters(frontLeftLeader.getPosition().getValueAsDouble()), 
                    nativeUnitsToDistanceMeters(frontRightLeader.getPosition().getValueAsDouble()));
    
    a = new JoystickButton(Driver, XboxController.Button.kA.value);
    ky = new JoystickButton(Driver, XboxController.Button.kY.value);
    kx = new JoystickButton(Driver, XboxController.Button.kX.value);
    b = new JoystickButton(Driver, XboxController.Button.kB.value);
    right = new JoystickButton(Driver, XboxController.Button.kRightBumper.value);
    left = new JoystickButton(Driver, XboxController.Button.kLeftBumper.value);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch());
    SmartDashboard.putNumber("Angle", gyro.getAngle());
    SmartDashboard.putNumber("heading", gyro.getCompassHeading());
    SmartDashboard.putNumber("Front Left Enc Position", frontLeftLeader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Front Right Enc Position", frontRightLeader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Back Left Enc Position", backLeftFollower.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Back Right Enc Position", backRightFollower.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Left Enc", leftEnc);
    SmartDashboard.putNumber("Right Enc", rightEnc);
    SmartDashboard.putNumber("What the flip", avgEncDistance());
    SmartDashboard.putNumber("gro", gyro.getAngle());
    SmartDashboard.putBoolean("y butt", ky.getAsBoolean());
    SmartDashboard.putBoolean("x butt", kx.getAsBoolean());
    SmartDashboard.putBoolean("a butt", a.getAsBoolean());
    SmartDashboard.putBoolean("b butt", b.getAsBoolean());
    SmartDashboard.putBoolean("right bum", right.getAsBoolean());
    SmartDashboard.putBoolean("left bum", left.getAsBoolean());
    //SmartDashboard.putNumber("Right Encoder Vel", -rightE.getVelocity());
    //SmartDashboard.putNumber("left Encoder Vel", leftE.getVelocity());

  }
}