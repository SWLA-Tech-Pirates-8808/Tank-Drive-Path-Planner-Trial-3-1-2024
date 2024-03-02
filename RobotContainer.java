// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/* You will need to import all commands and subsystems here. Use the below method to import all with only 2 lines of code */
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

//import com.pathplanner.lib.commands.PathPlannerAuto;
//import com.pathplanner.lib.path.PathPlannerPath;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private final Lets_freakin_DRIVEEEE s_Lets_freakin_DRIVEEEE = new Lets_freakin_DRIVEEEE();
  private final ShooterMan s_ShooterMan = new ShooterMan();
  private final Kicker s_Kicker = new Kicker();
  private final woowee s_woowee = new woowee();
  private final Intake s_Intake = new Intake();
  private final Shield s_Shield = new Shield();
  public final static Joystick Drew = new Joystick(0);
  public final static Joystick bazinga = new Joystick(1);


  //private final JoystickButton follow = new JoystickButton(Drew, XboxController.Button.kY.value);
  private final JoystickButton BIGEAT = new JoystickButton(Drew, XboxController.Button.kRightBumper.value);
  private final JoystickButton spit = new JoystickButton(Drew, XboxController.Button.kLeftBumper.value);
  private final JoystickButton IntakePosition = new JoystickButton(bazinga, XboxController.Button.kB.value);
  private final JoystickButton spinAim = new JoystickButton(bazinga, XboxController.Button.kLeftBumper.value);
  private final JoystickButton KICK = new JoystickButton(bazinga, XboxController.Button.kRightBumper.value);
  //private final JoystickButton ampAim = new JoystickButton(bazinga, XboxController.Button.kY.value);
  private final JoystickButton stow = new JoystickButton(bazinga, XboxController.Button.kA.value);
  private final JoystickButton SpeakAim = new JoystickButton(bazinga, XboxController.Button.kX.value);
  private final JoystickButton AutoTest = new JoystickButton(bazinga, XboxController.Button.kY.value);

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    s_Lets_freakin_DRIVEEEE.setDefaultCommand(new DrivinCommand(s_Lets_freakin_DRIVEEEE, Drew));
    s_woowee.setDefaultCommand(new UpeeDownCommand(s_woowee, bazinga));
    s_Shield.setDefaultCommand(new ShieldCommand(s_Shield, bazinga));

    configureBindings();

    //PathPlannerPath path = PathPlannerPath.fromPathFile("Straight");

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@litnk edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
      // follow.whileTrue(new autoMove(s_Lets_freakin_DRIVEEEE, 0, 0.35, 0.25, Drew)); 
       BIGEAT.whileTrue(new Radintake(s_Intake, s_Kicker));  // need to add set positions with woowee
       spit.whileTrue(new unEat(s_Intake, s_Kicker)); // also need to add set positions with woowee but just for
       IntakePosition.whileTrue(new WooweePosition(s_woowee, 318));
       spinAim.whileTrue(new aimBOY(s_woowee, s_ShooterMan)); // this will aim and spinup shooter
       KICK.whileTrue(new KickerCommand(s_Kicker, 0.25, false));
       //ampAim.whileTrue(new WooweePosition(s_Woowee, 278)); // set position for amp idk what that is yet
       stow.whileTrue(new WooweePosition(s_woowee, 90)); // set position for stow
       //trap.whileTrue(new WooweePosition(s_Woowee, 278)); // set position for trap idek what that is yet???????
       SpeakAim.whileTrue(new WooweeAprilTag(s_woowee));
       AutoTest.whileTrue(new autDrive(s_Lets_freakin_DRIVEEEE));
       

       

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   // An example command will be run in autonomous
    return new autDrive(s_Lets_freakin_DRIVEEEE);
  }
}
