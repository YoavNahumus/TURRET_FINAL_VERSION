// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.commands.Calibration;
import frc.robot.commands.ControlWithJoystick;
import frc.robot.commands.DistancePID;
import frc.robot.commands.FindFF;
import frc.robot.commands.VisionRotate;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Turret turret;
  FindFF find;
  Joystick joystick;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Vision vis = new Vision();
    // Configure the button bindings
    joystick = new Joystick(Constants.JOYSTICK_ID);
    turret = new Turret(this);
     find = new FindFF(turret);
     VisionRotate visrot = new VisionRotate(turret, vis);
    Shuffleboard.getTab("FIND").add(find);
    Shuffleboard.getTab("vis").add(visrot);
    turret.setDefaultCommand(visrot);
    //turret.setDefaultCommand(new ControlWithJoystick(turret, joystick));
  }

   
   

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


   
  public Command getAutonomousCommand() {
    return new Calibration(turret);
  }
}
