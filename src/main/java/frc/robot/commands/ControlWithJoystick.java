// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class ControlWithJoystick extends CommandBase {
  /** Creates a new ControlWithJoystick. */
  Turret turret;
  Joystick joystick;
  double angle = 0;
  public ControlWithJoystick(Turret turret, Joystick joystick) {
    this.turret = turret;
    this.joystick = joystick;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double angle = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = joystick.getY();
   // angle += Math.pow(3, joystick.getY());
   angle += joystick.getY()*3;
   double a = angle; 
   angle = turret.optimizeAngle(angle);
   System.out.println("a= " + a + " opt= " + angle + " cur= " + turret.getAngle());
    //System.out.println("---------------------------------------------");
    turret.setAngle(angle);
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
