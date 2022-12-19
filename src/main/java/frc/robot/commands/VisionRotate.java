// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class VisionRotate extends CommandBase {
  /** Creates a new VisionRotate. */
  Turret turret;
  Vision vision;
  double sp = 0;
  public VisionRotate(Turret turret, Vision vision) {
    this.turret = turret;
    this.vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.validTarget()){
      sp = turret.getAngle() + vision.targetXAngle();
    }
   // System.out.println(sp);
    System.out.println(vision.validTarget());

    turret.setAngle(turret.optimizeAngle(sp));
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
