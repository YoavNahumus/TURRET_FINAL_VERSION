// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class VisionRotate extends CommandBase {
  /** Creates a new VisionRotate. */
  Turret turret;
  Vision vision;
  double sp = 0;
  private double tempAngleForOptimize = 0;
  boolean goAround = false;
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
      sp = turret.getAngle() + vision.targetXAngle()-turret.getAngle()%0.1;
    }
    sp = optimizeAngleForVision(sp);
    if(!goAround){
      turret.setAngle(sp);
    }
  }
  
  public void setGoAround(boolean e){
    System.out.println("Change");
    goAround = e;
  }


  public double optimizeAngleForVision(double wantedAngle) {
    double currentAngle = turret.getAngle();
    if (wantedAngle > 180)
      wantedAngle -= 360;
    else if (wantedAngle < -180)
      wantedAngle += 360;

      double diff = wantedAngle - turret.getAngle();
    if (currentAngle + diff > Constants.MOTION_RANGE) {
      System.out.println("BIGGET ++++++++");
      diff -= 360;
      setGoAround(true); 
      tempAngleForOptimize = turret.getAngle();
    }
    if (currentAngle + diff < -Constants.MOTION_RANGE) {
      System.out.println("BIGGER --------");
      diff += 360;
      setGoAround(true); 
      tempAngleForOptimize = turret.getAngle();
    }

    if (getGoAround()) {
      if (Math.abs(tempAngleForOptimize - turret.getAngle()) > 180) {
        setGoAround(false); 
        tempAngleForOptimize = 0;
      }
    }
    return diff + currentAngle;
  }

  public boolean getGoAround(){
    return goAround;
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  public double getError(){
    return (sp-turret.getAngle());
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.addDoubleProperty("ERROR", this::getError, null);
  }
}
