// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.util.Utilities;
public class DistancePID extends CommandBase {
  private final Turret turret;
  private double sp;
  private double error;
  private double ff;
  private boolean reset = false;
  private final Utilities utilities = new Utilities();
  /** Creates a new DistancePID. */
  public DistancePID(Turret turret, double sp) {
    this.turret = turret;
    this.sp = sp;
    addRequirements(turret);

    //Shuffleboard.getTab("CMD").add(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setAngle(sp);
    System.out.println(sp);
  }

  public double getAngle(){
    return turret.getAngle();
  }

  public double getError(){
    return sp - getAngle();
  }

  public double getSP(){
    return sp;
  }
  
  public void setSP(double sp){
    this.sp = sp;
  }

  public double GetSP(){
    return sp;
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
     //Math.abs(sp-getAngle())<=0.1;
  }
}
