// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class Calibration extends CommandBase {
  Turret turret;
  enum modeEnum {Fast, Wait, Slow, End};
  modeEnum mode = modeEnum.Fast;
  private boolean sensorState;
  /** Creates a new Calibration. */
  public Calibration(Turret turret) {
    this.turret = turret;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.setPower(0.15);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(mode == modeEnum.Fast){
      if(turret.getSensorClockWise()){
        turret.setPower(0);
        sensorState = true;
        mode = modeEnum.Wait;
    }
  }

    if(mode == modeEnum.Wait){
      for(int i = 0; i < 10;i++){}
      mode = modeEnum.Slow;
    }
    
    if(mode == modeEnum.Slow){
      turret.setPower(Constants.FF_VALUE);
      if(!sensorState){
        turret.setPower(0);
        turret.setEncoder(160);
        mode = modeEnum.End;
      }
    }

  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mode == modeEnum.End;
  }
}
