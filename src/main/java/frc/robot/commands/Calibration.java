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
  private int nPeriods;
  private boolean passedsensor;
  /** Creates a new Calibration. */
  public Calibration(Turret turret) {
    this.turret = turret;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.setPower(0.35);
    nPeriods = 0;
    mode = modeEnum.Fast;
    passedsensor = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(mode) {
      case Fast:
        if(turret.getSensorClockWise()){
          passedsensor = true;
        }
        if(passedsensor && !turret.getSensorClockWise()){
          turret.setPower(0);
          mode = modeEnum.Wait;
          nPeriods = 0;
        }
        System.out.println("FAST");
        break;

      case Wait:
      turret.setIsBrake(true);
       nPeriods++;
       if(nPeriods==20){
         turret.setIsBrake(false);
            mode = modeEnum.Slow;
          }
      break;

      case Slow:
       turret.setPower(-0.07);
       System.out.println("SLOW");
       if(turret.getSensorClockWise()){
          turret.setPower(0);
          turret.setEncoder(160*Constants.PULSE_PER_DEGREE);
          mode = modeEnum.End;
        }
        break;
      }

System.out.println(turret.getSensorClockWise());

  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(turret.getAngle());
    boolean idk = false;
    turret.setAngle(0);
    System.out.println("END");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mode == modeEnum.End;
  }
}
