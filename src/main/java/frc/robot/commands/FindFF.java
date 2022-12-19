// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Turret;

public class FindFF extends CommandBase {
  private double encoder;
  Turret turret;
  private static double ffval = 0.039;
  int cycle = 0;
  /** Creates a new FindFF. */
  public FindFF(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoder = turret.getEncoder();
    cycle = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(cycle == 0) {
         turret.setPower(ffval);
    }
    cycle++;
    if(cycle >=13){
    if(turret.getEncoder() == encoder){
      ffval += 0.00001;
      }
      cycle = 0;
    }

  }

  public static double getFFValue(){
    return ffval;
  }

  // Called once the command ends or is interr;}upted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double getFindFF(){
    return ffval;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("FINDFF-val", this::getFindFF, null);
  }
}
