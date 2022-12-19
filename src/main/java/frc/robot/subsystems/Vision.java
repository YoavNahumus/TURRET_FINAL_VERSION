// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  NetworkTable table;
  NetworkTableEntry tv;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  /** Creates a new Vision. */
  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("gloworm");
    tv = table.getEntry("hasTarget");
    tx = table.getEntry("targetYaw");
    ty = table.getEntry("targetPitch");
    ta = table.getEntry("targetArea");
  }


  public boolean validTarget() {
    return tv.getDouble(0) != 0;
  }
  
  public double targetXAngle() {
    if(validTarget()) {
      return tx.getDouble(198);
    }else{
     return 180;
    }
  }
  public double targetYAngle() {
    if(validTarget()) {
      return ty.getDouble(180);
    }
    return 180;
  }

  @Override
  public void periodic() {
    //System.out.println(targetXAngle());
    System.out.println(validTarget());
  }
}
