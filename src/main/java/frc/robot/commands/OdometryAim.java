// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class OdometryAim extends CommandBase {
  private final Turret turret;
  private final Translation2d targetxy;
  private Rotation2d currentRobotHeading;
  private Translation2d currentRobotTranslation;
  private Translation2d robotToTargetXY;
  private Rotation2d robotToTargetRotation;
  private double tempX;
  private double tempY;
  private double angleToGo;
  private Pose2d currentRobotPose = new Pose2d(new Translation2d(1,2), new Rotation2d(8));
  /** Creates a new AimToXY. */
  public OdometryAim(Turret turret, Translation2d targetxy) {
    this.turret = turret;
    this.targetxy = targetxy;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentRobotPose = turret.getRobotPose();
    tempX = currentRobotPose.getX()-
    this.targetxy.getX();
    tempY = currentRobotPose.getY()-this.targetxy.getY();
    robotToTargetXY = new Translation2d(tempX,tempY);
    robotToTargetRotation = new Rotation2d(robotToTargetXY.getX(), robotToTargetXY.getY());
    angleToGo = robotToTargetRotation.minus(currentRobotPose.getRotation()).getDegrees();
   // turret.setAngle(angleToGo);
  }

  

  private Pose2d getRobotPose(){
    //TODO return real pose
    Pose2d p = new Pose2d();
    return p;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    turret.setPower(0);
    return false;
  }
}
