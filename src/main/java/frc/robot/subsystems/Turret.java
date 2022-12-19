// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AimToXY;
import frc.robot.util.Utilities;

public class Turret extends SubsystemBase {
  private final TalonFX motor;
  private AimToXY aimToBasket;
  private AimToXY aimToOurAlliance;
  private double kpValue = Constants.TURRET_MOTOR_KP;
  private double testTarget;
  private final Translation2d basketXY = new Translation2d(Constants.TARGET_X, Constants.TARGET_Y);
  private final Translation2d ourAllianceXY = new Translation2d(Constants.OUR_ALLIACNE_X, Constants.OUR_ALLIANCE_Y);
  private double ff = 0;
  private double targetAngle = 0;
  private double ffsearch;
  private double ffsearch2;
  private DigitalInput sensor1; 

  public enum TurretMode {
    FOLLOW, SIDE, STOP
  };

  public TurretMode mode = TurretMode.FOLLOW;
  RobotContainer robotContainer;
  private final Utilities utilities;

  /** Creates a new turret. */
  public Turret(RobotContainer robotContainer) {
    super();
    aimToBasket = new AimToXY(this, basketXY);
    aimToOurAlliance = new AimToXY(this, ourAllianceXY);
    utilities = new Utilities();
    this.robotContainer = robotContainer;
    sensor1 = new DigitalInput(Constants.SENSOR_CLOCKWISE);
    motor = new TalonFX(Constants.MOTOR_ID);
    motor.config_kP(0, kpValue);
    ffsearch = 0.1;
    // TODO - init motor - brake, PID, ....
    motor.setSelectedSensorPosition(0);
    //motor.setInverted(InvertType.InvertMotorOutput);
  }

  public void setPower(double power) {
    motor.set(ControlMode.PercentOutput, power);
  }

  public void setEncoder(double enc){
    motor.setSelectedSensorPosition(enc);
  }

  public void setIsBrake(boolean brake){
    if(brake){
      motor.setNeutralMode(NeutralMode.Brake);
    }else{
      motor.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void setAngle(double angle) { //gets optimized angle
    targetAngle = angle;
    double error = angle - getAngle();
    ff = Constants.FF_VALUE;
    if(Math.abs(error) < 0.06) {
      ff = 0;
    } else if(error < 0) {
      ff = -ff;
    }
     motor.set(ControlMode.Position, (int) (angle * Constants.PULSE_PER_DEGREE), DemandType.ArbitraryFeedForward, ff);
    }

  public void setSensorPositionAngle(double angle) {
    motor.setSelectedSensorPosition(angle * Constants.PULSE_PER_DEGREE);
  }

  public double getAngle() {
    return motor.getSelectedSensorPosition() / Constants.PULSE_PER_DEGREE;
  }

  public double getEncoder(){
    return motor.getSelectedSensorPosition();
  }

  public double getFF(){
    return ff;
  }

  public double getOptimizedAngle(){
    return targetAngle;
  }
  // TODO move to utilities 
  public double optimizeAngle(double wantedAngle) {
    double currentAngle = getAngle(), diff = wantedAngle - getAngle();
    if (wantedAngle > 180)
      diff -= 360;
    else if (wantedAngle < -180)
      diff += 360;

    if (currentAngle + diff > Constants.MOTION_RANGE)
      diff -= 360;
    else if (currentAngle + diff < -Constants.MOTION_RANGE)
      diff += 360;
      
    return diff + currentAngle;
  }

  // TODO DELETE!!!!!!!!!!!!!!!!!!!!!!! should be in chassis. no use of root`s location in subsystem, only in commands!!!!!!!!!!!!!!!!!!!!
  public Pose2d getRobotPose(){
    return utilities.getRobotPose ();
  }

  // TODO BETTER NAMES FOR SENSOR GETTERS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  public boolean getSensorClockWise() {
    return !sensor1.get();
  }

  public boolean getSensorCounterClockWise() {
    return false;
  }

  public TurretMode getMode() {
    return mode;
  }

  public void setMode(TurretMode mode) {
    this.mode = mode;
  }

  public void setKP(double newKP){
    kpValue = newKP;
    motor.config_kP(0, kpValue);
  }

  public double getKp(){
    return kpValue;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // set POSE from Vision Data
    utilities.setPoseFromVisionData(robotContainer);
    checkLimit();
    switch (mode) {
      case FOLLOW:
        periodicFollow();
        break;
      case STOP:
        periodicStop();
        break;
      case SIDE:
        periodicSide();
    }
      //ffsearch2 = SmartDashboard.getNumber("REALGETFF", 0.0);
      //setPower(ffsearch2);
  }

  private void checkLimit() {
    if (getSensorClockWise()) {

    }

  }

  private void periodicFollow() {
    aimToBasket.execute();
  }

  public void periodicSide() {
    aimToBasket.execute();
    Shuffleboard.getTab("title").add(this);
  }

  public void setVelocity(double vel){
    motor.set(ControlMode.Velocity, vel);
  }

  public void periodicStop() {
    // set motor to break
  }

  public double getFfValue(){
    return Constants.TURRET_MOTOR_KP;
  }

  public void setff(double newff){
    ff = newff;
  }

  public void setFfSearch(double newff){
   // ffsearch = newff;
  }

  public double getFfSearch(){
    return ffsearch;
  }

  

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Angle", this::getAngle, null);
    builder.addDoubleProperty("KP", this::getKp, null);
    builder.addDoubleProperty("ENCODER", this::getEncoder, null);
    builder.addDoubleProperty("FF", this::getFF, null);
    builder.addDoubleProperty("TARGET ANGLE", this::getOptimizedAngle, null);
    builder.addDoubleProperty("CHANGEFF", this::getFfSearch, this::setFfSearch);
    SmartDashboard.putNumber("REALGETFF", 0);
    builder.addBooleanProperty("SENSOR", this::getSensorClockWise, null);
  }
}
