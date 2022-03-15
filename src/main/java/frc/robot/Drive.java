/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {


    public WPI_TalonFX leftDrive;
    public WPI_TalonFX leftDriveSlave;
    public WPI_TalonFX rightDrive;
    public WPI_TalonFX rightDriveSlave;

    public MotorControllerGroup leftMotorController;
    public MotorControllerGroup rightMotorController;

    public DifferentialDrive diffDrive;

    private final AHRS navX;
    private double gyroAngle;
    private double gyroPitch;

    public DifferentialDriveOdometry odometry;
    public DifferentialDriveKinematics kinematics;
    public SimpleMotorFeedforward feedForward;
  

    public Drive(final int leftDriveID, final int leftDriveSlaveID, final int rightDriveID, final int rightDriveSlaveID){
      leftDrive = new WPI_TalonFX(leftDriveID);
      leftDriveSlave = new WPI_TalonFX(leftDriveSlaveID);
      rightDrive = new WPI_TalonFX(rightDriveID);
      rightDriveSlave = new WPI_TalonFX(rightDriveSlaveID);

      leftDrive.configFactoryDefault();
      rightDrive.configFactoryDefault();
      leftDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
		  rightDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);

      leftDrive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 0);
		  leftDrive.setSensorPhase(true);
		  //leftDrive.configClosedloopRamp(0);
		  leftDrive.configNominalOutputForward(0, 30);
		  leftDrive.configNominalOutputReverse(0, 30);
		  leftDrive.configPeakOutputForward(1, 30);
		  leftDrive.configPeakOutputReverse(-1, 30);
		  leftDrive.config_kF(0, .35, 30); // should get close to distance defined with everything else zero
		  leftDrive.config_kP(0, 1.2, 30); // occilate around error
		  leftDrive.config_kI(0, 0.005, 30);
		  leftDrive.config_kD(0, 25, 30);
      leftDrive.config_IntegralZone(0, 0, 30);
      

      rightDrive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 0);
      rightDrive.configAuxPIDPolarity(true);
		  //rightDrive.setSensorPhase(true);
		  //rightDrive.configClosedloopRamp(0);
		  rightDrive.configNominalOutputForward(0, 30);
		  rightDrive.configNominalOutputReverse(0, 30);
		  rightDrive.configPeakOutputForward(1, 30);
		  rightDrive.configPeakOutputReverse(-1, 30);
		  rightDrive.config_kF(0, .35, 30);
		  rightDrive.config_kP(0, 1.2, 30);
		  rightDrive.config_kI(0, 0.005, 30);
		  rightDrive.config_kD(0, 25, 30);
		  rightDrive.config_IntegralZone(0, 0, 30);
      
      leftDrive.setSelectedSensorPosition(0, 0, 0);
		  rightDrive.setSelectedSensorPosition(0, 0, 0);

		  leftDrive.set(ControlMode.Position, 0);
		  rightDrive.set(ControlMode.Position, 0);

		  DriverStation.reportError("left position: " + leftDrive.getSelectedSensorPosition(0), false);
		  DriverStation.reportError("right position: " + rightDrive.getSelectedSensorPosition(0), false);


		  leftDrive.setInverted(false);
      leftDriveSlave.setInverted(false);

		  rightDrive.setInverted(true);
      rightDriveSlave.setInverted(true);

      
      leftDrive.setNeutralMode(NeutralMode.Brake);
      leftDriveSlave.setNeutralMode(NeutralMode.Brake);
      rightDrive.setNeutralMode(NeutralMode.Brake);
      rightDriveSlave.setNeutralMode(NeutralMode.Brake);


      leftDrive.configOpenloopRamp(0.3);
      leftDriveSlave.configOpenloopRamp(0.3);
      rightDrive.configOpenloopRamp(0.3);
      rightDriveSlave.configOpenloopRamp(0.3);

      currentLimit(leftDrive);
      currentLimit(leftDriveSlave);
      currentLimit(rightDrive);
      currentLimit(rightDriveSlave);

      leftMotorController = new MotorControllerGroup(leftDrive, leftDriveSlave);
      rightMotorController = new MotorControllerGroup(rightDrive, rightDriveSlave);



      diffDrive = new DifferentialDrive(leftMotorController, rightMotorController);


      navX = new AHRS(SPI.Port.kMXP);

      kinematics = new DifferentialDriveKinematics(Constants.WHEEL_BASE);
      odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
      feedForward = new SimpleMotorFeedforward(0.1, 1.0, 0.2);
    }

    
    public void drive(double speed, double rotation) {
      diffDrive.arcadeDrive(speed, rotation, true);
    }


    public void currentLimit(final TalonFX talon) {
      talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30,0));
    }

    public void setToBrake(){
      leftDrive.setNeutralMode(NeutralMode.Brake);
      rightDrive.setNeutralMode(NeutralMode.Brake);
    }

    public void setToCoast(){
      leftDrive.setNeutralMode(NeutralMode.Coast);
      rightDrive.setNeutralMode(NeutralMode.Coast);
    }

    
    public void resetEncoders() {
      leftDrive.setSelectedSensorPosition(0, 0, 0);
      rightDrive.setSelectedSensorPosition(0, 0, 0);
      DriverStation.reportWarning("resetting encoders", false);
      while (Math.abs(getDistance()) < -1 && Math.abs(getDistance()) > 1) {
        leftDrive.setSelectedSensorPosition(0, 0, 0);
        rightDrive.setSelectedSensorPosition(0, 0, 0);
        DriverStation.reportWarning("Stuck in loop", false);
      }
    }

    public double getDistance() {
      return (toDistance(rightDrive) + toDistance(leftDrive)) / 2;
    }

    public double getLeftVelocity() {
      return leftDrive.getSelectedSensorVelocity();
    }

    public double getRightVelocity() {
      return rightDrive.getSelectedSensorVelocity();
    }
  
    public double getLeftDistance() {
      return toDistance(leftDrive);
    }

    public double getRightDistance() {
      return toDistance(rightDrive);
    }
  
    private static double toDistance(final TalonFX talon) {
      final double distance = (talon.getSelectedSensorPosition()/ (2048.0/((13.0/50.0)*(24.0/50.0)))) * 2.0 * Math.PI * Units.inchesToMeters(3);
      return distance;
    }


    public void updateGyro() {
      gyroAngle = navX.getYaw();
      gyroPitch = navX.getPitch();
    }

    public void setGyroAngle(double angle){
      navX.setAngleAdjustment(angle);
    }

    public double getGyroAngle(){
      updateGyro();
      return gyroAngle;
    }

    public double getGyroPitch(){
      updateGyro();
      return gyroPitch;
    }

    public void zeroGyro() {
      navX.zeroYaw();
    }

    public void changeGyroAngle(double angle){
      navX.setAngleAdjustment(angle);
    }
  

    public void updateOdometry(){
      odometry.update(Rotation2d.fromDegrees(-1*navX.getAngle()), getLeftDistance(), getRightDistance());
    }

    public Pose2d getPose(){
      return odometry.getPoseMeters();
    }

    public void resetOdometry(){
      resetEncoders();
    odometry.resetPosition(new Pose2d(0, 0, new Rotation2d(0)), Rotation2d.fromDegrees(-1*getGyroAngle()));
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
      return new DifferentialDriveWheelSpeeds(
          (leftDrive.getSelectedSensorVelocity()/(2048.0/((13.0/50.0)*(24.0/50.0))) * 2.0 * Math.PI * Units.inchesToMeters(3)),
          (rightDrive.getSelectedSensorVelocity()/(2048.0/((13.0/50.0)*(24.0/50.0))) * 2.0 * Math.PI * Units.inchesToMeters(3)));
    }

    public DifferentialDriveKinematics getKinematics(){
      return kinematics;
    }

    public SimpleMotorFeedforward getFeedforward(){
      return feedForward;
    }

  public void setOutput(double leftVolts, double rightVolts) {
    leftDrive.setVoltage(leftVolts);
    leftDriveSlave.setVoltage(leftVolts);
    rightDrive.setVoltage(rightVolts);
    rightDriveSlave.setVoltage(rightVolts);
    diffDrive.feed();

    SmartDashboard.putNumber("left drive voltage", leftVolts);
    SmartDashboard.putNumber("right drive voltage", rightVolts);
  }    

  public void leftWheelDrive(final double speed){
    leftDrive.set(ControlMode.PercentOutput, speed);
    leftDriveSlave.set(ControlMode.PercentOutput, speed);
  }

  public void rightWheelDrive(final double speed){
    rightDrive.set(ControlMode.PercentOutput, speed);
    rightDriveSlave.set(ControlMode.PercentOutput, speed);
  }

  public void autonTankDrive(final double left, final double right){
    leftWheelDrive(left);
    rightWheelDrive(right);
  }

}
