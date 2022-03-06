// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controllers.PlasmaJoystick;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  public PlasmaJoystick joystick;
  public Drive drive;
  public Shooter shooter;
  public Intake intake;
  public Turret turret;
  public Climb climb;

  private Compressor compressor;
  private PowerDistribution pdh;

  private double shooterSpeed;

  private double indexSpeed;      //convert to constant after testing
  private double turretSpeed;

  private boolean isBraked;
  private boolean clearStickyFaults;

  
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tv;
  NetworkTableEntry ts;
  
  double vision_X;
  double vision_Y;
  double vision_Area;

  double turretTargetAngle;

  double distanceFromTarget;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    joystick = new PlasmaJoystick(Constants.JOYSTICK1_PORT);
    drive = new Drive(Constants.L_DRIVE_ID, Constants.L_DRIVE_SLAVE_ID, Constants.R_DRIVE_ID, Constants.R_DRIVE_SLAVE_ID);
    shooter = new Shooter(Constants.SHOOTER_MAIN_MOTOR_ID);
    intake = new Intake(Constants.INTAKE_ID, Constants.KICKER_ID, Constants.INDEX_ID, Constants.FRONT_INDEX_SENSOR_ID);
    turret = new Turret(Constants.TURRET_ID);
    climb = new Climb(Constants.CLIMB_ID);

    compressor = new Compressor(PneumaticsModuleType.REVPH);
    pdh = new PowerDistribution(Constants.POWER_DISTRIBUTION_HUB, ModuleType.kRev);

    drive.zeroGyro();

    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    ts = table.getEntry("ts");

    //table.getEntry("ledMode").setNumber(1);
    //table.getEntry("pipeline").setNumber(0);

    turretTargetAngle = 0;
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    vision_X = tx.getDouble(0.0);
    vision_Y = ty.getDouble(0.0);
    vision_Area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", vision_X);
    SmartDashboard.putNumber("LimelightY", vision_Y);
    SmartDashboard.putNumber("LimelightArea", vision_Area);

    distanceFromTarget = (((Constants.UPPER_HUB_HEIGHT - Constants.CAMERA_HEIGHT) / Math.tan(Constants.CAMERA_ANGLE*Math.PI/180 + (vision_Y)*Math.PI/180))/12 - 2);
    SmartDashboard.putNumber("Distance from Target", distanceFromTarget);

    shooterSpeed = (double) SmartDashboard.getNumber("Set Shooter Speed", 0.0);
    SmartDashboard.putNumber("Set Shooter Speed", shooterSpeed);

    turretSpeed = (double) SmartDashboard.getNumber("Turret Percent Output", 0.0);
    SmartDashboard.putNumber("Turret Percent Output", turretSpeed);
    SmartDashboard.putNumber("Turret speed", turret.getTurretSpeed());


    isBraked = (boolean) SmartDashboard.getBoolean("Brake Mode", false);
    SmartDashboard.putBoolean("Brake Mode", isBraked);
    if(isBraked) {
      drive.setToBrake();
    }
    else {
      drive.setToCoast();
    }

    clearStickyFaults = (boolean) SmartDashboard.getBoolean("Clear Sticky Faults", false);
    SmartDashboard.putBoolean("Clear Sticky Faults", clearStickyFaults);
    if(clearStickyFaults){
      pdh.clearStickyFaults();
      clearStickyFaults = false;
    }

    SmartDashboard.putNumber("Turret Position", turret.getTurretPosition());
    SmartDashboard.putNumber("Turret Angle", turret.getTurretAngle());

    SmartDashboard.putBoolean("Front Index Sensor State", intake.getFrontIndexSensorState());

    SmartDashboard.putNumber("Index Position", intake.getIndexPosition());

    SmartDashboard.putNumber("gyro angle", drive.getGyroAngle());

    SmartDashboard.putNumber("Turret Target Angle", turretTargetAngle);
    
    SmartDashboard.putNumber("Shooter Speed", shooter.getShooterSpeed());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    compressor.enableDigital();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    driverControls(joystick);
    //visionControls();
  }

  public void driverControls(final PlasmaJoystick joystick){
    drive.drive(joystick.LeftY.getFilteredAxis(), joystick.RightX.getFilteredAxis());

    if(joystick.RT.isPressed()){
      //shooter.spinFlyWheel(shooterSpeed); //0.55, 0.7
      SmartDashboard.putNumber("target speed", Math.round(shooter.getTargetShootSpeed(distanceFromTarget)));
      if(vision_Area != 0){
        shooter.autoShoot(distanceFromTarget);
        double errorValue = Math.abs(shooter.getTargetShootSpeed(distanceFromTarget) - shooter.getShooterSpeed());
        SmartDashboard.putNumber("shooter error", errorValue);
        if(errorValue < 400){
          intake.runIndex(Constants.INDEX_SPEED);
        }
        else {
          intake.stopIndex();
        }
      }
    }
    else if(intake.getFrontIndexSensorState() == false){
       intake.advanceBall();
    }
    else {
      shooter.stopShooter();
      shooter.retractHood();
      intake.stopIndex();
    }

    // if(joystick.B.isPressed()){
    //   intake.runIndex(Constants.INDEX_SPEED);
    // }
    // else if(intake.getFrontIndexSensorState() == false){
    //   intake.advanceBall();
    // }
    // else {
    //   intake.stopIndex();
    // }

    if(joystick.X.isToggledOn()){
      intake.extendIntake();
    }
    else {
      intake.retractIntake();
    }

    if(joystick.A.isToggledOn()){
      //shooter.extendHood();
    }
    else {
      //shooter.retractHood();
    }


    if(joystick.LT.isPressed()){
      //intake.extendIntake();
      intake.runIntake(Constants.INTAKE_SPEED);
      intake.runKicker(Constants.KICKER_SPEED);
    }
    else{
      //intake.retractIntake();
      intake.stopIntake();
      intake.stopKicker();
    }
    

    
    if(joystick.Y.isPressed()){
      visionControls();
    }
    else if(joystick.RB.isPressed()){
      turret.turn(Constants.TURRET_SPEED);
    }
    else if(joystick.LB.isPressed()){
      turret.turn(-Constants.TURRET_SPEED);
    }
    else{
      turret.turn(0);
    }

    if(joystick.A.isPressed()){
      //intake.zeroIndexPosition();
      //turret.zeroTurretPosition();
    }


  }

  public void visionControls(){
    /*if(vision_Area != 0){

      double visionSpeed = 0.03 * Math.pow(Math.abs(vision_X), 0.38);

      if(vision_X > 2){
        turret.turn(visionSpeed);
      }
      else if(vision_X < 2){
        turret.turn(-visionSpeed);
      }
      else {
        turret.turn(0);
      }
    }
    else {
      turret.setTurretPosition(0.0);
    }*/

    
    /*float Kp = -0.1f;
    float min_command = 0.05f;
    if(vision_Area != 0){
      float steeringTurretAdust = 0;

      if(vision_X > 1.0){
        steeringTurretAdust = Kp * (float)vision_X - min_command;
      }else if(vision_X < 1.0){
        steeringTurretAdust = Kp * (float)vision_X + min_command;
      }

      turret.turn(steeringTurretAdust);

    }*/

    if(vision_Area != 0){
      if(Math.abs(vision_X) > 10){
        turretTargetAngle = turret.getTurretAngle()/1.03 + vision_X;
        turret.setTurretPosition(turretTargetAngle);
      }
    }
    else {
      turret.setTurretPosition(0.0);
    }


  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
