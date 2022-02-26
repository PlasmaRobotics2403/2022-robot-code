// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
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

  private Compressor compressor;

  private double shooterSpeed;
  private double secondShooterSpeed;

  private double intakeSpeed;     //convert to constant after testing
  private double indexSpeed;      //convert to constant after testing


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
    shooter = new Shooter(Constants.SHOOTER_MAIN_MOTOR_ID,Constants.SHOOTER_ACCELERATOR_MOTOR_ID);
    intake = new Intake(Constants.INTAKE_ID, Constants.INDEX_ID);

    compressor = new Compressor(PneumaticsModuleType.REVPH);
    
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

    shooterSpeed = (double) SmartDashboard.getNumber("Front Shooter Percent Output", 0.0);
    SmartDashboard.putNumber("Front Shooter Percent Output", shooterSpeed);

    secondShooterSpeed = (double) SmartDashboard.getNumber("Back Shooter Percent Output", 0.0);
    SmartDashboard.putNumber("Back Shooter Percent Output", secondShooterSpeed);

    intakeSpeed = (double) SmartDashboard.getNumber("Intake Percent Output", 0.0);
    SmartDashboard.putNumber("Intake Percent Output", intakeSpeed);

    indexSpeed = (double) SmartDashboard.getNumber("Index Percent Output", 0.0);
    SmartDashboard.putNumber("Index Percent Output", indexSpeed);

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
  }

  public void driverControls(final PlasmaJoystick joystick){
    drive.drive(joystick.LeftY.getFilteredAxis(), joystick.RightX.getFilteredAxis());

    if(joystick.RT.isPressed()){
      shooter.spinFlyWheel(shooterSpeed); //0.55, 0.7
      shooter.spinAcceleratorWheel(secondShooterSpeed); //1.0
    }
    else {
      shooter.stopShooter();
    }

    if(joystick.X.isPressed()){
      intake.extendIntake();
    }
    else if(joystick.Y.isPressed()){
      intake.retractIntake();
    }

    if(joystick.LT.isPressed()){
      intake.runIntake(intakeSpeed);
      intake.runIndex(indexSpeed);
    }
    else{
      intake.stopIntake();
      intake.stopIndex();
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
