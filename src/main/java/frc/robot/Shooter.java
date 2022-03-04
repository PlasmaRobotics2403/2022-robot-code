package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Shooter {
    TalonFX mainMotor;
    Solenoid shooterPiston;


    public Shooter(final int mainMotorID){
        mainMotor = new TalonFX(mainMotorID);

        mainMotor.setInverted(true);
        mainMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        mainMotor.setSelectedSensorPosition(0, 0, 0);
        mainMotor.setNeutralMode(NeutralMode.Brake);
        //limitCurrent(mainMotor);

        
        shooterPiston = new Solenoid(Constants.PNUEMATIC_HUB_ID, PneumaticsModuleType.REVPH, Constants.SHOOTER_SOLENOID_CHANNEL);

    }

    public void stopShooter(){
        mainMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void spinFlyWheel(double speed){
        mainMotor.set(ControlMode.PercentOutput, speed);
    }

    public double getShooterSpeed(){
        return mainMotor.getSelectedSensorVelocity();
    }

    public void limitCurrent(final TalonFX talon){
        talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 35, 0));
    }

    public void limitCurrent(final TalonSRX talon){
        talon.configPeakCurrentDuration(0, 1000);
        talon.configPeakCurrentLimit(15, 1000);
        talon.configContinuousCurrentLimit(15, 1000);
        talon.enableCurrentLimit(true);
    }

    public void extendShooter(){
        shooterPiston.set(true);
    }

    public void retractShooter(){
        shooterPiston.set(false);
    }
}
