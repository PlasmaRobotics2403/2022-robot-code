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

        mainMotor.setInverted(false);
        mainMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        mainMotor.setSelectedSensorPosition(0, 0, 0);
        mainMotor.setNeutralMode(NeutralMode.Brake);
        limitCurrent(mainMotor);

        //mainMotor.configClosedloopRamp(0, 300);
        //mainMotor.configOpenloopRamp(0, 300);
        
        mainMotor.config_kF(0, 0.060, 300);
        mainMotor.config_kP(0, 0.2, 300);
        mainMotor.config_kI(0, 0.00, 300);
        mainMotor.config_kD(0, 10, 300);
        mainMotor.config_IntegralZone(0, 0, 300);

        //mainMotor.getClosedLoopError()

        
        shooterPiston = new Solenoid(Constants.PNUEMATIC_HUB_ID, PneumaticsModuleType.REVPH, Constants.SHOOTER_SOLENOID_CHANNEL);

    }

    public void stopShooter(){
        mainMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void spinFlyWheel(double speed){
        mainMotor.set(ControlMode.Velocity, speed);
    }

    public double getShooterSpeed(){
        return mainMotor.getSelectedSensorVelocity();
    }

    public void autoShoot(double distance){
        //distance in feet
        double speed = 0;
        if(distance > 2.5){
            speed = 240.57 * distance + 5716.2;
            extendHood();
        }
        else if(distance > 0) {
            speed = 500 * Math.pow(distance, 2) - 3 * Math.pow(10, -11) * distance + 6500; //7500
            retractHood();
        }
        mainMotor.set(ControlMode.Velocity, speed);
    }

    public double getTargetShootSpeed(double distance){
        if(distance > 2.5){
            return 240.57 * distance + 5716.2; //5471.4
        } 
        else if(distance > 0){
            return 500 * Math.pow(distance, 2) - 3 * Math.pow(10, -11) * distance + 6500; //7500
        }
        else {
            return 0;
        }
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

    public void extendHood(){
        shooterPiston.set(true);
    }

    public void retractHood(){
        shooterPiston.set(false);
    }
}
