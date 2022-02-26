package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Shooter {
    TalonFX mainMotor;
    TalonSRX acceleratorMotor;


    public Shooter(final int mainMotorID){
        mainMotor = new TalonFX(mainMotorID);
        //acceleratorMotor = new TalonSRX(acceleratorMotorID);

        mainMotor.setInverted(true);
        mainMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        mainMotor.setSelectedSensorPosition(0, 0, 0);
        mainMotor.setNeutralMode(NeutralMode.Brake);
        //limitCurrent(mainMotor);

        //acceleratorMotor.setInverted(true);
        //acceleratorMotor.setSelectedSensorPosition(0, 0, 0);
        //acceleratorMotor.setNeutralMode(NeutralMode.Brake);
        //limitCurrent(acceleratorMotor);

    }

    public void stopShooter(){
        mainMotor.set(ControlMode.PercentOutput, 0.0);
        //acceleratorMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void spinFlyWheel(double speed){
        mainMotor.set(ControlMode.PercentOutput, speed);
    }

    public void spinAcceleratorWheel(double speed){
        //acceleratorMotor.set(ControlMode.PercentOutput, speed);
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
}
