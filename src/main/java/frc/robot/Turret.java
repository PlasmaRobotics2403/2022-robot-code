package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Turret {
    TalonSRX turretMotor;
    
    public Turret(final int turretMotorID){
        turretMotor = new TalonSRX(turretMotorID);

        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        turretMotor.setSelectedSensorPosition(0, 0, 0);
        turretMotor.configClosedloopRamp(0, 300);
        turretMotor.configOpenloopRamp(0, 300);

        turretMotor.config_kF(0, 0.0, 300);
        turretMotor.config_kP(0, 0.1, 300);
        turretMotor.config_kI(0, 0.0, 300);
        turretMotor.config_kD(0, 0, 300);
        turretMotor.config_IntegralZone(0, 30, 300);

        limitCurrent(turretMotor);
    }

    

    public void limitCurrent(final TalonSRX talon){
        talon.configPeakCurrentDuration(0, 1000);
        talon.configPeakCurrentLimit(15, 1000);
        talon.configContinuousCurrentLimit(15, 1000);
        talon.enableCurrentLimit(true);
    }
}