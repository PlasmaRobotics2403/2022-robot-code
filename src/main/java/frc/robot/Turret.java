package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret {
    TalonSRX turretMotor;

    DigitalInput minLimit;
    DigitalInput maxLimit;

    boolean isCalibrated;
    boolean isTracking;

    double targetAngle;
    double turretOffSet;
    
    public Turret(final int turretMotorID){
        turretMotor = new TalonSRX(turretMotorID);

        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        turretMotor.setSelectedSensorPosition(0, 0, 0);

        turretMotor.configClosedloopRamp(0.2);
        turretMotor.configOpenloopRamp(0);

        turretMotor.config_kF(0, 0.0, 30);
        turretMotor.config_kP(0, 0.35, 30);
        turretMotor.config_kI(0, 0.005, 30);
        turretMotor.config_kD(0, 30, 30);
        turretMotor.config_IntegralZone(0, 500, 30);

        turretMotor.setNeutralMode(NeutralMode.Brake);

        turretMotor.setInverted(true);

        limitCurrent(turretMotor);
    }

    public void turn(double turnVal){
        if(turretMotor.getSelectedSensorPosition() > Constants.MIN_LIMIT_DISTANCE && turretMotor.getSelectedSensorPosition() < Constants.MAX_LIMIT_DISTANCE){
            turnVal *= 1;//Constants.MAX_TURRET_SPEED;
        }
        else if(turretMotor.getSelectedSensorPosition() > Constants.MAX_LIMIT_DISTANCE){
            if(turnVal < 0){
                turnVal *= 1;
            }
            else {
                turnVal = 0;
            }
        }
        else if(turretMotor.getSelectedSensorPosition() < Constants.MIN_LIMIT_DISTANCE){
            if(turnVal > 0){
                turnVal *= 1;
            }
            else {
                turnVal = 0;
            }
        }
        else {
            turnVal = 0;
        }

        turretMotor.set(ControlMode.PercentOutput, turnVal);

    }

    public double getTurretSpeed(){
        return turretMotor.getMotorOutputPercent();
    }

    public double getTurretPosition(){
        return turretMotor.getSelectedSensorPosition();
    }

    public double getTurretAngle(){
        return turretMotor.getSelectedSensorPosition() * (360.0 / Constants.ENCODER_TICKS_PER_ROTATION);
    }

    public void setTurretPosition(double angle){
        int position = angleToEncoderPosition(angle);
        turretMotor.set(ControlMode.Position, position);
        SmartDashboard.putNumber("turret Target Position", angleToEncoderPosition(angle));
    }

    public int angleToEncoderPosition(double angle){
        int position = (int) (angle * (Constants.ENCODER_TICKS_PER_ROTATION / 360));
        return position;
    }

    public void zeroTurretPosition(){
        turretMotor.setSelectedSensorPosition(0, 0, 0);
    }

    public void limitCurrent(final TalonSRX talon){
        talon.configPeakCurrentDuration(0, 1000);
        talon.configPeakCurrentLimit(15, 1000);
        talon.configContinuousCurrentLimit(15, 1000);
        talon.enableCurrentLimit(true);
    }
}
