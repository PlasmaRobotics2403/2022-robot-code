package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Climb {

    TalonFX climbMotor;
    TalonFX pivotMotor; 
    Solenoid climbPiston;

    public Climb(final int climbMotorID, final int pivotMotorID){
        climbMotor = new WPI_TalonFX(climbMotorID);
        pivotMotor = new WPI_TalonFX(pivotMotorID);


        climbMotor.setInverted(true);
        climbMotor.setNeutralMode(NeutralMode.Brake);
        climbMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        climbMotor.setSelectedSensorPosition(0, 0, 0);

        climbMotor.configClosedloopRamp(0.2, 300);
        climbMotor.configOpenloopRamp(0, 300);
        climbMotor.config_kF(0, 0.0, 300);
        climbMotor.config_kP(0, 0.075, 300);
        climbMotor.config_kI(0, 0.4, 300);
        climbMotor.config_kD(0, 0.008, 300);
        climbMotor.config_IntegralZone(0, 30, 300);

        currentLimit(climbMotor);

        pivotMotor.setInverted(false);
        pivotMotor.setNeutralMode(NeutralMode.Brake);
        pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        pivotMotor.setSelectedSensorPosition(0, 0, 0);

        pivotMotor.configClosedloopRamp(0, 300);
        pivotMotor.configOpenloopRamp(0, 300);
        pivotMotor.config_kF(0, 0.00, 300);
        pivotMotor.config_kP(0, 0.0, 300);
        pivotMotor.config_kI(0, 0.0, 300);
        pivotMotor.config_kD(0, 0, 300);
        pivotMotor.config_IntegralZone(0, 30, 300);

        currentLimit(pivotMotor);

        climbPiston = new Solenoid(Constants.PNUEMATIC_HUB_ID, PneumaticsModuleType.REVPH, Constants.CLIMB_SOLENOID_CHANNEL);

    }

    public void extendArms(){
        climbPiston.set(true);
    }

    public void retractArms(){
        climbPiston.set(false);
    }

    public void runClimb(double speed){
        //climbMotor.set(ControlMode.PercentOutput, speed);
        if(speed > 0 && getMainClimbPosition() < Constants.MAX_CLIMB_DISTANCE){
            climbMotor.set(ControlMode.PercentOutput, speed);
        }
        else if(speed < 0 && getMainClimbPosition() > Constants.MIN_CLIMB_DISTANCE){
            climbMotor.set(ControlMode.PercentOutput, speed);
        }
        else{
            climbMotor.set(ControlMode.PercentOutput, 0.0);
        }
    }

    public void setClimbPosition(double position){
        climbMotor.set(ControlMode.Position, position);
    }

    public double getMainClimbPosition(){
        return climbMotor.getSelectedSensorPosition();
    }

    public void runPivotMotor(double speed){
        pivotMotor.set(ControlMode.PercentOutput, speed);
    }

    public double getPivotMotorPosition(){
        return pivotMotor.getSelectedSensorPosition();
    }

    public void currentLimit(final TalonFX talon) {
        talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30,0));
      }
}
