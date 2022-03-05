package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Climb {

    TalonFX climbMotor;
    Solenoid climbPiston;

    public Climb(final int climbMotorID){
        climbMotor = new WPI_TalonFX(climbMotorID);

        climbMotor.setInverted(false);
        climbMotor.setNeutralMode(NeutralMode.Brake);
        climbMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        climbMotor.setSelectedSensorPosition(0, 0, 0);

        climbMotor.configClosedloopRamp(0, 300);
        climbMotor.configOpenloopRamp(0, 300);
        climbMotor.config_kF(0, 0.0, 300);
        climbMotor.config_kP(0, 0.1, 300);
        climbMotor.config_kI(0, 0.0, 300);
        climbMotor.config_kD(0, 0, 300);
        climbMotor.config_IntegralZone(0, 30, 300);

        currentLimit(climbMotor);

        climbPiston = new Solenoid(Constants.PNUEMATIC_HUB_ID, PneumaticsModuleType.REVPH, Constants.CLIMB_SOLENOID_CHANNEL);

    }

    public void extendArms(){
        climbPiston.set(true);
    }

    public void retractArms(){
        climbPiston.set(false);
    }

    public void currentLimit(final TalonFX talon) {
        talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30,0));
      }
}
