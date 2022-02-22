package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake {
    VictorSPX intakeMotor;
    VictorSPX indexMotor;

    Solenoid intakePiston;
    
    public Intake(final int intakeMotorID, final int indexMotorID){
        intakeMotor = new VictorSPX(intakeMotorID);
        indexMotor = new VictorSPX(indexMotorID);

        intakeMotor.setInverted(true);
        indexMotor.setInverted(false);

        intakePiston = new Solenoid(Constants.PNUEMATIC_HUB_ID, PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID_CHANNEL);
    }

    public void extendIntake(){
        intakePiston.set(true);
    }

    public void retractIntake(){
        intakePiston.set(false);
    }

    public void toggleIntake(){
        intakePiston.toggle();
    }

    public void runIntake(double speed){
        intakeMotor.set(ControlMode.PercentOutput, speed);
        //indexMotor.set(ControlMode.PercentOutput, 0.5);
    }

    public void stopIntake(){
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
        //indexMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void runIndex(double speed){
        indexMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopIndex(){
        indexMotor.set(ControlMode.PercentOutput, 0.0);
    }
}
