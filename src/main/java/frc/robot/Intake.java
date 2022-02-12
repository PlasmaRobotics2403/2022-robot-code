package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake {
    Solenoid leftPiston;
    Solenoid rightPiston;
    
    public Intake( ){
        leftPiston = new Solenoid(Constants.PNUEMATIC_HUB_ID, PneumaticsModuleType.REVPH, Constants.L_INTAKE_SOLENOID_CHANNEL);
        rightPiston = new Solenoid(Constants.PNUEMATIC_HUB_ID, PneumaticsModuleType.REVPH, Constants.R_INTAKE_SOLENOID_CHANNEL);
    }

    public void extendIntake(){
        leftPiston.set(true);
        rightPiston.set(true);
    }

    public void retractIntake(){
        leftPiston.set(false);
        rightPiston.set(false);
    }
}
