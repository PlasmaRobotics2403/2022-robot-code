package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake {
    VictorSPX intakeMotor;
    VictorSPX kickerMotor;

    Solenoid intakePiston;

    TalonSRX indexMotor;

    DigitalInput frontIndexSensor;
    DigitalInput backIndexSensor;
    
    public Intake(final int intakeMotorID, final int kickerMotorID, final int indexMotorID, int frontIndexSensorID, int backIndexSensorID){
        intakeMotor = new VictorSPX(intakeMotorID);
        kickerMotor = new VictorSPX(kickerMotorID);
        indexMotor = new TalonSRX(indexMotorID);

        intakeMotor.setInverted(true);
        kickerMotor.setInverted(false);
        indexMotor.setInverted(false);

        intakePiston = new Solenoid(Constants.PNUEMATIC_HUB_ID, PneumaticsModuleType.REVPH, Constants.INTAKE_SOLENOID_CHANNEL);

        indexMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        indexMotor.setSelectedSensorPosition(0, 0, 0);
        indexMotor.configClosedloopRamp(0, 300);
        indexMotor.configOpenloopRamp(0, 300);

        indexMotor.config_kF(0, 0.0, 300);
        indexMotor.config_kP(0, 0.1, 300);
        indexMotor.config_kI(0, 0.0, 300);
        indexMotor.config_kD(0, 0, 300);
        indexMotor.config_IntegralZone(0, 30, 300);

        limitCurrent(indexMotor);

        frontIndexSensor = new DigitalInput(frontIndexSensorID);
        backIndexSensor = new DigitalInput(backIndexSensorID);
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

    public void runKicker(double speed){
        kickerMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopKicker(){
        kickerMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void runIndex(double speed){
        indexMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopIndex(){
        indexMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void limitCurrent(final TalonSRX talon){
        talon.configPeakCurrentDuration(0, 1000);
        talon.configPeakCurrentLimit(15, 1000);
        talon.configContinuousCurrentLimit(15, 1000);
        talon.enableCurrentLimit(true);
    }


    public boolean getFrontIndexSensorState(){
        return frontIndexSensor.get();
    }

    public boolean getBackIndexSensorState(){
        return backIndexSensor.get();
    }


}
