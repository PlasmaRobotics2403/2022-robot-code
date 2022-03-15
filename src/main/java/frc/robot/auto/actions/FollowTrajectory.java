package frc.robot.auto.actions;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Drive;
import frc.robot.Intake;
import frc.robot.auto.util.Action;

public class FollowTrajectory  implements Action{
    Drive drive;
    Intake intake;
    //GenerateTrajectory generateTrajectory;

    RamseteCommand ramsete;
    Trajectory trajectory0;
    Trajectory trajectory1;
    TrajectoryConfig config0;
    TrajectoryConfig config1;

    Trajectory[] trajectoryArray;
    int trajectoryNumber;

    /*  WARNING  WARNING  WARNING  */
    //  these values are experimental
    //  they need serious tuning
    //  use robot tuner software
    //  when you have the chance
    double ksVolts = 0.67;
    double kvVoltSecondsPerMeter = 1.0;
    double kaVoltSecondSquaredPerMeter = 0.2;
    double kMaxSpeedMetersPerSecond = 0.6;
    double kMaxAccelerationMetersPerSecondSquared = 0.6;

    double kp = 0.1;
    double kd = 0.01;

    public FollowTrajectory(int trajectoryNumber, Drive drive){
        this.trajectoryNumber = trajectoryNumber;
        this.drive = drive;

        DriverStation.reportWarning("getting trajectory", false);


        

        config0 = new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
            .setKinematics
                (new DifferentialDriveKinematics(Constants.WHEEL_BASE))
            .addConstraint(
                new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondSquaredPerMeter), 
                    new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11));
        trajectory0 = TrajectoryGenerator.generateTrajectory(
            // starting position
            new Pose2d(0, 0, new Rotation2d(0)), 
            // interior points
            List.of(
                new Translation2d(0.6, 0)
            ),
            // end position
            new Pose2d(1.2192, 0, new Rotation2d(0)),
            config0);


        config1 = new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
            .setKinematics
                (new DifferentialDriveKinematics(Constants.WHEEL_BASE))
            .addConstraint(
                new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondSquaredPerMeter), 
                    new DifferentialDriveKinematics(Constants.WHEEL_BASE), 11))
            .setReversed(true);
        trajectory1 = TrajectoryGenerator.generateTrajectory(
            // starting position
            new Pose2d(0, 0, new Rotation2d(0)), 
            // interior points
            List.of(
                new Translation2d(-0.6, 0)
            ),
            // end position
            new Pose2d(-1.2192, 0, new Rotation2d(0)),
            config1);

        trajectoryArray = new Trajectory[5];
        trajectoryArray[0] = trajectory0;
        trajectoryArray[1] = trajectory1;

    }

    @Override
    public boolean isFinished(){
        return ramsete.isFinished();
    }

    @Override
    public void start(){
        DriverStation.reportWarning("before ramsete", false);
        ramsete = new RamseteCommand(trajectoryArray[trajectoryNumber],
             drive::getPose,
             new RamseteController(),
             new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondSquaredPerMeter), 
             new DifferentialDriveKinematics(Constants.WHEEL_BASE), 
             drive::getWheelSpeeds, 
             new PIDController(kp, 0.0, kd), 
             new PIDController(kp, 0.0, kd), 
             drive::setOutput,
             drive
             );

        ramsete.initialize();
        DriverStation.reportWarning("finished creating ramsete", false);
    }

    public FollowTrajectory(Drive drive) {
        this.drive = drive;
    }

    @Override
    public void update(){
        SmartDashboard.putNumber("Left Error", drive.leftDrive.getClosedLoopError(0));
        SmartDashboard.putNumber("Right Error", drive.rightDrive.getClosedLoopError(0));
        ramsete.execute();
        DriverStation.reportWarning("updated", false);
    }

    @Override
    public void end(){
        drive.zeroGyro();
        drive.leftDrive.set(ControlMode.PercentOutput, 0);
        drive.rightDrive.set(ControlMode.PercentOutput, 0);
        drive.leftDriveSlave.set(ControlMode.PercentOutput, 0);
        drive.rightDriveSlave.set(ControlMode.PercentOutput, 0);
        drive.resetEncoders();
        drive.resetOdometry();
    }
}
