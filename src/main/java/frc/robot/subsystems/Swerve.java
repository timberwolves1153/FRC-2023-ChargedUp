package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public ADIS16470_IMU driveGyro;
    public double initRoll;
    public SlewRateLimiter filter;
    public SlewRateLimiter filter2;

    public Swerve() {
        gyro = new WPI_Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        gyro.clearStickyFaults();
        filter = new SlewRateLimiter(10);
        filter2 = new SlewRateLimiter(10);
        //driveGyro = new ADIS16470_IMU();
        //driveGyro.calibrate();
        
        zeroGyro();

        initRoll = gyro.getRoll();

        if (Constants.competitionRobot) {
            mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.CompMod0.constants),
                new SwerveModule(1, Constants.Swerve.CompMod1.constants),
                new SwerveModule(2, Constants.Swerve.CompMod2.constants),
                new SwerveModule(3, Constants.Swerve.CompMod3.constants)
            };
        } else {
            mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.ProtoMod0.constants),
                new SwerveModule(1, Constants.Swerve.ProtoMod1.constants),
                new SwerveModule(2, Constants.Swerve.ProtoMod2.constants),
                new SwerveModule(3, Constants.Swerve.ProtoMod3.constants)
            };
        }

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    filter.calculate(translation.getX()), 
                                    filter2.calculate(translation.getY()), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                  translation.getX(), 
                                  translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }  
    
    public double getInitRoll() {
        return this.initRoll;
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
       gyro.setYaw(0);
      // driveGyro.reset();
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    // public Rotation2d getDriveYaw() {
    //     return Rotation2d.fromDegrees(driveGyro.getAngle());
    // }

    public double getRoll() {
        return gyro.getRoll();
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void xPosition(boolean isOpenLoop){
        SwerveModuleState[] swerveModuleStates =
            new SwerveModuleState[]{
                new SwerveModuleState(1, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(1, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(1, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(1, Rotation2d.fromDegrees(45))
            };
        
       // return swerveModuleStates;

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }

        System.out.println("Set to X Position");
    }

    public void stop() {
        drive(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            0 * Constants.Swerve.maxAngularVelocity, 
            true, 
            true
        );
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  

        if (Constants.tuneSwerve) {
            for(SwerveModule mod : mSwerveMods){
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            }
        }

        SmartDashboard.putNumber("Drive Gyro Heading", gyro.getYaw());
        SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Gyro Roll", gyro.getRoll());
    }
}