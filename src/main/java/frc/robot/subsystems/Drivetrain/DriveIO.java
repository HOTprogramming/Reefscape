package frc.robot.subsystems.Drivetrain;


import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.DriverStation;

public interface DriveIO {
    // Data to be logged 
    class DriveIOdata { 
        public SwerveDriveState state = new SwerveDriveState();


        public boolean frontLeftDriveconnected = true;
        public double frontLeftDrivePositionRads = 0.0;
        public double frontLeftDriveVelocityRpm = 0.0;
        public double frontLeftDriveAppliedVolts = 0.0;
        public double frontLeftDriveSupplyCurrentAmps = 0.0;
        public double frontLeftDriveTorqueCurrentAmps = 0.0;
        public double frontLeftDriveTempCelsius = 0.0;
    
        public boolean frontRightDriveconnected = true;
        public double frontRightDrivePositionRads = 0.0;
        public double frontRightDriveVelocityRpm = 0.0;
        public double frontRightDriveAppliedVolts = 0.0;
        public double frontRightDriveSupplyCurrentAmps = 0.0;
        public double frontRightDriveTorqueCurrentAmps = 0.0;
        public double frontRightDriveTempCelsius = 0.0;

        public boolean backLeftDriveconnected = true;
        public double backLeftDrivePositionRads = 0.0;
        public double backLeftDriveVelocityRpm = 0.0;
        public double backLeftDriveAppliedVolts = 0.0;
        public double backLeftDriveSupplyCurrentAmps = 0.0;
        public double backLeftDriveTorqueCurrentAmps = 0.0;
        public double backLeftDriveTempCelsius = 0.0;

        public boolean backRightDriveconnected = true;
        public double backRightDrivePositionRads = 0.0;
        public double backRightDriveVelocityRpm = 0.0;
        public double backRightDriveAppliedVolts = 0.0;
        public double backRightDriveSupplyCurrentAmps = 0.0;
        public double backRightDriveTorqueCurrentAmps = 0.0;
        public double backRightDriveTempCelsius = 0.0;

        public boolean frontLeftSteerconnected = true;
        public double frontLeftSteerPositionRads = 0.0;
        public double frontLeftSteerVelocityRpm = 0.0;
        public double frontLeftSteerAppliedVolts = 0.0;
        public double frontLeftSteerSupplyCurrentAmps = 0.0;
        public double frontLeftSteerTorqueCurrentAmps = 0.0;
        public double frontLeftSteerTempCelsius = 0.0;
    
        public boolean frontRightSteerconnected = true;
        public double frontRightSteerPositionRads = 0.0;
        public double frontRightSteerVelocityRpm = 0.0;
        public double frontRightSteerAppliedVolts = 0.0;
        public double frontRightSteerSupplyCurrentAmps = 0.0;
        public double frontRightSteerTorqueCurrentAmps = 0.0;
        public double frontRightSteerTempCelsius = 0.0;

        public boolean backLeftSteerconnected = true;
        public double backLeftSteerPositionRads = 0.0;
        public double backLeftSteerVelocityRpm = 0.0;
        public double backLeftSteerAppliedVolts = 0.0;
        public double backLeftSteerSupplyCurrentAmps = 0.0;
        public double backLeftSteerTorqueCurrentAmps = 0.0;
        public double backLeftSteerTempCelsius = 0.0;

        public boolean backRightSteerconnected = true;
        public double backRightSteerPositionRads = 0.0;
        public double backRightSteerVelocityRpm = 0.0;
        public double backRightSteerAppliedVolts = 0.0;
        public double backRightSteerSupplyCurrentAmps = 0.0;
        public double backRightSteerTorqueCurrentAmps = 0.0;
        public double backRightSteerTempCelsius = 0.0;

        public Translation2d[] m_moduleLocations;
    }

    default DriveIOdata update() {
        return null;
    }

    default void updateStats(DriveIOdata stats) {}

    default void setSwerveRequest(SwerveRequest requestToApply) {}

    /**
     * @apiNote NOT NEEDED UNLESS YOU HAVE CAMERAS
     */
    default void setTeamRotation(DriverStation.Alliance alliance) {}

    default void resetPidgeon() {}

    /**
     * @apiNote if you have cameras, this should override odometry with the best input
     */
    default void seedFieldRelative(Pose2d seedling) {}

    default void updateVision(Pose2d calculatedPose, double timestamp, Matrix<N3, N1> stDevs) {}

    default void setOperatorPerspective(Rotation2d rotation2d) {}
}
