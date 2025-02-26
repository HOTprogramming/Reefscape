package frc.robot.subsystems.Drivetrain;


import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import java.util.EnumMap;
import java.util.Map;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.DriverStation;

public interface DriveIO {

    public enum SwervePoses{
        FRONT_LEFT,
        FRONT_RIGHT, 
        BACK_LEFT, 
        BACK_RIGHT
    }
    // Data to be logged 
    class DriveIOdata { 
        public SwerveDriveState state = new SwerveDriveState();

        public Rotation3d pigeon;

        Map<SwervePoses, Boolean> driveConnected = new EnumMap<>(SwervePoses.class);
        Map<SwervePoses, Double> drivePosition = new EnumMap<>(SwervePoses.class);
        Map<SwervePoses, Double> driveVelocity = new EnumMap<>(SwervePoses.class);
        Map<SwervePoses, Double> driveVolts = new EnumMap<>(SwervePoses.class);
        Map<SwervePoses, Double> driveSupplyCurrent = new EnumMap<>(SwervePoses.class);
        Map<SwervePoses, Double> driveTorqueCurrent = new EnumMap<>(SwervePoses.class);
        Map<SwervePoses, Double> driveTemp = new EnumMap<>(SwervePoses.class);

        Map<SwervePoses, Boolean> steerConnected = new EnumMap<>(SwervePoses.class);
        Map<SwervePoses, Double> steerPosition = new EnumMap<>(SwervePoses.class);
        Map<SwervePoses, Double> steerVelocity = new EnumMap<>(SwervePoses.class);
        Map<SwervePoses, Double> steerVolts = new EnumMap<>(SwervePoses.class);
        Map<SwervePoses, Double> steerSupplyCurrent = new EnumMap<>(SwervePoses.class);
        Map<SwervePoses, Double> steerTorqueCurrent = new EnumMap<>(SwervePoses.class);
        Map<SwervePoses, Double> steerTemp = new EnumMap<>(SwervePoses.class);

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

    default void setNeutralMode(NeutralModeValue neutralModeValue) {}
}
