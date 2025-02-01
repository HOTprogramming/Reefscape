package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.Drivetrain.DriveConstants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import com.ctre.phoenix6.swerve.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;





public class DriveKraken extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DriveIO {


    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

    private Pigeon2 m_Pigeon2;


    private SwerveDriveState currentState;

    private final StatusSignal<Angle> frontLeftDrivePosition;
    private final StatusSignal<AngularVelocity> frontLeftDriveVelocity;
    private final StatusSignal<Voltage> frontLeftDriveAppliedVolts;
    private final StatusSignal<Current> frontLeftDriveSupplyCurrent;
    private final StatusSignal<Current> frontLeftDriveTorqueCurrent;
    private final StatusSignal<Temperature> frontLeftDriveTempCelsius;

    private final StatusSignal<Angle> frontRightDrivePosition;
    private final StatusSignal<AngularVelocity> frontRightDriveVelocity;
    private final StatusSignal<Voltage> frontRightDriveAppliedVolts;
    private final StatusSignal<Current> frontRightDriveSupplyCurrent;
    private final StatusSignal<Current> frontRightDriveTorqueCurrent;
    private final StatusSignal<Temperature> frontRightDriveTempCelsius;

    private final StatusSignal<Angle> backLeftDrivePosition;
    private final StatusSignal<AngularVelocity> backLeftDriveVelocity;
    private final StatusSignal<Voltage> backLeftDriveAppliedVolts;
    private final StatusSignal<Current> backLeftDriveSupplyCurrent;
    private final StatusSignal<Current> backLeftDriveTorqueCurrent;
    private final StatusSignal<Temperature> backLeftDriveTempCelsius;

    private final StatusSignal<Angle> backRightDrivePosition;
    private final StatusSignal<AngularVelocity> backRightDriveVelocity;
    private final StatusSignal<Voltage> backRightDriveAppliedVolts;
    private final StatusSignal<Current> backRightDriveSupplyCurrent;
    private final StatusSignal<Current> backRightDriveTorqueCurrent;
    private final StatusSignal<Temperature> backRightDriveTempCelsius;

    private final StatusSignal<Angle> frontLeftSteerPosition;
    private final StatusSignal<AngularVelocity> frontLeftSteerVelocity;
    private final StatusSignal<Voltage> frontLeftSteerAppliedVolts;
    private final StatusSignal<Current> frontLeftSteerSupplyCurrent;
    private final StatusSignal<Current> frontLeftSteerTorqueCurrent;
    private final StatusSignal<Temperature> frontLeftSteerTempCelsius;

    private final StatusSignal<Angle> frontRightSteerPosition;
    private final StatusSignal<AngularVelocity> frontRightSteerVelocity;
    private final StatusSignal<Voltage> frontRightSteerAppliedVolts;
    private final StatusSignal<Current> frontRightSteerSupplyCurrent;
    private final StatusSignal<Current> frontRightSteerTorqueCurrent;
    private final StatusSignal<Temperature> frontRightSteerTempCelsius;

    private final StatusSignal<Angle> backLeftSteerPosition;
    private final StatusSignal<AngularVelocity> backLeftSteerVelocity;
    private final StatusSignal<Voltage> backLeftSteerAppliedVolts;
    private final StatusSignal<Current> backLeftSteerSupplyCurrent;
    private final StatusSignal<Current> backLeftSteerTorqueCurrent;
    private final StatusSignal<Temperature> backLeftSteerTempCelsius;

    private final StatusSignal<Angle> backRightSteerPosition;
    private final StatusSignal<AngularVelocity> backRightSteerVelocity;
    private final StatusSignal<Voltage> backRightSteerAppliedVolts;
    private final StatusSignal<Current> backRightSteerSupplyCurrent;
    private final StatusSignal<Current> backRightSteerTorqueCurrent;
    private final StatusSignal<Temperature> backRightSteerTempCelsius;

    private DriveIOdata iOdata = new DriveIOdata();



    public DriveKraken() {
        // drivetrain constants
        
        super(TalonFX::new, TalonFX::new, CANcoder::new,
        DriveConfig.DRIVETRAIN(), 
        DriveConfig.FRONT_LEFT(), 
        DriveConfig.FRONT_RIGHT(), 
        DriveConfig.BACK_LEFT(), 
        DriveConfig.BACK_RIGHT());

        m_Pigeon2 = getPigeon2();

        this.iOdata.m_moduleLocations = getModuleLocations();

        frontLeftDrivePosition = getModule(0).getDriveMotor().getPosition();
        frontLeftDriveVelocity = getModule(0).getDriveMotor().getVelocity();
        frontLeftDriveAppliedVolts = getModule(0).getDriveMotor().getMotorVoltage();
        frontLeftDriveSupplyCurrent = getModule(0).getDriveMotor().getSupplyCurrent();
        frontLeftDriveTorqueCurrent = getModule(0).getDriveMotor().getTorqueCurrent();
        frontLeftDriveTempCelsius = getModule(0).getDriveMotor().getDeviceTemp();

        frontRightDrivePosition = getModule(1).getDriveMotor().getPosition();
        frontRightDriveVelocity = getModule(1).getDriveMotor().getVelocity();
        frontRightDriveAppliedVolts = getModule(1).getDriveMotor().getMotorVoltage();
        frontRightDriveSupplyCurrent = getModule(1).getDriveMotor().getSupplyCurrent();
        frontRightDriveTorqueCurrent = getModule(1).getDriveMotor().getTorqueCurrent();
        frontRightDriveTempCelsius = getModule(1).getDriveMotor().getDeviceTemp();

        backLeftDrivePosition = getModule(2).getDriveMotor().getPosition();
        backLeftDriveVelocity = getModule(2).getDriveMotor().getVelocity();
        backLeftDriveAppliedVolts = getModule(2).getDriveMotor().getMotorVoltage();
        backLeftDriveSupplyCurrent = getModule(2).getDriveMotor().getSupplyCurrent();
        backLeftDriveTorqueCurrent = getModule(2).getDriveMotor().getTorqueCurrent();
        backLeftDriveTempCelsius = getModule(2).getDriveMotor().getDeviceTemp();

        backRightDrivePosition = getModule(3).getDriveMotor().getPosition();
        backRightDriveVelocity = getModule(3).getDriveMotor().getVelocity();
        backRightDriveAppliedVolts = getModule(3).getDriveMotor().getMotorVoltage();
        backRightDriveSupplyCurrent = getModule(3).getDriveMotor().getSupplyCurrent();
        backRightDriveTorqueCurrent = getModule(3).getDriveMotor().getTorqueCurrent();
        backRightDriveTempCelsius = getModule(3).getDriveMotor().getDeviceTemp();

        frontLeftSteerPosition = getModule(0).getSteerMotor().getPosition();
        frontLeftSteerVelocity = getModule(0).getSteerMotor().getVelocity();
        frontLeftSteerAppliedVolts = getModule(0).getSteerMotor().getMotorVoltage();
        frontLeftSteerSupplyCurrent = getModule(0).getSteerMotor().getSupplyCurrent();
        frontLeftSteerTorqueCurrent = getModule(0).getSteerMotor().getTorqueCurrent();
        frontLeftSteerTempCelsius = getModule(0).getSteerMotor().getDeviceTemp();

        frontRightSteerPosition = getModule(1).getSteerMotor().getPosition();
        frontRightSteerVelocity = getModule(1).getSteerMotor().getVelocity();
        frontRightSteerAppliedVolts = getModule(1).getSteerMotor().getMotorVoltage();
        frontRightSteerSupplyCurrent = getModule(1).getSteerMotor().getSupplyCurrent();
        frontRightSteerTorqueCurrent = getModule(1).getSteerMotor().getTorqueCurrent();
        frontRightSteerTempCelsius = getModule(1).getSteerMotor().getDeviceTemp();

        backLeftSteerPosition = getModule(2).getSteerMotor().getPosition();
        backLeftSteerVelocity = getModule(2).getSteerMotor().getVelocity();
        backLeftSteerAppliedVolts = getModule(2).getSteerMotor().getMotorVoltage();
        backLeftSteerSupplyCurrent = getModule(2).getSteerMotor().getSupplyCurrent();
        backLeftSteerTorqueCurrent = getModule(2).getSteerMotor().getTorqueCurrent();
        backLeftSteerTempCelsius = getModule(2).getSteerMotor().getDeviceTemp();

        backRightSteerPosition = getModule(3).getSteerMotor().getPosition();
        backRightSteerVelocity = getModule(3).getSteerMotor().getVelocity();
        backRightSteerAppliedVolts = getModule(3).getSteerMotor().getMotorVoltage();
        backRightSteerSupplyCurrent = getModule(3).getSteerMotor().getSupplyCurrent();
        backRightSteerTorqueCurrent = getModule(3).getSteerMotor().getTorqueCurrent();
        backRightSteerTempCelsius = getModule(3).getSteerMotor().getDeviceTemp();
        

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
             frontLeftDrivePosition,
             frontLeftDriveVelocity,
             frontLeftDriveAppliedVolts,
             frontLeftDriveSupplyCurrent,
             frontLeftDriveTorqueCurrent,
             frontLeftDriveTempCelsius,
        
             frontRightDrivePosition,
             frontRightDriveVelocity,
             frontRightDriveAppliedVolts,
             frontRightDriveSupplyCurrent,
             frontRightDriveTorqueCurrent,
             frontRightDriveTempCelsius,
        
             backLeftDrivePosition,
             backLeftDriveVelocity,
             backLeftDriveAppliedVolts,
             backLeftDriveSupplyCurrent,
             backLeftDriveTorqueCurrent,
             backLeftDriveTempCelsius,
        
             backRightDrivePosition,
             backRightDriveVelocity,
             backRightDriveAppliedVolts,
             backRightDriveSupplyCurrent,
             backRightDriveTorqueCurrent,
             backRightDriveTempCelsius,

             frontLeftSteerPosition,
             frontLeftSteerVelocity,
             frontLeftSteerAppliedVolts,
             frontLeftSteerSupplyCurrent,
             frontLeftSteerTorqueCurrent,
             frontLeftSteerTempCelsius,
        
             frontRightSteerPosition,
             frontRightSteerVelocity,
             frontRightSteerAppliedVolts,
             frontRightSteerSupplyCurrent,
             frontRightSteerTorqueCurrent,
             frontRightSteerTempCelsius,
        
             backLeftSteerPosition,
             backLeftSteerVelocity,
             backLeftSteerAppliedVolts,
             backLeftSteerSupplyCurrent,
             backLeftSteerTorqueCurrent,
             backLeftSteerTempCelsius,
        
             backRightSteerPosition,
             backRightSteerVelocity,
             backRightSteerAppliedVolts,
             backRightSteerSupplyCurrent,
             backRightSteerTorqueCurrent,
             backRightSteerTempCelsius
        );
    }

    public void setCurrentLimits() { // used for setting different current limits for auton/teleop
        SwerveModule<TalonFX, TalonFX, CANcoder>[] m_modules = getModules();
        
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].getDriveMotor().getConfigurator().apply(DriveConstants.driveInitialConfigs.CurrentLimits, 1.0);
            m_modules[i].getSteerMotor().getConfigurator().apply(DriveConstants.steerInitialConfigs.CurrentLimits, 1.0);
            // apply these seperatly (not in a TalonFXConfiguration obj) to ensure PID doesn't get overwritten
            m_modules[i].getDriveMotor().getConfigurator().apply(DriveConstants.driveInitialConfigs.TorqueCurrent, 1.0);
            m_modules[i].getSteerMotor().getConfigurator().apply(DriveConstants.steerInitialConfigs.TorqueCurrent, 1.0);
        }
    }

    @Override
    public void setSwerveRequest(SwerveRequest requestToApply) {
        setControl(requestToApply);
    }

    @Override
    public void updateStats(DriveIOdata stats) {
        stats.frontLeftDriveconnected =
        BaseStatusSignal.refreshAll(
        frontLeftDrivePosition,
        frontLeftDriveVelocity,
        frontLeftDriveAppliedVolts,
        frontLeftDriveSupplyCurrent,
        frontLeftDriveTorqueCurrent,
        frontLeftDriveTempCelsius)
        .isOK();
        
        stats.frontRightDriveconnected =
        BaseStatusSignal.refreshAll(
        frontRightDrivePosition,
        frontRightDriveVelocity,
        frontRightDriveAppliedVolts,
        frontRightDriveSupplyCurrent,
        frontRightDriveTorqueCurrent,
        frontRightDriveTempCelsius)
        .isOK();

        stats.backLeftDriveconnected =
        BaseStatusSignal.refreshAll(
        backLeftDrivePosition,
        backLeftDriveVelocity,
        backLeftDriveAppliedVolts,
        backLeftDriveSupplyCurrent,
        backLeftDriveTorqueCurrent,
        backLeftDriveTempCelsius)
        .isOK();

        stats.backRightDriveconnected =
        BaseStatusSignal.refreshAll(
        backRightDrivePosition,
        backRightDriveVelocity,
        backRightDriveAppliedVolts,
        backRightDriveSupplyCurrent,
        backRightDriveTorqueCurrent,
        backRightDriveTempCelsius)
        .isOK();

        stats.frontLeftSteerconnected =
        BaseStatusSignal.refreshAll(
        frontLeftSteerPosition,
        frontLeftSteerVelocity,
        frontLeftSteerAppliedVolts,
        frontLeftSteerSupplyCurrent,
        frontLeftSteerTorqueCurrent,
        frontLeftSteerTempCelsius)
        .isOK();

        stats.frontRightSteerconnected =
        BaseStatusSignal.refreshAll(
        frontRightSteerPosition,
        frontRightSteerVelocity,
        frontRightSteerAppliedVolts,
        frontRightSteerSupplyCurrent,
        frontRightSteerTorqueCurrent,
        frontRightSteerTempCelsius)
        .isOK();

        stats.backLeftSteerconnected =
        BaseStatusSignal.refreshAll(
        backLeftSteerPosition,
        backLeftSteerVelocity,
        backLeftSteerAppliedVolts,
        backLeftSteerSupplyCurrent,
        backLeftSteerTorqueCurrent,
        backLeftSteerTempCelsius)
        .isOK();

        stats.backRightSteerconnected =
        BaseStatusSignal.refreshAll(
        backRightSteerPosition,
        backRightSteerVelocity,
        backRightSteerAppliedVolts,
        backRightSteerSupplyCurrent,
        backRightSteerTorqueCurrent,
        backRightSteerTempCelsius)
        .isOK();

        stats.frontLeftDrivePositionRads = frontLeftDrivePosition.getValueAsDouble();
        stats.frontLeftDriveVelocityRpm = frontLeftDriveVelocity.getValueAsDouble();
        stats.frontLeftDriveAppliedVolts = frontLeftDriveAppliedVolts.getValueAsDouble();
        stats.frontLeftDriveSupplyCurrentAmps = frontLeftDriveSupplyCurrent.getValueAsDouble();
        stats.frontLeftDriveTorqueCurrentAmps = frontLeftDriveTorqueCurrent.getValueAsDouble();
        stats.frontLeftDriveTempCelsius = frontLeftDriveTempCelsius.getValueAsDouble();

        stats.frontRightDrivePositionRads = frontRightDrivePosition.getValueAsDouble();
        stats.frontRightDriveVelocityRpm = frontRightDriveVelocity.getValueAsDouble();
        stats.frontRightDriveAppliedVolts = frontRightDriveAppliedVolts.getValueAsDouble();
        stats.frontRightDriveSupplyCurrentAmps = frontRightDriveSupplyCurrent.getValueAsDouble();
        stats.frontRightDriveTorqueCurrentAmps = frontRightDriveTorqueCurrent.getValueAsDouble();
        stats.frontRightDriveTempCelsius = frontRightDriveTempCelsius.getValueAsDouble();

        stats.backLeftDrivePositionRads = backLeftDrivePosition.getValueAsDouble();
        stats.backLeftDriveVelocityRpm = backLeftDriveVelocity.getValueAsDouble();
        stats.backLeftDriveAppliedVolts = backLeftDriveAppliedVolts.getValueAsDouble();
        stats.backLeftDriveSupplyCurrentAmps = backLeftDriveSupplyCurrent.getValueAsDouble();
        stats.backLeftDriveTorqueCurrentAmps = backLeftDriveTorqueCurrent.getValueAsDouble();
        stats.backLeftDriveTempCelsius = backLeftDriveTempCelsius.getValueAsDouble();

        stats.backRightDrivePositionRads = backRightDrivePosition.getValueAsDouble();
        stats.backRightDriveVelocityRpm = backRightDriveVelocity.getValueAsDouble();
        stats.backRightDriveAppliedVolts = backRightDriveAppliedVolts.getValueAsDouble();
        stats.backRightDriveSupplyCurrentAmps = backRightDriveSupplyCurrent.getValueAsDouble();
        stats.backRightDriveTorqueCurrentAmps = backRightDriveTorqueCurrent.getValueAsDouble();
        stats.backRightDriveTempCelsius = backRightDriveTempCelsius.getValueAsDouble();

        stats.frontLeftSteerPositionRads = frontLeftSteerPosition.getValueAsDouble();
        stats.frontLeftSteerVelocityRpm = frontLeftSteerVelocity.getValueAsDouble();
        stats.frontLeftSteerAppliedVolts = frontLeftSteerAppliedVolts.getValueAsDouble();
        stats.frontLeftSteerSupplyCurrentAmps = frontLeftSteerSupplyCurrent.getValueAsDouble();
        stats.frontLeftSteerTorqueCurrentAmps = frontLeftSteerTorqueCurrent.getValueAsDouble();
        stats.frontLeftSteerTempCelsius = frontLeftSteerTempCelsius.getValueAsDouble();

        stats.frontRightSteerPositionRads = frontRightSteerPosition.getValueAsDouble();
        stats.frontRightSteerVelocityRpm = frontRightSteerVelocity.getValueAsDouble();
        stats.frontRightSteerAppliedVolts = frontRightSteerAppliedVolts.getValueAsDouble();
        stats.frontRightSteerSupplyCurrentAmps = frontRightSteerSupplyCurrent.getValueAsDouble();
        stats.frontRightSteerTorqueCurrentAmps = frontRightSteerTorqueCurrent.getValueAsDouble();
        stats.frontRightSteerTempCelsius = frontRightSteerTempCelsius.getValueAsDouble();

        stats.backLeftSteerPositionRads = backLeftSteerPosition.getValueAsDouble();
        stats.backLeftSteerVelocityRpm = backLeftSteerVelocity.getValueAsDouble();
        stats.backLeftSteerAppliedVolts = backLeftSteerAppliedVolts.getValueAsDouble();
        stats.backLeftSteerSupplyCurrentAmps = backLeftSteerSupplyCurrent.getValueAsDouble();
        stats.backLeftSteerTorqueCurrentAmps = backLeftSteerTorqueCurrent.getValueAsDouble();
        stats.backLeftSteerTempCelsius = backLeftSteerTempCelsius.getValueAsDouble();

        stats.backRightSteerPositionRads = backRightSteerPosition.getValueAsDouble();
        stats.backRightSteerVelocityRpm = backRightSteerVelocity.getValueAsDouble();
        stats.backRightSteerAppliedVolts = backRightSteerAppliedVolts.getValueAsDouble();
        stats.backRightSteerSupplyCurrentAmps = backRightSteerSupplyCurrent.getValueAsDouble();
        stats.backRightSteerTorqueCurrentAmps = backRightSteerTorqueCurrent.getValueAsDouble();
        stats.backRightSteerTempCelsius = backRightSteerTempCelsius.getValueAsDouble();
    }

    @Override
    public DriveIOdata update() {
        this.currentState = getState();

        if (currentState != null) {
            this.iOdata.state = this.currentState;
        }

        // TODO ADD MODULE DATA
        return this.iOdata;
    }

    @Override
    public void setTeamRotation(Alliance alliance) {
        if (alliance == DriverStation.Alliance.Red) {
            this.setOperatorPerspectiveForward(RedAlliancePerspectiveRotation);
        } else {
            this.setOperatorPerspectiveForward(BlueAlliancePerspectiveRotation);
        }
    }

    @Override
    public void resetPidgeon() {
        m_Pigeon2.setYaw(0);
    }

    @Override
    public void seedFieldRelative(Pose2d seedling) {
        resetPose(seedling);
    }

    @Override
    public void updateVision(Pose2d calculatedPose, double timestamp, Matrix<N3, N1> stDevs) {
        addVisionMeasurement(calculatedPose, Utils.fpgaToCurrentTime(timestamp), stDevs);
    }

    @Override
    public void setOperatorPerspective(Rotation2d rotation2d) {
        setOperatorPerspectiveForward(rotation2d);
    }
}
