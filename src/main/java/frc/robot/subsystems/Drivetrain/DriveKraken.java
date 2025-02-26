package frc.robot.subsystems.Drivetrain;

import static frc.robot.subsystems.Drivetrain.DriveConstants.DriveConfig;

import java.util.ArrayList;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
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

    private final ArrayList<StatusSignal<Angle>> drivePosition;
    private final ArrayList<StatusSignal<AngularVelocity>> driveVelocity;
    private final ArrayList<StatusSignal<Voltage>> driveAppliedVolts;
    private final ArrayList<StatusSignal<Current>> driveSupplyCurrent;
    private final ArrayList<StatusSignal<Current>> driveTorqueCurrent;
    private final ArrayList<StatusSignal<Temperature>> driveTempCelsius;

    private final ArrayList<StatusSignal<Angle>> steerPosition;
    private final ArrayList<StatusSignal<AngularVelocity>> steerVelocity;
    private final ArrayList<StatusSignal<Voltage>> steerAppliedVolts;
    private final ArrayList<StatusSignal<Current>> steerSupplyCurrent;
    private final ArrayList<StatusSignal<Current>> steerTorqueCurrent;
    private final ArrayList<StatusSignal<Temperature>> steerTempCelsius;

    private SwervePoses swervePoses;

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

        drivePosition = new ArrayList<>();
        driveVelocity = new ArrayList<>();
        driveAppliedVolts = new ArrayList<>();
        driveSupplyCurrent = new ArrayList<>();
        driveTorqueCurrent = new ArrayList<>();
        driveTempCelsius = new ArrayList<>();

        steerPosition = new ArrayList<>();
        steerVelocity = new ArrayList<>();
        steerAppliedVolts = new ArrayList<>();
        steerSupplyCurrent = new ArrayList<>();
        steerTorqueCurrent = new ArrayList<>();
        steerTempCelsius = new ArrayList<>();

        swervePoses = SwervePoses.FRONT_LEFT;

        for (int i = 0; i==3; ++i) {
            drivePosition.add(getModule(i).getDriveMotor().getPosition());
            driveVelocity.add(getModule(i).getDriveMotor().getVelocity());
            driveAppliedVolts.add(getModule(i).getDriveMotor().getMotorVoltage());
            driveSupplyCurrent.add(getModule(i).getDriveMotor().getSupplyCurrent());
            driveTorqueCurrent.add(getModule(i).getDriveMotor().getTorqueCurrent());
            driveTempCelsius.add(getModule(i).getDriveMotor().getDeviceTemp());

            steerPosition.add(getModule(i).getSteerMotor().getPosition());
            steerVelocity.add(getModule(i).getSteerMotor().getVelocity());
            steerAppliedVolts.add(getModule(i).getSteerMotor().getMotorVoltage());
            steerSupplyCurrent.add(getModule(i).getSteerMotor().getSupplyCurrent());
            steerTorqueCurrent.add(getModule(i).getSteerMotor().getTorqueCurrent());
            steerTempCelsius.add(getModule(i).getSteerMotor().getDeviceTemp());

            BaseStatusSignal.setUpdateFrequencyForAll(
                100.0,
                drivePosition.get(i),
                driveVelocity.get(i),
                driveAppliedVolts.get(i),
                driveSupplyCurrent.get(i),
                driveTorqueCurrent.get(i),
                driveTempCelsius.get(i),

                steerPosition.get(i),
                steerVelocity.get(i),
                steerAppliedVolts.get(i),
                steerSupplyCurrent.get(i),
                steerTorqueCurrent.get(i),
                steerTempCelsius.get(i)
            );
        }
    }

    public void setCurrentLimits() { // used for setting different current limits for auton/teleop
        SwerveModule<TalonFX, TalonFX, CANcoder>[] m_modules = getModules();
        
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].getDriveMotor().setNeutralMode(NeutralModeValue.Coast, 1.0);
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
        for (int i = 0; i==3; ++i) {
            switch (i) {
                case 0: swervePoses = SwervePoses.FRONT_LEFT;
                case 1: swervePoses = SwervePoses.FRONT_RIGHT;
                case 2: swervePoses = SwervePoses.BACK_LEFT;
                case 3: swervePoses = SwervePoses.BACK_RIGHT;
            }
            stats.driveConnected.put(swervePoses,
            BaseStatusSignal.refreshAll(
            drivePosition.get(i),
            driveVelocity.get(i),
            driveAppliedVolts.get(i),
            driveSupplyCurrent.get(i),
            driveTorqueCurrent.get(i),
            driveTempCelsius.get(i))
            .isOK()
            );

            stats.steerConnected.put(swervePoses,
            BaseStatusSignal.refreshAll(
            steerPosition.get(i),
            steerVelocity.get(i),
            steerAppliedVolts.get(i),
            steerSupplyCurrent.get(i),
            steerTorqueCurrent.get(i),
            steerTempCelsius.get(i))
            .isOK()
            );
        }

        this.iOdata.driveConnected.forEach((key, connected) -> {
            if(connected) {
                this.iOdata.driveVelocity.put(key, getModule(key.ordinal()).getDriveMotor().getVelocity().getValueAsDouble());
                this.iOdata.driveVolts.put(key, getModule(key.ordinal()).getDriveMotor().getMotorVoltage().getValueAsDouble());
                this.iOdata.driveSupplyCurrent.put(key, getModule(key.ordinal()).getDriveMotor().getSupplyCurrent().getValueAsDouble());
                this.iOdata.driveTorqueCurrent.put(key, getModule(key.ordinal()).getDriveMotor().getTorqueCurrent().getValueAsDouble());
                this.iOdata.driveTemp.put(key, getModule(key.ordinal()).getDriveMotor().getDeviceTemp().getValueAsDouble());

                SmartDashboard.putNumber(key.toString() + "_Velo", this.iOdata.driveVelocity.get(key));
                SmartDashboard.putNumber(key.toString() + "_Volts", this.iOdata.driveVolts.get(key));
                SmartDashboard.putNumber(key.toString() + "_Supply", this.iOdata.driveSupplyCurrent.get(key));
                SmartDashboard.putNumber(key.toString() + "_Torque", this.iOdata.driveTorqueCurrent.get(key));
                SmartDashboard.putNumber(key.toString() + "_Temp", this.iOdata.driveTemp.get(key));

                this.iOdata.steerPosition.put(key, getModule(key.ordinal()).getSteerMotor().getPosition().getValueAsDouble());
                this.iOdata.steerVelocity.put(key, getModule(key.ordinal()).getSteerMotor().getVelocity().getValueAsDouble());
                this.iOdata.steerVolts.put(key, getModule(key.ordinal()).getSteerMotor().getMotorVoltage().getValueAsDouble());
                this.iOdata.steerSupplyCurrent.put(key, getModule(key.ordinal()).getSteerMotor().getSupplyCurrent().getValueAsDouble());
                this.iOdata.steerTorqueCurrent.put(key, getModule(key.ordinal()).getSteerMotor().getTorqueCurrent().getValueAsDouble());
                this.iOdata.steerTemp.put(key, getModule(key.ordinal()).getSteerMotor().getDeviceTemp().getValueAsDouble());

                SmartDashboard.putNumber(key.toString() + "_Position", this.iOdata.steerPosition.get(key));
                SmartDashboard.putNumber(key.toString() + "_Velo", this.iOdata.steerVelocity.get(key));
                SmartDashboard.putNumber(key.toString() + "_Volts", this.iOdata.steerVolts.get(key));
                SmartDashboard.putNumber(key.toString() + "_Supply", this.iOdata.steerSupplyCurrent.get(key));
                SmartDashboard.putNumber(key.toString() + "_Torque", this.iOdata.steerTorqueCurrent.get(key));
                SmartDashboard.putNumber(key.toString() + "_Temp", this.iOdata.steerTemp.get(key));
            }
        });
    }

    @Override
    public DriveIOdata update() {
        this.currentState = getState();

        if (currentState != null) {
            this.iOdata.state = this.currentState;
        }

        this.iOdata.pigeon = m_Pigeon2.getRotation3d();

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
        if (seedling != null) {
            if (currentState.Pose.getTranslation().getDistance(seedling.getTranslation()) > .5) {
                resetPose(seedling);
            }
        }
        
    }

    @Override
    public void updateVision(Pose2d calculatedPose, double timestamp, Matrix<N3, N1> stDevs) {
        addVisionMeasurement(calculatedPose, Utils.fpgaToCurrentTime(timestamp), stDevs);
    }

    @Override
    public void setOperatorPerspective(Rotation2d rotation2d) {
        setOperatorPerspectiveForward(rotation2d);
    }

    public void setNeutralMode(NeutralModeValue neutralModeValue) {
        SwerveModule<TalonFX, TalonFX, CANcoder>[] m_modules = getModules();

        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].getDriveMotor().setNeutralMode(neutralModeValue, 1.0);
        }
    }
}
