package frc.robot.subsystems.GameSpec.Arm;

public class ArmConstants {
    
    public static final int armMotorID = 32;
    public static final int armEncoderID = 51;
    public static final double kReduction = (1.0 / 2.0);
    public static final double kMaxAccelerationRpmPerSec = 9000.0; 
    public static final MMGains armGains = new MMGains(200, 100, 200, 15, 0.0, 0.0, 0, 0);
    
    public record MMGains(double CruiseVelocity, double Acceleration, double Jerk, double kP, double kI, double kD, double kV, double kS) {} 
}