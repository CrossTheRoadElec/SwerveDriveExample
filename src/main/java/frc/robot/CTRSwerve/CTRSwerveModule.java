package frc.robot.CTRSwerve;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TorqueCurrentConfigs;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class CTRSwerveModule {
    private TalonFX m_driveMotor;
    private TalonFX m_steerMotor;
    private CANcoder m_cancoder;

    private StatusSignalValue<Double> m_drivePosition;
    private StatusSignalValue<Double> m_driveVelocity;
    private StatusSignalValue<Double> m_steerPosition;
    private StatusSignalValue<Double> m_steerVelocity;
    private BaseStatusSignalValue[] m_signals;
    private double m_driveRotationsPerMeter = 0;

    private PositionVoltage m_angleSetter = new PositionVoltage(0);
    private VelocityTorqueCurrentFOC m_velocitySetter = new VelocityTorqueCurrentFOC(0);

    private SwerveModulePosition m_internalState = new SwerveModulePosition();

    public CTRSwerveModule(SwerveModuleConstants constants, String canbusName) {
        m_driveMotor = new TalonFX(constants.DriveMotorId, canbusName);
        m_steerMotor = new TalonFX(constants.SteerMotorId, canbusName);
        m_cancoder = new CANcoder(constants.CANcoderId, canbusName);

        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

        talonConfigs.Slot0 = constants.DriveMotorGains;
        talonConfigs.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        m_driveMotor.getConfigurator().apply(talonConfigs);
        
        /* Undo changes for torqueCurrent */
        talonConfigs.TorqueCurrent = new TorqueCurrentConfigs();

        talonConfigs.Slot0 = constants.SteerMotorGains;
        // Modify configuration to use remote CANcoder fused
        talonConfigs.Feedback.FeedbackRemoteSensorID = constants.CANcoderId;
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonConfigs.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;

        talonConfigs.ClosedLoopGeneral.ContinuousWrap =
                true; // Enable continuous wrap for swerve modules
        m_steerMotor.getConfigurator().apply(talonConfigs);

        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = constants.CANcoderOffset;
        m_cancoder.getConfigurator().apply(cancoderConfigs);

        m_drivePosition = m_driveMotor.getPosition();
        m_driveVelocity = m_driveMotor.getVelocity();
        m_steerPosition = m_cancoder.getPosition();
        m_steerVelocity = m_cancoder.getVelocity();

        m_signals = new BaseStatusSignalValue[4];
        m_signals[0] = m_drivePosition;
        m_signals[1] = m_driveVelocity;
        m_signals[2] = m_steerPosition;
        m_signals[3] = m_steerVelocity;

        /* Calculate the ratio of drive motor rotation to meter on ground */
        double rotationsPerWheelRotation = constants.DriveMotorGearRatio;
        double metersPerWheelRotation = 2 * Math.PI * Units.inchesToMeters(constants.WheelRadius);
        m_driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
    }

    public SwerveModulePosition getPosition() {
        /* Refresh all signals */
        m_drivePosition.refresh();
        m_driveVelocity.refresh();
        m_steerPosition.refresh();
        m_steerVelocity.refresh();

        /* Now latency-compensate our signals */
        double drive_rot =
                m_drivePosition.getValue()
                        + (m_driveVelocity.getValue() * m_drivePosition.getTimestamp().getLatency());
        double angle_rot =
                m_steerPosition.getValue()
                        + (m_steerVelocity.getValue() * m_steerPosition.getTimestamp().getLatency());

        /* And push them into a SwerveModuleState object to return */
        m_internalState.distanceMeters = drive_rot / m_driveRotationsPerMeter;
        /* Angle is already in terms of steer rotations */
        m_internalState.angle = Rotation2d.fromRotations(angle_rot);

        return m_internalState;
    }

    public void apply(SwerveModuleState state) {
        var optimized = SwerveModuleState.optimize(state, m_internalState.angle);

        double angleToSetDeg = optimized.angle.getRotations();
        m_steerMotor.setControl(m_angleSetter.withPosition(angleToSetDeg));
        double velocityToSet = optimized.speedMetersPerSecond * m_driveRotationsPerMeter;
        m_driveMotor.setControl(m_velocitySetter.withVelocity(velocityToSet));
    }

    BaseStatusSignalValue[] getSignals() {
        return m_signals;
    }
}
