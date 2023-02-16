// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenixpro.configs.Slot0Configs;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.CTRSwerve.CANdleConstants;
import frc.robot.CTRSwerve.CANdleManager;
import frc.robot.CTRSwerve.CTRSwerveDrivetrain;
import frc.robot.CTRSwerve.SwerveDriveTrainConstants;
import frc.robot.CTRSwerve.SwerveModuleConstants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    SwerveDriveTrainConstants drivetrain =
            new SwerveDriveTrainConstants().withPigeon2Id(1).withCANbusName("Fred");

    Slot0Configs steerGains = new Slot0Configs();
    Slot0Configs driveGains = new Slot0Configs();

    {
        steerGains.kP = 30;
        steerGains.kD = 0.2;
        driveGains.kP = 1;
    }

    /**
     * Note: WPI's coordinate system is X forward, Y to the left so make sure all locations are with
     * respect to this coordinate system
     *
     * <p>This particular drive base is 22" x 22"
     */
    SwerveModuleConstants frontRight =
            new SwerveModuleConstants()
                    .withDriveMotorId(1)
                    .withSteerMotorId(0)
                    .withCANcoderId(0)
                    .withCANcoderOffset(-0.538818)
                    .withDriveMotorGearRatio(10)
                    .withWheelRadius(3)
                    .withLocationX(Units.inchesToMeters(22.0 / 2.0))
                    .withLocationY(Units.inchesToMeters(-22.0 / 2.0))
                    .withSteerMotorGains(steerGains)
                    .withDriveMotorGains(driveGains)
                    .withSlipCurrent(17);

    SwerveModuleConstants frontLeft =
            new SwerveModuleConstants()
                    .withDriveMotorId(3)
                    .withSteerMotorId(2)
                    .withCANcoderId(1)
                    .withCANcoderOffset(-0.474609)
                    .withDriveMotorGearRatio(10)
                    .withWheelRadius(3)
                    .withLocationX(Units.inchesToMeters(22.0 / 2.0))
                    .withLocationY(Units.inchesToMeters(22.0 / 2.0))
                    .withSteerMotorGains(steerGains)
                    .withDriveMotorGains(driveGains)
                    .withSlipCurrent(17);
    SwerveModuleConstants backRight =
            new SwerveModuleConstants()
                    .withDriveMotorId(5)
                    .withSteerMotorId(4)
                    .withCANcoderId(2)
                    .withCANcoderOffset(-0.928467)
                    .withDriveMotorGearRatio(10)
                    .withWheelRadius(3)
                    .withLocationX(Units.inchesToMeters(-22.0 / 2.0))
                    .withLocationY(Units.inchesToMeters(-22.0 / 2.0))
                    .withSteerMotorGains(steerGains)
                    .withDriveMotorGains(driveGains)
                    .withSlipCurrent(17);
    SwerveModuleConstants backLeft =
            new SwerveModuleConstants()
                    .withDriveMotorId(7)
                    .withSteerMotorId(6)
                    .withCANcoderId(3)
                    .withCANcoderOffset(-0.756348)
                    .withDriveMotorGearRatio(10)
                    .withWheelRadius(3)
                    .withLocationX(Units.inchesToMeters(-22.0 / 2.0))
                    .withLocationY(Units.inchesToMeters(22.0 / 2.0))
                    .withSteerMotorGains(steerGains)
                    .withDriveMotorGains(driveGains)
                    .withSlipCurrent(17);

    CTRSwerveDrivetrain m_drivetrain =
            new CTRSwerveDrivetrain(drivetrain, frontLeft, frontRight, backLeft, backRight);

    XboxController m_joystick = new XboxController(0);

    CANdleConstants frontCandle = new CANdleConstants().withId(2).withLocationX(1).withLocationY(0);
    CANdleConstants rightCandle = new CANdleConstants().withId(1).withLocationX(0).withLocationY(-1);
    CANdleConstants leftCandle = new CANdleConstants().withId(4).withLocationX(0).withLocationY(1);
    CANdleConstants backCandle = new CANdleConstants().withId(3).withLocationX(-1).withLocationY(0);
    Color8Bit posX = new Color8Bit(0, 0, 255);
    Color8Bit posY = new Color8Bit(255, 0, 0);
    Color8Bit negX = new Color8Bit(255, 255, 255);
    Color8Bit negY = new Color8Bit(0, 255, 0);
    CANdleManager m_candleManager =
            new CANdleManager(
                    "Fred", posY, posX, negY, negX, frontCandle, leftCandle, rightCandle, backCandle);

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {}

    @Override
    public void robotPeriodic() {
        double leftY = -m_joystick.getLeftY();
        double leftX = m_joystick.getLeftX();
        double rightX = m_joystick.getRightX();

        if (Math.abs(leftY) < 0.1 && Math.abs(leftX) < 0.1) {
            leftY = 0;
            leftX = 0;
        }
        if (Math.abs(rightX) < 0.1) {
            rightX = 0;
        }

        var directions = new ChassisSpeeds();
        directions.vxMetersPerSecond = leftY * 1;
        directions.vyMetersPerSecond = leftX * -1;
        directions.omegaRadiansPerSecond = rightX * -10;

        /* If we're pressing Y, don't move, otherwise do normal movement */
        if (m_joystick.getYButton()) {
            m_drivetrain.driveStopMotion();
        } else {
            m_drivetrain.driveFieldCentric(directions);
        }

        if (m_joystick.getAButton()) {
            m_drivetrain.seedFieldRelative();
        }

        m_candleManager.orient(m_drivetrain.getPoseMeters().getRotation());
        m_candleManager.color();
    }

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
