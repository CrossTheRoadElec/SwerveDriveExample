// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenixpro.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.CTRSwerve.CANdleConstants;
import frc.robot.CTRSwerve.CANdleManager;
import frc.robot.CTRSwerve.CTRSwerveDrivetrain;
import frc.robot.CTRSwerve.SwerveDriveConstantsCreator;
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
            new SwerveDriveTrainConstants().withPigeon2Id(1).withCANbusName("Fred").withTurnKp(5);

    Slot0Configs steerGains = new Slot0Configs();
    Slot0Configs driveGains = new Slot0Configs();

    {
        steerGains.kP = 30;
        steerGains.kD = 0.2;
        driveGains.kP = 1;
    }

    SwerveDriveConstantsCreator m_constantsCreator =
            new SwerveDriveConstantsCreator(
                    10, // 10:1 ratio for the drive motor
                    12.8, // 12.8:1 ratio for the steer motor
                    3, // 3 inch radius for the wheels
                    17, // Only apply 17 stator amps to prevent slip
                    steerGains, // Use the specified steer gains
                    driveGains, // Use the specified drive gains
                    false // CANcoder not reversed from the steer motor. For WCP Swerve X this should be true.
                    );

    /**
     * Note: WPI's coordinate system is X forward, Y to the left so make sure all locations are with
     * respect to this coordinate system
     *
     * <p>This particular drive base is 22" x 22"
     */
    SwerveModuleConstants frontRight =
            m_constantsCreator.createModuleConstants(
                    0, 1, 0, -0.538818, Units.inchesToMeters(22.0 / 2.0), Units.inchesToMeters(-22.0 / 2.0));

    SwerveModuleConstants frontLeft =
            m_constantsCreator.createModuleConstants(
                    2, 3, 1, -0.474609, Units.inchesToMeters(22.0 / 2.0), Units.inchesToMeters(22.0 / 2.0));
    SwerveModuleConstants backRight =
            m_constantsCreator.createModuleConstants(
                    4, 5, 2, -0.928467, Units.inchesToMeters(-22.0 / 2.0), Units.inchesToMeters(-22.0 / 2.0));
    SwerveModuleConstants backLeft =
            m_constantsCreator.createModuleConstants(
                    6, 7, 3, -0.756348, Units.inchesToMeters(-22.0 / 2.0), Units.inchesToMeters(22.0 / 2.0));

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
    Rotation2d m_lastTargetAngle = new Rotation2d();

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
        double rightY = -m_joystick.getRightY();

        if (Math.abs(leftY) < 0.1 && Math.abs(leftX) < 0.1) {
            leftY = 0;
            leftX = 0;
        }
        if (Math.abs(rightX) < 0.1 && Math.abs(rightY) < 0.1) {
            rightX = 0;
            rightY = 0;
        }

        var directions = new ChassisSpeeds();
        directions.vxMetersPerSecond = leftY * 1;
        directions.vyMetersPerSecond = leftX * -1;
        directions.omegaRadiansPerSecond = rightX * -10;

        /* If we're pressing Y, don't move, otherwise do normal movement */
        if (m_joystick.getYButton()) {
            m_drivetrain.driveStopMotion();
        } else {
            /* If we're fully field centric, we need to be pretty deflected to target an angle */
            if (Math.abs(rightX) > 0.7 || Math.abs(rightY) > 0.7) {
                m_lastTargetAngle = new Rotation2d(rightY, -rightX);
            }
            m_drivetrain.driveFullyFieldCentric(leftY * 1, leftX * -1, m_lastTargetAngle);
        }

        if (m_joystick.getAButton()) {
            m_drivetrain.seedFieldRelative();
            // Make us target forward now to avoid jumps
            m_lastTargetAngle = new Rotation2d();
        }

        m_candleManager.orient(m_drivetrain.getPoseMeters().getRotation());
        m_candleManager.color();
    }

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        m_lastTargetAngle = m_drivetrain.getPoseMeters().getRotation();
    }

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
