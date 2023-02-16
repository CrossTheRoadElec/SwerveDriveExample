package frc.robot.CTRSwerve;

public class SwerveDriveTrainConstants {
    /** CAN ID of the Pigeon2 on the drivetrain */
    public int Pigeon2Id = 0;
    /** Name of CANivore the swerve drive is on */
    public String CANbusName = "rio";

    public SwerveDriveTrainConstants withPigeon2Id(int id) {
        this.Pigeon2Id = id;
        return this;
    }

    public SwerveDriveTrainConstants withCANbusName(String name) {
        this.CANbusName = name;
        return this;
    }
}
