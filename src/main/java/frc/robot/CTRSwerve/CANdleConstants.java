package frc.robot.CTRSwerve;

public class CANdleConstants {
    public int id = 0;
    public double locationX = 0;
    public double locationY = 0;

    public CANdleConstants withId(int id) {
        this.id = id;
        return this;
    }
    public CANdleConstants withLocationX(double locationX) {
        this.locationX = locationX;
        return this;
    }
    public CANdleConstants withLocationY(double locationY) {
        this.locationY = locationY;
        return this;
    }
}