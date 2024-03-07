package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ChenryLib.PID;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.UpperState;

public class VisionSub extends SubsystemBase{
    
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private final PID facingPID = new PID(
      SwerveConstants.faceKP,
      SwerveConstants.faceKI,
      SwerveConstants.faceKD,
      SwerveConstants.faceiWindup,
      SwerveConstants.faceiLimit);

    public double getTx() {
        return table.getDoubleTopic("tx").subscribe(0.0).get();
    }

    public double getTy() {
        return table.getDoubleTopic("ty").subscribe(0.0).get();
    }

    public double getTv() {
        return table.getDoubleTopic("tv").subscribe(0.0).get();
    }

    public double getTa() {
        return table.getDoubleTopic("ta").subscribe(0.0).get();
    }

    public double getTid() {
        return table.getDoubleTopic("tid").subscribe(0.0).get();
    }

    public boolean hasTarget() {
        // return getTv() < 1.0 ? false : true;
        if(Robot.alliance == "RED") {
            if(Constants.state == UpperState.AUTO && getTid() == 4) return true;
            else if(Constants.state == UpperState.AMP && getTid() == 5) return true;
            else return false;
        } else if(Robot.alliance == "BLUE") {
            if(Constants.state == UpperState.AUTO && getTid() == 7) return true;
            else if(Constants.state == UpperState.AMP && getTid() == 6) return true;
            else return false;
        } else return false;
    }

    public Pose2d getRobotPose() {
        return new Pose2d(getRobotX(), getRobotY(), Rotation2d.fromDegrees(getRobotDEG()));
    }

    public double getRobotX() {
        return table.getEntry("botpose").getDoubleArray(new double[6])[0];
    }
    
    public double getRobotY() {
        return table.getEntry("botpose").getDoubleArray(new double[6])[1];
    }

    public double getRobotDEG() {
        return table.getEntry("botpose").getDoubleArray(new double[6])[5];
    }

    public double calculateAutoFacing() {
        return facingPID.calculate(getTx());
    }

    public double calculateAutoAiming() {
        if(Robot.alliance == "RED") {
            return FieldConstants.autoAimDataRed.get(new Translation2d(getRobotX(), getRobotY()));
        } else if(Robot.alliance == "BLUE") {
            return FieldConstants.autoAimDataBlue.get(new Translation2d(getRobotX(), getRobotY()));
        } else return 404;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("RobotPose_X", getRobotX());
        SmartDashboard.putNumber("RobotPose_Y", getRobotY());
        SmartDashboard.putNumber("RobotPose_r", getRobotDEG());
        SmartDashboard.putBoolean("HasTarget", hasTarget());
         }
}