package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.ChenryLib.PID;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSub;

public class AutoSwerve extends Command{
    
    private final Swerve swerve;
    private final VisionSub vision;

    double x,y,z;
    boolean facing;
    Translation2d trans2d;
    double rotate2d;

    private final PID xyPID = new PID(3, 0, 1, 0, 0);
    private final PID zPID = new PID(5, 0, 4, 0, 0);

    public AutoSwerve(Swerve swerve, VisionSub vision, double x, double y, double z, boolean facing) {
        this.swerve = swerve;
        this.vision = vision;
        this.x = x;
        this.y = y;
        this.z = z;
        this.facing = facing;
        addRequirements(swerve);
        addRequirements(vision);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        trans2d = new Translation2d(xyPID.calculate(x - swerve.getPose().getX()), xyPID.calculate(y - swerve.getPose().getY()));
        rotate2d = facing ? vision.hasTarget() ? -vision.calculateAutoFacing() : -zPID.calculate(z - swerve.getPose().getRotation().getRotations()) : -zPID.calculate(z - swerve.getPose().getRotation().getRotations());
        swerve.drive(trans2d, rotate2d, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, true, false);
    }

    @Override
    public boolean isFinished() {
        if( Math.abs(x - swerve.getPose().getX()) < 0.02 &&
            Math.abs(y - swerve.getPose().getY()) < 0.02 &&
            Math.abs(z - swerve.getPose().getRotation().getRotations()) < 0.005
        )  return true;
        return false;
    }
}
