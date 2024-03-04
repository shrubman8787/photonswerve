package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.ChenryLib.PID;
import frc.robot.subsystems.Swerve;

public class AutoSwerve extends Command{
        private final Swerve swerve;
    double x,y,z;

    private final PID xyPID = new PID(3, 0, 1, 0, 0);
    private final PID zPID = new PID(10, 0, 8, 0, 0);

    public AutoSwerve(Swerve swerve, double x, double y, double z) {
        this.swerve = swerve;
        this.x = x;
        this.y = y;
        this.z = z;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        System.out.println("AutoSwerve.initialize()");
    }

    @Override
    public void execute() {
        swerve.drive(
            new Translation2d(xyPID.calculate(x - swerve.getPose().getX()), xyPID.calculate(y - swerve.getPose().getY())), 
            -zPID.calculate(z - swerve.getPose().getRotation().getRotations()), true, false
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, true, false);
        System.out.println("AutoSwerve.end()");
    }

    @Override
    public boolean isFinished() {
        if(
            Math.abs(x - swerve.getPose().getX()) < 0.02 &&
            Math.abs(y - swerve.getPose().getY()) < 0.02 &&
            Math.abs(z - swerve.getPose().getRotation().getRotations()) < 0.005
        )  return true;
        return false;
    }
}
