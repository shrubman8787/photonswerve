package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;        
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UpperSub;
import frc.robot.subsystems.VisionSub;

public class AutoAim extends Command{
    private final Swerve s_Swerve;
    private final UpperSub s_Upper;
    private final VisionSub s_Vision;
    
    private final Optional<Alliance> ally = DriverStation.getAlliance();
    private final Translation2d speaker = ally.get() == Alliance.Red ? VisionConstants.Speaker_red : VisionConstants.Speaker_blue;

    public AutoAim(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) {
        this.s_Swerve = s_Swerve;
        this.s_Upper = s_Upper;
        this.s_Vision = s_Vision;
        addRequirements(s_Swerve, s_Vision);
    }

    public Pair<Double,Double> calculate() {
        double facing = Math.atan2(
            speaker.getY() - s_Swerve.getPose().getY(),
            speaker.getX() - s_Swerve.getPose().getX()
        );

        // double distance = s_Vision.getRobotPose().getTranslation().getDistance(speaker);
        double elbowAngle = -0.204589;
        
        return new Pair<Double,Double>(facing,elbowAngle);
    }

    public ParallelCommandGroup getCommand() {
        Pair<Double,Double> data = calculate();
        return new ParallelCommandGroup(
            new AutoSwerve(s_Swerve, s_Vision, 0, 0, data.getFirst(), false),
            new AutoUpper(s_Upper, data.getSecond(), 0, 0, 1)
        );
    }
}
