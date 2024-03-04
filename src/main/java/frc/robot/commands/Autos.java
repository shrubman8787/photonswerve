package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.UpperState;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UpperSub;
import frc.robot.subsystems.VisionSub;

public class Autos {

    public SequentialCommandGroup X1() {
        return new SequentialCommandGroup(
            
        );
    }

    public SequentialCommandGroup X2(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) {
        return new SequentialCommandGroup(
            new AutoUpper(s_Upper, UpperState.SPEAKER),
            new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, -0.92, 0, 0),
                new AutoUpper(s_Upper, -0.236328, 0.03, 0.3, 5)
            ),
            new AutoUpper(s_Upper, -0.16, -1, 0, 1.5),
            new AutoUpper(s_Upper, -0.16, -1, 1, 0.5),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, -6.65, 1.57, 0),
                new AutoUpper(s_Upper, -0.236328, 0.03, 0.3, 5)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, -3, 1.57, 0),
                new AutoUpper(s_Upper, -0.204589, -1, 0, 5)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, 0, 0, 0),
                new AutoUpper(s_Upper, -0.204589, -1, 0, 5)
            ),
            new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5)
        );
    }

    public SequentialCommandGroup X3() {
        return new SequentialCommandGroup(
            
        );
    }
}
