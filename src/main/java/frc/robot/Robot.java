// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.robotConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TeleopUpper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UpperSub;
import frc.robot.subsystems.VisionSub;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final Swerve m_Swerve = new Swerve();
  private final UpperSub m_upper = new UpperSub();
  private final VisionSub m_vision = new VisionSub();

  private final XboxController driverController = new XboxController(robotConstants.DriverControllerID);

  private final TeleopSwerve teleopSwerve = new TeleopSwerve(m_Swerve, m_vision, driverController);
  private final TeleopUpper teleopUpper = new TeleopUpper(m_upper, driverController);

  public static String alliance;
  private String command;
  private int delay;

  SendableChooser<String> m_Alliance = new SendableChooser<>();
  SendableChooser<String> m_AutoCommand = new SendableChooser<>();
  SendableChooser<Integer> m_Delay = new SendableChooser<>();

  @Override
  public void robotInit() {
    
    m_Alliance.setDefaultOption("RED", "RED");
    m_Alliance.addOption("BLUE", "BLUE");

    // left=L mid=M right=R base=B red=r blue=b   combination -> initial pos(L,M,B)+base or not(B or N)+preload and note place or only leave(PXaYb or PL,(a=1,2,3... , b=1,2,3...))+how many notes(xn, x=1, 2, 3...)+red or blue(r,b)
    // 0 note
    m_AutoCommand.setDefaultOption("OURAUTOISFUCKED", "OURAUTOISFUCKED");

    // 1 note
    m_AutoCommand.addOption("LBP1n", "LBP1n");
    m_AutoCommand.addOption("MBP1n", "MBP1n");
    m_AutoCommand.addOption("RBP1n", "RBP1n");
    m_AutoCommand.addOption("LBPL1n", "LBPL1n");
    m_AutoCommand.addOption("RBPL1n", "RBPL1n");

    // 2 note
    m_AutoCommand.addOption("MBPX12n", "MBPX12n");
    m_AutoCommand.addOption("MBPX22n", "MBPX22n");
    m_AutoCommand.addOption("MBPX32n", "MBPX32n");

    // 3 note
    m_AutoCommand.addOption("MBPX2X13n", "MBPX2X13n");
    m_AutoCommand.addOption("MBPX2X33n", "MBPX2X33n");

    // 4 note
    m_AutoCommand.addOption("MBPX2X1X34n", "MBPX2X1X34n");

    m_Delay.setDefaultOption("0", 0);
    for(int i=1;i<=15;i++) m_Delay.addOption("" + i, i);

    SmartDashboard.putData("Alliance Team", m_Alliance);
    SmartDashboard.putData("Auto Path", m_AutoCommand);
    SmartDashboard.putData("Auto Delay", m_Delay);
  }

  @Override
  public void robotPeriodic() {
    alliance = m_Alliance.getSelected();
    command = m_AutoCommand.getSelected();
    delay = m_Delay.getSelected();

    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    robotConstants.mode = "DISABLED";
  }

  @Override
  public void disabledPeriodic() {
    m_upper.charge(255, 0, 0, (driverController.getLeftY() + driverController.getRightY()));
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    robotConstants.mode = "AUTO";
    switch(alliance){
      case "RED":
        switch(command){
          // 0 note
          case "OURAUTOISFUCKED":
            m_autonomousCommand = null;
            break;

          // 1 note
          case "LBP1n":
            m_autonomousCommand = new Autos().Left_1n(m_Swerve, m_upper);
            break;
          case "MBP1n":
            m_autonomousCommand = new Autos().Mid_1n(m_Swerve, m_upper);
            break;
          case "RBP1n":
            m_autonomousCommand = new Autos().Right_1n(m_Swerve, m_upper);
            break;
          case "LBPL1n":
            m_autonomousCommand = new Autos().LeftLeave_r1n(m_Swerve, m_upper, delay);
            break;
          case "RBPL1n":
            m_autonomousCommand = new Autos().RightLeave_r1n(m_Swerve, m_upper, delay);
            break;

          // 2 notes
          case "MBPX12n":
            m_autonomousCommand = new Autos().MidX1Base_2n(m_Swerve, m_upper);
            break;
          case "MBPX22n":
            m_autonomousCommand = new Autos().MidX2Base_2n(m_Swerve, m_upper);
            break;
          case "MBPX32n":
            m_autonomousCommand = new Autos().MidX3Base_2n(m_Swerve, m_upper);
            break;

          // 3 note
          case "MBPX2X13n":
            m_autonomousCommand = new Autos().MidX2BaseX1Base_3n(m_Swerve, m_upper);
            break;
          case "MBPX2X33n":
            m_autonomousCommand = new Autos().MidX2BaseX3Base_3n(m_Swerve, m_upper);
            break;

          // 4 note 0=left 1=right
          case "MBPX2X1X34n":
            m_autonomousCommand = new Autos().MidX2BaseX1BaseX3Base_4n(m_Swerve, m_upper, alliance, delay);
            break;
        }
        break;
      case "BLUE":
        switch(command){
          // 0 note
          case "OURAUTOISFUCKED":
            m_autonomousCommand = null;
            break;

          // 1 note
          case "LBP1n":
            m_autonomousCommand = new Autos().Left_1n(m_Swerve, m_upper);
            break;
          case "MBP1n":
            m_autonomousCommand = new Autos().Mid_1n(m_Swerve, m_upper);
            break;
          case "RBP1n":
            m_autonomousCommand = new Autos().Right_1n(m_Swerve, m_upper);
            break;
          case "LBPL1n":
            m_autonomousCommand = new Autos().LeftLeave_b1n(m_Swerve, m_upper, delay);
            break;
          case "RBPL1n":
            m_autonomousCommand = new Autos().RightLeave_b1n(m_Swerve, m_upper, delay);
            break;

          // 2 note
          case "MBPX12n":
            m_autonomousCommand = new Autos().MidX1Base_2n(m_Swerve, m_upper);
            break;
          case "MBPX22n":
            m_autonomousCommand = new Autos().MidX2Base_2n(m_Swerve, m_upper);
            break;
          case "MBPX32n":
            m_autonomousCommand = new Autos().MidX3Base_2n(m_Swerve, m_upper);
            break;    

          // 3 note
          case "MBPX2X13n":
            m_autonomousCommand = new Autos().MidX2BaseX1Base_3n(m_Swerve, m_upper);
            break;
          case "MBPX2X33n":
            m_autonomousCommand = new Autos().MidX2BaseX3Base_3n(m_Swerve, m_upper);
            break;

          // 4 note
          case "MBPX2X1X34n":
            m_autonomousCommand = new Autos().MidX2BaseX1BaseX3Base_4n(m_Swerve, m_upper, alliance, delay);
            break;
        }
        break;
    } 

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    robotConstants.mode = "TELE";
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_Swerve.setDefaultCommand(teleopSwerve);
    m_upper.setDefaultCommand(teleopUpper);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}