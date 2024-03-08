// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.robotConstants;
import frc.robot.commands.AutoAim;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TeleopUpper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UpperSub;
import frc.robot.subsystems.VisionSub;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final Swerve m_Swerve = new Swerve();
  private final UpperSub m_upper = new UpperSub();
  private final VisionSub m_vision = new VisionSub();

  private final XboxController driverController = new XboxController(robotConstants.DriverControllerID);

  private final TeleopSwerve teleopSwerve = new TeleopSwerve(m_Swerve, m_vision, driverController);
  private final TeleopUpper teleopUpper = new TeleopUpper(m_upper, m_vision, driverController);

  private final AutoAim autoAim = new AutoAim(m_Swerve, m_upper, m_vision);

  public static String alliance;
  private String command;
  // private String X;
  // private String Y;
  // private String Z;

  SendableChooser<String> m_Alliance = new SendableChooser<>();
  SendableChooser<String> m_AutoCommand = new SendableChooser<>();
  // SendableChooser<String> m_X = new SendableChooser<>();
  // SendableChooser<String> m_Y = new SendableChooser<>();
  // SendableChooser<String> m_Z = new SendableChooser<>();

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();

    m_robotContainer = new RobotContainer(m_Swerve, m_upper, m_vision);

    m_Alliance.setDefaultOption("RED", "RED");
    m_Alliance.addOption("BLUE", "BLUE");

    m_AutoCommand.setDefaultOption("X1Y1", "X1Y1");
    m_AutoCommand.addOption("X2Y2Z3", "X2Y2Z3");
    m_AutoCommand.addOption("X3Y3", "X3Y3");

    // m_X.setDefaultOption("X1", "X1");
    // m_X.addOption("X2", "X2");
    // m_X.addOption("X3", "X3");

    // m_Y.setDefaultOption("Y1", "Y1");
    // m_Y.addOption("Y2", "Y2");
    // m_Y.addOption("Y3", "Y3");

    // m_Y.setDefaultOption("Z1", "Z1");
    // m_Y.addOption("Z2", "Z2");
    // m_Y.addOption("Z3", "Z3");
    // m_Y.addOption("Z4", "Z4");
    // m_Y.addOption("Z5", "Z5");

    SmartDashboard.putData("Alliance Team", m_Alliance);
    SmartDashboard.putData("Auto Path", m_AutoCommand);
    // SmartDashboard.putData("X", m_X);
    // SmartDashboard.putData("Y", m_Y);
    // SmartDashboard.putData("Z", m_Z);
  }

  @Override
  public void robotPeriodic() {
    alliance = m_Alliance.getSelected();
    command = m_AutoCommand.getSelected();
    // X = m_X.getSelected();
    // Y = m_Y.getSelected();
    // Z = m_Z.getSelected();

    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    if(alliance == "RED") {
      switch(command){
        case "X1Y1":
          m_autonomousCommand = new Autos().X1Y1_r(m_Swerve, m_upper, m_vision);
          break;
        case "X2Y2Z3":
          m_autonomousCommand = new Autos().X2Y2Z3_r(m_Swerve, m_upper, m_vision);
          break;
        case "X3Y3":
          m_autonomousCommand = new Autos().X3Y3_r(m_Swerve, m_upper, m_vision);
          break;
      }
    } else if(alliance == "BLUE") {
      switch(command){
        case "X1Y1":
          m_autonomousCommand = new Autos().X1Y1_b(m_Swerve, m_upper, m_vision);
          break;
        case "X2Y2Z3":
          m_autonomousCommand = new Autos().X2Y2Z3_b(m_Swerve, m_upper, m_vision);
          break;
        case "X3Y3":
          m_autonomousCommand = new Autos().X3Y3_b(m_Swerve, m_upper, m_vision);
          break;
      }
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