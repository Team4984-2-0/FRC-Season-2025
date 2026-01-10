package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.resetheading;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.cscore.VideoSource;
import frc.robot.commands.ElevatorGoUp;
import frc.robot.commands.ElevatorGodown;
import frc.robot.subsystems.Climber;
import frc.robot.commands.Climb;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.ElevatorAutoPickup;
import frc.robot.commands.ElevatorAutoL2;
import frc.robot.commands.ElevatorAutoL3;
import frc.robot.commands.ElevatorAutoL4;
import frc.robot.commands.Climbbutback;
import frc.robot.commands.ArmDown;
import frc.robot.commands.ArmUP;
import frc.robot.commands.ArmAuto1;
import frc.robot.commands.ArmAuto2;
import frc.robot.commands.ShootAuto;
import frc.robot.commands.ArmAutoS;

import frc.robot.subsystems.Arm;




import frc.robot.commands.Launch;
import frc.robot.commands.Launchfast;

import frc.robot.commands.Intake;
import frc.robot.commands.ChangeSpeedHalf;
import frc.robot.commands.ChangeSpeedMax;
import frc.robot.commands.ChangeSpeedDanger;
public class RobotContainer {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
      
        private final Climber climber = new Climber();
  

        private final Arm arm = new Arm();

        private final Elevator elevator = new Elevator();

        private final Launcher launcher = new Launcher();
        
        private final SendableChooser<Command> autoChooser;

        private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);

        private final XboxController operatorJoytick = new XboxController(OIConstants.kOperatorControllerPort);
        SendableChooser<Command> m_Chooser = new SendableChooser<>();

        public RobotContainer() {
          
                NamedCommands.registerCommand("ElevatorAutoL4", (new ElevatorAutoL4(elevator)));
                NamedCommands.registerCommand("ArmAuto1", (new ArmAuto1(launcher)));

                NamedCommands.registerCommand("ArmUP", (new ArmUP(arm)));

                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("Auto Chooser", autoChooser);

                //m_Chooser.setDefaultOption("Auto Command", getAutonomousCommand());
                SmartDashboard.putData("Auto Mode",m_Chooser);
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                                () -> -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
                // launcher.setDefaultCommand(new Shooter(launcher));
                configureButtonBindings();
                // Ported Camera Code
                //SmartDashboard.putData("Auto Mode",m_Chooser);
                CameraThread myCameraThread = null;

                try {
                        myCameraThread = new CameraThread();
                        CameraServer.getServer("test");
                        // CameraServer.startAutomaticCapture();
                        usbCamera1 = CameraServer.startAutomaticCapture(myCameraThread.CAMERA1);
                        // usbCamera2 = CameraServer.startAutomaticCapture(myCameraThread.CAMERA2);
                        // CameraServer.getServer();
                        myCameraThread.server = CameraServer.getServer();
                        usbCamera1.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
                        myCameraThread.setCameraConfig();

                        myCameraThread.start();
                        myCameraThread.setResolutionHigh();
                        // myCameraThread.getCameraConfig();
                } finally {
                        myCameraThread = null;
                }
        }

        private void configureButtonBindings () {
                new JoystickButton(driverJoytick, 2).whileTrue(new resetheading(swerveSubsystem));
                new JoystickButton(driverJoytick, 1).whileTrue(new ChangeSpeedHalf(swerveSubsystem));
                new JoystickButton(driverJoytick, 3).whileTrue(new ChangeSpeedMax(swerveSubsystem));
                new JoystickButton(driverJoytick, 4).whileTrue(new ChangeSpeedDanger(swerveSubsystem));
            
                // Elevator & Arm for L4
                new JoystickButton(operatorJoytick,1).whileTrue(new ArmAuto1(launcher));
                new JoystickButton(operatorJoytick, 1).whileTrue(new ElevatorAutoL4(elevator));
                
                 // Elevator & Arm for Source
                new JoystickButton(operatorJoytick, 11).whileTrue(new ElevatorAutoPickup(elevator));
                new JoystickButton(operatorJoytick,11).whileTrue(new ArmAutoS(launcher));

                 // Elevator & Arm for L3
                new JoystickButton(operatorJoytick, 2).whileTrue(new ElevatorAutoL3(elevator));
                new JoystickButton(operatorJoytick,2).whileTrue(new ArmAuto2(launcher));

                 // Elevator & Arm for L2
                new JoystickButton(operatorJoytick, 3).whileTrue(new ElevatorAutoL2(elevator));
                new JoystickButton(operatorJoytick,3).whileTrue(new ArmAuto2(launcher));


                // Intake & Outake
                new JoystickButton(operatorJoytick, 10).whileTrue(new ArmDown(arm));
                new JoystickButton(operatorJoytick,9).whileTrue(new ArmUP(arm));
                new JoystickButton(operatorJoytick, 6).whileTrue(new Launchfast(arm));


                // Elevator
                new JoystickButton(operatorJoytick, 5).whileTrue(new ElevatorGoUp(elevator));
                new JoystickButton(operatorJoytick,4).whileTrue(new ElevatorGodown(elevator));

                // Arm
                new JoystickButton(operatorJoytick, 8).whileTrue(new Launch(launcher));
                new JoystickButton(operatorJoytick, 7).whileTrue(new Intake(launcher));


/*8 
                new JoystickButton(operatorJoytick, 8).whileTrue(new Climb(climber));
                new JoystickButton(operatorJoytick, 7).whileTrue(new Climbbutback(climber));

                new JoystickButton(operatorJoytick, 1).whileTrue(new ElevatorAutoL2(elevator));
               
              // XboxController.Button.
               // new JoystickButton(operatorJoytick, 7).whileTrue(new Launch(launcher)); */
               
        }

          public Command getAutonomousCommand() {
        // This method loads the auto when it is called, however, it is recommended
       // to first load your paths/autos when code starts, then return the
       // pre-loaded auto/path
       return autoChooser.getSelected();
       }
        public static UsbCamera usbCamera1 = null;

        // public static UsbCamera usbCamera2 = null;
        public class CameraThread extends Thread {
          final int CAMERA1 = 0;
          // final int CAMERA2 = 1;
          private final int currentCamera = CAMERA1; // UNCOMMENT WHEN RUNNING THE PROGRAM THRU ROBORIO!!!!
      
          VideoSink server;
      
          public void run() {
            System.out.println("CameraThread running");
      
          }
      
          public void setResolutionLow() {
            System.out.println("CameraThread rsetResolutionLow running");
            usbCamera1.setResolution(150, 150);
            usbCamera1.setFPS(Constants.CAMERA1_FPS);
      
          }
      
          public void setResolutionHigh() {
            System.out.println("CameraThread rsetResolutionHigh running");
            usbCamera1.setResolution(150, 150);
            usbCamera1.setFPS(Constants.CAMERA1_FPS);
          }
      
          public void setCameraSource() {
            System.out.println("CameraThread setCameraSource running");
            server.setSource(usbCamera1);
            SmartDashboard.putString("My Key", "Hello");
          }
      
          public void getCameraConfig() {
            System.out.println("CameraThread getPrintCameraConfig running");
            String cameraConfig;
            // issue when camera is not plugged in at start
            cameraConfig = usbCamera1.getConfigJson();
            if (cameraConfig.isEmpty() == false) {
              // System.out.println(cameraConfig.toString()); //print to console
            }
          }
      
          public void setCameraConfig() {
            System.out.println("CameraThread setPrintCameraConfig running");
      
            usbCamera1.setFPS(Constants.CAMERA1_FPS);
            usbCamera1.setBrightness(Constants.CAMERA1_BRIGHTNESS);
            usbCamera1.setExposureAuto();
          }
        }
      }