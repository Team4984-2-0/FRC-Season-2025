package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.subsystems.AutoFeeder;
import frc.robot.commands.ElevatorAuto;
import frc.robot.commands.Launch;
import frc.robot.commands.Intake;

public class RobotContainer {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
      
        private final Climber climber = new Climber();
  
        private final AutoFeeder elevatorAuto = new AutoFeeder();
       
        private final Launcher launcher = new Launcher();
        

        private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);

        private final XboxController operatorJoytick = new XboxController(OIConstants.kOperatorControllerPort);

        public RobotContainer() {
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                                () -> -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
                // launcher.setDefaultCommand(new Shooter(launcher));
                configureButtonBindings();
                // Ported Camera Code
 
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
                new JoystickButton(operatorJoytick, 5).whileTrue(new Intake(launcher));
                new JoystickButton(operatorJoytick, 8).whileTrue(new Climb(climber));
                new JoystickButton(operatorJoytick, 6).whileTrue(new Launch(launcher));
                new JoystickButton(operatorJoytick, 2).whileTrue(new ElevatorGoUp(elevatorAuto));
               new JoystickButton(operatorJoytick, 3).whileTrue(new ElevatorAuto(elevatorAuto));
                new JoystickButton(operatorJoytick, 1).whileTrue(new ElevatorGodown(elevatorAuto));
              // XboxController.Button.
               // new JoystickButton(operatorJoytick, 7).whileTrue(new Launch(launcher));
               
        }

        public Command getAutonomousCommand() {
                // 1. Create trajectory settings
                TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                                AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                .setKinematics(DriveConstants.kDriveKinematics);

                // 2. Generate trajectory
                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(
                                                new Translation2d(1, 0),
                                                new Translation2d(1, -1)),
                                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                                trajectoryConfig);

                // 3. Define PID controllers for tracking trajectory
                PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
                PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
                ProfiledPIDController thetaController = new ProfiledPIDController(
                                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                // 4. Construct command to follow trajectory
                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                                trajectory,
                                swerveSubsystem::getPose,
                                DriveConstants.kDriveKinematics,
                                xController,
                                yController,
                                thetaController,
                                swerveSubsystem::setModuleStates,
                                swerveSubsystem);

                // 5. Add some init and wrap-up, and return everything
                return new SequentialCommandGroup(
                                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                                swerveControllerCommand,
                                new InstantCommand(() -> swerveSubsystem.stopModules()));
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