package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.TestFly;
import frc.robot.subsystems.TestHang;
import frc.robot.subsystems.TestIntake;
import frc.robot.subsystems.TestHopper;
// import frc.robot.subsystems.arm.Arm;
// import frc.robot.subsystems.arm.ArmIO;
// import frc.robot.subsystems.arm.ArmIOCTRE;
// import frc.robot.subsystems.arm.ArmIOSIM;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.drive.requests.SwerveSetpointGen;
// import frc.robot.subsystems.elevator.Elevator;
// import frc.robot.subsystems.elevator.ElevatorIO;
// import frc.robot.subsystems.elevator.ElevatorIOSIM;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSIM;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSIM;
import frc.robot.utils.TunableController;
import frc.robot.utils.TunableController.TunableControllerType;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
        private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;

        // private final TunablePController joystick = new TunableController(0)
        // .withControllerType(TunableControllerType.QUADRATIC);

        // private final TunableController joystick2 = new TunableController(1)
        // .withControllerType(TunableControllerType.LINEAR);

        private final CommandPS5Controller joystick = new CommandPS5Controller(0);
        private final CommandPS5Controller joystick2 = new CommandPS5Controller(1);

        private final LoggedDashboardChooser<Command> autoChooser;

        public final Drive drivetrain;
        // CTRE Default Drive Request
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed.times(0.1))
                        .withRotationalDeadband(Constants.MaxAngularRate.times(0.1)) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        // private final Flywheel flywheel;
        // private final Elevator elevator;
        // private final Arm arm;

        private final TestFly fly;
        private final TestIntake intake;
        private final TestHang hang;
        private final TestHopper hopper;

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        public RobotContainer() {
                DriveIOCTRE currentDriveTrain = TunerConstants.createDrivetrain();
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                drivetrain = new Drive(currentDriveTrain);

                                new Vision(
                                                drivetrain::addVisionData,
                                                new VisionIOLimelight("limelight-fl", drivetrain::getVisionParameters),
                                                new VisionIOLimelight("limelight-fr", drivetrain::getVisionParameters),
                                                new VisionIOLimelight("limelight-bl", drivetrain::getVisionParameters),
                                                new VisionIOLimelight("limelight-br", drivetrain::getVisionParameters));

                                // flywheel = new Flywheel(new FlywheelIOCTRE()); // Disabled to prevent robot
                                // movement if
                                // deployed to a real robot
                                // flywheel = new Flywheel(new FlywheelIO() {});
                                // elevator = new Elevator(new ElevatorIOCTRE()); // Disabled to prevent robot
                                // movement if
                                // deployed to a real robot
                                // elevator = new Elevator(new ElevatorIO() {});
                                // arm = new Arm(new ArmIOCTRE()); // Disabled to prevent robot movement if
                                // deployed to a
                                // real robot
                                // arm = new Arm(new ArmIO() {});

                                fly = new TestFly();

                                intake = new TestIntake();
                                hang = new TestHang();
                                hopper = new TestHopper();

                                break;

                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                drivetrain = new Drive(currentDriveTrain);

                                new Vision(
                                                drivetrain::addVisionData,
                                                new VisionIOPhotonVisionSIM(
                                                                "Front Camera",
                                                                new Transform3d(
                                                                                new Translation3d(0.2, 0.0, 0.8),
                                                                                new Rotation3d(0, Math.toRadians(20),
                                                                                                Math.toRadians(0))),
                                                                drivetrain::getVisionParameters),
                                                new VisionIOPhotonVisionSIM(
                                                                "Back Camera",
                                                                new Transform3d(
                                                                                new Translation3d(-0.2, 0.0, 0.8),
                                                                                new Rotation3d(0, Math.toRadians(20),
                                                                                                Math.toRadians(180))),
                                                                drivetrain::getVisionParameters),
                                                new VisionIOPhotonVisionSIM(
                                                                "Left Camera",
                                                                new Transform3d(
                                                                                new Translation3d(0.0, 0.2, 0.8),
                                                                                new Rotation3d(0, Math.toRadians(20),
                                                                                                Math.toRadians(90))),
                                                                drivetrain::getVisionParameters),
                                                new VisionIOPhotonVisionSIM(
                                                                "Right Camera",
                                                                new Transform3d(
                                                                                new Translation3d(0.0, -0.2, 0.8),
                                                                                new Rotation3d(0, Math.toRadians(20),
                                                                                                Math.toRadians(-90))),
                                                                drivetrain::getVisionParameters));

                                // flywheel = new Flywheel(new FlywheelIOSIM());
                                // elevator = new Elevator(new ElevatorIOSIM());
                                // arm = new Arm(new ArmIOSIM());
                                fly = new TestFly();

                                intake = new TestIntake();
                                hang = new TestHang();
                                hopper = new TestHopper();

                                break;

                        default:
                                // Replayed robot, disable IO implementations
                                drivetrain = new Drive(new DriveIO() {
                                });

                                new Vision(
                                                drivetrain::addVisionData,
                                                new VisionIO() {
                                                },
                                                new VisionIO() {
                                                },
                                                new VisionIO() {
                                                },
                                                new VisionIO() {
                                                });

                                // flywheel = new Flywheel(new FlywheelIO() {});
                                // elevator = new Elevator(new ElevatorIO() {});
                                // arm = new Arm(new ArmIOCTRE() {});
                                fly = new TestFly();
                                intake = new TestIntake();
                                hang = new TestHang();
                                hopper = new TestHopper();

                                break;
                }

                // Set up auto routines
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

                // Set up SysId routines
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Forward)",
                                drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Reverse)",
                                drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                autoChooser.addOption(
                                "Drive SysId (Dynamic Forward)",
                                drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Dynamic Reverse)",
                                drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
                autoChooser.addOption(
                                "Drive Wheel Radius Characterization",
                                DriveCommands.wheelRadiusCharacterization(drivetrain));
                fly.setDefaultCommand(fly.runTakeOnce(0));
                hopper.setDefaultCommand(
                                hopper.runTake(() -> 0));
                intake.setDefaultCommand(
                                intake.runTake(() -> joystick.getL2Axis() -
                                                joystick.getR2Axis()));

                configureBindings();
        }

        private void configureBindings() {

                // hang.setDefaultCommand(
                // hang.runTake(() -> joystick2.getLeftY()));

                joystick.cross().toggleOnTrue(hopper.runTake(() -> -1).alongWith(fly.runTakeOnce(1)))
                                .toggleOnFalse(hopper.runTake(() -> 0).alongWith(fly.runTakeOnce(0)));

                // joystick2.axisMagnitudeGreaterThan(PS5Controller.Axis.kLeftY.value,
                // 0.01).onTrue(intake.runPivot(()-> joystick2.getLeftY()));

                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(
                                                () -> drive
                                                                .withVelocityX(
                                                                                MaxSpeed.times(
                                                                                                joystick
                                                                                                                .getLeftY())) // Drive
                                                                                                                              // forward
                                                                                                                              // with
                                                                                                                              // negative
                                                                                                                              // Y
                                                                                                                              // (forward)
                                                                .withVelocityY(
                                                                                MaxSpeed.times(
                                                                                                joystick.getLeftX())) // Drive
                                                                                                                      // left
                                                                                                                      // with
                                                                                                                      // negative
                                                                                                                      // X
                                                                                                                      // (left)
                                                                .withRotationalRate(
                                                                                Constants.MaxAngularRate.times(
                                                                                                -joystick
                                                                                                                .getRightX())))); // Drive
                                                                                                                                  // counterclockwise
                                                                                                                                  // with
                                                                                                                                  // negative
                                                                                                                                  // X
                                                                                                                                  // (left)

                // joystick.a().onTrue(Commands.runOnce(() ->
                // drivetrain.resetPose(Pose2d.kZero)));
                joystick.circle()
                                .whileTrue(
                                                drivetrain.applyRequest(
                                                                () -> point.withModuleDirection(
                                                                                new Rotation2d(-joystick.getLeftY(),
                                                                                                -joystick.getLeftX()))));

                // Custom Swerve Request that use PathPlanner Setpoint Generator. Tuning NEEDED.
                // Instructions
                // can be found here
                // https://hemlock5712.github.io/Swerve-Setup/talonfx-swerve-tuning.html
                SwerveSetpointGen setpointGen = new SwerveSetpointGen(
                                drivetrain.getChassisSpeeds(),
                                drivetrain.getModuleStates(),
                                drivetrain::getRotation)
                                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
                joystick
                                .square()
                                .whileTrue(
                                                drivetrain.applyRequest(
                                                                () -> setpointGen
                                                                                .withVelocityX(MaxSpeed.times(
                                                                                                -joystick.getLeftY()))
                                                                                .withVelocityY(MaxSpeed.times(
                                                                                                -joystick.getLeftX()))
                                                                                .withRotationalRate(
                                                                                                Constants.MaxAngularRate
                                                                                                                .times(-joystick.getRightX()))
                                                                                .withOperatorForwardDirection(drivetrain
                                                                                                .getOperatorForwardDirection())));

        }

        public Command getAutonomousCommand() {
                return autoChooser.get();
        }
}
