// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import static edu.wpi.first.units.Units.Meter;

import java.lang.reflect.Field;
import java.util.function.DoubleBinaryOperator;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.s_Turret;
import frc.robot.subsystems.Touchboard.Touchboard;
import yams.mechanisms.swerve.SwerveDrive;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public class Turret extends SubsystemBase implements StateSubsystem {
  /** Creates a new TurretTracker. */

  private TurretStates desiredState, currentState = TurretStates.TRACKING;

  private s_Turret turret = s_Turret.getInstance();
  private static Turret m_Instance;

  private CommandSwerveDrivetrain s_swerve = TunerConstants.getInstance();
  private Supplier<Pose2d> robotPoseSupplier = () -> s_swerve.getState().Pose;

  private Supplier<ChassisSpeeds> chassisSpeedSupplier = ()-> ChassisSpeeds.fromRobotRelativeSpeeds(s_swerve.getState().Speeds, robotPoseSupplier.get().getRotation());

  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable driveStateTable = networkTable.getTable("DriveState/TurretTurntable");
  private final NetworkTable stateTable = networkTable.getTable("RobotStates");

  private final StructPublisher<Pose2d> turretPose = driveStateTable.getStructTopic("turretPose", Pose2d.struct)
      .publish();
  private final StructPublisher<Pose2d> goalPose = driveStateTable.getStructTopic("goalPose", Pose2d.struct).publish();
  private final StructPublisher<Pose2d> virtualGoalPose = driveStateTable
      .getStructTopic("virtualGoalPose", Pose2d.struct).publish();

  private final DoublePublisher storedRotation = driveStateTable.getDoubleTopic("storedRotation").publish();

  private final StringPublisher stateShower = stateTable.getStringTopic("TurretState").publish();

  private boolean unwindCommandBound = false;

  private Pose2d goalPosition = new Pose2d(4.620419, 4.034631, new Rotation2d());

  public static final Pose2d HUB_BLUE_POSE = new Pose2d(4.620419, 4.034631, new Rotation2d());
  public static final Pose2d HUB_RED_POSE = FlippingUtil.flipFieldPose(HUB_BLUE_POSE);

  public static final Pose2d FEED_BLUE_LEFT = new Pose2d(0.5, 6, new Rotation2d());
  public static final Pose2d FEED_BLUE_RIGHT = new Pose2d(0.5, 1.6, new Rotation2d());
  public static final Pose2d FEED_RED_LEFT = FlippingUtil.flipFieldPose(FEED_BLUE_RIGHT);
  public static final Pose2d FEED_RED_RIGHT = FlippingUtil.flipFieldPose(FEED_BLUE_LEFT);

  private double previousRotation = 0.0;
  private double currentRotation = 0.0;
  String alliance = "";

  public Turret() {
    turret.setDegreeCommand();
    goalPose.set(goalPosition);

    this.setDesiredState(desiredState);

    RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop()).onTrue(Commands.runOnce(() -> {
      if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
          goalPosition = FlippingUtil.flipFieldPose(goalPosition);
          alliance = "red";

          goalPose.set(goalPosition);
        } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
          alliance = "blue";
        }
      }

    }));
  }

  public static Turret getInstance() {
    if (m_Instance == null) {
      m_Instance = new Turret();
    }
    return m_Instance;
  }

  public void setGoal(Supplier<Pose2d> goalPosition) {
    this.goalPosition = goalPosition.get();
    goalPose.set(goalPosition.get());
  }

  public Pose2d getGoal() {
    return this.goalPosition;
  }

  final double blueDistMeters = FieldConstants.ALLIANCE_ZONE.in(Meter) + 1.2192;
  final double redDistMeters = FieldConstants.FIELD_LENGTH.minus(FieldConstants.ALLIANCE_ZONE).in(Meter) - 1.2192;

  public boolean inAllianceZone(Pose2d turretPose) {

    switch (alliance) {
      case "":
        if (DriverStation.getAlliance().isPresent()) {
          if (DriverStation.getAlliance().get() == Alliance.Blue) {
            alliance = "blue";
          } else if (DriverStation.getAlliance().get() == Alliance.Red) {
            alliance = "red";
          }
          return true;
        }
        break;
      case "blue":
        if (turretPose.getX() < blueDistMeters) {
          return true;
        }
        return false;
      case "red":
        if (turretPose.getX() > redDistMeters) {
          return true;
        }
        return false;
    }
    return true;
  }

  private void setHubAsGoal(String alliance) {
    switch (alliance) {
      case "":
        this.setGoal(() -> HUB_BLUE_POSE);
        break;
      case "blue":
        this.setGoal(() -> HUB_BLUE_POSE);
        break;
      case "red":
        this.setGoal(() -> HUB_RED_POSE);
        break;
      default:
        break;
    }
  }

  double halfField = FieldConstants.FIELD_WIDTH.div(2).in(Meter);
  Pose2d feedGoal = FEED_BLUE_LEFT;

  private void setFeedAsGoal(Pose2d turretPose, String alliance) {

    if (alliance == "red") {
      if (turretPose.getY() > halfField) {
        feedGoal = FEED_RED_LEFT;
      } else {
        feedGoal = FEED_RED_RIGHT;
      }

    } else {
      if (turretPose.getY() > halfField) {
        feedGoal = FEED_BLUE_LEFT;
      } else {
        feedGoal = FEED_BLUE_RIGHT;
      }
    }

    this.setGoal(() -> feedGoal);
  }

  double dist = 0;

  public double findSpeedModifier(double speed) {
    switch (alliance) {
      case "":
        return -1.0 * speed;
      case "blue":
        return -1.0 * speed;
      case "red":
        return 1.0 * speed;
    }
    return 1.0 * speed;

  }

  Pose2d robotPose;
  Pose2d translatedTurretPose;
  Pose2d translatedGoalPose;
  ChassisSpeeds speeds;

  Rotation2d toGoal;
  Rotation2d robotRealtiveRotation;
  double currentRotationChange;
  Command turretResetter;

  private final CommandXboxController coPilot = new CommandXboxController(1);

  public void update() {
    if (turret.inaccurate.get() && currentState == TurretStates.TRACKING) {
      setDesiredState(TurretStates.INACCURATE);
    } else if (!turret.inaccurate.get() && currentState == TurretStates.INACCURATE) {
      setDesiredState(TurretStates.TRACKING);
    }
    switch (currentState) {
      case IDLE:
        break;
      case BROKEN:
        break;
      case INACCURATE:
      case TRACKING:
        robotPose = robotPoseSupplier.get();

        translatedTurretPose = robotPose.transformBy(new Transform2d(0.17145000, 0.0, new Rotation2d()));

        speeds = chassisSpeedSupplier.get();

        translatedGoalPose = goalPosition
            .transformBy(new Transform2d(findSpeedModifier(speeds.vxMetersPerSecond),
                findSpeedModifier(speeds.vyMetersPerSecond), new Rotation2d()));

        if (inAllianceZone(translatedTurretPose)) {
          setHubAsGoal(alliance);
        } else {
          setFeedAsGoal(translatedTurretPose, alliance);
        }

        virtualGoalPose.set(translatedGoalPose);

        toGoal = Rotation2d.fromRadians(Math.atan2(translatedGoalPose.getY() - translatedTurretPose.getY(),
            translatedGoalPose.getX() - translatedTurretPose.getX()));

        turretPose.set(translatedTurretPose);

        robotRealtiveRotation = Rotation2d
            .fromRadians(toGoal.getRadians() - robotPose.getRotation().getRadians());

        currentRotationChange = (robotRealtiveRotation.getDegrees() - previousRotation);

        if (currentRotationChange > 180) {
          currentRotationChange = currentRotationChange - 360;
        } else if (currentRotationChange < -180) {
          currentRotationChange = currentRotationChange + 360;
        }
        currentRotation += currentRotationChange;

        if (currentRotation > 360) {
          currentRotation = (currentRotation % 360);

        } else if (currentRotation <= -360) {
          currentRotation = (currentRotation % 360);

        }

        turret.setDegrees(currentRotation);

        storedRotation.set(currentRotation);

        previousRotation = robotRealtiveRotation.getDegrees();
        break;
      case UNWINDING:
        if (!unwindCommandBound) {
          unwindCommandBound = true;

          turretResetter = turret.getDegreeSetter(currentRotation).andThen(new InstantCommand(() -> {
            unwindCommandBound = false;
            setDesiredState(TurretStates.TRACKING);
            currentRotation = currentRotation % 360;
          }));

          CommandScheduler.getInstance().schedule(turretResetter);
        }

        break;
      case ZEROING:
        turret.setSpeed(coPilot.getRightX());
        break;
      case MANUAL:

      default:
        break;
    }

  }

  public void setDesiredState(State state) {
    if (this.desiredState != state) {
      desiredState = (TurretStates) state;
      handleStateTransition();
    }
  }

  double currentOffset = 0;

  public void increaseDeg() {
    currentOffset += 0.75;
    turret.setOffset(currentOffset);
  }

  public void decreaseDeg() {
    currentOffset -= 0.75;
    turret.setOffset(currentOffset);

  }

  public void handleStateTransition() {

    if (currentState == TurretStates.TRACKING && desiredState == TurretStates.INACCURATE) {
      stateShower.set("INACCURATEstateSkip");

      currentState = desiredState;

      return;
    } else if (currentState == TurretStates.INACCURATE && desiredState == TurretStates.TRACKING) {
      stateShower.set("TRACKINGstateSkip");

      currentState = desiredState;

      return;
    }

    if (currentState == TurretStates.ZEROING) {
      turret.setCurrentPoseAsZero();
    }

    switch (desiredState) {
      case IDLE:
        stateShower.set("IDLE");
        break;
      case BROKEN:
        stateShower.set("BROKEN");

        turret.stop();
        break;
      case TRACKING:
        stateShower.set("TRACKING");
        unwindCommandBound = false;
        turret.setDegreeCommand();
        goalPose.set(goalPosition);
        break;
      case INACCURATE:
        stateShower.set("INACCURATE");
        unwindCommandBound = false;
        turret.setDegreeCommand();
        goalPose.set(goalPosition);
        break;
      case UNWINDING:
        stateShower.set("UNWINDING");

        turret.stop();
        break;
      case SYSID:
        stateShower.set("SYSID");

        turret.runSYSID();
        break;
      case ZEROING:
        stateShower.set("ZEROING");
        turret.setZeroCommand();

        break;
      default:
        stateShower.set("UNKNOWN");
        break;
    }

    currentState = desiredState;
  }

  @Override
  public void periodic() {
    update();
    // This method will be called once per scheduler run
  }

  public enum TurretStates implements State {
    IDLE,
    BROKEN,
    TRACKING,
    INACCURATE,
    MANUAL,
    UNWINDING,
    ZEROING,
    SYSID,
  }
}