// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.generated.TunerConstants;
import frc.robot.handlers.Turret;
import static edu.wpi.first.units.Units.Feet;

/** Add your docs here. */
public class u_Lut {
    public u_Lut() {
    }

    private static boolean initialized = false;

    private static InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

    private static void initialize() {
        rpmMap.put(7.45, 1700.0);
        angleMap.put(7.45, 22.0);
        tofMap.put(7.45, .93);

        rpmMap.put(10.73, 1850.0);//short
        angleMap.put(10.73, 24.2);
        tofMap.put(10.73, .79);

        rpmMap.put(13.39, 2000.0);//Short
        angleMap.put(13.39, 25.0);
        tofMap.put(13.39, 1.33);

        rpmMap.put(15.42, 2100.0);
        angleMap.put(15.42, 25.0);
        tofMap.put(15.42, 1.44);



        
        rpmMap.put(16.91, 2200.0);
        angleMap.put(16.91, 26.0);
        tofMap.put(16.91, 1.38);

        
        rpmMap.put(18.22, 200.0);
        angleMap.put(18.22, 25.0);
        tofMap.put(18.22, 1.44);


        //       rpmMap.put(7.45, 1650.0);
        // angleMap.put(7.45, 22.0);
        // tofMap.put(7.45, .88);

        // rpmMap.put(10.73, 1875.0);//short
        // angleMap.put(10.73, 24.2);
        // tofMap.put(10.73, 1.2);

        // rpmMap.put(13.39, 2005.0);//Short
        // angleMap.put(13.39, 26.0);
        // tofMap.put(13.39, 1.2);

        // rpmMap.put(15.42, 2100.0);
        // angleMap.put(15.42, 26.0);
        // tofMap.put(15.42, 1.2);

        
        // rpmMap.put(16.91, 2200.0);
        // angleMap.put(16.91, 29.0);
        // tofMap.put(16.91, .9);

        // rpmMap.put(19.84, 2280.0);
        // angleMap.put(19.84, 29.0);
        // tofMap.put(16.84, .9);

        // rpmMap.put(5.05, 1610.0);
        // angleMap.put(5.05, 15.0);
        // tofMap.put(5.05,1.01);

        // rpmMap.put(6.6, 1660.0);
        // angleMap.put(6.6, 18.0);
        // tofMap.put(6.6,1.10);

        // rpmMap.put(8.24, 1810.0);
        // angleMap.put(8.24, 20.0);
        // tofMap.put(8.24,1.10);

        // rpmMap.put(8.24, 1810.0);
        // angleMap.put(8.24, 20.0);
        // tofMap.put(8.24,1.10);

        
        // rpmMap.put(10.66, 1860.0);
        // angleMap.put(10.66, 21.5);
        // tofMap.put(10.66,1.16);

        
        // rpmMap.put(13.52, 1960.0);
        // angleMap.put(13.52, 23.5);
        // tofMap.put(13.52,1.16);

               
        // rpmMap.put(15.45, 209.0);
        // angleMap.put(15.45, 25.5);
        // tofMap.put(15.45,1.15);
        initialized = true;
    }

    public static double getRpmFrom(double feet) {
        if (!initialized)
            initialize();

        return rpmMap.get(feet);
    }

    public static double getAngleFrom(double feet) {
        if (!initialized)
            initialize();

        return angleMap.get(feet);
    }

    public static double getTofFrom(double feet) {
        if (!initialized)
            initialize();

        return tofMap.get(feet);
    }

    public static double getTof() {
        if (!initialized)
            initialize();

        return tofMap.get(getDist());
    }

    static Pose2d currentGoalPosition;
    static Pose2d translatedGoalPose;
    static Pose2d translatedTurretPose;

    static ChassisSpeeds speeds;

    static double gears;

    final static private CommandSwerveDrivetrain s_swerve = TunerConstants.getInstance();
    final static private QuestNavSubsystem QNav = QuestNavSubsystem.getInstance();
    final static private Supplier<Pose2d> robotPoseSupplier = () -> QNav.getPose();
    
    final static private Supplier<ChassisSpeeds> chassisSpeedSupplier = () -> ChassisSpeeds
            .fromRobotRelativeSpeeds(s_swerve.getState().Speeds, robotPoseSupplier.get().getRotation());

    private static final Supplier<Pose2d> goalPosition = () -> Turret.getInstance().getGoal();

    public static double getDist(){
        speeds = chassisSpeedSupplier.get();

        currentGoalPosition = goalPosition.get();

        translatedTurretPose = robotPoseSupplier.get().transformBy(new Transform2d(0.196, 0.0, new Rotation2d()));

        return Meter.of(Math.sqrt(Math.pow((translatedTurretPose.getX() - currentGoalPosition.getX()), 2)
            + Math.pow((translatedTurretPose.getY() - currentGoalPosition.getY()), 2))).in(Feet);
    }
}
