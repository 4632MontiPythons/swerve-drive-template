package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    //Prevent instantiation
    private Constants() {
    }

    public static final class Drive {
        //Most Drive constants are located in /generated/TunerConstants.java
        public static final double odometryXYStdDevs = 0.03;
        public static final double odometryThetaStdDev = 0.015;
        
        //Auto config
        public final static PPHolonomicDriveController ppController =
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), //translation
                new PIDConstants(4.0, 0.0, 0.0)  //rotation
            );

        //sysID
        public static final double translationRampRate = 0.75; //for quasistatic (volts per second)
        public static final double translationStep = 4; //for dynamic (volts)
        public static final double timeout = 10;
        //  assuming max speed of 4.5 m/s, feet traveled during sysID rotuine = 0.615 *timeout^2 *ramprate

    }

    public static final class Vision {
        public static final int minTags = 2; // Min. tags seen in a measurement for it to be considered valid
        //*Should always be at least 1*; decide between 1 and 2. This year I think 2 is better because the tags usually comes in pairs(two on each side of both hubs), so it's ok to ignore singular tags.
        //We'll really only see single tags when in the neutral zone, and we don't need pose estimation there, and the information we gleaned will be slightly messed up after going over the bump anyways

        public static final double maxYawRate_DegPerSec = 300; //ignore visual measurements when yaw rate is greater than this value
        public static final double maxTagDistance_Meters = 5.0; //ignore visual measurements when distance to tag is greater than this value.
        public static final double minStdDev_Meters = 0.01; //std dev for xy vision measurement can never be smaller than this value
        public static final double stdDevPerMeter = 0.02; //Std dev increase per meter; farther from tag means less accurate measurement
    }

    public static final class OI {
        public static final double deadband = 0.05; //percentage of max speed/rotational rate. e.g. 10% deadband should be 0.10
        public static final int driverControllerPort = 0;
        public static final double slewRate = 3.0; //limits change to 100k% per second, meaning would take 1/k seconds to go from 0 to full throttle
    }
    // public static final class Shooter {
    //     public static final Translation2d redGoalXY = new Translation2d(11.915394, 4.034536); //meters
    //     public static final Translation2d blueGoalXY = new Translation2d(4.625594, 4.034536); //meters

    // }  
}