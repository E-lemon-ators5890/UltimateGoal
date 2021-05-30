package org.firstinspires.ftc.teamcode.inperson;

import com.acmerobotics.dashboard.config.Config;

public class VisionConstants {
    @Config
    public static class RED_LEFT_VISION {
        public static double WIDTH = 0.96;
        public static double BOTTOM_HEIGHT = 0.5;
        public static double TOP_HEIGHT = 0.37;
    }

    @Config
    public static class RED_RIGHT_VISION {
        public static double WIDTH = 0.02;
        public static double BOTTOM_HEIGHT = 0.54;
        public static double TOP_HEIGHT = 0.41;
    }

    @Config
    public static class BLUE_RIGHT_VISION {
        public static double WIDTH = 0.02;
        public static double BOTTOM_HEIGHT = 0.55;
        public static double TOP_HEIGHT = 0.41;
    }

    @Config
    public static class BLUE_LEFT_VISION {
        public static double WIDTH = 0.96;
        public static double BOTTOM_HEIGHT = 0.50;
        public static double TOP_HEIGHT = 0.36;
    }
}
