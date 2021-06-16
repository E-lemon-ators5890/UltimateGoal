package org.firstinspires.ftc.teamcode.inperson;

import com.acmerobotics.dashboard.config.Config;

public class VisionConstants {
    @Config
    public static class RED_LEFT_VISION {
        public static double WIDTH = 0.95;
        public static double BOTTOM_HEIGHT = 0.399999999999;
        public static double TOP_HEIGHT = 0.2599999999;
    }

    @Config
    public static class RED_RIGHT_VISION {
        public static double WIDTH = 0.05;
        public static double BOTTOM_HEIGHT = 0.339999999999;
        public static double TOP_HEIGHT = 0.19999999999;
    }

    @Config
    public static class BLUE_RIGHT_VISION {
        public static double WIDTH = 0.01;
        public static double BOTTOM_HEIGHT = 0.339999999;
        public static double TOP_HEIGHT = 0.1999999999;
    }

    @Config
    public static class BLUE_LEFT_VISION {
        public static double WIDTH = 0.94;
        public static double BOTTOM_HEIGHT = 0.398999999;
        public static double TOP_HEIGHT = 0.2699999;
    }
}
