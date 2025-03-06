package org.firstinspires.ftc.teamcode.HelperClasses;



public class Colors {
    public static class Color{
        public double r, g, b;
        public Color(double r, double g, double b){
            this.r = r;
            this.g = g;
            this.b = b;
        }

    }
    public enum ColorType {
        BLUE(new Color(125, 125, 255)),
//        GREEN(new Color(0, 255, 0)),
        RED(new Color(255, 198, 126)),
        YELLOW(new Color(200, 255, 116)),
        BLACK(new Color(0, 0, 0)),
        WHITE(new Color(255, 255, 223)),
        GREEN(new Color(0, 255, 0)),
//        CYAN(new Color(0, 255, 255)),
//        PURPLE(new Color(255, 0, 255)),
//        ORANGE(new Color(255, 128, 0)),
        NONE(new Color(50, 50, 50));
        private final Color color;

        ColorType(Color c) {
            this.color = c;
        }

        public Color getColor(){ return color; }

    }
    public static double getColorDistance(Color c1, Color c2) {
        double rDiff = c1.r - c2.r;
        double gDiff = c1.g - c2.g;
        double bDiff = c1.b - c2.b;
        return Math.cbrt(rDiff * rDiff + gDiff * gDiff + bDiff * bDiff);
    }


    public static ColorType getColorFromRGB(Color input){
        ColorType detected = ColorType.NONE;
        double minDist = Double.MAX_VALUE;
        for(ColorType ct : ColorType.values()){
            if(ct == ColorType.NONE) continue;
            double dist = getColorDistance(input, ct.getColor());
            if(dist < minDist){
                detected = ct;
                minDist = dist;
            }
        }
        return detected;
    }

}
