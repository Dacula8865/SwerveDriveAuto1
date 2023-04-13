package frc.robot;

public class MB_Math {
    public static double angDiffDeg(double ang1, double ang2){
        if(Math.abs(ang1-ang2)>180){
            if(ang1>ang2) return -ang1+ang2+360;
            else return -ang1+ang2-360;
        }
        return ang2-ang1;
    }
}
