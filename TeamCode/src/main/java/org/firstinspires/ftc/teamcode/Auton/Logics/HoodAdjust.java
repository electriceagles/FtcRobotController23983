package org.firstinspires.ftc.teamcode.Auton.Logics;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class HoodAdjust {
    public RobotHardware hardwareMap;
    private Servo hood;
    private final LimeDistance limeDistance = new LimeDistance();

    public void init() {
        limeDistance.init();
        hood = hardwareMap.hood;
    }

    public void update() {
        if (limeDistance.distance <= 5){
            hood.setPosition(1);
        }
        if (limeDistance.distance <= 10 && limeDistance.distance > 5){
            hood.setPosition(0.95);
        }
        if (limeDistance.distance <= 20 && limeDistance.distance > 10){
            hood.setPosition(0.9);
        }
        if (limeDistance.distance <= 30 && limeDistance.distance > 20){
            hood.setPosition(0.85);
        }
        if (limeDistance.distance <= 40 && limeDistance.distance > 30){
            hood.setPosition(0.8);
        }
        if (limeDistance.distance <= 50 && limeDistance.distance > 40){
            hood.setPosition(0.75);
        }
        if (limeDistance.distance <= 60 && limeDistance.distance > 50){
            hood.setPosition(0.7);
        }
        if (limeDistance.distance <= 70 && limeDistance.distance > 60){
            hood.setPosition(0.65);
        }
        if (limeDistance.distance <= 80 && limeDistance.distance > 70){
            hood.setPosition(0.6);
        }
        if (limeDistance.distance <= 90 && limeDistance.distance > 80){
            hood.setPosition(0.55);
        }
        if (limeDistance.distance <= 100 && limeDistance.distance > 90){
            hood.setPosition(0.5);
        }
        if (limeDistance.distance <= 110 && limeDistance.distance > 100){
            hood.setPosition(0.45);
        }
        if (limeDistance.distance <= 120 && limeDistance.distance > 110){
            hood.setPosition(0.4);
        }
        if (limeDistance.distance <= 130 && limeDistance.distance > 120){
            hood.setPosition(0.35);
        }

    }
}
