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
        if (limeDistance.distance >= 100) {
            hood.setPosition(0.5);
            }
        if (limeDistance.distance >= 110){
            hood.setPosition(0.7);
        }
    }
}
