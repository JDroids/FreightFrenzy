package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Point;

@Autonomous
public class BlueCyclingWarehouseSideAuto extends CyclingWarehouseSideAuto {
    public BlueCyclingWarehouseSideAuto() {
        super(Alliance.BLUE,
                new Point(30,140),
                new Point(170,140),
                new Point(300,140));
    }
}
