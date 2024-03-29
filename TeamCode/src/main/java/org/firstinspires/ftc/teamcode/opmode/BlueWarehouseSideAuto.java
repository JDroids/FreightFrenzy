package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Autonomous
public class BlueWarehouseSideAuto extends WarehouseSideAuto {
    public BlueWarehouseSideAuto() {
        super(Alliance.BLUE,
                new Point(30,140),
                new Point(170,140),
                new Point(300,140));
    }
}
