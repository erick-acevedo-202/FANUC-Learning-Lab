using RobotDynamics.MathUtilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobotDynamics.Robots
{
    public class FanucLRMate : Robot
    { 
        public FanucLRMate()
        {
             
            Robot Ro = new Robot()
                .AddJoint('y', new Vector(0, 0, 0))
                .AddJoint('z', new Vector(0.075, 0, 0))
                .AddJoint('z', new Vector(0.075, 0.300, 0))
                .AddJoint('x', new Vector(0.075, 0.375, 0))
                .AddJoint('z', new Vector(0.395, 0.375, 0))
                .AddJoint('x', new Vector(0.475, 0.375, 0));
            /*
                        Robot Ro = new Robot()
                    .AddJoint('z', new Vector(0.075, 0, 0)) // Joint 1: rotación z, offset a2 = 0.075 m
                    .AddJoint('z', new Vector(0, 0.300, 0)) // Joint 2: rotación z, offset a2 = 0.300 m
                    .AddJoint('z', new Vector(0, 0.075, 0)) // Joint 3: rotación z, offset a3 = 0.075 m
                    .AddJoint('x', new Vector(0, 0, 0.320)) // Joint 4: rotación x, offset d4 = 0.320 m
                    .AddJoint('z', new Vector(0, 0, 0.080)) // Joint 5: rotación z, offset d5 = 0.080 m
                    .AddJoint('x', new Vector(0, 0, 0));*/

            Links = Ro.Links;
        }
    }
}
