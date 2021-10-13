using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace Microsoft.Samples.Kinect.BodyBasics
{
    public class clsUtil
    {
        #region Basic Frame Counter

        private static int lastTick;
        private static int lastFrameRate;
        private static int frameRate;

        public static int CalculateFrameRate()
        {
            if (System.Environment.TickCount - lastTick >= 1000)
            {
                lastFrameRate = frameRate;
                frameRate = 0;
                lastTick = System.Environment.TickCount;
            }
            frameRate++;
            return lastFrameRate;
        }
        #endregion

        public static double getAngle(Joint p0, Joint p1, Joint p2)
        {
            //producto escalar
            double p0x2 = p0.Position.X - p1.Position.X;
            double p0y2 = p0.Position.Y - p1.Position.Y;
            double p0z2 = p0.Position.Z - p1.Position.Z;

            double p2x2 = p2.Position.X - p1.Position.X;
            double p2y2 = p2.Position.Y - p1.Position.Y;
            double p2z2 = p2.Position.Z - p1.Position.Z;

            double p = p0x2 * p2x2 + p0y2 * p2y2 + p0z2 * p2z2;
            double modP0 = Math.Sqrt(Math.Pow(p0x2, 2) + Math.Pow(p0y2, 2) + Math.Pow(p0z2, 2));
            double modP2 = Math.Sqrt(Math.Pow(p2x2, 2) + Math.Pow(p2y2, 2) + Math.Pow(p2z2, 2));
            if (modP0 * modP2 == 0)
            {
                return 0;
            }
            else
            {
                return Math.Acos(p / (modP0 * modP2)) * 180 / Math.PI;
            }
        }

        public static double distance(Joint p0, Joint p1)
        {
            double pX = Math.Pow((p1.Position.X - p0.Position.X), 2);
            double pY = Math.Pow((p1.Position.Y - p0.Position.Y), 2);
            double pZ = Math.Pow((p1.Position.Z - p0.Position.Z), 2);
            double d = Math.Sqrt(pX + pY + pZ);
            return d;
        }

    }

}
