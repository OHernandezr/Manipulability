using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace Microsoft.Samples.Kinect.BodyBasics
{
    class clsInformationArm
    {
        public long TimeStamp { get; set; }
        public double AngleElbowRight { get; set; }
        public double AngleWristRight { get; set; }
        public double AngleShoulderRight { get; set; }
        public double AngleElbowLeft { get; set; }
        public double AngleWristLeft { get; set; }
        public double AngleShoulderLeft { get; set; }

        public double LShoulderElbow { get; set; }
        public double LElbowWrist { get; set; }

        public string BodyInfoJson { get; set; }
        public Joint Elbow {
            get;
            set;
        }

        public override string ToString()
        {
            return String.Format("TimeStamp: {0}, AngleElbowLeft: {1}, AngleShoulderLeft: {2}, AngleWristLeft: {3}, AngleElbowRight: {4}, AngleShoulderRight {5}, AngleWristRight {6}, LShoulderElbow {7}, LElbowWrist {8}",
                                this.TimeStamp,this.AngleElbowLeft, this.AngleShoulderLeft, this.AngleWristLeft, this.AngleElbowRight, this.AngleShoulderRight, this.AngleWristRight, this.LShoulderElbow,this.LElbowWrist);
        }
    }
}
