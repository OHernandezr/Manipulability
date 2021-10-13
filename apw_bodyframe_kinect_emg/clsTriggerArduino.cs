using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Microsoft.Samples.Kinect.BodyBasics
{
    class clsTriggerArduino
    {
        System.IO.Ports.SerialPort ArduinoPort;
        public clsTriggerArduino()
        {
            ArduinoPort = new System.IO.Ports.SerialPort();
            ArduinoPort.PortName = "COM3";  
            ArduinoPort.BaudRate = 9600;
            ArduinoPort.Open();
        }

        public bool Start()
        {
            try
            {
                ArduinoPort.Write("1");
                return true;
            }catch(Exception e)
            {
                return false;
            }

            
        }

        public void End()
        {
            ArduinoPort.Write("0");
        }

        public void ClosePort()
        {
            ArduinoPort.Close();
        }
    }
}
