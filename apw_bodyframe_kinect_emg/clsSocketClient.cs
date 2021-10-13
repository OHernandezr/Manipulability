using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace Microsoft.Samples.Kinect.BodyBasics
{

    public class clsSocketClient
    {

        private const int PORT = 8000;
        private const string HOST = "raspberrypi";
        TcpClient socketClient = null;
        NetworkStream stream; //Link stream
        public bool StartClient()
        {
            try
            {
                socketClient = new TcpClient(HOST, PORT);
                Console.WriteLine("TCP Client correct... ");
                return true;
            }
            catch (Exception e)
            {
                Console.WriteLine("ERROR connect TCP Client ... " + e.ToString());
                return false;
            }
        }

        public void CloseSocket()
        {
            stream.Close(); //Closes data stream
            socketClient.Close(); //Closes socket
        }


        public void SendData(String data)
        {
            string message; //Message to send
            int byteCount; //Raw bytes to send

            byte[] sendData; //Raw data to send
            try
            {
                message = data; //Set message variable to input
                byteCount = Encoding.ASCII.GetByteCount(message); //Measures bytes required to send ASCII data
                sendData = new byte[byteCount]; //Prepares variable size for required data
                sendData = Encoding.ASCII.GetBytes(message); //Sets the sendData variable to the actual binary data (from the ASCII)
                stream = socketClient.GetStream(); //Opens up the network stream
                stream.Write(sendData, 0, sendData.Length); //Transmits data onto the stream

            }
            catch (System.NullReferenceException) //Error if socket not open
            {
                Console.WriteLine("ERROR send data... ");
            }
        }

    }



}
