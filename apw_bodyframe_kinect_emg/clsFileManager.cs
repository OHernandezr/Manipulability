using CsvHelper;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Microsoft.Samples.Kinect.BodyBasics
{
    class clsFileManager
    {
        private List<clsInformationArm> cInfoArm;
        private bool _autoGenerateName;
        private string _nameFile;
        private string _log=string.Empty;

        public clsFileManager(bool autoGene, string nameFile) {
            cInfoArm = new List<clsInformationArm>();
            this._autoGenerateName = autoGene;
            this._nameFile = nameFile;
            if (!autoGene && nameFile == string.Empty)
            {
                this._log = "Error: Autogenerate is false and name file is empty";
            }

        }


        public bool addRecord(clsInformationArm clInfo)
        {
            try
            {
                if (this._log != string.Empty)
                {
                    Console.Write(this._log);
                    return false;
                }
                this.cInfoArm.Add(clInfo);
                return true;
            }catch(Exception e)
            {
                return false;
            }
            
        }
        

        public string getNameFile()
        {
            if (!this._autoGenerateName && this._nameFile != string.Empty)
            {
                return this._nameFile;
            }
            else
            {
                DateTime localDate = DateTime.Now;
                String d=String.Format("{0:dd_MMMM_yyyy_HH_mm}", localDate)+".csv";
                return d;
            }
        }


        public bool saveFile()
        {
            try
            {
                Console.WriteLine(CultureInfo.InvariantCulture);
                using (var writer = new StreamWriter("output\\" + this.getNameFile()))
                using (var csv = new CsvWriter(writer, CultureInfo.InvariantCulture))
                {
                    csv.Configuration.Delimiter = ";";
                    csv.WriteRecords(cInfoArm);
                    csv.Flush();
                }
            }catch(Exception e)
            {
                Console.Write("Error " + e.Message);
                return false;

            }
          
            
            return true;
        }

    }
}
