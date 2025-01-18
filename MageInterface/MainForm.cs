
using System.Xml;
using MageInterface.Kinematics;
using System;
using System.IO;
using System.Diagnostics;
using System.Windows.Forms;
using System.Runtime.InteropServices;
using System.Management;
// install System.Management by Nuget 

namespace MageInterface
{
    public partial class MainForm : Form
  {
    // add function
    [DllImport("kernel32.dll")]
    static extern IntPtr GetConsoleWindow();
    [DllImport("user32.dll")]
    [return: MarshalAs(UnmanagedType.Bool)]
    static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);
    const int SW_HIDE = 0;
    const int SW_SHOW = 5;

    private IniFile ini;
    // machine
    private Kinematics.Kinematics kin = new KinTrunnionBC();
    // parameter
    private double nozzle = 200.0;
    private double bed = 60.0;
    private double wallFirst = 5.0;
    private double wallSecond = 5.0;
    private double infillFirst = 5.0;
    private double infillSecond = 5.0;
    private double supportFirst = 5.0;
    private double supportSecond = 5.0;
    private double moveFirst = 5.0;
    private double moveSecond = 5.0;
    private double fanFirst = 0.0;
    private double fanSecond = 100.0;
    private double retLength = 2.0;
    private double retPull = 10.0;
    private double retPush = 2.0;
    private double zHop = 5.0;
    private bool isCollision = true;
    private bool isThread = false;

    public MainForm()
    {
      // ini file
      string filepath = string.Format("{0}\\config.ini", Directory.GetCurrentDirectory());
      this.ini = new IniFile(filepath);      

      // init component
      InitializeComponent();

      this.textInputFile.Text = "C:\\Develop\\Slicer\\slicer_models\\output_viewer.csv";
      this.textOutputFile.Text = "C:\\Develop\\Slicer\\slicer_models\\output_viewer.gcode";

      this.numericNozzle.Maximum = (decimal)270.0;
      this.numericNozzle.Minimum = (decimal)170.0;
      this.numericNozzle.Increment = (decimal)5.0;
      this.numericNozzle.DecimalPlaces = 0;
      this.numericBed.Maximum = (decimal)120.0;
      this.numericBed.Minimum = (decimal)30.0;
      this.numericBed.Increment = (decimal)5.0;
      this.numericBed.DecimalPlaces = 0;

      this.numericWallFirst.Maximum = (decimal)25.0;
      this.numericWallFirst.Minimum = (decimal)0.5;
      this.numericWallFirst.Increment = (decimal)0.5;
      this.numericWallFirst.DecimalPlaces = 1;
      this.numericWallSecond.Maximum = (decimal)25.0;
      this.numericWallSecond.Minimum = (decimal)0.5;
      this.numericWallSecond.Increment = (decimal)0.5;
      this.numericWallSecond.DecimalPlaces = 1;

      this.numericInfillFirst.Maximum = (decimal)25.0;
      this.numericInfillFirst.Minimum = (decimal)0.5;
      this.numericInfillFirst.Increment = (decimal)0.5;
      this.numericInfillFirst.DecimalPlaces = 1;
      this.numericInfillSecond.Maximum = (decimal)25.0;
      this.numericInfillSecond.Minimum = (decimal)0.5;
      this.numericInfillSecond.Increment = (decimal)0.5;
      this.numericInfillSecond.DecimalPlaces = 1;

      this.numericSupportFirst.Maximum = (decimal)25.0;
      this.numericSupportFirst.Minimum = (decimal)0.5;
      this.numericSupportFirst.Increment = (decimal)0.5;
      this.numericSupportFirst.DecimalPlaces = 1;
      this.numericSupportSecond.Maximum = (decimal)25.0;
      this.numericSupportSecond.Minimum = (decimal)0.5;
      this.numericSupportSecond.Increment = (decimal)0.5;
      this.numericSupportSecond.DecimalPlaces = 1;

      this.numericMoveFirst.Maximum = (decimal)50.0;
      this.numericMoveFirst.Minimum = (decimal)0.5;
      this.numericMoveFirst.Increment = (decimal)0.5;
      this.numericMoveFirst.DecimalPlaces = 1;
      this.numericMoveSecond.Maximum = (decimal)50.0;
      this.numericMoveSecond.Minimum = (decimal)0.5;
      this.numericMoveSecond.Increment = (decimal)0.5;
      this.numericMoveSecond.DecimalPlaces = 1;

      this.numericFanFirst.Maximum = (decimal)100.0;
      this.numericFanFirst.Minimum = (decimal)0.0;
      this.numericFanFirst.Increment = (decimal)10.0;
      this.numericFanFirst.DecimalPlaces = 0;
      this.numericFanSecond.Maximum = (decimal)100.0;
      this.numericFanSecond.Minimum = (decimal)0.0;
      this.numericFanSecond.Increment = (decimal)10.0;
      this.numericFanSecond.DecimalPlaces = 0;

      this.numericRetractLength.Maximum = (decimal)10.0;
      this.numericRetractLength.Minimum = (decimal)0.0;
      this.numericRetractLength.Increment = (decimal)0.5;
      this.numericRetractLength.DecimalPlaces = 1;
      this.numericPullSpeed.Maximum = (decimal)50.0;
      this.numericPullSpeed.Minimum = (decimal)0.5;
      this.numericPullSpeed.Increment = (decimal)0.5;
      this.numericPullSpeed.DecimalPlaces = 1;
      this.numericPushSpeed.Maximum = (decimal)50.0;
      this.numericPushSpeed.Minimum = (decimal)0.5;
      this.numericPushSpeed.Increment = (decimal)0.5;
      this.numericPushSpeed.DecimalPlaces = 1;

      this.numericZhop.Maximum = (decimal)10.0;
      this.numericZhop.Minimum = (decimal)0.0;
      this.numericZhop.Increment = (decimal)0.5;
      this.numericZhop.DecimalPlaces = 1;

      this.valueInsert();
    }

    private void checkBoxCollision_CheckedChanged(object sender, EventArgs e)
    {
      if (this.checkBoxCollision.Checked)
      {
        this.isCollision = true;
        this.checkBoxThread.Enabled = true;
      }
      else
      {
        this.isCollision = false;
        this.isThread = false;
        this.checkBoxThread.Checked = false;
        this.checkBoxThread.Enabled = false;
      }
    }

    private void checkBoxThread_CheckedChanged(object sender, EventArgs e)
    {
      if (this.checkBoxThread.Checked)
      {
        this.isThread = true;
      }
      else
      {
        this.isThread = false;
      }
    }

    private void valueCheck()
    {
      this.nozzle = (double)this.numericNozzle.Value;
      this.bed = (double)this.numericBed.Value;
      this.wallFirst = (double)this.numericWallFirst.Value;
      this.wallSecond = (double)this.numericWallSecond.Value;
      this.infillFirst = (double)this.numericInfillFirst.Value;
      this.infillSecond = (double)this.numericInfillSecond.Value;
      this.supportFirst = (double)this.numericSupportFirst.Value;
      this.supportSecond = (double)this.numericSupportSecond.Value;
      this.moveFirst = (double)this.numericMoveFirst.Value;
      this.moveSecond = (double)this.numericMoveSecond.Value;
      this.fanFirst = (double)this.numericFanFirst.Value;
      this.fanSecond = (double)this.numericFanSecond.Value;
      this.retLength = (double)this.numericRetractLength.Value;
      this.retPull = (double)this.numericPullSpeed.Value;
      this.retPush = (double)this.numericPushSpeed.Value;
      this.zHop = (double)this.numericZhop.Value;
    }

    private void valueInsert()
    {
      this.numericNozzle.Value = (decimal)this.nozzle;
      this.numericBed.Value = (decimal)this.bed;
      this.numericWallFirst.Value = (decimal)this.wallFirst;
      this.numericWallSecond.Value = (decimal)this.wallSecond;
      this.numericInfillFirst.Value = (decimal)this.infillFirst;
      this.numericInfillSecond.Value = (decimal)this.infillSecond;
      this.numericSupportFirst.Value = (decimal)this.supportFirst;
      this.numericSupportSecond.Value = (decimal)this.supportSecond;
      this.numericMoveFirst.Value = (decimal)this.moveFirst;
      this.numericMoveSecond.Value = (decimal)this.moveSecond;
      this.numericFanFirst.Value = (decimal)this.fanFirst;
      this.numericFanSecond.Value = (decimal)this.fanSecond;
      this.numericRetractLength.Value = (decimal)this.retLength;
      this.numericPullSpeed.Value = (decimal)this.retPull;
      this.numericPushSpeed.Value = (decimal)this.retPush;
      this.numericZhop.Value = (decimal)this.zHop;
      this.checkBoxCollision.Checked = this.isCollision;
      this.checkBoxThread.Checked = this.isThread;
    }

    private void buttonSlice_Click(object sender, EventArgs e)
    {
      valueCheck();

      if(this.kin.MachineFile == null)
      {
        MessageBox.Show("Settings are missing.");
      }
      else
      {
        GcodeChanger chg = new GcodeChanger(this.kin, this.textInputFile.Text, this.textOutputFile.Text,
          this.nozzle, this.bed,
          this.wallFirst, this.wallSecond, this.infillFirst, this.infillSecond,
          this.supportFirst, this.supportSecond, this.moveFirst, this.moveSecond,
          this.fanFirst, this.fanSecond, this.retLength, this.retPull, this.retPush,
          this.zHop);
        chg.ChangeGcode();
        MessageBox.Show("Completion of G-code conversion.");
      }
    }
    static void WaitForChildProcesses(int parentId)
    {
        while (true)
        {
            bool hasChildren = false;
            using (var searcher = new ManagementObjectSearcher($"SELECT * FROM Win32_Process WHERE ParentProcessId={parentId}"))
            {
                foreach (var obj in searcher.Get())
                {
                    hasChildren = true;
                    int childId = Convert.ToInt32(obj["ProcessId"]);
                    Console.WriteLine($"子プロセスID: {childId}");

                    // 子プロセスの終了を待機
                    WaitForChildProcesses(childId);
                }
            }

            if (!hasChildren)
            {
                break;
            }
            System.Threading.Thread.Sleep(500); // 少し待機して再チェック
        }
    }

    private void buttonSimulator_Click(object sender, EventArgs e)
    {
      valueCheck();

      if(this.textOutputFile.Text == "" || this.kin.MachineFile == null)
      {
        MessageBox.Show("Settings are missing.");
      }
      else
      {
        ProcessStartInfo processStartInfo = new ProcessStartInfo();
        processStartInfo.FileName = this.ini.Read("Settings", "SimulatorPath");
        MessageBox.Show(processStartInfo.FileName);
        processStartInfo.Arguments = string.Format("{0} {1} {2} {3}", this.textOutputFile.Text, this.kin.MachineFile, this.checkBoxCollision.Checked, this.checkBoxThread.Checked);
        MessageBox.Show(processStartInfo.Arguments);
        processStartInfo.UseShellExecute = false;
        processStartInfo.CreateNoWindow = false;
        processStartInfo.RedirectStandardOutput = true;
        processStartInfo.RedirectStandardError = true;
        Process process = new Process();
        process.StartInfo = processStartInfo;
        process.Start();
        process.BeginOutputReadLine();
        process.BeginErrorReadLine();
        process.WaitForExit();
        WaitForChildProcesses(process.Id);
        MessageBox.Show("Exit.");
      }

    }

    private void buttonSettingOpen_Click(object sender, EventArgs e)
    {

      OpenFileDialog ofd = new OpenFileDialog();
      ofd.InitialDirectory = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
      ofd.FileName = "slicer_setting.xml";
      ofd.Filter = "XML file(*.xml)|*.xml";
      ofd.Title = "Slicer setting xml file";
      ofd.RestoreDirectory = true;
      ofd.CheckFileExists = true;
      ofd.CheckPathExists = true;
      if (ofd.ShowDialog() == DialogResult.OK)
      {
        string filename = ofd.FileName;
        XmlDocument doc = new XmlDocument();
        doc.Load(filename);
        XmlElement root = doc.DocumentElement;
        // slicer
        XmlElement xmlSlicer = root["Slicer"];
        XmlElement xmlNozzle = xmlSlicer["Nozzle"];
        this.nozzle = double.Parse(xmlNozzle.InnerText);
        XmlElement xmlBed = xmlSlicer["Bed"];
        this.bed = double.Parse(xmlBed.InnerText);
        XmlElement xmlWallFirst = xmlSlicer["WallFirst"];
        this.wallFirst = double.Parse(xmlWallFirst.InnerText);
        XmlElement xmlWallSecond = xmlSlicer["WallSecond"];
        this.wallSecond = double.Parse(xmlWallSecond.InnerText);
        XmlElement xmlInfillFirst = xmlSlicer["InfillFirst"];
        this.infillFirst = double.Parse(xmlInfillFirst.InnerText);
        XmlElement xmlInfillSecond = xmlSlicer["InfillSecond"];
        this.infillSecond = double.Parse(xmlInfillSecond.InnerText);
        XmlElement xmlSupportFirst = xmlSlicer["SupportFirst"];
        this.supportFirst = double.Parse(xmlSupportFirst.InnerText);
        XmlElement xmlSupportSecond = xmlSlicer["SupportSecond"];
        this.supportSecond = double.Parse(xmlSupportSecond.InnerText);
        XmlElement xmlMoveFirst = xmlSlicer["MoveFirst"];
        this.moveFirst = double.Parse(xmlMoveFirst.InnerText);
        XmlElement xmlMoveSecond = xmlSlicer["MoveSecond"];
        this.moveSecond = double.Parse(xmlMoveSecond.InnerText);
        XmlElement xmlFanFirst = xmlSlicer["FanFirst"];
        this.fanFirst = double.Parse(xmlFanFirst.InnerText);
        XmlElement xmlFanSecond = xmlSlicer["FanSecond"];
        this.fanSecond = double.Parse(xmlFanSecond.InnerText);
        XmlElement xmlRetLength = xmlSlicer["RetractLength"];
        this.retLength = double.Parse(xmlRetLength.InnerText);
        XmlElement xmlPullSpeed = xmlSlicer["RetractPullSpeed"];
        this.retPull = double.Parse(xmlPullSpeed.InnerText);
        XmlElement xmlPushSpeed = xmlSlicer["RetractPushSpeed"];
        this.retPush = double.Parse(xmlPushSpeed.InnerText);
        XmlElement xmlZHop = xmlSlicer["ZHop"];
        this.zHop = double.Parse(xmlZHop.InnerText);
        // simulator
        XmlElement xmlSimulator = root["Simulator"];
        XmlElement xmlCollision = xmlSimulator["EnableCollision"];
        this.isCollision = bool.Parse(xmlCollision.InnerText);
        XmlElement xmlThread = xmlSimulator["EnableMultiThread"];
        this.isThread = bool.Parse(xmlThread.InnerText);

        this.valueInsert();
      }
    }

    private void buttonSettingSave_Click(object sender, EventArgs e)
    {
      valueCheck();

      OpenFileDialog ofd = new OpenFileDialog();
      ofd.InitialDirectory = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
      ofd.FileName = "slicer_setting.xml";
      ofd.Filter = "XML file(*.xml)|*.xml";
      ofd.Title = "Slicer setting xml file";
      ofd.RestoreDirectory = true;
      ofd.CheckFileExists = false;
      ofd.CheckPathExists = true;
      if (ofd.ShowDialog() == DialogResult.OK)
      {
        string filename = ofd.FileName;
        XmlDocument doc = new XmlDocument();
        XmlDeclaration xmlDeclaration = doc.CreateXmlDeclaration("1.0", "UTF-8", null);
        doc.AppendChild(xmlDeclaration);
        // structure
        XmlElement root = doc.CreateElement("Setting");
        doc.AppendChild(root);
        XmlElement xmlKinematics = doc.CreateElement("Kinematics");
        root.AppendChild(xmlKinematics);
        XmlElement xmlSlicer = doc.CreateElement("Slicer");
        root.AppendChild(xmlSlicer);
        XmlElement xmlSimulator = doc.CreateElement("Simulator");
        root.AppendChild(xmlSimulator);
        // slicer
        XmlElement xmlNozzle = doc.CreateElement("Nozzle");
        xmlNozzle.InnerText = this.nozzle.ToString();
        xmlSlicer.AppendChild(xmlNozzle);
        XmlElement xmlBed = doc.CreateElement("Bed");
        xmlBed.InnerText = this.bed.ToString();
        xmlSlicer.AppendChild(xmlBed);
        XmlElement xmlWallFirst = doc.CreateElement("WallFirst");
        xmlWallFirst.InnerText = this.wallFirst.ToString();
        xmlSlicer.AppendChild(xmlWallFirst);
        XmlElement xmlWallSecond = doc.CreateElement("WallSecond");
        xmlWallSecond.InnerText = this.wallSecond.ToString();
        xmlSlicer.AppendChild(xmlWallSecond);
        XmlElement xmlInfillFirst = doc.CreateElement("InfillFirst");
        xmlInfillFirst.InnerText = this.infillFirst.ToString();
        xmlSlicer.AppendChild(xmlInfillFirst);
        XmlElement xmlInfillSecond = doc.CreateElement("InfillSecond");
        xmlInfillSecond.InnerText = this.infillSecond.ToString();
        xmlSlicer.AppendChild(xmlInfillSecond);
        XmlElement xmlSupportFirst = doc.CreateElement("SupportFirst");
        xmlSupportFirst.InnerText = this.supportFirst.ToString();
        xmlSlicer.AppendChild(xmlSupportFirst);
        XmlElement xmlSupportSecond = doc.CreateElement("SupportSecond");
        xmlSupportSecond.InnerText = this.supportSecond.ToString();
        xmlSlicer.AppendChild(xmlSupportSecond);
        XmlElement xmlMoveFirst = doc.CreateElement("MoveFirst");
        xmlMoveFirst.InnerText = this.moveFirst.ToString();
        xmlSlicer.AppendChild(xmlMoveFirst);
        XmlElement xmlMoveSecond = doc.CreateElement("MoveSecond");
        xmlMoveSecond.InnerText = this.moveSecond.ToString();
        xmlSlicer.AppendChild(xmlMoveSecond);
        XmlElement xmlFanFirst = doc.CreateElement("FanFirst");
        xmlFanFirst.InnerText = this.fanFirst.ToString();
        xmlSlicer.AppendChild(xmlFanFirst);
        XmlElement xmlFanSecond = doc.CreateElement("FanSecond");
        xmlFanSecond.InnerText = this.fanSecond.ToString();
        xmlSlicer.AppendChild(xmlFanSecond);
        XmlElement xmlRetLength = doc.CreateElement("RetractLength");
        xmlRetLength.InnerText = this.retLength.ToString();
        xmlSlicer.AppendChild(xmlRetLength);
        XmlElement xmlPullSpeed = doc.CreateElement("RetractPullSpeed");
        xmlPullSpeed.InnerText = this.retPull.ToString();
        xmlSlicer.AppendChild(xmlPullSpeed);
        XmlElement xmlPushSpeed = doc.CreateElement("RetractPushSpeed");
        xmlPushSpeed.InnerText = this.retPush.ToString();
        xmlSlicer.AppendChild(xmlPushSpeed);
        XmlElement xmlZHop = doc.CreateElement("ZHop");
        xmlZHop.InnerText = this.zHop.ToString();
        xmlSlicer.AppendChild(xmlZHop);
        // simulator
        XmlElement xmlCollision = doc.CreateElement("EnableCollision");
        xmlCollision.InnerText = this.checkBoxCollision.Checked.ToString();
        xmlSimulator.AppendChild(xmlCollision);
        XmlElement xmlThread = doc.CreateElement("EnableMultiThread");
        xmlThread.InnerText = this.checkBoxThread.Checked.ToString();
        xmlSimulator.AppendChild(xmlThread);
        // save
        doc.Save(filename);
      }
    }

    private void buttonInputOpen_Click(object sender, EventArgs e)
    {
      OpenFileDialog ofd = new OpenFileDialog();
      ofd.InitialDirectory = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
      ofd.FileName = "output.csv";
      ofd.Filter = "CSV file(*.csv)|*.csv";
      ofd.Title = "Input file";
      ofd.RestoreDirectory = true;
      ofd.CheckFileExists = true;
      ofd.CheckPathExists = true;
      if (ofd.ShowDialog() == DialogResult.OK)
      {
        this.textInputFile.Text = ofd.FileName;
        this.textInputFile.Enabled = false;
      }
    }

    private void buttonOutputOpen_Click(object sender, EventArgs e)
    {
      OpenFileDialog ofd = new OpenFileDialog();
      ofd.InitialDirectory = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
      ofd.FileName = "output_viewer.gcode";
      ofd.Filter = "G-code file(*.gcode)|*.gcode";
      ofd.Title = "Output file";
      ofd.RestoreDirectory = true;
      ofd.CheckFileExists = false;
      ofd.CheckPathExists = true;
      if (ofd.ShowDialog() == DialogResult.OK)
      {
        this.textOutputFile.Text = ofd.FileName;
        this.textOutputFile.Enabled = false;
      }
    }

    private void buttonMachine_Click(object sender, EventArgs e)
    {
      OpenFileDialog ofd = new OpenFileDialog();
      ofd.InitialDirectory = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
      ofd.FileName = "machine.xml";
      ofd.Filter = "XML file(*.xml)|*.xml";
      ofd.Title = "Machine setting file";
      ofd.RestoreDirectory = true;
      ofd.CheckFileExists = true;
      ofd.CheckPathExists = true;
      if (ofd.ShowDialog() == DialogResult.OK)
      {
        // xml
        XmlDocument doc = new XmlDocument();
        doc.Load(ofd.FileName);
        XmlElement root = doc.DocumentElement;
        XmlElement xmlKinematics = root["Kinematics"];
        this.labelKinematics.Text = xmlKinematics.InnerText;
        switch(xmlKinematics.InnerText)
        {
          case "CoreXY-BC":
            kin = new KinTrunnionBC();
            kin.MachineFile = ofd.FileName;
            break;
          case "CoreXY":
            kin = new KinCartesian();
            kin.MachineFile = ofd.FileName;
            break;
          case "BedSlingerY":
            kin = new KinCartesian();
            kin.MachineFile = ofd.FileName;
            break;
          case "Delta":
            kin = new KinCartesian();
            kin.MachineFile = ofd.FileName;
            break;
          default:
            MessageBox.Show("An unsupported Kinematics Machine file was loaded. Exit the application.");
            this.Close();
            break;
        }
        XmlElement xmlChanger = root["Changer"];
        this.kin.FilamentDiameter = double.Parse(xmlChanger["FilamentDiameter"].InnerText);
        this.kin.NozzleDiameter = double.Parse(xmlChanger["NozzleDiameter"].InnerText);
        this.kin.OriginX = double.Parse(xmlChanger["OriginX"].InnerText);
        this.kin.OriginY = double.Parse(xmlChanger["OriginY"].InnerText);
        this.kin.OriginZ = double.Parse(xmlChanger["OriginZ"].InnerText);
        this.kin.HoppingDistance = double.Parse(xmlChanger["HoppingDistance"].InnerText);
        if(xmlChanger["HoppingDegreeA"] == null)
        {
          this.kin.HoppingDegreeA = 2.0;
        }
        else
        {
          this.kin.HoppingDegreeA = double.Parse(xmlChanger["HoppingDegreeA"].InnerText);
        }
        if(xmlChanger["HoppingDegreeB"] == null)
        {
          this.kin.HoppingDegreeB = 2.0;
        }
        else
        {
          this.kin.HoppingDegreeB = double.Parse(xmlChanger["HoppingDegreeB"].InnerText);
        }
        if(xmlChanger["HoppingDegreeC"] == null)
        {
          this.kin.HoppingDegreeC = 2.0;
        }
        else
        {
          this.kin.HoppingDegreeC = double.Parse(xmlChanger["HoppingDegreeC"].InnerText);
        }
        if(xmlChanger["Type"] == null)
        {
          this.kin.Type = "Normal";
        }
        else
        {
          this.kin.Type = xmlChanger["Type"].InnerText;
        }
      }
    }
  }
}
