using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static System.Runtime.InteropServices.JavaScript.JSType;
using System.Collections;
using System.Text.RegularExpressions;
using System.Security.Policy;
using static System.Net.Mime.MediaTypeNames;

namespace MageInterface
{
    class GcodeChanger
  {
    private Kinematics.Kinematics kin;
    private double nozzle;
    private double bed;
    private double wallFirst;
    private double wallSecond;
    private double infillFirst;
    private double infillSecond;
    private double supportFirst;
    private double supportSecond;
    private double moveFirst;
    private double moveSecond;
    private double fanFirst;
    private double fanSecond;
    private double retLength;
    private double retPull;
    private double retPush;
    private double zHop;
    private string inputFile;
    private string outputFile;
    private bool debugLine = true;
    public GcodeChanger(Kinematics.Kinematics kin, string inputFile, string outputFile, 
      double nozzle, double bed,
      double wallFirst, double wallSecond, double infillFirst, double infillSecond,
      double supportFirst, double supportSecond, double moveFirst, double moveSecond,
      double fanFirst, double fanSecond, double retLength, double retPull, double retPush,
      double zHop)
    {
      this.kin = kin;
      this.inputFile = inputFile;
      this.outputFile = outputFile;
      this.nozzle = nozzle;
      this.bed = bed;
      this.wallFirst = wallFirst;
      this.wallSecond = wallSecond;
      this.infillFirst = infillFirst;
      this.infillSecond = infillSecond;
      this.supportFirst = supportFirst;
      this.supportSecond = supportSecond;
      this.moveFirst = moveFirst;
      this.moveSecond = moveSecond;
      this.fanFirst = fanFirst;
      this.fanSecond = fanSecond;
      this.retLength = retLength;
      this.retPull = retPull;
      this.fanFirst = fanFirst;
      this.fanSecond = fanSecond;
      this.retPush = retPush;
      this.zHop = zHop;
    }

    public void ChangeGcode()
    {
      FileStream infile = new FileStream(this.inputFile, FileMode.Open);
      StreamReader reader = new StreamReader(infile, Encoding.GetEncoding("UTF-8"));
      FileStream outfile = new FileStream(this.outputFile, FileMode.Create);
      UTF8Encoding encoding = new UTF8Encoding(false);
      //StreamWriter writer = new StreamWriter(outfile, Encoding.GetEncoding("UTF-8"));
      StreamWriter writer = new StreamWriter(outfile, encoding);

      // init values
      double pd = 0.0;
      double pw = 0.0;
      double[] rp_xyz = {0.0, 0.0, 0.0 };
      double[] rp_ijk = {0.0, 0.0, 1.0 };
      int pre_block = -1;
      int pre_layer = -1;
      int pre_loop = -1;
      int pre_curve = -1;
      int prem = 0;
      double pe = 0.0;
      double[] vp_gcd = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

      string startGcd = string.Format("PRINT_START BED_TEMP={0:F2} EXTRUDER_TEMP={1:F2} ;(MACRO)",
        this.bed, this.nozzle);
      writer.WriteLine(startGcd);
      string fanGcd = string.Format("SET_FAN_SPEED FAN=fan SPEED={0:F3} ;(MACRO)", this.fanFirst / 100.0);
      writer.WriteLine(fanGcd);

      // read
      string line;
      while((line = reader.ReadLine()) != null)
      {
        string[] buf = line.Split(',');
        int rnm = int.Parse(buf[0]);
        double rnx = double.Parse(buf[1]);
        double rny = double.Parse(buf[2]);
        double rnz = double.Parse(buf[3]);
        double rni = double.Parse(buf[4]);
        double rnj = double.Parse(buf[5]);
        double rnk = double.Parse(buf[6]);
        //if(rnk < 0.0)
        //{
        //  rni = -rni;
        //  rnj = -rnj;
        //  rnk = -rnk;
        //}
        double d = double.Parse(buf[7]);
        double w = double.Parse(buf[8]);
        int block = int.Parse(buf[9]);
        int layer = int.Parse(buf[10]);
        int loop = int.Parse(buf[11]);
        int curve = int.Parse(buf[12]);

        // kinematics
        double[] rn_xyz = {rnx, rny, rnz };
        double[] rn_ijk = {rni, rnj, rnk };
        double[] vn_gcd;
        this.kin.InverseKinematics(rn_xyz, rn_ijk, vp_gcd, out vn_gcd);

        // calc distance
        double dist = Math.Sqrt((rn_xyz[0] - rp_xyz[0]) * (rn_xyz[0] - rp_xyz[0])
          + (rn_xyz[1] - rp_xyz[1]) * (rn_xyz[1] - rp_xyz[1])
          + (rn_xyz[2] - rp_xyz[2]) * (rn_xyz[2] - rp_xyz[2]));
        double area1 = (w - d) * d + d * 2 / 4.0 * Math.PI;
        double area2 = (pw - pd) * pd + pd * pd / 4.0 * Math.PI;
        double mass = (area1 + area2) * dist / 2.0;
        double edelta = mass / (kin.FilamentDiameter * kin.FilamentDiameter / 4.0 * Math.PI);
        
        // fan etc
        if(block != pre_block || layer != pre_layer)
        {
          string text = string.Format(";(block:{0:D},layer:{1:D})", block, layer);
          writer.WriteLine(text);
          if(pre_block == 0 && pre_layer == 0)
          {
            string bufGcd = string.Format("SET_FAN_SPEED FAN=fan SPEED={0:F3} ;(MACRO)", this.fanSecond / 100.0);
            writer.WriteLine(bufGcd);
          }
        }

        // output gcode
        string[] gcd = {"" };
        if(
          //Math.Abs(vn_gcd[3] - vp_gcd[3]) > Math.Abs(vp_gcd[3]) ||
          //Math.Abs(vn_gcd[4] - vp_gcd[4]) > Math.Abs(vp_gcd[4]) ||
          //Math.Abs(vn_gcd[5] - vp_gcd[5]) > Math.Abs(vp_gcd[5]) ||
          Math.Abs(vn_gcd[3] - vp_gcd[3]) > this.kin.HoppingDegreeA / 180.0 * Math.PI ||
          Math.Abs(vn_gcd[4] - vp_gcd[4]) > this.kin.HoppingDegreeB / 180.0 * Math.PI ||
          Math.Abs(vn_gcd[5] - vp_gcd[5]) > this.kin.HoppingDegreeC / 180.0 * Math.PI ||
          Math.Abs(rn_xyz[0] - rp_xyz[0]) > this.kin.HoppingDistance ||
          Math.Abs(rn_xyz[1] - rp_xyz[1]) > this.kin.HoppingDistance ||
          Math.Abs(rn_xyz[2] - rp_xyz[2]) > this.kin.HoppingDistance 
          )
        {
          double[] buf_gcd;
          string[] add_gcd = {"" };
          this.kin.InverseKinematicsWithABC(rp_xyz, vn_gcd[3], vn_gcd[4], vn_gcd[5], vp_gcd, out buf_gcd);
          if(block == 0 && layer == 0)
          {
            switch(rnm)
            {
	            case 11://OuterWallMiddle = 11,
                gcd = this.kin.OutputGcodeHopping(buf_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveFirst, vp_gcd);
                add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.wallFirst, out pe);
                break;
	            case 12://OuterWallStart = 12,
                gcd = this.kin.OutputGcodeHopping(vn_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveFirst, vp_gcd);
                //add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, 0.0, this.wallFirst, out pe);
                break;
	            case 13://OuterWallEnd = 13,
                gcd = this.kin.OutputGcodeHopping(buf_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveFirst, vp_gcd);
                add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.wallFirst, out pe);
                break;
	            case 21://InnerWallMiddle = 21,
                gcd = this.kin.OutputGcodeHopping(buf_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveFirst, vp_gcd);
                add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.wallFirst, out pe);
                break;
	            case 22://InnerWallStart = 22,
                gcd = this.kin.OutputGcodeHopping(vn_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveFirst, vp_gcd);
                //add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, 0.0, this.wallFirst, out pe);
                break;
	            case 23://InnerWallEnd = 23,
                gcd = this.kin.OutputGcodeHopping(buf_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveFirst, vp_gcd);
                add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.wallFirst, out pe);
                break;
	            case 31://InfillMiddle = 31,
                gcd = this.kin.OutputGcodeHopping(buf_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveFirst, vp_gcd);
                add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.infillFirst, out pe);
                break;
	            case 32://InfillStart = 32,
                gcd = this.kin.OutputGcodeHopping(vn_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveFirst, vp_gcd);
                //add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, 0.0, this.infillFirst, out pe);
                break;
	            case 33://InfillEnd = 33,
                gcd = this.kin.OutputGcodeHopping(buf_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveFirst, vp_gcd);
                add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.infillFirst, out pe);
                break;
	            case 41://SupportMiddle = 41,
                gcd = this.kin.OutputGcodeHopping(buf_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveFirst, vp_gcd);
                add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.supportFirst, out pe);
                break;
	            case 42://SupportStart = 42,
                gcd = this.kin.OutputGcodeHopping(vn_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveFirst, vp_gcd);
                //add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, 0.0, this.supportFirst, out pe);
                break;
	            case 43://SupportEnd = 43,
                gcd = this.kin.OutputGcodeHopping(buf_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveFirst, vp_gcd);
                add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.supportFirst, out pe);
                break;
	            case 51://Saving = 51,
                add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, 0.0, this.moveFirst, out pe);
                break;
	            case 0://None = 0
                break;
              default:
                break;
            }
          }
          else
          {
            switch(rnm)
            {
	            case 11://OuterWallMiddle = 11,
                gcd = this.kin.OutputGcodeHopping(buf_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveSecond, vp_gcd);
                add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.wallSecond, out pe);
                break;
	            case 12://OuterWallStart = 12,
                gcd = this.kin.OutputGcodeHopping(vn_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveSecond, vp_gcd);
                //add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, 0.0, this.wallSecond, out pe);
                break;
	            case 13://OuterWallEnd = 13,
                gcd = this.kin.OutputGcodeHopping(buf_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveSecond, vp_gcd);
                add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.wallSecond, out pe);
                break;
	            case 21://InnerWallMiddle = 21,
                gcd = this.kin.OutputGcodeHopping(buf_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveSecond, vp_gcd);
                add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.wallSecond, out pe);
                break;
	            case 22://InnerWallStart = 22,
                gcd = this.kin.OutputGcodeHopping(vn_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveSecond, vp_gcd);
                //add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, 0.0, this.wallSecond, out pe);
                break;
	            case 23://InnerWallEnd = 23,
                gcd = this.kin.OutputGcodeHopping(buf_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveSecond, vp_gcd);
                add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.wallSecond, out pe);
                break;
	            case 31://InfillMiddle = 31,
                gcd = this.kin.OutputGcodeHopping(buf_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveSecond, vp_gcd);
                add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.infillSecond, out pe);
                break;
	            case 32://InfillStart = 32,
                gcd = this.kin.OutputGcodeHopping(vn_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveSecond, vp_gcd);
                //add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, 0.0, this.infillSecond, out pe);
                break;
	            case 33://InfillEnd = 33,
                gcd = this.kin.OutputGcodeHopping(buf_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveSecond, vp_gcd);
                add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.infillSecond, out pe);
                break;
	            case 41://SupportMiddle = 41,
                gcd = this.kin.OutputGcodeHopping(buf_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveSecond, vp_gcd);
                add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.supportSecond, out pe);
                break;
	            case 42://SupportStart = 42,
                gcd = this.kin.OutputGcodeHopping(vn_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveSecond, vp_gcd);
                //add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, 0.0, this.supportSecond, out pe);
                break;
	            case 43://SupportEnd = 43,
                gcd = this.kin.OutputGcodeHopping(buf_gcd, pe, 
                  this.retLength, this.retPull, this.retPush, this.zHop, this.moveSecond, vp_gcd);
                add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.supportSecond, out pe);
                break;
	            case 51://Saving = 51,
                add_gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, 0.0, this.moveSecond, out pe);
                break;
	            case 0://None = 0
                break;
              default:
                break;
            }
          }
          List<string> out_gcd = new List<string>();
          for(int g = 0; g < gcd.Length; g++)
          {
            out_gcd.Add(gcd[g]);
          }
          for(int g = 0; g < add_gcd.Length; g++)
          {
            out_gcd.Add(add_gcd[g]);
          }
          gcd = out_gcd.ToArray();
        }
        else
        {
          if(block == 0 && layer == 0)
          {
            switch(rnm)
            {
	            case 11://OuterWallMiddle = 11,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.wallFirst, out pe);
                break;
	            case 12://OuterWallStart = 12,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.wallFirst, out pe);
                break;
	            case 13://OuterWallEnd = 13,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.wallFirst, out pe);
                break;
	            case 21://InnerWallMiddle = 21,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.wallFirst, out pe);
                break;
	            case 22://InnerWallStart = 22,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.wallFirst, out pe);
                break;
	            case 23://InnerWallEnd = 23,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.wallFirst, out pe);
                break;
	            case 31://InfillMiddle = 31,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.infillFirst, out pe);
                break;
	            case 32://InfillStart = 32,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.infillFirst, out pe);
                break;
	            case 33://InfillEnd = 33,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.infillFirst, out pe);
                break;
	            case 41://SupportMiddle = 41,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.supportFirst, out pe);
                break;
	            case 42://SupportStart = 42,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.supportFirst, out pe);
                break;
	            case 43://SupportEnd = 43,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.supportFirst, out pe);
                break;
	            case 51://Saving = 51,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, 0.0, this.moveFirst, out pe);
                break;
	            case 0://None = 0
                break;
              default:
                break;
            }
          }
          else
          {
            switch(rnm)
            {
	            case 11://OuterWallMiddle = 11,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.wallSecond, out pe);
                break;
	            case 12://OuterWallStart = 12,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.wallSecond, out pe);
                break;
	            case 13://OuterWallEnd = 13,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.wallSecond, out pe);
                break;
	            case 21://InnerWallMiddle = 21,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.wallSecond, out pe);
                break;
	            case 22://InnerWallStart = 22,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.wallSecond, out pe);
                break;
	            case 23://InnerWallEnd = 23,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.wallSecond, out pe);
                break;
	            case 31://InfillMiddle = 31,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.infillSecond, out pe);
                break;
	            case 32://InfillStart = 32,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.infillSecond, out pe);
                break;
	            case 33://InfillEnd = 33,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.infillSecond, out pe);
                break;
	            case 41://SupportMiddle = 41,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.supportSecond, out pe);
                break;
	            case 42://SupportStart = 42,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.supportSecond, out pe);
                break;
	            case 43://SupportEnd = 43,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, edelta, this.supportSecond, out pe);
                break;
	            case 51://Saving = 51,
                gcd = this.kin.OutputGcodeNormal(vn_gcd, pe, 0.0, this.moveSecond, out pe);
                break;
	            case 0://None = 0
                break;
              default:
                break;
            }
          }
        }
        // output gcd
        foreach(string text in gcd)
        {
          if(this.debugLine)
          {
            string str = string.Format("{0};{1}", text, line);
            writer.WriteLine(str);
          }
          else
          {
            writer.WriteLine(text);
          }
        }

        // next
        rp_xyz = rn_xyz;
        rp_ijk = rn_ijk;
        vp_gcd = vn_gcd;
        pd = d;
        pw = w;
        prem = rnm;
        pre_block = block;
        pre_layer = layer;
        pre_loop = loop;
        pre_curve = curve;
      }

      string endGcd = string.Format("PRINT_END ;(MACRO)");
      writer.WriteLine(endGcd);

      writer.Close();
      reader.Close();
      infile.Close();
    }
  }
}
