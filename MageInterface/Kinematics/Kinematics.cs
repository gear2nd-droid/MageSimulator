﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MageInterface.Kinematics
{
  abstract class Kinematics
  {
    public string MachineFile;
    public double FilamentDiameter;
    public double NozzleDiameter;
    public double OriginX;
    public double OriginY;
    public double OriginZ;
    public double HoppingDistance;
    public double HoppingDegreeA;
    public double HoppingDegreeB;
    public double HoppingDegreeC;
    public string Type;
    abstract public void InverseKinematics(double[] tar_pos, double[] tar_ijk,
      double[] pre_gcd, out double[] next);
    abstract public void InverseKinematicsWithABC(double[] tar_pos, double tar_a, double tar_b, double tar_c,
      double[] pre_gcd, out double[] next);
    abstract public string[] OutputGcodeNormal(double[] next, double pe, double edelta, double printFval, out double ne);
    abstract public string[] OutputGcodeHopping(double[] next, double pe,
      double retLength, double retPull, double retPush, double hopZ, double moveFval, double[] prev);
  }
}
