using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace MageInterface
{
  internal class IniFile
  {
    private string path;
    public IniFile(string iniPath)
    {
        path = iniPath;
    }

    [DllImport("kernel32", CharSet = CharSet.Auto)]
    private static extern int GetPrivateProfileString(string section, string key, string def, StringBuilder retVal, int size, string filePath);

    [DllImport("kernel32", CharSet = CharSet.Auto)]
    private static extern long WritePrivateProfileString(string section, string key, string val, string filePath);

    public string Read(string section, string key)
    {
        StringBuilder result = new StringBuilder(255);
        GetPrivateProfileString(section, key, "", result, 255, path);
        return result.ToString();
    }

    public void Write(string section, string key, string value)
    {
        WritePrivateProfileString(section, key, value, path);
    }
  }
}
