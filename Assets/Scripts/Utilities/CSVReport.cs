/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CSVReport
{
    private static string reportDirectory = "Reports";
    private static string reportSeparator = ",";

    private string reportName;
    private string[] reportHeaders;

    public CSVReport(string reportName, string[] reportHeaders)
    {
        this.reportName = reportName;
        this.reportHeaders = reportHeaders;
    }

    public void Append(string[] strings)
    {
        VerifyDirectory();
        VerifyFile();
        using (StreamWriter sw = File.AppendText(GetFilePath()))
        {
            string finalString = "";
            for (int i = 0; i < strings.Length; i++)
            {
                if (finalString != "")
                    finalString += reportSeparator;

                finalString += strings[i];
            }
            sw.WriteLine(finalString);
        }
    }

    public void Create()
    {
        VerifyDirectory();
        using (StreamWriter sw = File.CreateText(GetFilePath()))
        {
            string finalString = "";
            for (int i = 0; i < reportHeaders.Length; i++)
            {
                if (finalString != "")
                    finalString += reportSeparator;

                finalString += reportHeaders[i];
            }
            sw.WriteLine(finalString);
        }
    }


    private static void VerifyDirectory()
    {
        string dir = GetDirectoryPath();
        if (!Directory.Exists(dir))
            Directory.CreateDirectory(dir);
    }

    private void VerifyFile()
    {
        string file = GetFilePath();
        if (!File.Exists(file))
            Create();
    }

    private static string GetDirectoryPath()
    {
        return Application.dataPath + "/" + reportDirectory;
    }

    private string GetFilePath()
    {
        return GetDirectoryPath() + "/" + reportName;
    }

    private static string GetTimeStamp()
    {
        return Time.fixedTime.ToString();
    }
}
