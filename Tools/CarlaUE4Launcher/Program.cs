using System;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Windows.Forms;

internal static class Program
{
    [STAThread]
    private static int Main(string[] args)
    {
        string root = ResolveCarlaRoot();
        string target = root == null
            ? null
            : Path.Combine(root, "CarlaUE4", "Binaries", "Win64", "CarlaUE4-Win64-Shipping.exe");

        if (root == null || !File.Exists(target))
        {
            MessageBox.Show(
                "CARLA executable was not found.\r\n\r\n" +
                "Move this launcher into the CARLA package, or start it from a shortcut whose working directory is the package root.",
                "CARLA Launcher",
                MessageBoxButtons.OK,
                MessageBoxIcon.Error);
            return 1;
        }

        try
        {
            ProcessStartInfo startInfo = new ProcessStartInfo(target)
            {
                Arguments = QuoteArguments(args),
                WorkingDirectory = root,
                UseShellExecute = false
            };

            using (Process process = Process.Start(startInfo))
            {
                process.WaitForExit();
                return process.ExitCode;
            }
        }
        catch (Exception ex)
        {
            MessageBox.Show(
                "Failed to start CARLA:\r\n" + ex.Message,
                "CARLA Launcher",
                MessageBoxButtons.OK,
                MessageBoxIcon.Error);
            return 1;
        }
    }

    private static string ResolveCarlaRoot()
    {
        string[] candidates = new[]
        {
            Environment.CurrentDirectory,
            AppDomain.CurrentDomain.BaseDirectory,
        };

        foreach (string candidate in candidates)
        {
            string root = FindCarlaRoot(candidate);
            if (root != null)
            {
                return root;
            }
        }

        return null;
    }

    private static string FindCarlaRoot(string startPath)
    {
        if (string.IsNullOrWhiteSpace(startPath))
        {
            return null;
        }

        DirectoryInfo directory = new DirectoryInfo(Path.GetFullPath(startPath));
        while (directory != null)
        {
            string target = Path.Combine(
                directory.FullName,
                "CarlaUE4",
                "Binaries",
                "Win64",
                "CarlaUE4-Win64-Shipping.exe");

            if (File.Exists(target))
            {
                return directory.FullName;
            }

            directory = directory.Parent;
        }

        return null;
    }

    private static string QuoteArguments(string[] args)
    {
        if (args == null || args.Length == 0)
        {
            return string.Empty;
        }

        string[] quoted = new string[args.Length];
        for (int i = 0; i < args.Length; i++)
        {
            quoted[i] = QuoteArgument(args[i]);
        }

        return string.Join(" ", quoted);
    }

    private static string QuoteArgument(string arg)
    {
        if (string.IsNullOrEmpty(arg))
        {
            return "\"\"";
        }

        bool needsQuotes = arg.IndexOfAny(new[] { ' ', '\t', '\n', '\r', '"' }) >= 0;
        if (!needsQuotes)
        {
            return arg;
        }

        StringBuilder builder = new StringBuilder();
        builder.Append('"');

        int backslashes = 0;
        foreach (char current in arg)
        {
            if (current == '\\')
            {
                backslashes++;
                continue;
            }

            if (current == '"')
            {
                builder.Append('\\', backslashes * 2 + 1);
                builder.Append('"');
                backslashes = 0;
                continue;
            }

            builder.Append('\\', backslashes);
            builder.Append(current);
            backslashes = 0;
        }

        builder.Append('\\', backslashes * 2);
        builder.Append('"');
        return builder.ToString();
    }
}
