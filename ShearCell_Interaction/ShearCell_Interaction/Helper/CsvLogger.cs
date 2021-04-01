using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.ExceptionServices;

namespace ShearCell_Interaction.Helper
{
    public class CsvLogger
    {
        private readonly string _filename;
        private readonly string _delimiter;

        private readonly Dictionary<object, string> _prefixes;

        public CsvLogger(string filename, string delimiter = ";")
        {
            _filename = filename;
            _delimiter = delimiter;
            _prefixes = new Dictionary<object, string>();
        }

        public void WriteHeader(params string[] headers)
        {
            using (var file = new StreamWriter(_filename, true))
            {
                for (int i = 0; i < headers.Length; i++)
                {
                    if (i < headers.Length - 1)
                        file.Write(headers[i] + _delimiter);
                    else
                        file.Write(headers[i]);
                }
                file.Write(Environment.NewLine);
            }
        }

        public void Log(params string[] values)
        {
            var logString = DateTime.Now.ToString("O") + _delimiter;

            var enumerator = _prefixes.GetEnumerator();

            while (enumerator.MoveNext())
            {
                logString += enumerator.Current.Value + _delimiter;
            }

            enumerator.Dispose();

            foreach (var value in values)
            {
                logString += value + _delimiter;
            }

            using (var file = new StreamWriter(_filename, true))
            {
                file.WriteLine(logString);
            }
        }

        public void AddOrChangePrefix(string value, object sender)
        {
            if (_prefixes.ContainsKey(sender))
                _prefixes[sender] = value;
            else
                _prefixes.Add(sender, value);
        }
    }
}