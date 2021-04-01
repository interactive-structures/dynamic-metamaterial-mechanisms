using ShearCell_Interaction.Model;
using ShearCell_Interaction.Simulation;
using ShearCell_Interaction.View;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Windows;

namespace ShearCell_Interaction.Randomizer
{
    class LargePatternRandomizer
    {
        public int MinDoF { get; set; }
        public int MaxDoF { get; set; }
        public int MinRigidCells { get; set; }
        public int MaxRigidCells { get; set; }
        public int MaxIterations { get; set; }
        
        private int _numberCells;
        private int _arrangementWidth;
        private int _arrangementHeight;
        private Random _random;

        private int _numberRigidCells;
        private string _targetFolder;

        private readonly ViewModel _viewModel;
        private readonly MetamaterialModel _model;

        public LargePatternRandomizer(int arrangementWidth, int arrangementHeight, ViewModel viewModel, MetamaterialModel model)
        {
            _viewModel = viewModel;
            _model = model;
            //ViewModel.CellScale = 10;

            _random = new Random(Guid.NewGuid().GetHashCode());
        }

        public void GeneratePersistedVariations()
        {
            /*
             * create bitcode, (draw for debug)
             * check dof, set non-moving shear cells rigid, 
             * check if is in list (mirrored, rotated)
             */

            var hasMatchedDoF = false;

            while (!hasMatchedDoF)
            {
                var bitcode = GenerateCode();
                //var bitcode = "1101011111101111010111111111101111110100101111111111111111111111";
                CodeToModel(bitcode);

                Debug.WriteLine("code: " + bitcode);
                Debug.WriteLine("# DoF: {0}", _model.CurrentDoF);

                hasMatchedDoF = _model.CurrentDoF >= MinDoF && _model.CurrentDoF <= MaxDoF;
            }

            ShowCode();
        }

        public void SystematicallyRandomizePatterns(int startSize, int endSize, int increment, bool randomizeRectangular = false)
        {
            const int numberTrials = 500000;
            Debug.WriteLine("\r\n ------ Large patterns ({0} - {1}, incr. {2}) ------ \r\n", startSize, endSize, increment);

            var now = DateTime.Now; 
            var dateTimeString = $"{now.Year}_{now.Month}_{now.Day}_{now.Hour}.{now.Minute}.{now.Second}";

            var parentFolder = Path.GetFullPath(@"..\..\..\");
            var trialFolder = dateTimeString + $"__sizes_{startSize}--{endSize}__(i{increment}_t{numberTrials})";
            var targetFolder = Path.Combine(parentFolder, "__experiments\\large-patterns\\", trialFolder);
                
            if (!Directory.Exists(targetFolder))
                Directory.CreateDirectory(targetFolder);

            for (var width = startSize; width <= endSize; width += increment)
            {
                var startHeight = width;
                var endHeight = randomizeRectangular ? endSize : width;
                //var endHeight = randomizeRectangular && width+increment <= endSize ? width + increment : width;

                for (var height = width; height <= endHeight; height += increment)
                {
                    bool canEnumerateAll = width * height <= 20 ? true : false;

                    var startTime = DateTime.Now;

                    var fileName = $"{width}x{height}.txt";
                    var filePath = Path.Combine(targetFolder, fileName);

                    using (var file = new StreamWriter(filePath))
                    {
                        file.WriteLine(
                            "size; DoF; trial; " +
                            "randomized code; number random. rigid cells; percent random. rigid cells; number rand. shear cells; percent rand. shear cells; has occurred random; " +
                            "effective code; number effect. rigid cells; percent effect. rigid cells; number effect. shear cells; percent effect. shear cells; has occurred effect"
                        );

                        Console.WriteLine(" ------ {0} x {1} ------", width, height);
                        Console.WriteLine("        size = {0}, can enumerate: {1}", width*height, canEnumerateAll);
                        Initialize(width, height);

                        var randomlyOccurred = new HashSet<string>();
                        var effectivelyOccurred = new HashSet<string>();

                        if (canEnumerateAll)
                        {
                            var allCodes = GenerateCompleteBitSequence();

                            for (var j = 0; j < allCodes.Count; j++)
                            {
                                var code = allCodes[j];
                                Console.WriteLine("    ({0}/{1}) (enumerated code)", j, allCodes.Count);

                                LogCode(code, width * height, j, file, randomlyOccurred, effectivelyOccurred);
                            }
                        }
                        else
                        {
                            for (var trial = 1; trial <= numberTrials; trial++)
                            {
                                var randomCode = GenerateCode();
                                Console.WriteLine("    ({0}/{1}) (random code)", trial, numberTrials);

                                LogCode(randomCode, width * height, trial, file, randomlyOccurred, effectivelyOccurred);
                            }
                        }

                        file.WriteLine("---- start time: {0}", startTime.ToString());
                        file.WriteLine("---- end time:   {0}", DateTime.Now.ToString());
                        file.Flush();
                    }
                }
            }

            //for (var size = startSize; size <= endSize; size += increment)
            //{
            //    bool canEnumerateAll = size*size <= 25 ? true : false;

            //    var startTime = DateTime.Now;

            //    var fileName = $"{size}x{size}.txt"; 
            //    var filePath = Path.Combine(targetFolder, fileName);

            //    using (var file = new StreamWriter(filePath))
            //    {
            //        file.WriteLine(
            //            "size; DoF; trial; " +
            //            "randomized code; number random. rigid cells; percent random. rigid cells; number rand. shear cells; percent rand. shear cells; has occurred random; " +
            //            "effective code; number effect. rigid cells; percent effect. rigid cells; number effect. shear cells; percent effect. shear cells; has occurred effect"
            //        );

            //        Console.WriteLine(" ------ {0} x {1} ------", size, size);
            //        Initialize(size, size);

            //        var randomlyOccurred = new HashSet<string>();
            //        var effectivelyOccurred = new HashSet<string>();

            //        if (canEnumerateAll)
            //        {
            //            var allCodes = GenerateCompleteBitSequence();

            //            for (var j = 0; j < allCodes.Count; j++)
            //            {
            //                var code = allCodes[j];
            //                Console.WriteLine("    ({0}/{1}) (enumerated code)", j, allCodes.Count);

            //                LogCode(code, size, j, file, randomlyOccurred, effectivelyOccurred);
            //            }
            //        }
            //        else
            //        {
            //            for (var trial = 1; trial <= numberTrials; trial++)
            //            {
            //                var randomCode = GenerateCode();
            //                Console.WriteLine("    ({0}/{1}) (random code)", trial, numberTrials);

            //                LogCode(randomCode, size, trial, file, randomlyOccurred, effectivelyOccurred);

            //                //_model.Clear();

            //                //var randomCode = GenerateCode();
            //                //CodeToModel(randomCode);
            //                //Console.WriteLine("    ({0}) (random code)", t);

            //                //file.Write(i + "; " + _model.CurrentDoF + "; " + t);
            //                //file.Write("; " + LogConfiguration(randomCode, randomlyOccurred));
            //                //randomlyOccurred.Add(randomCode);

            //                //_model.SetNonShearingCells();
            //                //var effectiveCode = _model.GetEncoding();

            //                //file.Write("; " + LogConfiguration(effectiveCode, effectivelyOccurred));
            //                //effectivelyOccurred.Add(effectiveCode);

            //                //file.WriteLine();
            //                //file.Flush();
            //            }
            //        }

            //        file.WriteLine("---- start time: {0}", startTime.ToString());
            //        file.WriteLine("---- end time:   {0}", DateTime.Now.ToString());
            //        file.Flush();
            //    }
            //}
        }

        private void LogCode(string code, int size, int trial, StreamWriter file, HashSet<string> randomlyOccurred, HashSet<string> effectivelyOccurred)
        {
            _model.Clear();

            //var randomCode = GenerateCode();
            var randomCode = code;
            CodeToModel(randomCode);

            file.Write(size + "; " + _model.CurrentDoF + "; " + trial);
            file.Write("; " + LogConfiguration(randomCode, randomlyOccurred));
            randomlyOccurred.Add(randomCode);

            _model.SetNonShearingCells();
            var effectiveCode = _model.GetEncoding();

            file.Write("; " + LogConfiguration(effectiveCode, effectivelyOccurred));
            effectivelyOccurred.Add(effectiveCode);

            file.WriteLine();
            file.Flush();
        }

        private string LogConfiguration(string code, HashSet<string> occurred)
        {
            //var code = _model.GetEncoding();

            //var numCells = _model.Cells.Count;
            //var numRigid = _model.Cells.Count(cell => cell is RigidCell);
            //var numShear = _model.Cells.Count(cell => cell is ShearCell);

            var numCells = code.Count();
            var numRigid = code.Count(c => c == '0');
            var numShear = code.Count(c => c == '1');

            var log = new string[]{ code, numRigid.ToString(), (numRigid/(double)numCells).ToString(), numShear.ToString(), (numShear/(double)numCells).ToString(), occurred.Contains(code).ToString() };
            return string.Join("; ", log);
        }

        private void Initialize(int arrangementWidth, int arrangementHeight)
        {
            _arrangementWidth = arrangementWidth;
            _arrangementHeight = arrangementHeight;
            _numberCells = _arrangementWidth * _arrangementHeight;

            MinDoF = 1;
            MaxDoF = int.MaxValue;

            MinRigidCells = 0;
            MaxRigidCells = _numberCells;

            MaxIterations = 2000;

            var parentFolder = System.IO.Path.GetFullPath(@"..\..\..\");
            _targetFolder = parentFolder + _arrangementWidth + " x " + _arrangementHeight;
        }

        private string GenerateCode()
        {
            //var random = new Random(Guid.NewGuid().GetHashCode());

            _numberRigidCells = _random.Next(MinRigidCells, MaxRigidCells + 1);
            Debug.WriteLine("# rigid cells: {0} ({1}%)", _numberRigidCells, _numberRigidCells / (double)_numberCells);

            var rigidPositions = GetRigidCellPositions();

            char[] bitcode = new char[_numberCells];
            for (var i = 0; i < _numberCells; i++)
                bitcode[i] = '1';

            foreach (var rigidPosition in rigidPositions)
                bitcode[rigidPosition] = '0';

            return new string(bitcode);
        }

        private List<string> GenerateCompleteBitSequence()
        {
            var numberSequences = Math.Pow(2, _numberCells);
            var bitCodes = new List<string>();

            for (int i = 0; i < numberSequences; i++)
                AddNewBitSequence((uint)i, bitCodes);

            return bitCodes;
        }

        private void AddNewBitSequence(long value, List<string> bitCodes)
        {
            string bitString = Convert.ToString(value, 2);

            //fill up with zeros
            if (bitString.Length < _numberCells)
            {
                var toInsert = "";
                for (var j = 0; j < _numberCells - bitString.Length; j++)
                    toInsert += "0";

                bitString = bitString.Insert(0, toInsert);
            }

            //Debug.WriteLine("{0}: {1}", i, bitString);

            bitCodes.Add(bitString);
        }


        private void CodeToModel(string bitcode)
        {
            List<Cell> cells;
            List<Vector> positions;
            CreateCellConfiguration(bitcode, new Vector(0, 0), out cells, out positions);

            for (var i = 0; i < cells.Count; i++)
                _model.AddCell(cells[i], positions[i], new Size(1, 1), false);

            _model.UpdateConstraintsGraph();
        }

        private void ShowCode()
        {
            _viewModel.Redraw();
            //_model.Clear();
        }

        private List<int> GetRigidCellPositions()
        {
            var rigidPositions = new List<int>();
            //var random = new Random(DateTime.Now.Millisecond);

            while(rigidPositions.Count < _numberRigidCells)
            {
                var rigidPosition = _random.Next(0, _numberCells);
                if (rigidPositions.Contains(rigidPosition))
                    continue;

                rigidPositions.Add(rigidPosition);
            }

            return rigidPositions;
        }

        #region from filtered randomizer

        public void CreateCellConfiguration(string bitCode, Vector configurationPosition, out List<Cell> cells, out List<Vector> cellPositions)
        {
            cells = new List<Cell>();
            cellPositions = new List<Vector>();

            var cellPosition = new Vector(configurationPosition.X, configurationPosition.Y);

            for (var j = 0; j < bitCode.Length; ++j)
            {
                var bit = int.Parse("" + bitCode[j]);
                Cell cell = new ShearCell();
                if (bit == 0)
                    cell = new RigidCell();

                cellPosition = new Vector(configurationPosition.X + j % _arrangementWidth, cellPosition.Y);
                if (j > 0 && j % _arrangementWidth == 0)
                    cellPosition.Y += 1;

                cells.Add(cell);
                cellPositions.Add(cellPosition);
            }
        }

        #endregion from filtered randomizer

    }
}
