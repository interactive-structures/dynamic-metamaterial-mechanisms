using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Media;
using System.Windows.Shapes;
using ShearCell_Interaction.Helper;
using ShearCell_Interaction.Model;
using ShearCell_Interaction.Simulation;
using ShearCell_Interaction.View;
using Path = System.IO.Path;

namespace ShearCell_Interaction.Randomizer
{
    class FilteredRandomizer
    {
        private static bool _canEnumerateAll = true;

        private readonly List<int> _flags;
        private List<string> _bitCodes;
        private List<string> _uniqueBitCodes;

        private List<string> _uniqueMovingCodes;
        private List<List<string>> _uniqueCodesWithDoF;
        private List<HashSet<string>> _dofs;

        private Dictionary<string, string> _alteredMovingCodes;

        private readonly int _numberSequences;
        private readonly int _numberCells;
        private readonly int _arrangementWidth;
        private readonly int _arrangementHeight;

        private readonly string _targetFolder;

        private readonly ViewModel _viewModel;
        private readonly MetamaterialModel _model;

        public FilteredRandomizer(int arrangementWidth, int arrangementHeight, ViewModel viewModel, MetamaterialModel model)
        {
            _arrangementWidth = arrangementWidth;
            _arrangementHeight = arrangementHeight;
            _numberCells = _arrangementWidth * _arrangementHeight;

            var samplingSize = (int) Math.Pow(2, 20);

            var intendedSequences = Math.Pow(2, _numberCells);
            var maxCells = (int) Math.Pow(2, 25);
            if (intendedSequences > maxCells)
            {
                _numberSequences = samplingSize;
                _canEnumerateAll = false;
            }
            else
            {
                _numberSequences = (int)intendedSequences;
                _canEnumerateAll = true;
            }

            _viewModel = viewModel;
            _model = model;
            ViewModel.CellScale = 10;

            _flags = new List<int>(_numberSequences);
            _bitCodes = new List<string>(_numberSequences);
            _uniqueBitCodes = new List<string>();

            _alteredMovingCodes = new Dictionary<string, string>();
            _uniqueMovingCodes = new List<string>();

            //var parentFolder = "D:\\Dropbox\\HPI\\2018-UIST-Understanding metamaterial mechanisms\\Cells Exploration\\generating cells\\";
            var parentFolder = Path.GetFullPath(@"..\..\..\");
            _targetFolder = parentFolder + _arrangementWidth + " x " + _arrangementHeight;
        }

        internal void GeneratePersistedVariations()
        {
            Console.WriteLine();
            Console.WriteLine("---------- START {0} x {1} ----------", _arrangementWidth, _arrangementHeight);
            Console.WriteLine();

            GeneratePersistBitSequences();
            FilterPersistCompleteSequences();
            FilterPersistMovingConfigurations();
            ClusterPersistByDegreesOfFreedom();

            Console.WriteLine("DONE {0} x {1}", _arrangementWidth, _arrangementHeight);
            Console.WriteLine();
        }

        private void GeneratePersistBitSequences()
        {
            Console.WriteLine(@"start generating bit sequences: {0}x{1} ({2})", _arrangementWidth, _arrangementHeight, DateTime.Now.ToString(new CultureInfo("de-DE")));

            GenerateBitSequence();

            Console.WriteLine(@"done generating bit sequences: {0} ({1})", _bitCodes.Count, DateTime.Now.ToString(new CultureInfo("de-DE")));


            Console.WriteLine("persisting...");

            ViewModel.WindowWidth = 1500;

            if (!Directory.Exists(_targetFolder))
                Directory.CreateDirectory(_targetFolder);

            //RenderBitmapOfCodes(_bitCodes, targetFolder + "\\1 all codes.jpg");
            //WriteCodesToFile(_bitCodes, _targetFolder + "\\1 all codes.txt");
            WriteCodesToFile(_bitCodes, _targetFolder + "\\1 all codes.txt");
            GC.Collect();
            GC.WaitForPendingFinalizers();

            Console.WriteLine("done.");
            Console.WriteLine();
        }

        private void FilterPersistCompleteSequences()
        {
            Console.WriteLine("remove rotated and mirrored variations ({0})", DateTime.Now.ToString(new CultureInfo("de-DE")));

            _uniqueBitCodes = RemoveVariations(_bitCodes);

            Console.WriteLine("variation removal done. remaining codes: {0} ({1})", _uniqueBitCodes.Count, DateTime.Now.ToString(new CultureInfo("de-DE")));


            Console.WriteLine("persisting...");

            //RenderBitmapOfCodes(_uniqueBitCodes, targetFolder + "\\2 unique codes.jpg");
            WriteCodesToFile(_uniqueBitCodes, _targetFolder + "\\2 unique codes.txt");

            _bitCodes = null;
            GC.Collect();
            GC.WaitForPendingFinalizers();

            Console.WriteLine("done.");
            Console.WriteLine();
        }

        private void FilterPersistMovingConfigurations()
        {
            Console.WriteLine("filtering variations---change non-shearing shear cells to rigid cells. ({0})", DateTime.Now.ToString(new CultureInfo("de-DE")));

            FilterMovingConfigurations(_uniqueBitCodes);
            _uniqueMovingCodes = RemoveVariations(_uniqueMovingCodes);

            Console.WriteLine("filtering done. remaining codes: {0} ({1})", _uniqueMovingCodes.Count, DateTime.Now.ToString(new CultureInfo("de-DE")));


            Console.WriteLine("persisting...");

            //RenderBitmapOfCodes(_alteredMovingCodes, targetFolder + "\\4 moving codes.jpg");
            //WriteCodesToFile(_alteredMovingCodes, targetFolder + "\\4 moving codes.txt");
            RenderBitmapOfCodes(_alteredMovingCodes, _targetFolder + "\\3 altered codes.jpg");
            WriteCodesToFile(_alteredMovingCodes, _targetFolder + "\\3 altered codes.txt");

            _uniqueBitCodes = null;
            _alteredMovingCodes = null;
            GC.Collect();
            GC.WaitForPendingFinalizers();

            RenderBitmapOfCodes(_uniqueMovingCodes, _targetFolder + "\\5 moving unique codes.jpg");
            WriteCodesToFile(_uniqueMovingCodes, _targetFolder + "\\5 moving unique codes.txt");

            GC.Collect();
            GC.WaitForPendingFinalizers();

            Console.WriteLine("done.");
            Console.WriteLine();
        }

        private void ClusterPersistByDegreesOfFreedom()
        {
            Console.WriteLine("clustering cells by DoF. ({0})", DateTime.Now.ToString(new CultureInfo("de-DE")));

            ClusterByDegreesOfFreedom();

            Console.WriteLine("clustering by DoF done. ({0})", DateTime.Now.ToString(new CultureInfo("de-DE")));
            Console.WriteLine();


            Console.WriteLine("persisting...");

            RenderBitmapOfCodes(_uniqueCodesWithDoF, _targetFolder + "\\6 moving unique codes with .jpg");
            WriteCodesToFile(_uniqueCodesWithDoF, _targetFolder + "\\6 moving unique codes with DoF.txt");
            WriteStatsFile(_uniqueCodesWithDoF, _targetFolder + "\\7 stats "+_arrangementWidth + "x" + _arrangementHeight+".txt");

            _uniqueCodesWithDoF = null;
            GC.Collect();
            GC.WaitForPendingFinalizers();

            Console.WriteLine("done.");
            Console.WriteLine();
        }

        private void GenerateBitSequence()
        {
            if (_canEnumerateAll)
                GenerateCompleteBitSequence();
            else
                GenerateStochasticBitSequence();
        }

        private void GenerateCompleteBitSequence()
        {
            for (int i = 0; i < _numberSequences; i++)
                AddNewBitSequence((uint)i);
        }

        private void GenerateStochasticBitSequence()
        {
            long bucketSize = (long)Math.Pow(2, _numberCells) / _numberSequences;
            long bucket = 0;

            var random = new Random(DateTime.Now.Millisecond);

            for (int i = 0; i < _numberSequences; i++)
            {
                var randomFactor = random.NextDouble();
                long value = (long)(bucketSize*randomFactor + bucket);
                AddNewBitSequence(value);
                //AddNewBitSequence(bucket);

                bucket += bucketSize;

                if(i % 1000000 == 1)
                    Debug.WriteLine("at element " + i);
            }
        }

        private void AddNewBitSequence(long value)
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

            _bitCodes.Add(bitString);
            _flags.Add(-1);
        }

        private void FilterMovingConfigurations(List<string> sourceCodes)
        {
            _dofs = new List<HashSet<string>>(_numberCells);
            for (var i = 0; i < _numberCells; i++)
                _dofs.Add(new HashSet<string>());

            for (var j = 0; j < sourceCodes.Count; j++)
            {
                var code = sourceCodes[j];

                List<Cell> cells;
                List<Vector> positions;
                BitcodeHelper.CreateCellConfiguration(code, new Vector(0, 0), out cells, out positions);

                for (var i = 0; i < cells.Count; i++)
                    _model.AddCell(cells[i], positions[i], new Size(1,1));

                //var connectedComponents = _model.GetConstraintsGraph();

                var alteredCode = string.Copy(code);
                var changedIndices = new List<int>();

                for (var i = 0; i < _model.Cells.Count; i++)
                {
                    var cell = _model.Cells[i];
                    if(cell is RigidCell)
                        continue;


                    foreach (var component in _model.ConstraintGraph)
                    {
                        var flattend = component.SelectMany(e => e);
                        var intersected = flattend.Intersect(cell.CellEdges).ToList();

                        if (intersected.Count == 4)
                            SetCellTypeAt(ref alteredCode, ref changedIndices, '0', 0, cell.IndexVertex.InitialX, cell.IndexVertex.InitialY);
                    }
                }

                _alteredMovingCodes.Add(code, alteredCode);

                //var dof = _uniqueCodesWithDoF[connectedComponents.Count - 1];
                //if(!dof.Contains(alteredCode))
                //    dof.Add(alteredCode);

                _dofs[_model.ConstraintGraph.Count - 1].Add(alteredCode);

                //var codeValue = Convert.ToInt64(alteredCode, 2);
                //if (index < _numberSequences && index != -1)
                //    _flags[index] = connectedComponents.Count - 1;

                _model.Clear();
            }

            _uniqueMovingCodes = _alteredMovingCodes.Values.Distinct().ToList();
        }

        private void ClusterByDegreesOfFreedom()
        {
            _uniqueCodesWithDoF = new List<List<string>>(_numberCells);
            for (var i = 0; i < _numberCells; i++)
                _uniqueCodesWithDoF.Add(new List<string>());

            //for (var i = 0; i < _uniqueMovingCodes.Count; i++)
            //{
            //    var code = _uniqueMovingCodes[i];
            //    var codeValue = Convert.ToInt64(code, 2);
            //    var dof = _flags[index];

            //    if (dof < 0)
            //    {
            //        Debug.WriteLine("ERROR! dof is -2!");
            //        continue;
            //    }

            //    _uniqueCodesWithDoF[dof].Add(code);
            //}

            for (var i = 0; i < _uniqueMovingCodes.Count; i++)
            {
                var code = _uniqueMovingCodes[i];
                for (var dof = 0; dof < _dofs.Count; dof++)
                {
                    if (_dofs[dof].Contains(code))
                    {
                        _uniqueCodesWithDoF[dof].Add(code);
                        break;
                    }
                }
            }


            for (var i = 0; i < _uniqueCodesWithDoF.Count; i++)
            {
                var list = _uniqueCodesWithDoF[i];

                list.Sort(delegate (string current, string other)
                {
                    var currentShearCells = current.Count(c => c == '0');
                    var otherShearCells = other.Count(c => c == '0');

                    if (currentShearCells > otherShearCells)
                        return 1;
                    if (currentShearCells < otherShearCells)
                        return -1;

                    return 0;
                });
            }
        }

        #region variation filtering
        private List<string> RemoveVariations(List<string> sourceCodes)
        {
            var targetCodes = new System.Collections.Generic.HashSet<string>(sourceCodes);

            for (int i = 0; i < sourceCodes.Count; i++)
            {
                if (i % 1000000 == 1)
                    Debug.WriteLine("at element " + i);

                var code = sourceCodes[i];
                if(!targetCodes.Contains(code))
                    continue;

                var mirroredVertically = MirrorCodeVertically(code);
                if (mirroredVertically != code && targetCodes.Contains(mirroredVertically))
                    targetCodes.Remove(mirroredVertically);

                var mirroredHorizontally = MirrorCodeHorizontally(code);
                if (mirroredHorizontally != code && targetCodes.Contains(mirroredHorizontally))
                    targetCodes.Remove(mirroredHorizontally);

                var toRotate = code;

                for (var j = 0; j < 3; j++)
                {
                    var rotated = RotateCode(toRotate);
                    toRotate = rotated;

                    if (rotated != code && targetCodes.Contains(rotated))
                        targetCodes.Remove(rotated);

                    mirroredVertically = MirrorCodeVertically(rotated);
                    if (mirroredVertically != code && targetCodes.Contains(mirroredVertically))
                        targetCodes.Remove(mirroredVertically);

                    mirroredHorizontally = MirrorCodeHorizontally(rotated);
                    if (mirroredHorizontally != code && targetCodes.Contains(mirroredHorizontally))
                        targetCodes.Remove(mirroredHorizontally);
                }
            }

            return targetCodes.ToList();
        }
        //private List<string> RemoveVariations_index(List<string> sourceCodes)
        //{
        //    var targetCodes = new List<string>();

        //    for (int i = 0; i < sourceCodes.Count; i++)
        //    {
        //        if (i % 1000000 == 0)
        //            Debug.WriteLine("at element " + i);

        //        var code = sourceCodes[i];
        //        var codeValue = Convert.ToInt64(code, 2);
        //        var index = _indexLookup.IndexOf(codeValue);

        //        if (index < _numberSequences && index != -1 && _flags[index] == -2)
        //            continue;

        //        var mirroredVertically = MirrorCodeVertically(code);
        //        var mirroredVerticallyValue = Convert.ToInt64(mirroredVertically, 2);
        //        //if (mirroredVertically != code)
        //        if (mirroredVerticallyValue != codeValue)
        //        {
        //            index = _indexLookup.IndexOf(mirroredVerticallyValue);
        //            if (index != -1) _flags[index] = -2;
        //        }

        //        var mirroredHorizontally = MirrorCodeHorizontally(code);
        //        var mirroredHorizontallyValue = Convert.ToInt64(mirroredHorizontally, 2);
        //        //if (mirroredHorizontally != code)
        //        if (mirroredHorizontallyValue != codeValue)
        //        {
        //            index = _indexLookup.IndexOf(mirroredHorizontallyValue);
        //            if (index != -1) _flags[index] = -2;
        //        }

        //        var toRotate = code;

        //        for (var j = 0; j < 3; j++)
        //        {
        //            var rotated = RotateCode(toRotate);
        //            toRotate = rotated;

        //            var rotatedValue = Convert.ToInt64(rotated, 2);
        //            //if (rotated != code)
        //            if (rotatedValue != codeValue)
        //            {
        //                index = _indexLookup.IndexOf(rotatedValue);
        //                if (index != -1) _flags[index] = -2;
        //            }

        //            mirroredVertically = MirrorCodeVertically(rotated);
        //            mirroredVerticallyValue = Convert.ToInt64(mirroredVertically, 2);
        //            //if (mirroredVertically != code)
        //            if (mirroredVerticallyValue != codeValue)
        //            {
        //                index = _indexLookup.IndexOf(mirroredVerticallyValue);
        //                if (index != -1) _flags[index] = -2;
        //            }

        //            mirroredHorizontally = MirrorCodeHorizontally(rotated);
        //            mirroredHorizontallyValue = Convert.ToInt64(mirroredHorizontally, 2);
        //            //if (mirroredHorizontally != code)
        //            if (mirroredHorizontallyValue != codeValue)
        //            {
        //                index = _indexLookup.IndexOf(mirroredHorizontallyValue);
        //                if (index != -1) _flags[index] = -2;
        //            }
        //        }

        //        targetCodes.Add(code);
        //    }

        //    return targetCodes;
        //}

        //private void SetFlag(uint codeValue, int flagValue)
        //{
        //    var index = _indexLookup.IndexOf(codeValue);
        //    if(index != -1) 
        //        _flags[index] = flagValue;
        //}

        private string MirrorCodeVertically(string code)
        {
            var mirrored = "";

            for (var group = 0; group < _arrangementWidth; group++)
                for (var i = code.Length / _arrangementWidth - 1; i >= 0; i--)
                    mirrored += code[i + group * _arrangementWidth];

            return mirrored;
        }

        private string MirrorCodeHorizontally(string code)
        {
            var mirrored = "";

            for (var group = _arrangementWidth - 1; group >= 0; group--)
                for (var i = 0; i < code.Length / _arrangementWidth; i++)
                    mirrored += code[i + group * _arrangementWidth];

            return mirrored;
        }

        private string RotateCode(string code)
        {
            var rotated = "";

            for (var i = code.Length / _arrangementWidth - 1; i >= 0; i--)
                for (var j = 0; j < _arrangementWidth; j++)
                    rotated += code[i + j * _arrangementWidth];

            return rotated;
        }
        #endregion variation filtering

        #region drawing functions
        private List<Shape> DrawCategorizedCells(Dictionary<string,string> cellCodes, Vector startPosition)
        {
            var shapes = new List<Shape>();

            var configurationsInARow = CalculateConfigurationsInARowForComparison();
            var configurationPosition = startPosition;

            var rowTotal = 0;

            foreach (var codesKey in cellCodes.Keys)
            {
                if (rowTotal >= configurationsInARow)
                {
                    configurationPosition.Y += _arrangementHeight + 1;
                    configurationPosition.X = startPosition.X;

                    rowTotal = 0;
                }

                var originShapes = DrawCellConfiguration(codesKey, configurationPosition);
                shapes.AddRange(originShapes);
                configurationPosition.X += _arrangementWidth + 1;

                var alteredShapes = DrawCellConfiguration(cellCodes[codesKey], configurationPosition);
                shapes.AddRange(alteredShapes);
                configurationPosition.X += _arrangementWidth + 2;

                rowTotal++;
            }

            return shapes;
        }

        private List<Shape> DrawRandomCells(List<string> cellCodes, Vector startPosition)
        {
            var shapes = new List<Shape>();

            var configurationsInARow = CalculateConfigurationsInARowForList();
            var configurationPosition = startPosition;

            for (var i = 0; i < cellCodes.Count; i++)
            {
                if (i > 0 && i % configurationsInARow == 0)
                {
                    configurationPosition.Y += _arrangementHeight + 1;
                    configurationPosition.X -= (_arrangementWidth + 1) * configurationsInARow;
                }

                var configurationShapes = DrawCellConfiguration(cellCodes[i], configurationPosition);
                shapes.AddRange(configurationShapes);

                configurationPosition.X += _arrangementWidth + 1;
            }

            return shapes;
        }

        private List<Shape> DrawCellConfiguration(string bitCode, Vector configurationPosition)
        {
            var shapes = new List<Shape>();

            List<Cell> cells;
            List<Vector> positions;
            BitcodeHelper.CreateCellConfiguration(bitCode, configurationPosition, out cells, out positions);

            for (var i = 0; i < cells.Count; i++)
            {
                var shape = VisualizeDummyCells(cells[i], positions[i]);
                shapes.Add(shape);
            }

            return shapes;
        }
        
        private Shape VisualizeDummyCells(Cell cell, Vector cellPosition)
        {
            var vertexOffsets = new List<Vector>
            {
                new Vector(0, 0),
                new Vector(1, 0),
                new Vector(1, 1),
                new Vector(0, 1)
            };

            var points = new PointCollection();

            foreach (var offset in vertexOffsets)
            {
                var screenPoint = CoordinateConverter.ConvertGlobalToScreenCoordinates(Vector.Add(cellPosition, offset));
                points.Add(new Point(screenPoint.X, screenPoint.Y));
            }

            var polygon = new Polygon
            {
                Points = points,
                Stroke = Brushes.Gray,
                Fill = cell is RigidCell ? Brushes.DarkGray : Brushes.Transparent,
                StrokeLineJoin = PenLineJoin.Bevel
            };

            //_viewModel.InputShapeItems.Add(polygon);
            return polygon;
        }

        private int CalculateConfigurationsInARowForList()
        {
            return ViewModel.WindowWidth / (int)(ViewModel.CellScale * (_arrangementWidth + 1));
        }
        private int CalculateConfigurationsInARowForComparison()
        {
            return ViewModel.WindowWidth / (int)(ViewModel.CellScale * (_arrangementWidth * 2 + 3));
        }

        private void VisualizeConnectedComponents(List<List<List<Edge>>> connectedComponents)
        {
            for (var i = 0; i < connectedComponents.Count; i++)
            {
                var component = connectedComponents[i].SelectMany(e => e);

                var hue = i / (double)connectedComponents.Count;
                var colorRGB = ColorConversionHelper.ColorFromHSL(hue, 1.0, 0.5);
                var brush = new SolidColorBrush(colorRGB);

                foreach (var edge in component)
                {
                    var point1 = CoordinateConverter.ConvertGlobalToScreenCoordinates(edge.Vertex1.ToInitialVector());
                    var point2 = CoordinateConverter.ConvertGlobalToScreenCoordinates(edge.Vertex2.ToInitialVector());

                    var polyline = new Polyline
                    {
                        Stroke = brush,
                        Points = { new Point(point1.X, point1.Y), new Point(point2.X, point2.Y) }
                    };

                    _viewModel.InputShapeItems.Add(polyline);
                }
            }
        }
        #endregion drawing functions

        #region persisting results
        private void WriteCodesToFile(List<string> codesToWrite, string filename)
        {
            const int flushFrequency = 100;

            using (var file = new StreamWriter(filename))
            {
                file.WriteLine(GetFileHeader(codesToWrite.Count));

                var count = 0;
                foreach (var code in codesToWrite)
                {
                    if (count % flushFrequency == 0)
                        file.Flush();

                    file.WriteLine(code);

                    count++;
                }
            }
        }

        private void WriteCodesToFile(Dictionary<string, string> codesToWrite, string filename)
        {
            const int flushFrequency = 100;

            using (var file = new StreamWriter(filename))
            {
                file.WriteLine(GetFileHeader(codesToWrite.Count));

                var count = 0;
                foreach (var code in codesToWrite)
                {
                    if (count % flushFrequency == 0)
                        file.Flush();

                    file.WriteLine(code.Key + "-->" + code.Value);

                    count++;
                }
            }
        }

        private void WriteStatsFile(List<List<string>> codesToWrite, string filename)
        {
            const int flushFrequency = 100;

            using (var file = new StreamWriter(filename))
            {
                file.WriteLine("total cells; DoF; code; number rigid cells; set size; mean rigid; min rigid; max rigid");

                var flushCount = 0;
                for (var dof = 0; dof < codesToWrite.Count; dof++)
                {
                    var dofCodes = codesToWrite[dof];

                    if(dofCodes.Count == 0)
                        continue;

                    var sumRigidCells = 0;

                    for (var j = 0; j < dofCodes.Count; j++)
                    {
                        if (flushCount % flushFrequency == 0)
                            file.Flush();

                        var code = dofCodes[j];
                        var numberRigidCells = code.Count(c => c == '0');

                        file.WriteLine(
                            _numberCells + "; " + 
                            dof + "; " + 
                            code + "; " + 
                            numberRigidCells
                            ); 

                        flushCount++;
                        sumRigidCells += numberRigidCells;
                    }

                    //file.WriteLine("total cells; DoF; code; number rigid cells; set size; mean rigid; min rigid; max rigid");

                    file.WriteLine(
                        _numberCells + "; " +
                        dof + "; " +
                        "; " +
                        "; " +
                        dofCodes.Count + "; " +
                        sumRigidCells /(double) dofCodes.Count + "; " +
                        dofCodes[0].Count(c => c == '0') + "; " +
                        dofCodes[dofCodes.Count - 1].Count(c => c == '0')
                    );

                }
            }
        }

        private void WriteCodesToFile(List<List<string>> codesToWrite, string filename)
        {
            const int flushFrequency = 100;

            using (var file = new StreamWriter(filename))
            {
                file.WriteLine(GetFileHeader(codesToWrite.Count));

                var flushCount = 0;
                for (var i = 0; i < codesToWrite.Count; i++)
                {
                    var dofCodes = codesToWrite[i];

                    if(dofCodes.Count == 0)
                        continue;

                    file.WriteLine("#####");
                    file.WriteLine("degrees of freedom: " + i);
                    file.WriteLine("#####");

                    for (var j = 0; j < dofCodes.Count; j++)
                    {
                        if (flushCount % flushFrequency == 0)
                            file.Flush();

                        file.WriteLine(dofCodes[j]);
                        flushCount++;
                    }

                    file.WriteLine("   min rigid: " + dofCodes[0].Count(c => c == '0'));
                    file.WriteLine("   max rigid: " + dofCodes[dofCodes.Count-1].Count(c => c == '0'));
                    file.WriteLine("   sum: " + dofCodes.Count);
                    file.WriteLine();
                }
            }
        }

        private string GetFileHeader(int count)
        {
            //var now = DateTime.Now;

            var header = "" +
                DateTime.Now.ToString(new CultureInfo("de-DE")) + "\r\n" +
                //now.Hour + ":" + now.Minute + ":" + now.Second + "\r\n" +
                "###########################################" + "\r\n" +
                "Configuration: " + _arrangementWidth + " x " + _arrangementHeight + "\r\n" +
                "Total: " + count + "\r\n" +
                "###########################################\r\n";

            return header;
        }

        private void RenderBitmapOfCodes(List<string> codesToDraw, string filename)
        {
            CalculateCanvasHeight(codesToDraw.Count, CalculateConfigurationsInARowForList());
            var shapes = DrawRandomCells(codesToDraw, new Vector(1, 1));
            BitmapHelper.RenderBitmap(shapes, filename);
        }

        private void RenderBitmapOfCodes(Dictionary<string, string> codesToDraw, string filename)
        {
            CalculateCanvasHeight(codesToDraw.Count, CalculateConfigurationsInARowForComparison());
            var shapes = DrawCategorizedCells(codesToDraw, new Vector(1, 1));
            BitmapHelper.RenderBitmap(shapes, filename);
        }

        private void RenderBitmapOfCodes(List<List<string>> codesToDraw, string filename)
        {
            for (var i = 0; i < codesToDraw.Count; i++)
            {
                var dofCodes = codesToDraw[i];

                if (dofCodes.Count == 0)
                    continue;

                CalculateCanvasHeight(dofCodes.Count, CalculateConfigurationsInARowForList());
                var shapes = DrawRandomCells(dofCodes, new Vector(1, 1));

                var extensionIndex = filename.LastIndexOf('.');
                var name = filename.Insert(extensionIndex, i + " DoF");
                BitmapHelper.RenderBitmap(shapes, name);
            }
        }


        private void CalculateCanvasHeight(int numberCodesToDraw, int numberConfigurationsInARow)
        {
            //var numberConfigurationsInARow = ViewModel.WindowWidth / (int)(ViewModel.CellScale * (_arrangementWidth + 1));
            var numberConfigurationRows = (int)Math.Ceiling(numberCodesToDraw / (double)numberConfigurationsInARow);
            var numberCellRows = numberConfigurationRows * (_arrangementHeight + 1);
            var canvasHeight = (numberCellRows + 1) * (int)ViewModel.CellScale;

            ViewModel.WindowHeight = canvasHeight;
        }
        #endregion persisting results

        #region cell position functions
        private void SetCellTypeAt(ref string code, ref List<int> changedIndices, char type, int index, int offsetX = 0, int offsetY = 0)
        {
            var offsetIndex = GetOffsetIndex(index, offsetX, offsetY);
            if (offsetIndex == -1)
                return;

            var newCode = code.ToCharArray();
            if (newCode[offsetIndex] == type)
                return;

            newCode[offsetIndex] = type;
            code = new string(newCode);

            changedIndices.Add(offsetIndex);
        }

        private int GetOffsetIndex(int index, int offsetX, int offsetY)
        {
            var x = index % _arrangementWidth + offsetX;
            if (x > _arrangementWidth - 1 || x < 0)
                return -1;

            var y = index / _arrangementWidth + offsetY;
            if (y > _arrangementHeight - 1 || y < 0)
                return -1;

            var offsetIndex = x + y * _arrangementWidth;
            return offsetIndex;
        }

        private int GetCellTypeAt(string code, int index, int offsetX = 0, int offsetY = 0)
        {
            var offsetIndex = GetOffsetIndex(index, offsetX, offsetY);
            if (offsetIndex == -1)
                return -1;

            return int.Parse(code[offsetIndex].ToString());
        }

        private int GetCellTypeAt(string code, Vector position)
        {
            var x = (int)position.X;
            var y = (int)position.Y;

            if (x > _arrangementWidth - 1 || y > _arrangementWidth - 1)
                return -1;

            var index = x + _arrangementWidth * y;
            return int.Parse(code[index].ToString());
        }

        private Vector GetCellPosition(int index, int offsetX = 0, int offsetY = 0)
        {
            var positionX = (index + offsetX) % _arrangementWidth;
            var positionY = (index + offsetY * _arrangementWidth) / _arrangementWidth;

            return new Vector(positionX, positionY);
        }
#endregion cell position functions

        #region alex rigidity filtering
        //private void FilterMovingConfigurations_old()
        //{
        //    foreach (var code in _uniqueBitCodes)
        //    {
        //        var alteredCode = PropagateRigidity(code);

        //        if (!alteredCode.Contains('1'))
        //            _rigidCodes.Add(code, alteredCode);
        //        else
        //            _alteredMovingCodes.Add(code, alteredCode);
        //    }

        //    foreach (var movingCode in _alteredMovingCodes.Values.ToList())
        //    {
        //        if (!_uniqueMovingCodes.Contains(movingCode))
        //            _uniqueMovingCodes.Add(movingCode);
        //    }
        //}

        private string PropagateRigidity(string code)
        {
            //var testCode = "123456789";
            //var code = "000101110";
            //var code = "011100101";
            //code = "011110010";

            //var alreadyCheckedCells = new List<int>();
            var cellsToCheck = new List<int>();

            for (var i = 0; i < _numberCells; i++)
            {
                if (code[i] == '0')
                    cellsToCheck.Add(i);
            }

            while (cellsToCheck.Count > 0)
            {
                var index = cellsToCheck[0];
                var changedIndices = ConstrainNeighborCells(ref code, index);

                cellsToCheck.AddRange(changedIndices);
                cellsToCheck.RemoveAt(0);
                //alreadyCheckedCells.Add(index);
            }



            //for (var i = 0; i < _numberCells; i++)
            //{
            //    if(!code.Contains('1'))
            //        break;

            //    var cellType = int.Parse(code[i].ToString());
            //    if(cellType == 1)
            //        continue;

            //    var rightNeighbor = GetCellTypeAt(code, i, 1, 0);
            //    if (rightNeighbor == 0)
            //        code = ConstrainVerticalNeighborCells(code, i, 1);


            //    var leftNeighbor = GetCellTypeAt(code, i, -1, 0);
            //    if (leftNeighbor == 0)
            //        code = ConstrainVerticalNeighborCells(code, i, -1);


            //    var topNeighbor = GetCellTypeAt(code, i, 0, 1);
            //    if (topNeighbor == 0)
            //        code = ConstrainHorizontalNeighborCells(code, i, 1);

            //    var bottomNeighbor = GetCellTypeAt(code, i, 0, -1);
            //    if(bottomNeighbor == 0)
            //        code = ConstrainHorizontalNeighborCells(code, i, -1);
            //}

            return code;
        }

        private List<int> ConstrainNeighborCells(ref string code, int index)
        {
            var newCellsToCheck = new List<int>();
            List<int> changedIndices;

            var rightNeighbor = GetCellTypeAt(code, index, 1, 0);
            if (rightNeighbor == 0)
            {
                changedIndices = ConstrainVerticalNeighborCells(ref code, index, 1);
                newCellsToCheck.AddRange(changedIndices);
            }

            var leftNeighbor = GetCellTypeAt(code, index, -1, 0);
            if (leftNeighbor == 0)
            {
                changedIndices = ConstrainVerticalNeighborCells(ref code, index, -1);
                newCellsToCheck.AddRange(changedIndices);
            }

            var topNeighbor = GetCellTypeAt(code, index, 0, 1);
            if (topNeighbor == 0)
            {
                changedIndices = ConstrainHorizontalNeighborCells(ref code, index, 1);
                newCellsToCheck.AddRange(changedIndices);
            }

            var bottomNeighbor = GetCellTypeAt(code, index, 0, -1);
            if (bottomNeighbor == 0)
            {
                changedIndices = ConstrainHorizontalNeighborCells(ref code, index, -1);
                newCellsToCheck.AddRange(changedIndices);
            }

            return newCellsToCheck;
        }

        private List<int> ConstrainVerticalNeighborCells(ref string code, int index, int offsetX)
        {
            //var horizontalNeighbor = GetCellTypeAt(code, index, offsetX, 0);
            //if (horizontalNeighbor != 0)
            //    return;

            var changedIndices = new List<int>();
            var indexY = index / _arrangementWidth;

            for (var y = 0; y < _arrangementHeight; y++)
            {
                if (y == indexY)
                    continue;

                var offsetY = y - indexY;

                var top = GetCellTypeAt(code, index, 0, offsetY);
                var topRight = GetCellTypeAt(code, index, offsetX, offsetY);

                //if (top == -1)
                //    break;

                if (top != topRight)
                {
                    SetCellTypeAt(ref code, ref changedIndices, '0', index, 0, offsetY);
                    SetCellTypeAt(ref code, ref changedIndices, '0', index, offsetX, offsetY);
                }
            }

            return changedIndices;
        }

        private List<int> ConstrainHorizontalNeighborCells(ref string code, int index, int offsetY)
        {
            var changedIndices = new List<int>();
            var indexX = index % _arrangementWidth;

            for (var x = 0; x < _arrangementWidth; x++)
            {
                if (x == indexX)
                    continue;

                var offsetX = x - indexX;

                var horizontal = GetCellTypeAt(code, index, offsetX, 0);
                var horizontalNeighbor = GetCellTypeAt(code, index, offsetX, offsetY);

                //if (top == -1)
                //    break;

                if (horizontal != horizontalNeighbor)
                {
                    SetCellTypeAt(ref code, ref changedIndices, '0', index, offsetX, 0);
                    SetCellTypeAt(ref code, ref changedIndices, '0', index, offsetX, offsetY);
                }
            }

            return changedIndices;
        }
        #endregion alex rigidity filtering

    }
}
