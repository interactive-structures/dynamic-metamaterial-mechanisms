using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Text.RegularExpressions;
using System.Threading;
using System.Windows;
using ShearCell_Interaction.Model;
using ShearCell_Interaction.View;

using NativeWrapper;
using System.Linq;

namespace ShearCell_Interaction.Simulation
{
    // call C++ code: https://stackoverflow.com/questions/9407616/how-to-call-c-code-from-c-sharp

    internal class OptimizedSimulationController
    {
        private string _optimizerPath;// = Path.GetFullPath(@"..\..\..\");//@"D:\hpi\__workspace\metamaterial-mechanisms (analyze-space)\1-analyze\ShearCell_Analysis\ShearCell_Optimization\build\x64\";
        private string _optimizerFile;// = _optimizerPath + @"Release\ShearCell_Optimization.exe";
        private const string _cellsInputFile = @"cells.txt";
        private string _pointsOutputPath; // = _optimizerPath + @"points\";

        public MetamaterialModel Model { get; set; }

        private readonly ViewModel _viewModel;

        private List<List<Vector>> _framePositions;

        private Process _optimizerProcess;

        public delegate void SimulationInfoAvailableHandler(object sender, SimulationInfoEventArgs e);
        public event SimulationInfoAvailableHandler OnSimulationInfoAvailable;
        
        public OptimizedSimulationController(MetamaterialModel model, ViewModel viewModel)
        {
            _optimizerPath = Path.GetFullPath(@"..\..\..\..\ShearCell_Optimization\build__visual-studio-2015\x64\");//@"D:\hpi\__workspace\m
            _optimizerFile = _optimizerPath + @"Release\ShearCell_Optimization.exe";
            _pointsOutputPath = _optimizerPath + @"points\";

            Model = model;
            _viewModel = viewModel;
        }

        public void StartOptimizedPathSimulation(bool integratedSolution = true)
        {
            var allDataAvailable = CheckAllDataAvailable();

            if (!allDataAvailable)
            {
                MessageBox.Show("Not all data for simulation provided.");
                return;
            }
            
            if (integratedSolution)
            {
                RunOptimizedPathSimulationIntegrated(false);
            }
            else
            {
                WriteCurrentModelConfiguration();
                RunOptimizerProcess();
            }
        }

        public void RunOptimizedPathSimulationIntegrated(bool useOutputPath = true)
        {
            var modelNative = ConvertModelToNativeModel(useOutputPath);
            
            CommunicatorNative communicator = new CommunicatorNative();
            communicator.modelNative = modelNative;
            communicator.optimize();
            
            var result = communicator.resultNative;
            ConvertResultToFramePositions(result);
            
            if (OnSimulationInfoAvailable != null)
                Application.Current.Dispatcher.Invoke(() => OnSimulationInfoAvailable(this, new SimulationInfoEventArgs(_framePositions)));
        }

        private bool CheckAllDataAvailable()
        {
            if (Model.GetAnchors().Count < 1)
                return false;

            if (Model.InputPath == null || Model.InputPath.Count < 1)
                return false;

            return true;
        }

        private void RunOptimizerProcess()
        {
            _optimizerProcess = new Process();
            _optimizerProcess.StartInfo.FileName = _optimizerFile;
            _optimizerProcess.StartInfo.WorkingDirectory = Directory.GetParent(_optimizerFile).FullName;
            //_optimizerProcess.StartInfo.WindowStyle = ProcessWindowStyle.Hidden;

            _optimizerProcess.EnableRaisingEvents = true;
            _optimizerProcess.Exited -= OnOptimizationDone;
            _optimizerProcess.Exited += OnOptimizationDone;
            _optimizerProcess.Start();
            //Application.Current.MainWindow.Topmost = true;
        }

        private void OnOptimizationDone(object sender, EventArgs e)
        {
            ((Process)sender).Exited -= OnOptimizationDone;
            ((Process)sender).EnableRaisingEvents = false;

            SaveSimulationFrames();

            if (OnSimulationInfoAvailable != null)
                Application.Current.Dispatcher.Invoke(() => OnSimulationInfoAvailable(this, new SimulationInfoEventArgs(_framePositions)));

        }
        
        private void WriteCurrentModelConfiguration()
        {
            ModelSerializer.WriteToFile(Path.Combine(_optimizerPath, _cellsInputFile), Model, _viewModel, false);
        }

        private void SaveSimulationFrames()
        {
            if (!Directory.Exists(_pointsOutputPath))
                return;

            _framePositions = new List<List<Vector>>();
            for (var i = 0; i < Model.InputPath.Count; i++)
                _framePositions.Add(new List<Vector>());

            var frameFiles = Directory.GetFiles(_pointsOutputPath);

            foreach (var frameFile in frameFiles)
            {
                var filename = frameFile.Substring(frameFile.LastIndexOf('\\'));
                var numericPart = Regex.Match(filename, "\\d+").Value;

                var currentFrame = int.Parse(numericPart);

                using (var file = new StreamReader(frameFile))
                {
                    string line;
                    var i = 0;

                    while ((line = file.ReadLine()) != null)
                    {
                        if (string.IsNullOrWhiteSpace(line))
                            continue;

                        var split = line.Split(' ');

                        var vector = new Vector(double.Parse(split[0]), double.Parse(split[1]));
                        _framePositions[currentFrame].Add(vector);

                        i++;
                    }
                }
            }
        }

        private GridModelNative ConvertModelToNativeModel(bool useOutputPath = true)
        {
            GridModelNative modelNative = new GridModelNative();

            GridCellNative[] cellsNative = new GridCellNative[Model.Cells.Count];

            for (int i = 0; i < Model.Cells.Count; i++)
            {
                Cell cell = Model.Cells[i];
                GridCellNative cellNative = new GridCellNative();
                cellNative.a = Model.Vertices.IndexOf(cell.CellVertices[0]);
                cellNative.b = Model.Vertices.IndexOf(cell.CellVertices[1]);
                cellNative.c = Model.Vertices.IndexOf(cell.CellVertices[2]);
                cellNative.d = Model.Vertices.IndexOf(cell.CellVertices[3]);

                //0 Rigid, 1 Shear, 2 Void
                cellNative.type = cell is RigidCell ? 0 : 1;

                cellsNative[i] = cellNative;
            }

            modelNative.cells = cellsNative;

            modelNative.points = Model.Vertices.Select(vertex => new PointNative(vertex.X, vertex.Y)).ToArray();
            modelNative.anchors = Model.GetAnchors().Select(anchor => Model.Vertices.IndexOf(anchor)).ToArray();

            modelNative.targets = new int[] { Model.Vertices.IndexOf(Model.InputVertex) };
            modelNative.targetPaths = new PointNative[][] { Model.InputPath.Select(point => new PointNative(point.X, point.Y)).ToArray() };

            //output path currently not used
            if (useOutputPath)
            {
                modelNative.inputs = new int[] { Model.Vertices.IndexOf(Model.OutputVertex) };
                modelNative.inputPaths = new PointNative[][] { Model.OutputPath.Select(point => new PointNative(point.X, point.Y)).ToArray() };
            }
            else
            {
                modelNative.inputs = new int[0];
                modelNative.inputPaths = new PointNative[0][];
            }

            return modelNative;
        }
        
        private void ConvertResultToFramePositions(GridResultNative[] result)
        {
            _framePositions = new List<List<Vector>>();

            for (int i = 0; i < result.Length; i++)
            {
                var currentResultPoints = result[i].points;
                List<Vector> points = new List<Vector>();

                for (int j = 0; j < currentResultPoints.Length; j++)
                {
                    points.Add(new Vector(currentResultPoints[j].X, currentResultPoints[j].Y));
                }

                _framePositions.Add(points);
            }
        }
    }
}
