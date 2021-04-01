using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using Microsoft.Win32;
using ShearCell_Interaction.Helper;
using ShearCell_Interaction.Model;
using ShearCell_Interaction.Presenter;
using ShearCell_Interaction.Simulation;
using ShearCell_Interaction.View;
using Path = System.IO.Path;

namespace ShearCell_Interaction.Test
{
    class SimluationPathTests
    {
        public bool Started { get; set; }

        private readonly MetamaterialPresenter _presenter;
        private readonly MetamaterialModel _model;
        private readonly ViewModel _viewModel;
        private readonly MainWindow _mainWindow;

        private readonly List<string> _bitcodes;
        private int _currentSimulationStep;


        private int _currentBitcode;
        private int _currentStep;
        private string _outputPath;
        private DateTime _simulationStoreStartTime;

        //read list bitcoes
        //convert bit code to cell
        //set anchors, input, tracing point
        //start simulation for cell
        //save codes
        //go to next code

        public SimluationPathTests(MetamaterialPresenter presenter, MetamaterialModel model, ViewModel viewModel, MainWindow mainWindow)
        {
            _presenter = presenter;
            _model = model;
            _viewModel = viewModel;
            _mainWindow = mainWindow;

            _bitcodes = new List<string>();

            Started = false;
        }

        public void Start()
        {

            _viewModel.InteractionMode = InteractionMode.AddAnchor;

            var slnDirectory = Path.GetFullPath(@"..\..\..\");
            var dialog = new OpenFileDialog
            {
                DefaultExt = ".txt",
                InitialDirectory = slnDirectory,
                RestoreDirectory = true
            };

            var result = dialog.ShowDialog();

            if (result == false)
                return;

            Started = true;
            ReadBitCodes(dialog.FileName);

            var dimension = (int)Math.Sqrt(_bitcodes[0].Length);

            _outputPath = Path.GetFullPath(@"..\..\..\output\" + dimension + "x" + dimension + DateTime.Now.ToString("_yyyy-dd-M--HH-mm-ss") + @"\");

            if (!Directory.Exists(_outputPath))
                Directory.CreateDirectory(_outputPath);

            NextStep();
        }
        public void NextStep()
        {
            switch (_currentStep)
            {
                case 0:
                    _viewModel.InteractionMode = InteractionMode.AddAnchor;
                    DrawCurrentBitcode();
                    _currentStep++;
                    break;
                case 1:
                    if (_model.InputPath.Count <= 0)
                    {
                        SetInputPathToArc();
                        _currentStep++;
                    }
                    else
                    {
                        _currentStep++;
                        NextStep();
                    }
                    break;
                case 2:
                    AddInputAsTracingPoint();
                    RunSimulation();

                    _currentStep++;
                    break;
                case 3:
                    SaveOutput();

                    ClearModel();
                    _currentBitcode++;
                    _currentStep = 0;

                    if (_currentBitcode >= _bitcodes.Count)
                    {
                        Started = false;
                        _currentBitcode = 0;
                        _viewModel.Redraw();
                    }
                    else
                    {
                        NextStep();
                    }
                    break;
            }
        }

        private void AddInputAsTracingPoint()
        {
            _viewModel.AddTracingPoint(_model.InputVertex);
        }

        public void RunSimulationAndStoreSteps()
        {
            _simulationStoreStartTime = DateTime.Now;

            _currentSimulationStep = 0;

            _model.ResetDeformation();
            _viewModel.ResetTracingPaths();
            _viewModel.Redraw();

            var controller = new OptimizedSimulationController(_model, _viewModel);
            //controller.OnSimulationStep += OnOptimizedSimulationStep;
            controller.StartOptimizedPathSimulation();
        }

        private void RunSimulation()
        {
            var controller = new OptimizedSimulationController(_model, _viewModel);
            controller.StartOptimizedPathSimulation();
        }

        private void OnOptimizedSimulationStep(object sender, EventArgs e)
        {
            SaveCurrentCanvas();
            //SaveStepsToImagesAndClearSteps();
            _currentSimulationStep++;
        }

        private void SaveOutput(string outputNumber = "")
        {
            AddTracingPathToOutputPath();

            //SaveCurrentTracePathToFile();
            SaveImageOfCurrentCanvas(true, outputNumber);

            _viewModel.TracePointsEnabled = false;
            _model.ResetDeformation();
            _viewModel.Redraw();
            _viewModel.TracePointsEnabled = true;

            SaveImageOfCurrentCanvas(false, outputNumber);

            var bitcode = _bitcodes[_currentBitcode];
            var dimension = (int)Math.Sqrt(bitcode.Length);
            var filename = dimension + "x" + dimension + "_" + _currentBitcode.ToString().PadLeft(2, '0') + "_" +
                           _bitcodes[_currentBitcode];

            if (!string.IsNullOrEmpty(outputNumber))
                filename += "_" + outputNumber.PadLeft(2, '0') + "_";

            filename = _outputPath + filename;

            ModelSerializer.WriteToFile(filename, _model, _viewModel);
        }

        private void AddTracingPathToOutputPath()
        {
            if (_viewModel.TracedVertices.Keys.Count > 0)
            {
                var tracedLine = _viewModel.TracedVertices.Values.First().Line;
                _model.OutputPath.AddRange(tracedLine.Points.Select(point => CoordinateConverter.ConvertScreenToGlobalCellCoordinates(new Vector(point.X, point.Y))));
            }
        }

        private void ReadBitCodes(string filename)
        {
            using (var file = new StreamReader(filename))
            {
                string line;

                while ((line = file.ReadLine()) != null)
                {
                    if (string.IsNullOrWhiteSpace(line))
                        continue;

                    _bitcodes.Add(line);
                }
            }
        }

        private void DrawCurrentBitcode()
        {
            var bitcode = _bitcodes[_currentBitcode];

            List<Cell> cells;
            List<Vector> cellPositions;
            BitcodeHelper.CreateCellConfiguration(bitcode, new Vector(0, 0), out cells, out cellPositions);

            for (var i = 0; i < cells.Count; i++)
                _model.AddCell(cells[i], cellPositions[i], new Size(1, 1));

            _viewModel.Redraw();
        }

        
        internal void SetInputPathToArc(double startAngleDegree = 90, double angleDegree = -90, int numberPathPoints = 50)
        {
            _model.ResetDeformation();

            var startAngle = startAngleDegree * MathHelper.DegToRad;
            var angle = angleDegree / numberPathPoints * MathHelper.DegToRad;

            Vector pivot;

            if (_model.InputVertex == null)
            {
                var anchors = _model.GetAnchors();
                pivot = anchors.Last().ToInitialVector();

                _model.InputVertex = _model.Vertices.Find(v => v.Equals(pivot + new Vector(0, 1)));
                _viewModel.HighlightInputVertex();
            }
            else
            {
                pivot = _model.InputVertex.ToInitialVector() - new Vector(0, 1);
            }

            var inputPathPoints = new List<Vector>();
            for (var i = 0; i < numberPathPoints; i++)
            {
                var point = new Vector
                {
                    X = Math.Cos(startAngle + angle * i),
                    Y = Math.Sin(startAngle + angle * i)
                };

                inputPathPoints.Add(Vector.Add(point, pivot));
            }

            _model.InputPath = inputPathPoints;
            _viewModel.DrawInputPath();
        }


        private void SaveCurrentTracePathToFile()
        {
            if (_viewModel.TracedVertices.Count > 0)
            {
                var dimension = (int)Math.Sqrt(_bitcodes[_currentBitcode].Length);
                var filename = dimension + "x" + dimension + "_" + _currentBitcode.ToString().PadLeft(2, '0') + "_" + _bitcodes[_currentBitcode] + "_outputpath.svg";

                filename = _outputPath + filename;

                var line = _viewModel.TracedVertices.First().Value;
                SVGHelper.DrawPath(line.Line.Points.ToList(), filename, 25, 250, 2, Colors.Black);
            }
        }

        private void SaveCurrentCanvas()
        {
            if (_viewModel.InputShapeItems.Count > 0)
            {
                string filename;
                string outputDirectory;

                if (_bitcodes.Count > 0)
                {
                    var dimension = (int)Math.Sqrt(_bitcodes[_currentBitcode].Length);
                    outputDirectory = _outputPath + @"\" + dimension + "x" + dimension + "_" +
                                          _currentBitcode.ToString().PadLeft(2, '0') + "_" +
                                          _bitcodes[_currentBitcode] + @"_steps\";


                    filename = dimension + "x" + dimension + "_" + _currentBitcode.ToString().PadLeft(2, '0') + "_" +
                                   _bitcodes[_currentBitcode] + "_step_" + _currentSimulationStep.ToString().PadLeft(3, '0') + ".jpeg";

                }
                else
                {
                    outputDirectory = Path.GetFullPath(@"..\..\..\output\steps_" + _simulationStoreStartTime.ToString("yyyy-dd-M--HH-mm-ss") + @"\");
                    filename = "step_" + _currentSimulationStep.ToString().PadLeft(3, '0') + ".jpeg";
                }

                if (!Directory.Exists(outputDirectory))
                    Directory.CreateDirectory(outputDirectory);

                filename = outputDirectory + filename;

                var allShapes = new List<Shape>();

                //requires deepcopy because shapes are already children of a a canvas (can only have 1 parent)
                allShapes.AddRange(_viewModel.AnchorShapeItems.Select(BitmapHelper.GetDeepCopyOfShape));
                allShapes.AddRange(_viewModel.DeformedShapeItems.Select(BitmapHelper.GetDeepCopyOfShape));
                allShapes.AddRange(_viewModel.TracingVerticesShapeItems.Select(BitmapHelper.GetDeepCopyOfShape));
                allShapes.Add(BitmapHelper.GetDeepCopyOfShape(_mainWindow.SetInputPath));

                BitmapHelper.RenderBitmap(allShapes, filename, 250, 250, 100, (int)_mainWindow.MainCanvas.ActualHeight - 200);
            }
        }

        private void SaveImageOfCurrentCanvas(bool deformed, string outputNumber = "")
        {
            if (_viewModel.InputShapeItems.Count > 0)
            {
                var dimension = (int)Math.Sqrt(_bitcodes[_currentBitcode].Length);

                var allShapes = new List<Shape>();

                //requires deepcopy because shapes are already children of a a canvas (can only have 1 parent)
                allShapes.AddRange(_viewModel.AnchorShapeItems.Select(BitmapHelper.GetDeepCopyOfShape));
                allShapes.AddRange(_viewModel.DeformedShapeItems.Select(BitmapHelper.GetDeepCopyOfShape));
                allShapes.AddRange(_viewModel.TracingVerticesShapeItems.Select(BitmapHelper.GetDeepCopyOfShape));
                allShapes.Add(BitmapHelper.GetDeepCopyOfShape(_mainWindow.SetInputPath));

                var filename = dimension + "x" + dimension + "_" + _currentBitcode.ToString().PadLeft(2, '0') + "_" +
                               _bitcodes[_currentBitcode];

                if (!string.IsNullOrEmpty(outputNumber))
                    filename += "_" + outputNumber.PadLeft(2, '0') + "_";

                filename += (deformed ? "_01_deformed" : "_00_original") + ".jpeg";

                filename = _outputPath + filename;

                BitmapHelper.RenderBitmap(allShapes, filename, 250, 250, 100, (int)_mainWindow.MainCanvas.ActualHeight - 200);
            }
        }
        private void ClearModel()
        {
            _presenter.ResetDeformation();
            _presenter.Reset();
            _model.Clear();
        }

    }
}
