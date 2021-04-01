using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Windows;
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
    public class AutomaticPathSimulationTests
    {
        public bool Started { get; set; }

        private readonly MetamaterialPresenter _presenter;
        private readonly MetamaterialModel _model;
        private readonly ViewModel _viewModel;
        private readonly MainWindow _mainWindow;

        private List<List<string>> _bitCodeLists;
        private List<int> _degreeOfFreedomList;
        private int _currentSimulationStep;

        private List<string> _currentBitCodeList;
        private int _currentBitcodeIndex;
        private int _currentBitCodeListIndex;
        private int _currentDof;

        private string _outputPath;

        private int _currentAnchorCounter;
        private int _currentInputPath;
        private int _currentInputPathScale;

        private int[] stepsToSave = { 0, 7, 14, 21, 28, 35, 39 };
        private string _currentFileName;
        private List<Vertex> _alreadyVisitedAnchors;

        //read list bitcoes
        //convert bit code to cell
        //set anchors, input, tracing point
        //start simulation for cell
        //save codes
        //go to next code

        public AutomaticPathSimulationTests(MetamaterialPresenter presenter, MetamaterialModel model, ViewModel viewModel, MainWindow mainWindow)
        {
            _presenter = presenter;
            _model = model;
            _viewModel = viewModel;
            _mainWindow = mainWindow;

            Started = false;
        }

        public void Start()
        {
            _bitCodeLists = new List<List<string>>();
            _degreeOfFreedomList = new List<int>();
            _alreadyVisitedAnchors = new List<Vertex>();

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

            var shortFileName = Path.GetFileNameWithoutExtension(dialog.FileName);

            //var dimension = (int)Math.Sqrt(_currentBitCodeList[0].Length);
            //_outputPath = Path.GetFullPath(@"..\..\..\output\" + dimension + "x" + dimension + DateTime.Now.ToString("_yyyy-dd-M--HH-mm-ss") + @"\");
            _outputPath = Path.GetFullPath(@"..\..\..\output\" + shortFileName + "_" + DateTime.Now.ToString("_yyyy -dd-M--HH-mm-ss") + @"\");

            if (!Directory.Exists(_outputPath))
                Directory.CreateDirectory(_outputPath);

            _currentBitCodeList = _bitCodeLists[_currentBitCodeListIndex];
            _currentDof = _degreeOfFreedomList[_currentBitCodeListIndex];

            NextBitcode();
        }
        private void NextBitcode()
        {
            Debug.WriteLine("Next Bitcode is " + _currentBitcodeIndex + " in list " + _currentBitCodeListIndex + ". " + DateTime.Now);
            ClearModel();

            DrawCurrentBitcode();
            SetInputPoint();
            SetTracingPoint();
            AddInputAsTracingPoint();

            _currentInputPath = 0;
            _currentInputPathScale = 0;
            _currentAnchorCounter = 0;
            _alreadyVisitedAnchors.Clear();

            StartWithNextAnchor();
        }

        private void SetInputPoint()
        {
            _model.InputVertex = _model.Vertices[0];
        }

        public void StartWithNextAnchor()
        {
            RemoveAllAnchors();
            _model.ResetDeformation();
            _viewModel.ResetTracingPaths();
            _viewModel.Redraw();

            var nextRigidCellFound = false;

            while (!nextRigidCellFound && _currentAnchorCounter < _model.Cells.Count)
            {
                var cell = _model.Cells[_currentAnchorCounter];

                if (_alreadyVisitedAnchors.Contains(cell.CellVertices[0]) && _alreadyVisitedAnchors.Contains(cell.CellVertices[1]))
                {
                    _currentAnchorCounter++;
                    continue;
                }

                if (cell is RigidCell)
                {
                    nextRigidCellFound = true;
                }
                else
                {
                    _currentAnchorCounter++;
                }
            }

            if (nextRigidCellFound)
            {
                var rigidCell = (RigidCell)_model.Cells[_currentAnchorCounter];

                rigidCell.CellVertices[0].IsAnchor = true;
                rigidCell.CellVertices[1].IsAnchor = true;

                _model.PropagateAnchors(rigidCell.CellVertices[0]);
                _model.PropagateAnchors(rigidCell.CellVertices[1]);

                _alreadyVisitedAnchors.AddRange(_model.GetAnchors());

                var cellWithTracingPoint = _model.Cells.Find(cell => cell.CellVertices.Contains(_viewModel.TracedVertices.First().Key));

                if (!cellWithTracingPoint.CanMove())
                {
                    RemoveAllAnchors();
                    _currentAnchorCounter++;
                    StartWithNextAnchor();
                    return;
                }
               
                RemoveAllAnchors();

                rigidCell.CellVertices[0].IsAnchor = true;
                rigidCell.CellVertices[1].IsAnchor = true;

                if (rigidCell.CellVertices.Contains(_model.InputVertex) && !rigidCell.CanMove())
                {
                    _currentAnchorCounter++;
                    StartWithNextAnchor();
                    return;
                }
                

                NextInputPath();
                return;
            }

            _currentAnchorCounter = 0;

            if (_currentBitCodeListIndex >= _currentBitCodeList.Count)
            {
                Console.WriteLine("Done at " + DateTime.Now.ToString("F"));
                Started = false;
                _currentBitcodeIndex = 0;
                _currentBitCodeListIndex = 0;
                _currentDof = 0;
                _viewModel.Redraw();
            }
            else
            {
                _currentBitcodeIndex++;

                if (_currentBitcodeIndex >= _currentBitCodeList.Count)
                {
                    _currentBitcodeIndex = 0;
                    _currentBitCodeListIndex++;

                    if (_currentBitCodeListIndex < _bitCodeLists.Count)
                    {
                        _currentBitCodeList = _bitCodeLists[_currentBitCodeListIndex];
                        _currentDof = _degreeOfFreedomList[_currentBitCodeListIndex];

                        NextBitcode();
                    }
                }
                else
                {
                    NextBitcode();
                }
            }

        }

        private void NextInputPath()
        {
            SetCurrentInputPath();
            _viewModel.ResetTracingPaths();
            _viewModel.Redraw();

            SetCurrentFilename();

            _currentSimulationStep = 0;

            var controller = new OptimizedSimulationController(_model, _viewModel);
            //controller.OnSimulationStep += OnOptimizedSimulationStep;
            //controller.OnSimulationCompleted += OnSimulationCompleted;
            controller.StartOptimizedPathSimulation();
        }

        private void SetCurrentFilename()
        {
            var bitcode = _currentBitCodeList[_currentBitcodeIndex];
            var dimension = (int)Math.Sqrt(bitcode.Length);
            _currentFileName = dimension.ToString();
            _currentFileName += "_" + _currentDof;
            _currentFileName += "_" + _currentBitcodeIndex.ToString().PadLeft(4, '0');
            _currentFileName += "_" + _currentBitCodeList[_currentBitcodeIndex];
            _currentFileName += "_a" + _currentAnchorCounter;
            _currentFileName += "_i" + _currentInputPath;
            _currentFileName += "_s" + _currentInputPathScale;
        }

        private void OnOptimizedSimulationStep(object sender, EventArgs e)
        {
            if (stepsToSave.Contains(_currentSimulationStep))
                SaveCurrentCanvas();

            _currentSimulationStep++;
        }

        private void OnSimulationCompleted(object sender, EventArgs e)
        {
            SaveOutput();

            if (_currentInputPathScale == 1)
            {
                _currentInputPathScale = 0;
                _currentInputPath++;
            }
            else
            {
                _currentInputPathScale = 1;
            }

            if (_currentInputPath >= 4)
            {
                _currentInputPath = 0;
                _currentAnchorCounter++;
                StartWithNextAnchor();
            }
            else
            {
                NextInputPath();
            }
        }

        private void RemoveAllAnchors()
        {
            foreach (var vertex in _model.Vertices)
            {
                vertex.IsAnchor = false;
            }
        }

        private void SaveOutput()
        {
            //AddTracingPathToOutputPath();

            //SaveCurrentTracePathToFile();
            SaveImageOfCurrentCanvas(true);

            _viewModel.TracePointsEnabled = false;
            _model.ResetDeformation();
            _viewModel.Redraw();
            _viewModel.TracePointsEnabled = true;

            SaveImageOfCurrentCanvas(false);

            var modelFileName = _outputPath + _currentFileName;

            ModelSerializer.WriteToFile(modelFileName, _model, _viewModel);
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
                List<string> bitCodes = null;

                while ((line = file.ReadLine()) != null)
                {
                    if (string.IsNullOrWhiteSpace(line))
                        continue;

                    if (line.StartsWith("#"))
                    {
                        var currentDof = int.Parse(line.Substring(2, 1));
                        _degreeOfFreedomList.Add(currentDof);
                        bitCodes = new List<string>();
                        _bitCodeLists.Add(bitCodes);
                    }
                    else
                    {
                        bitCodes.Add(line);
                    }
                }
            }
        }

        private void DrawCurrentBitcode()
        {
            var bitcode = _currentBitCodeList[_currentBitcodeIndex];

            List<Cell> cells;
            List<Vector> cellPositions;
            BitcodeHelper.CreateCellConfiguration(bitcode, new Vector(0, 0), out cells, out cellPositions);

            for (var i = 0; i < cells.Count; i++)
                _model.AddCell(cells[i], cellPositions[i], new Size(1, 1));

            _viewModel.Redraw();
        }

        private void AddInputAsTracingPoint()
        {
            _viewModel.AddTracingPoint(_model.InputVertex);
        }

        private void SetTracingPoint()
        {
            Vertex topRight = _model.Vertices[0];

            for (var i = 1; i < _model.Vertices.Count; i++)
            {
                var vertex = _model.Vertices[i];

                if (vertex.X > topRight.X && vertex.Y > topRight.Y)
                    topRight = vertex;
            }

            _viewModel.AddTracingPoint(topRight);
        }

        private void SetCurrentInputPath()
        {
            switch (_currentInputPath)
            {
                case 0:
                    SetInputPathToLine(_currentInputPathScale + 1);
                    break;
                case 1:
                    SetInputPathToArc(_currentInputPathScale + 1);
                    break;
                case 2:
                    SetInputPathToCorner(_currentInputPathScale + 1);
                    break;
                case 3:
                    SetInputPathToSpiral(_currentInputPathScale + 1);
                    break;

            }
        }

        private void SetInputPathToArc(double scale, double startAngleDegree = 90, double angleDegree = -90, int numberPathPoints = 40)
        {
            _model.ResetDeformation();

            var startAngle = startAngleDegree * MathHelper.DegToRad;
            var angle = angleDegree / numberPathPoints * MathHelper.DegToRad;

            Vector pivot = _model.InputVertex.ToInitialVector() - new Vector(0, 1 * scale);

            var inputPathPoints = new List<Vector>();
            for (var i = 0; i < numberPathPoints; i++)
            {
                var point = new Vector
                {
                    X = Math.Cos(startAngle + angle * i) * scale,
                    Y = Math.Sin(startAngle + angle * i) * scale
                };

                inputPathPoints.Add(Vector.Add(point, pivot));
            }

            _model.InputPath = inputPathPoints;
            _viewModel.DrawInputPath();
        }

        private void SetInputPathToLine(int scale)
        {
            var points = new List<Point>();
            points.Add(new Point(_model.InputVertex.X, _model.InputVertex.Y));
            points.Add(new Point(_model.InputVertex.X + 1 * scale, _model.InputVertex.Y - 1 * scale));

            var sampledPoints = PolylineHelper.SamplePolygon(points, MetamaterialPresenter.GetPathSamplingFactor());

            var inputPathPoints = new List<Vector>();
            foreach (var sampledPoint in sampledPoints)
            {
                inputPathPoints.Add(new Vector(sampledPoint.X, sampledPoint.Y));
            }

            _model.InputPath = inputPathPoints;
            _viewModel.DrawInputPath();
        }


        private void SetInputPathToCorner(int scale)
        {
            var points = new List<Point>();
            points.Add(new Point(_model.InputVertex.X, _model.InputVertex.Y));
            points.Add(new Point(_model.InputVertex.X + 1 * scale, _model.InputVertex.Y));
            points.Add(new Point(_model.InputVertex.X + 1 * scale, _model.InputVertex.Y - 1 * scale));

            var sampledPoints = PolylineHelper.SamplePolygon(points, MetamaterialPresenter.GetPathSamplingFactor());

            var inputPathPoints = new List<Vector>();
            foreach (var sampledPoint in sampledPoints)
            {
                inputPathPoints.Add(new Vector(sampledPoint.X, sampledPoint.Y));
            }

            _model.InputPath = inputPathPoints;
            _viewModel.DrawInputPath();
        }


        private void SetInputPathToSpiral(int scale)
        {
            var points = new List<Point>();
            points.Add(new Point(_model.InputVertex.X, _model.InputVertex.Y));
            points.Add(new Point(_model.InputVertex.X, _model.InputVertex.Y + 0.5 * scale));
            points.Add(new Point(_model.InputVertex.X - 0.5 * scale, _model.InputVertex.Y + 0.5 * scale));
            points.Add(new Point(_model.InputVertex.X - 0.5 * scale, _model.InputVertex.Y - 0.5 * scale));
            points.Add(new Point(_model.InputVertex.X + 0.5 * scale, _model.InputVertex.Y - 0.5 * scale));
            points.Add(new Point(_model.InputVertex.X + 0.5 * scale, _model.InputVertex.Y + 0.5 * scale));

            var sampledPoints = PolylineHelper.SamplePolygon(points, MetamaterialPresenter.GetPathSamplingFactor());

            var inputPathPoints = new List<Vector>();
            foreach (var sampledPoint in sampledPoints)
            {
                inputPathPoints.Add(new Vector(sampledPoint.X, sampledPoint.Y));
            }

            _model.InputPath = inputPathPoints;
            _viewModel.DrawInputPath();
        }

        private void SaveCurrentCanvas()
        {
            if (_viewModel.InputShapeItems.Count > 0)
            {
                var outputDirectory = _outputPath + @"\_steps\";

                if (!Directory.Exists(outputDirectory))
                    Directory.CreateDirectory(outputDirectory);

                var filename = _currentFileName + "_s" + _currentSimulationStep.ToString().PadLeft(3, '0') + ".jpeg";
                filename = outputDirectory + filename;

                var allShapes = new List<Shape>();

                //requires deepcopy because shapes are already children of a a canvas (can only have 1 parent)
                allShapes.AddRange(_viewModel.AnchorShapeItems.Select(BitmapHelper.GetDeepCopyOfShape));
                allShapes.AddRange(_viewModel.DeformedShapeItems.Select(BitmapHelper.GetDeepCopyOfShape));
                allShapes.AddRange(_viewModel.TracingVerticesShapeItems.Select(BitmapHelper.GetDeepCopyOfShape));
                allShapes.Add(BitmapHelper.GetDeepCopyOfShape(_mainWindow.SetInputPath));

                BitmapHelper.RenderBitmap(allShapes, filename, 250, 250, 100, (int)_mainWindow.MainCanvas.ActualHeight - 200, 2);
            }
        }

        private void SaveImageOfCurrentCanvas(bool deformed)
        {
            if (_viewModel.InputShapeItems.Count > 0)
            {
                var allShapes = new List<Shape>();

                //requires deepcopy because shapes are already children of a a canvas (can only have 1 parent)
                allShapes.AddRange(_viewModel.AnchorShapeItems.Select(BitmapHelper.GetDeepCopyOfShape));
                allShapes.AddRange(_viewModel.DeformedShapeItems.Select(BitmapHelper.GetDeepCopyOfShape));
                allShapes.AddRange(_viewModel.TracingVerticesShapeItems.Select(BitmapHelper.GetDeepCopyOfShape));
                allShapes.Add(BitmapHelper.GetDeepCopyOfShape(_mainWindow.SetInputPath));

                var imageFileName = _currentFileName + (deformed ? "_1-deformed" : "_0-original") + ".jpeg";
                imageFileName = _outputPath + imageFileName;

                BitmapHelper.RenderBitmap(allShapes, imageFileName, 250, 250, 100, (int)_mainWindow.MainCanvas.ActualHeight - 200, 2);
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