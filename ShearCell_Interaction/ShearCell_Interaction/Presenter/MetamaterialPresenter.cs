using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using System.Windows;
using System.Windows.Forms.ComponentModel.Com2Interop;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using Microsoft.Win32;
using ShearCell_Interaction.Helper;
using ShearCell_Interaction.Model;
using ShearCell_Interaction.Optimisation;
using ShearCell_Interaction.Randomizer;
using ShearCell_Interaction.Simulation;
using ShearCell_Interaction.View;
using Path = System.IO.Path;

namespace ShearCell_Interaction.Presenter
{
    public class MetamaterialPresenter
    {
        public static int GetPathSamplingFactor()
        {
            return 40;
        }

        //private const int PathDeformationSteps = 100;
        private const int PathDeformationSegmentLength = 2; //in screen units, i.e. pixels
        private const int PathDeformationTimeStep = 100;

        public MetamaterialModel Model { get; set; }

        private readonly ViewModel _viewModel;

        //private readonly DeformationController _deformationController;
        private OptimizedSimulationController _simulationController;
        private SimulationVisualiser _simulationVisualiser;
        
        private HierarchicalOptimization _hierarchicalOptimizationController;

        private BackgroundWorker _worker;

        public MetamaterialPresenter(MetamaterialModel model, ViewModel viewModel)
        {
            Model = model;
            _viewModel = viewModel;

            //_deformationController = new DeformationController(_model);
            _simulationController = new OptimizedSimulationController(Model, _viewModel);
            _simulationVisualiser = new SimulationVisualiser( _viewModel);

            _hierarchicalOptimizationController = new HierarchicalOptimization(Model, _viewModel, this);
        }

        internal void CreateCell(Vector screenPosition, Type cellType, Size size, bool updateConstraints = true)
        {
            //if (_deformationController.IsSimulating)
            //    return;

            if (!_simulationVisualiser.IsPaused)
                _simulationVisualiser.PlayPauseSimulation();

            var indexPosition = CoordinateConverter.ConvertScreenToCellIndex(screenPosition);
            Debug.WriteLine("screen: " + screenPosition + "  index: " + indexPosition);

            if (cellType == typeof(ShearCell))
                Model.AddCell(new ShearCell(), indexPosition, size, updateConstraints);
            else if (cellType == typeof(RigidCell))
                Model.AddCell(new RigidCell(), indexPosition, size, updateConstraints);

            _viewModel.CurrentDoF = Model.CurrentDoF.ToString();
            _viewModel.Redraw();
        }

        internal void DeleteCell(Vector screenPosition)
        {
            //if (_deformationController.IsSimulating)
            //    return;

            if (!_simulationVisualiser.IsPaused)
                _simulationVisualiser.PlayPauseSimulation();

            var indexPosition = CoordinateConverter.ConvertScreenToCellIndex(screenPosition);
            Model.DeleteCell(indexPosition);

            _viewModel.CurrentDoF = Model.CurrentDoF.ToString();
            _viewModel.Redraw();
        }

        internal void AddAnchor(Vector ellipseScreenPosition)
        {
            var gridPosition = CoordinateConverter.ConvertScreenToGlobalCellCoordinates(ellipseScreenPosition);

            Model.ToggleAnchor(gridPosition);
            _viewModel.Redraw();
        }
        internal void Reset()
        {
            Model.Reset();
            _viewModel.Reset();
            _viewModel.Redraw();
        }


        internal void Clear()
        {
            Model.Clear();
            _viewModel.Reset();
            _viewModel.Redraw();
        }

        internal void ResetDeformation(bool resetTracingPoints = true)
        {
            if (resetTracingPoints)
            {
                _viewModel.ResetTracingPaths();
            }
            else
            {
                _viewModel.TracePointsEnabled = false;
            }

            _simulationVisualiser.ResetSimulationStep();
            Model.ResetDeformation();
            _viewModel.Redraw();

            _viewModel.TracePointsEnabled = true;
        }

        internal void AddTracingPoint(Vector vertexCenterScreenPosition)
        {
            var globalGridPosition = CoordinateConverter.ConvertScreenToGlobalCellCoordinates(vertexCenterScreenPosition);
            var existingVertex = Model.Vertices.Find(current => MathHelper.IsEqualDouble(current.X, globalGridPosition.X) && MathHelper.IsEqualDouble(current.Y, globalGridPosition.Y));

            if (existingVertex == null)
                return;

            _viewModel.AddTracingPoint(existingVertex);
        }

        internal void TraceAllPoints()
        {
            foreach (var vertex in Model.Vertices)
            {
                if (!vertex.IsAnchor && !vertex.Equals(Model.InputVertex))
                    _viewModel.AddTracingPoint(vertex);
            }
        }

        internal void SetSimulationOutputVertex(Vector vertexCenterScreenPosition)
        {
            //_model.DeformedCells.Clear();

            var globalGridPosition = CoordinateConverter.ConvertScreenToGlobalCellCoordinates(vertexCenterScreenPosition);
            var existingVertex = Model.Vertices.Find(current => MathHelper.IsEqualDouble(current.X, globalGridPosition.X) && MathHelper.IsEqualDouble(current.Y, globalGridPosition.Y));

            if (existingVertex == null)
                return;

            Model.OutputVertex = existingVertex;
            _viewModel.HighlightOutputVertex();
        }

        internal void SetSimulationInputVertex(Vector vertexCenterScreenPosition)
        {
            //_model.DeformedCells.Clear();

            var globalGridPosition =
                CoordinateConverter.ConvertScreenToGlobalCellCoordinates(vertexCenterScreenPosition);
            var existingVertex = Model.Vertices.Find(current =>
                MathHelper.IsEqualDouble(current.X, globalGridPosition.X) &&
                MathHelper.IsEqualDouble(current.Y, globalGridPosition.Y));

            if (existingVertex == null)
                return;

            Model.InputVertex = existingVertex;
            _viewModel.HighlightInputVertex();
        }

        internal void SetOutputPath(PathGeometry pathGeometry, Matrix transform)
        {
            var gridPathPoints = PolylineHelper.SamplePolygon(pathGeometry, GetPathSamplingFactor(), transform);

            Model.OutputPath = gridPathPoints;
            _viewModel.DrawOutputPath();
        }

        internal void SetInputPath(PathGeometry pathGeometry, Matrix transform)
        {
            var gridPathPoints = PolylineHelper.SamplePolygon(pathGeometry, GetPathSamplingFactor(), transform);

            Model.InputPath = gridPathPoints;
            _viewModel.DrawInputPath();
        }
        internal void SetInputPath(List<Vector> points, Matrix transform)
        {
            Model.InputPath = points;
            _viewModel.DrawInputPath();
        }

        //internal void StartPathSimulation()
        //{
        //    _viewModel.InteractionMode = InteractionMode.Simulate;

        //    if (_model.InputVertex == null)
        //        return;

        //    if (_model.InputPath.Count < 1)
        //        return;

        //    _worker = new BackgroundWorker();
        //    _worker.DoWork += OnDeformInBackground;
        //    _worker.RunWorkerCompleted += OnCompletedVisualizeDeformation;

        //    Debug.WriteLine("            RUN worker");
        //    _worker.RunWorkerAsync(0);
        //}

        //internal void UpdateDeformation(Vector screenPosition)
        //{
        //    var targetPoint = CoordinateConverter.ConvertScreenToGlobalCellCoordinates(screenPosition);

        //    Deform(targetPoint);
        //    _viewModel.Redraw();
        //}

        //private void Deform(Vector targetVector)
        //{
        //    if (_deformationController.IsSimulating)
        //        return;

        //    if (_model.InputVertex == null)
        //        return;

        //    var targetOffset = Vector.Subtract(new Vector(targetVector.X, targetVector.Y), _model.InputVertex.ToVector());

        //    Debug.IndentLevel = 0;
        //    Debug.WriteLine("PRESENTER.UpdateDeformation()");

        //    _deformationController.Deform(targetOffset);
        //}

        //private void OnDeformInBackground(object sender, DoWorkEventArgs e)
        //{
        //    var worker = sender as BackgroundWorker;
        //    if (worker == null)
        //        return;

        //    var currentPointIndex = (int)e.Argument;
        //    e.Result = currentPointIndex;

        //    Deform(_model.InputPath[currentPointIndex]);
        //    Thread.Sleep(PathDeformationTimeStep);

        //    Debug.WriteLine("Deform path point " + currentPointIndex);
        //}

        //private void OnCompletedVisualizeDeformation(object sender, RunWorkerCompletedEventArgs e)
        //{
        //    var currentPointIndex = (int)e.Result;

        //    _viewModel.Redraw();

        //    Debug.WriteLine("Done visualizing path point " + currentPointIndex);

        //    if (currentPointIndex < _model.InputPath.Count - 1)
        //        _worker.RunWorkerAsync(currentPointIndex + 1);
        //    else
        //        _model.InputPath.Clear();
        //}

        internal void ExportCurrentConfiguration(string filename = null)
        {
            var filepath = filename;

            if (filename == null)
            {
                var now = DateTime.Now;
                var defaultFilename = $"cells__{now.Year}_{now.Month}_{now.Day}_{now.Hour}.{now.Minute}.{now.Second}.txt";
                var slnDirectory = Path.GetFullPath(@"..\..\..\");

                var dialog = new SaveFileDialog
                {
                    DefaultExt = ".txt",
                    FileName = defaultFilename,
                    InitialDirectory = slnDirectory
                };

                var result = dialog.ShowDialog();

                if (result == false)
                    return;

                filepath = dialog.FileName;
            }

            //if (_viewModel.TracedVertices.Keys.Count > 0)
            //{
            //    var tracedLine = _viewModel.TracedVertices.Values.First().Line;
            //    _model.OutputPath.AddRange(tracedLine.Points.Select(point => CoordinateConverter.ConvertScreenToGlobalCellCoordinates(new Vector(point.X, point.Y))));
            //}

            ModelSerializer.WriteToFile(filepath, Model, _viewModel);
            _viewModel.Redraw();
        }

        internal void ImportConfiguration()
        {
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

            Model.Clear();
            _viewModel.Reset();

            var filename = dialog.FileName;
            ModelSerializer.ReadFromFile(filename, Model, _viewModel);

            Model.UpdateConstraintsGraph();
            _viewModel.CurrentDoF = Model.CurrentDoF.ToString();
            _viewModel.InteractionMode = InteractionMode.Simulate;

            //_viewModel.RedrawTracingPoints();
            //TraceAllPoints();
            _viewModel.Redraw();

            _viewModel.HighlightInputVertex();
            _viewModel.DrawInputPath();
            _viewModel.HighlightOutputVertex();
            _viewModel.DrawOutputPath();
        }
        
        internal void SaveCurrentCanvas(MainWindow window)
        {
            if (_viewModel.InputShapeItems.Count > 0)
            {
                var slnDirectory = Path.GetFullPath(@"..\..\..\");
                var dialog = new SaveFileDialog
                {
                    DefaultExt = ".jpeg",
                    InitialDirectory = slnDirectory,
                    RestoreDirectory = true
                };

                var result = dialog.ShowDialog();

                if (result == false)
                    return;

                var filename = dialog.FileName;

                if (!filename.EndsWith(".jpeg"))
                {
                    if (filename.Contains("."))
                        filename = filename.Substring(0, filename.LastIndexOf(".", StringComparison.Ordinal)) + ".jpeg";
                    else
                        filename += ".jpeg";
                }

                var allShapes = new List<Shape>();

                //requires deepcopy because shapes are already children of a a canvas (can only have 1 parent)
                allShapes.AddRange(_viewModel.AnchorShapeItems.Select(BitmapHelper.GetDeepCopyOfShape));
                allShapes.AddRange(_viewModel.DeformedShapeItems.Select(BitmapHelper.GetDeepCopyOfShape));
                allShapes.AddRange(_viewModel.TracingVerticesShapeItems.Select(BitmapHelper.GetDeepCopyOfShape));

                allShapes.Add(BitmapHelper.GetDeepCopyOfShape(window.SetInputPath));

                BitmapHelper.RenderBitmap(allShapes, filename, 250, 250, 100, (int)window.MainCanvas.ActualHeight - 200, 2);
            }
        }

        internal void RandomizeCellConfiguration()
        {
            Reset();

            var randomizer = new FourNeighborAutomaton(Model, _viewModel);
            randomizer.SetInitialCells(new List<Vector> { new Vector(10, 10), new Vector(3, 6), new Vector(8, 14) });
            randomizer.Run();
        }

        internal void RandomizeAllCellConfigurations()
        {
            for (var dimension = 4; dimension <= 4; dimension++)
            {
                var randomizer = new FilteredRandomizer(dimension, dimension, _viewModel, Model);
                randomizer.GeneratePersistedVariations();
            }

            //for (var height = 2; height <= 5; height++)
            //{
            //    for (width = 2; width <= 5; width++)
            //    {
            //        var randomizer = new FilteredRandomizer(width, height, _viewModel, _model);
            //        randomizer.GeneratePersistedVariations();
            //    }
            //}
        }

        private SuccessiveRandomizer _successiveRandomizer;

        internal void AddRandomCell()
        {
            //_model.Clear();
            //_viewModel.Redraw();

            //TestSuccsessiveRandomizer();

            _viewModel.InteractionMode = InteractionMode.Test;

            if (Model.Vertices.Count == 0)
            {
                var xPosition = (ViewModel.WindowWidth / ViewModel.CellScale) / 2;
                var yPosition = (ViewModel.WindowHeight / ViewModel.CellScale) / 2;
                Model.AddCell(new RigidCell(), new Vector(xPosition, yPosition), new Size(1, 1));
            }

            if (Model.GetAnchors().Count == 0)
            {
                Model.ToggleAnchor(Model.Vertices[0].ToVector());
                Model.ToggleAnchor(Model.Vertices[1].ToVector());
            }

            if (_successiveRandomizer == null)
                _successiveRandomizer = new SuccessiveRandomizer(Model, _viewModel);

            _successiveRandomizer.AddNextCell();
            _viewModel.Redraw();
        }

        internal void TestSuccsessiveRandomizer()
        {
            _viewModel.InteractionMode = InteractionMode.Test;

            var xPosition = (ViewModel.WindowWidth / ViewModel.CellScale) / 2;
            var yPosition = (ViewModel.WindowHeight / ViewModel.CellScale) / 2;
            Model.AddCell(new RigidCell(), new Vector(xPosition, yPosition), new Size(1, 1));

            Model.ToggleAnchor(Model.Vertices[0].ToVector());
            Model.ToggleAnchor(Model.Vertices[1].ToVector());
            //_viewModel.Redraw();

            _successiveRandomizer = new SuccessiveRandomizer(Model, _viewModel);

            //for (var i = 0; i < 15 * 15 - 1; i++)
                _successiveRandomizer.AddNextCell();

            _viewModel.Redraw();
        }

        internal void ToggleConnectedComponents()
        {
            _viewModel.IsDoFVisible = !_viewModel.IsDoFVisible;
            _viewModel.Redraw();
        }

        internal void ToggleMarkNonShearingCells()
        {
            _viewModel.AreNonShearingCellsMarked = !_viewModel.AreNonShearingCellsMarked;
            _viewModel.Redraw();
        }

        internal void StartOptimizedPathSimulation()
        {
            _viewModel.InteractionMode = InteractionMode.Simulate;
            _viewModel.ResetTracingPaths();

            if (_simulationController == null)
                _simulationController = new OptimizedSimulationController(_viewModel.Model, _viewModel);
            //else
            //    _simulationController.Clear();

            //_simulationController.LoopAnimation = _viewModel.LoopAnimation;

            _simulationController.Model = _viewModel.Model;

            _simulationController.OnSimulationInfoAvailable -= OnSimulationInfoAvailable;
            _simulationController.OnSimulationInfoAvailable += OnSimulationInfoAvailable;

            _simulationController.StartOptimizedPathSimulation();
        }

        internal void StartOptimization()
        {
            var simulationModel = new SimulationModel(Model, _viewModel);
            var optimizationController = new IterativeCellOptimizationController(simulationModel);
            optimizationController.StartOptimization();

            Model = simulationModel.Model;
            _viewModel.Model = Model;

            _viewModel.Redraw();
        }
        public void StartHierarchicalOptimization()
        {
            _hierarchicalOptimizationController.SetupLogger();
            _hierarchicalOptimizationController.Start();
        }

        private void OnSimulationInfoAvailable(object sender, SimulationInfoEventArgs e)
        {
            _simulationController.OnSimulationInfoAvailable -= OnSimulationInfoAvailable;

            if (_simulationVisualiser == null)
                _simulationVisualiser = new SimulationVisualiser(_viewModel);
            else
                _simulationVisualiser.Clear();

            _simulationVisualiser.LoopAnimation = _viewModel.LoopAnimation;
            _simulationVisualiser.OptimizationInfoAvailable(e.FramePosition);

           if (Model.InputVertex != null)
            {
                var inputIndex = Model.Vertices.IndexOf(Model.InputVertex);

                if (inputIndex >= 0)
                {
                    Console.WriteLine("input index is " + inputIndex);
                    var inputPositions = e.FramePosition.Select(frame => frame[inputIndex]).ToList();

                    if (inputPositions.Count == Model.InputPath.Count)
                    {
                        var distance = PolylineHelper.Distance(inputPositions, Model.InputPath);
                        _viewModel.InputPathDistance = distance;
                    }
                }

            }

            if (Model.OutputVertex != null)
            {
                var outputIndex = Model.Vertices.IndexOf(Model.OutputVertex);

                if (outputIndex >= 0)
                {
                    var outputPositions = e.FramePosition.Select(frame => frame[outputIndex]).ToList();

                    if (outputPositions.Count == Model.OutputPath.Count)
                    {
                        var distance = PolylineHelper.Distance(outputPositions, Model.OutputPath);
                        _viewModel.OutputPathDistance = distance;
                    }
                }
            }
        }

        internal void ToggleSimulationPlaying()
        {
            if(!_simulationVisualiser.SimulationAvailable)
                return;
            
            _simulationVisualiser.LoopAnimation = _viewModel.LoopAnimation;
            _simulationVisualiser.PlayPauseSimulation();
        }

        internal void StepSimulationStepsBack(int numberFrames)
        {
            if (!_simulationVisualiser.SimulationAvailable)
                return;

            _simulationVisualiser.StepSimulation(-numberFrames);
        }

        internal void StepSimulationStepsForward(int numberFrames)
        {
            if (!_simulationVisualiser.SimulationAvailable)
                return;

            _simulationVisualiser.StepSimulation(numberFrames);
        }

        internal void Scale(int scaleFactor)
        {
            Model.Scale(scaleFactor, _viewModel.ScalePathWithCells);

            Model.UpdateConstraintsGraph();
            _viewModel.CurrentDoF = Model.CurrentDoF.ToString();
            _viewModel.DrawInputPath();
            _viewModel.DrawOutputPath();
            _viewModel.Redraw();
        }

        internal void DecreaseScale()
        {
            Model.DecreaseScale(_viewModel.ScalePathWithCells);

            Model.UpdateConstraintsGraph();
            _viewModel.CurrentDoF = Model.CurrentDoF.ToString();
            _viewModel.DrawInputPath();
            _viewModel.DrawOutputPath();
            _viewModel.Redraw();
        }
    }
}
