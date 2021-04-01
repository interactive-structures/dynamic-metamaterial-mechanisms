using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Documents;
using System.Windows.Media;
using ShearCell_Interaction.Helper;
using ShearCell_Interaction.Model;
using ShearCell_Interaction.Presenter;
using ShearCell_Interaction.Simulation;
using ShearCell_Interaction.View;

namespace ShearCell_Interaction.Optimisation
{
    public class HierarchicalOptimization
    {
        private MetamaterialModel _model;
        private readonly ViewModel _viewModel;
        private readonly MetamaterialPresenter _presenter;

        private List<Cell> _originalCells;
        private List<Vector> _inputPath;
        private Vertex _inputVertex;
        private List<Vector> _outputPath;
        private Vertex _outputVertex;
        private List<Vertex> _originalAnchors;

        private const int NUM_MAX_OPTI_RESTARTS = 10;
        private const int NUM_PARALLEL_OPTI_RUNS = 1;

        public HierarchicalOptimization(MetamaterialModel model, ViewModel viewModel, MetamaterialPresenter presenter)
        {
            _model = model;
            _viewModel = viewModel;
            _presenter = presenter;
        }

        public CsvLogger Logger { get; set; }

        public void SetupLogger()
        {
            var filenameDate = DateTime.Now.ToString("yyyy-dd-M--HH-mm-ss");

            Logger = new CsvLogger("log_" + filenameDate + ".csv");

            Logger.WriteHeader("DateTime", "Prefix", "Iteration",
                                "Error", "InputError", "OutputError", "MinError",
                                "Accepted", "ForceAccept", "SimTemp",
                                "DoF", 
                                "InputPathLengthTarget", "OutputPathLengthTarget",
                                "InputPathLengthCurrent", "OutputPathLengthCurrent",
                                "InputInflectionPointsTarget", "OutputInflectionPointsTarget",
                                "InputPathLengthCurrent", "OutputPathLengthCurrent",
                                "BitCode");
        }

        public void Start()
        {
            StoreModel();

            var minAchor = _originalAnchors[0];
            var minAnchorZeroDistance = minAchor.X + minAchor.Y;

            foreach (var anchor in _originalAnchors)
            {
                var anchorZeroDistance = anchor.X + anchor.Y;

                if (anchorZeroDistance < minAnchorZeroDistance)
                {
                    minAchor = anchor;
                    minAnchorZeroDistance = anchorZeroDistance;
                }
            }

            var numCellHierarchies = _viewModel.NumScalesOptimization;
            var pathLengths = GetPathLengths();

            for (int i = 0; i < numCellHierarchies; i++)
            {
                _model.DecreaseScale(true);
                Console.WriteLine(@"Scale down during hierarchical optimizatio for scale " + i);
            }

            for (int i = 0; i <= numCellHierarchies; i++)
            {
                SetAnchorsBasedOnOriginalAnchors(numCellHierarchies, i, minAchor);

                Console.WriteLine(@"Run for scale " + i);

                var resampledInputPaths = ResamplePaths(pathLengths, _model.InputPath);
                var resampledOutputPaths = ResamplePaths(pathLengths, _model.OutputPath);

                for (int j = 0; j < pathLengths.Count; j++)
                {
                    SimulationModel minErrorModel = null;

                    var currentNumOptimizationRuns = 0;
                    var resultDidImprove = true;

                    while (resultDidImprove && currentNumOptimizationRuns <= NUM_MAX_OPTI_RESTARTS)
                    {
                        var modelsInRun = new List<SimulationModel>();

                        for (int k = 0; k < NUM_PARALLEL_OPTI_RUNS; k++)
                        {
                            var model = new SimulationModel(_model, _viewModel) { EnableSimulatedAnnealing = true };

                            //if (k == 0)
                            //{
                            //    model.EnableSimulatedAnnealing = false;
                            //}
                            //else
                            //{
                            //    model.EnableSimulatedAnnealing = true;
                            //}

                            if (minErrorModel != null)
                            {
                                model.AssignValues(minErrorModel);
                            }

                                model.Model.InputPath = new List<Vector>(resampledInputPaths[j]);
                            model.Model.OutputPath = new List<Vector>(resampledOutputPaths[j]);

                            modelsInRun.Add(model);
                        }

                        foreach (var currentModel in modelsInRun)
                        {
                            Console.WriteLine(@"Run for path length " + j + @" at scale " + i + @" with # of points " + currentModel.Model.InputPath.Count + @" in run " + currentNumOptimizationRuns);

                            var optimizer = new IterativeCellOptimizationController(currentModel);

                            if (Logger != null)
                            {
                                optimizer.Logger = Logger;
                                Logger.AddOrChangePrefix(i.ToString("D") + "_" + currentNumOptimizationRuns.ToString("D") + "_" + modelsInRun.IndexOf(currentModel), this);
                            }

                            optimizer.StartOptimization();
                        }

                        var minErrorModelInRun = modelsInRun[0];

                        foreach (var model in modelsInRun)
                        {
                            if (model.MinError < minErrorModelInRun.MinError)
                            {
                                minErrorModelInRun = model;
                            }
                        }

                        Console.WriteLine(@"Best result in run " + currentNumOptimizationRuns + @" has error " + minErrorModelInRun.MinError + @" and is index " + modelsInRun.IndexOf(minErrorModelInRun));

                        if (minErrorModel != null)
                        {
                            var minErrorImprovement = minErrorModel.MinError - minErrorModelInRun.MinError;
                            resultDidImprove = minErrorImprovement > SimulationModel.MinImprovement;

                            if (resultDidImprove)
                            {
                                minErrorModel = minErrorModelInRun;
                                _model = minErrorModel.Model;
                                _viewModel.Model = _model;
                            }
                        }
                        else
                        {
                            minErrorModel = minErrorModelInRun;
                            _model = minErrorModel.Model;
                            _viewModel.Model = _model;
                        }

                        currentNumOptimizationRuns++;
                    }

                    Console.WriteLine("Best result has error " + minErrorModel.MinError + " after num of iterations " + currentNumOptimizationRuns);

                    _viewModel.Model = _model;
                    _presenter.Model = _model;

                    _model.UpdateConstraintsGraph();

                    if (Logger != null)
                    {
                        _presenter.ExportCurrentConfiguration("_auto_" + i + "_" + j + "_solution");
                    }
                }

                if (i != numCellHierarchies)
                {
                    _model.Scale(2, true);
                    Console.WriteLine(@"Scale up during hierarchical optimization for scale " + i);
                }
            }

            Application.Current.Dispatcher.Invoke(_viewModel.Redraw);
        }

        private void SetAnchorsBasedOnOriginalAnchors(int numCellHierarchies, int currentHierarchy, Vertex minAchor)
        {
            //Remove old anchors
            foreach (var anchor in _model.GetAnchors())
            {
                _model.SetAnchor(anchor.ToVector(), false);
            }

            var anchorScaleFactor = Math.Pow(.5, numCellHierarchies - currentHierarchy);
            //Anchors are set relativ to minimum anchor. If all anchors fall on the same spot, move them away from the min anchor.
            Console.WriteLine("anchor scale is " + anchorScaleFactor + ". # anchors is " + _originalAnchors.Count);

            var scaledMinAnchor = minAchor.ToVector() * anchorScaleFactor;
            _model.SetAnchor(scaledMinAnchor, true);
            Console.WriteLine("set anchor at " + scaledMinAnchor);

            foreach (var anchor in _originalAnchors)
            {
                if (!anchor.Equals(minAchor))
                {
                    var minAnchorDistanceX = anchor.X - minAchor.X;
                    var minAnchorDistanceY = anchor.Y - minAchor.Y;

                    var scaleMinAnchorDistanceX = minAnchorDistanceX * anchorScaleFactor;
                    var scaleMinAnchorDistanceY = minAnchorDistanceY * anchorScaleFactor;

                    var anchorToMinAnchorDistanceX = scaleMinAnchorDistanceX >= 0
                        ? Math.Ceiling(scaleMinAnchorDistanceX)
                        : Math.Floor(scaleMinAnchorDistanceX);

                    var anchorToMinAnchorDistanceY = scaleMinAnchorDistanceY >= 0
                        ? Math.Ceiling(scaleMinAnchorDistanceY)
                        : Math.Floor(scaleMinAnchorDistanceY);

                    var anchorPosition = new Vector(scaledMinAnchor.X + anchorToMinAnchorDistanceX,
                        scaledMinAnchor.Y + anchorToMinAnchorDistanceY);
                    _model.SetAnchor(anchorPosition, true);
                    Console.WriteLine("set anchor at " + anchorPosition + " w distance " + anchorToMinAnchorDistanceX + " / " + anchorToMinAnchorDistanceY);
                }
            }
        }

        private List<int> GetPathLengths()
        {
            var pathLengths = new List<int>();

            var numPathHierarchies = _viewModel.NumPathDivides;

            if (numPathHierarchies < 1)
            {
                pathLengths.Add(_viewModel.MaxNumberPathSamples);
                return pathLengths;
            }

            var minMaxPathDifference = _viewModel.MaxNumberPathSamples - _viewModel.MinNumberPathSamples;
            int offset = minMaxPathDifference / numPathHierarchies;

            for (int i = numPathHierarchies; i > 0; i--)
            {
                pathLengths.Add(_viewModel.MaxNumberPathSamples - offset * i);
            }

            pathLengths.Add(_viewModel.MaxNumberPathSamples);

            return pathLengths;
        }

        private List<List<Vector>> ResamplePaths(List<int> pathLengths, List<Vector> targetPath)
        {
            var resampledPaths = new List<List<Vector>>();
            List<Point> targetPoints = targetPath.Select(p => new Point(p.X, p.Y)).ToList();

            foreach (var pathLength in pathLengths)
            {
                var path = PolylineHelper.SamplePolygon(targetPoints, pathLength, false).Select(p => new Vector(p.X, p.Y)).ToList();
                resampledPaths.Add(path);
            }

            return resampledPaths;
        }

        private void StoreModel()
        {
            _originalCells = new List<Cell>(_model.Cells);
            _originalAnchors = new List<Vertex>(_model.GetAnchors());

            _inputPath = new List<Vector>(_model.InputPath);
            _inputVertex = _model.InputVertex;

            _outputPath = new List<Vector>(_model.OutputPath);
            _outputVertex = _model.InputVertex;
        }
    }
}