#define VERBOSE

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Net.Mime;
using System.Windows;
using ShearCell_Interaction.Helper;
using ShearCell_Interaction.Model;
using ShearCell_Interaction.Simulation;
using ShearCell_Interaction.View;


namespace ShearCell_Interaction.Optimisation
{

    public class IterativeCellOptimizationController
    {
        private readonly SimulationModel _simModel;


        private readonly OptimizedSimulationController _simulationController;

        private List<Cell> _cellStorage;
        private readonly Random _random;
        public CsvLogger Logger { get; set; }

        private List<Cell> bestResult;
        private double bestResultError = Double.PositiveInfinity;
        private int bestResultDof;


        public IterativeCellOptimizationController(SimulationModel simModel)
        {
            _simModel = simModel;

            _simulationController = new OptimizedSimulationController(simModel.Model, simModel.ViewModel);
            _cellStorage = new List<Cell>();

            _random = new Random();

            bestResult = new List<Cell>();
        }

        public void StartOptimization()
        {
            _simModel.CurrentError = _simModel.MinError;

            Console.WriteLine("Generation optimization started at " + DateTime.Now);

            _simModel.StartErrorWasSet = false;
            _simModel.NumAcceptedSolutions = 0;
            _simModel.NumRejectedSolutions = 0;
            _simModel.NumAcceptedSolutionsSimAnn = 0;
            _simModel.CurrentNumIterations = 0;

            _simModel.InputPathLength = PolylineHelper.PathLength(_simModel.Model.InputPath);
            _simModel.OutputPathLength = PolylineHelper.PathLength(_simModel.Model.OutputPath);

            _simulationController.OnSimulationInfoAvailable -= OnSimulationInfoAvailable;
            _simulationController.OnSimulationInfoAvailable += OnSimulationInfoAvailable;

            _simulationController.StartOptimizedPathSimulation();
        }

        private void OnSimulationInfoAvailable(object sender, SimulationInfoEventArgs e)
        {
            _simModel.CurrentNumIterations++;

            _simModel.Model.UpdateConstraintsGraph();

            if (_simModel.MaxNumIterations > 10)
            {
                if (_simModel.CurrentNumIterations % (_simModel.MaxNumIterations / 10) == 0)
                    Console.WriteLine((_simModel.CurrentNumIterations / (double)_simModel.MaxNumIterations * 100.0)
                                      .ToString("F2") + @"% done");
            }

            //pointwise euclidian distances normalized with overall path length
            var inputError = GetInputError(e.FramePosition);
            var outputError = GetOutputError(e.FramePosition);

            var normalizedInputError = inputError / _simModel.InputPathLength;
            var normalizedOutputError = outputError / _simModel.OutputPathLength;

            // Simulation sometimes break if the input is rigid.
            // Ignore these solutions are restart trial.

            var inputIndex = _simModel.Model.Vertices.IndexOf(_simModel.Model.InputVertex);
            var inputPositions = e.FramePosition.Select(frame => frame[inputIndex]).ToList();
            var inputLength = PolylineHelper.PathLength(inputPositions);

            var ouputIndex = _simModel.Model.Vertices.IndexOf(_simModel.Model.OutputVertex);
            var outputPositions = e.FramePosition.Select(frame => frame[ouputIndex]).ToList();
            var outputLength = PolylineHelper.PathLength(outputPositions);

            var solutionIsRigid = inputLength < SimulationModel.MinErrorDelta || outputLength < SimulationModel.MinErrorDelta 
                                    || normalizedInputError > 30 || normalizedOutputError > 30;

            if (solutionIsRigid)
            {
                Console.WriteLine("## error " + inputLength.ToString("F4") + " / " + outputLength.ToString("F4"));

                ResetSolution(_cellStorage);

                _simModel.Model.UpdateConstraintsGraph();
                _simModel.NumRejectedSolutions--;

                Step();
                _simulationController.StartOptimizedPathSimulation();

                return;
            }

            _simModel.ViewModel.InputPathDistance = normalizedInputError;
            _simModel.ViewModel.OutputPathDistance = normalizedOutputError;

            _simModel.CurrentError = normalizedInputError * _simModel.InputWeight + normalizedOutputError * _simModel.OutputWeight;

            if (!_simModel.StartErrorWasSet)
            {
                _simModel.StartError = _simModel.CurrentError;
                _simModel.StartErrorWasSet = true;
            }

            var forceChange = false;
            var currentTemp = 0.0;
            var objective = 0.0;

            if (_simModel.EnableSimulatedAnnealing)
            {
                var errorDelta = _simModel.CurrentError - _simModel.MinError;

                //error should be small
                objective = 1.0 / _simModel.CurrentError;
                //currentTemp is between 0 and 0.5
                currentTemp = 1.0 / (1.0 + Math.Exp(objective / (_simModel.SimAnnStartTemperature * Math.Pow(SimulationModel.SimAnnFalloff, _simModel.CurrentNumIterations))));
                var random = new Random();
                forceChange = random.NextDouble() < currentTemp;
            }

            var doFDifference = _simModel.CurrentDof - _simModel.Model.CurrentDoF;
            var errorDifference = _simModel.MinError - _simModel.CurrentError;

            //accept if error is smaller than MinErrorDelta or slightly larger (-MinErrorDelta) but smaller DoF
            bool accepted = errorDifference > SimulationModel.MinErrorDelta || (errorDifference < SimulationModel.MinErrorDelta && errorDifference > -SimulationModel.MinErrorDelta && doFDifference > 0);

            if (Logger != null)
            {
                LogStep(e.FramePosition, normalizedInputError, normalizedOutputError, accepted, forceChange, currentTemp);
            }

            if (accepted)
            {
                _simModel.MinError = _simModel.CurrentError;

                StoreAsBestResult();
#if (VERBOSE)
                Console.WriteLine($"Iteraction {_simModel.CurrentNumIterations}/{_simModel.MaxNumIterations}. Solution accepted with error {_simModel.MinError.ToString("G0")}");
#endif
                _simModel.NumAcceptedSolutions++;
            }
            else if (forceChange)
            {
                _simModel.MinError = _simModel.CurrentError;
#if (VERBOSE)
                Console.WriteLine($"Iteraction {_simModel.CurrentNumIterations}/{_simModel.MaxNumIterations}. Solution accepted (SimAnn {currentTemp.ToString("F3")}, {objective.ToString("F3")}) with error {_simModel.MinError.ToString("G0")}");
#endif
                _simModel.Model.UpdateConstraintsGraph();
                _simModel.NumAcceptedSolutionsSimAnn++;
            }
            else
            {
#if (VERBOSE)
                Console.WriteLine($"Iteraction {_simModel.CurrentNumIterations}/{_simModel.MaxNumIterations}. Solution rejected with error {_simModel.CurrentError.ToString("G0")}");
#endif
                ResetSolution(_cellStorage);

                _simModel.Model.UpdateConstraintsGraph();
                _simModel.NumRejectedSolutions++;
            }

            if (_simModel.CurrentNumIterations >= _simModel.MaxNumIterations)
            {
                _simulationController.OnSimulationInfoAvailable -= OnSimulationInfoAvailable;
                CompleteOptimization();
            }
            else
            {
                Step();
                _simulationController.StartOptimizedPathSimulation();
            }
        }

        private double GetOutputError(List<List<Vector>> framePositions)
        {
            if (_simModel.Model.OutputVertex != null)
            {
                var outputIndex = _simModel.Model.Vertices.IndexOf(_simModel.Model.OutputVertex);

                if (outputIndex >= 0)
                {
                    var outputPositions = framePositions.Select(frame => frame[outputIndex]).ToList();
                    var length = PolylineHelper.PathLength(outputPositions);

                    if (length <= .01)
                        return -1;

                    var distance = PolylineHelper.Distance(outputPositions, _simModel.Model.OutputPath);
                    return distance;
                }
            }
            throw new Exception("Incomplete result. Missing output vertex or path for optimization.");
        }

        private double GetInputError(List<List<Vector>> framePositions)
        {
            if (_simModel.Model.InputVertex != null)
            {
                var inputIndex = _simModel.Model.Vertices.IndexOf(_simModel.Model.InputVertex);

                if (inputIndex >= 0)
                {
                    var inputPositions = framePositions.Select(frame => frame[inputIndex]).ToList();

                    var length = PolylineHelper.PathLength(inputPositions);

                    if (length <= .01)
                        return -1;

                    var distance = PolylineHelper.Distance(inputPositions, _simModel.Model.InputPath);
                    return distance;
                }
            }

            throw new Exception("Incomplete result. Missing input vertex or path for optimization.");
        }

        private void StoreAsBestResult()
        {
            bestResultError = _simModel.MinError;
            bestResultDof = _simModel.CurrentDof;

            bestResult.Clear();
            bestResult.AddRange(_simModel.Model.Cells);
        }

        private void ResetSolution(List<Cell> cells)
        {
            var tempAnchors = new List<Vertex>();

            foreach (var cell in cells)
            {
                var anchors = cell.CellVertices.Where(vertex => vertex.IsAnchor);
                tempAnchors.AddRange(anchors.ToList());
            }

            foreach (var cell in cells)
            {
                var indexPosition = cell.IndexVertex.ToVector();

                if (cell is ShearCell)
                {
                    _simModel.Model.AddCell(new ShearCell(), indexPosition, new Size(1, 1), false);
                }
                else
                {
                    _simModel.Model.AddCell(new RigidCell(), indexPosition, new Size(1, 1), false);
                }
            }

            foreach (var anchor in tempAnchors)
            {
                _simModel.Model.AddAnchor(anchor.ToInitialVector());
            }
        }

        private void Step()
        {
            _simModel.Model.UpdateConstraintsGraph();

            _simModel.CurrentDof = _simModel.Model.CurrentDoF;
            _simModel.Model.ResetDeformation();
            _cellStorage.Clear();

            var currentDof = _simModel.Model.CurrentDoF;

            var randomOperation = _random.NextDouble();

            var operationIsMerge = currentDof >= _simModel.MaxTargetDof || randomOperation < .5;
            var operationIsSplit = currentDof <= _simModel.MinTargetDof || !operationIsMerge;

            Debug.WriteLine(currentDof + " merge: " + operationIsMerge + " / split " + operationIsSplit);

            if (operationIsSplit)
            {
                SplitComponent();
            }
            else
            {
                MergeComponents();
            }
        }

        private void SplitComponent()
        {
            var componentIndex = _random.Next(0, _simModel.Model.ConstraintGraph.Count);

            //double rigidityThreshold = 0.1;
            //HashSet<HashSet<Edge>> component = null;

            //var i = 0;
            //foreach (var comp in _simModel.Model.ConstraintGraph)
            //{
            //    var rigidity = _simModel.Model.GetRigidityOfComponent(comp);

            //    if (rigidity >= rigidityThreshold)
            //    {
            //        component = comp;
            //        break;
            //    }

            //    i++;
            //}

            //if (component == null)
            //{
            //    Console.WriteLine("No component with rigidity >= {0} found. Return.", rigidityThreshold);
            //    return;
            //}

            var currentDof = _simModel.Model.CurrentDoF;

            while (currentDof <= _simModel.Model.CurrentDoF)
            {
                if (componentIndex >= _simModel.Model.ConstraintGraph.Count)
                    throw new Exception("Splitting a component lead to a decrease in connected components. Something is probably wrong.");

                var component = _simModel.Model.ConstraintGraph[componentIndex];

                var splitCandidates = _simModel.Model.GetCandidatesForSplitting(component);
                Debug.WriteLine("# split candidates: {0}", splitCandidates.Count);

                if (splitCandidates.Count < 1)
                    return;

                var candidateIndex = _random.Next(0, splitCandidates.Count);

                var splitModeIndex = _random.Next(0, 2);
                var splitMode = (SplitMode)splitModeIndex;

                Debug.WriteLine("Split component with {0} cells at {1} with mode {2}", component.Count, splitCandidates[candidateIndex], splitMode);

                var changedCells = _simModel.Model.SplitComponentAt(component, splitCandidates[candidateIndex], splitMode);
                _cellStorage.AddRange(changedCells);

                foreach (var cell in changedCells)
                {
                    var anchors = cell.CellVertices.Where(vertex => vertex.IsAnchor);
                    foreach (var anchor in anchors)
                        _simModel.Model.AddAnchor(anchor.ToInitialVector());
                }

                _simModel.Model.UpdateConstraintsGraph();
            }
        }

        private void MergeComponents()
        {
            var currentDof = _simModel.Model.CurrentDoF;
            var newDof = _simModel.Model.CurrentDoF;

            while (newDof >= currentDof)
            {
                int cc1, cc2;
                var success = _simModel.Model.TryPickRandomComponents(out cc1, out cc2);
                if (!success)
                {
                    Debug.WriteLine("Could not pick random merge components. No candidates.");
                    continue;
                }

                var mergeCandidates = _simModel.Model.GetCandidatesForMerging(_simModel.Model.ConstraintGraph[cc1], _simModel.Model.ConstraintGraph[cc2]);
                if (mergeCandidates.Count < 1)
                {
                    Debug.WriteLine("Could not merge. No candidates.");
                    continue;
                }

                var cell = mergeCandidates[_random.Next(0, mergeCandidates.Count)];

                var anchors = cell.CellVertices.Where(v => v.IsAnchor);

                Debug.WriteLine(">>> merge at {0}", cell);

                _cellStorage.Add(cell);
                _simModel.Model.AddCell(new RigidCell(), cell.IndexVertex.ToInitialVector(), new Size(1, 1));

                foreach (var anchor in anchors)
                    _simModel.Model.AddAnchor(anchor.ToInitialVector());

                _simModel.Model.UpdateConstraintsGraph();
                newDof = _simModel.Model.CurrentDoF;
            }
        }

        private void CompleteOptimization(bool restoreBest = true)
        {
            var doFDifference = _simModel.CurrentDof - bestResultDof;
            var errorDifference = _simModel.MinError - bestResultError;

            //accept if error is smaller than MinErrorDelta or slightly larger (-MinErrorDelta) but smaller DoF
            bool shouldRestore = errorDifference > SimulationModel.MinErrorDelta || (errorDifference < SimulationModel.MinErrorDelta && errorDifference > -SimulationModel.MinErrorDelta && doFDifference > 0);

            if (restoreBest && shouldRestore)
            {
                _simModel.MinError = bestResultError;
                _simModel.CurrentDof = bestResultDof;
                ResetSolution(bestResult);
#if (VERBOSE)
                Console.WriteLine("Restored best result from SimAnn.");
#endif
            }

            //Console.WriteLine("Generation optimization ended at " + DateTime.Now);
            PrintStats();
            //Application.Current.Dispatcher.Invoke(_simModel.ViewModel.Redraw);
        }

        private void PrintStats()
        {
            Console.WriteLine("Generation finished at " + DateTime.Now + $" Start error was {_simModel.StartError.ToString("G0")}. End error  {_simModel.MinError.ToString("G0")}. Accepted {_simModel.NumAcceptedSolutions}.  Accepted SimAnn{_simModel.NumAcceptedSolutionsSimAnn}. Rejected {_simModel.NumRejectedSolutions}");
        }

        private void LogStep(List<List<Vector>> framePositions, double normalizedInputError, double normalizedOutputError, bool accepted, bool forceChange, double currentTemp)
        {

            var inputIndex = _simModel.Model.Vertices.IndexOf(_simModel.Model.InputVertex);
            var inputPositions = framePositions.Select(frame => frame[inputIndex]).ToList();
            var inputLength = PolylineHelper.PathLength(inputPositions);

            var ouputIndex = _simModel.Model.Vertices.IndexOf(_simModel.Model.OutputVertex);
            var outputPositions = framePositions.Select(frame => frame[ouputIndex]).ToList();
            var outputLength = PolylineHelper.PathLength(outputPositions);

            var inputInflectionPointsInput = PolylineHelper.NumInflectionPoints(_simModel.Model.InputPath);
            var outputInflectionPointsInput = PolylineHelper.NumInflectionPoints(_simModel.Model.OutputPath);

            var inputInflectionPointsCurrent = PolylineHelper.NumInflectionPoints(inputPositions);
            var outputInflectionPointsCurrent = PolylineHelper.NumInflectionPoints(outputPositions);
            

            Logger.Log(
                (_simModel.CurrentNumIterations-1).ToString("D"),
                _simModel.CurrentError.ToString("F4"),
                normalizedInputError.ToString("F4"),
                normalizedOutputError.ToString("F4"),
                _simModel.MinError.ToString("F4"),
                accepted ? "TRUE" : "FALSE",
                forceChange ? "TRUE" : "FALSE",
                currentTemp.ToString("F4"),
                _simModel.Model.CurrentDoF.ToString("D"),
                _simModel.InputPathLength.ToString("F4"),
                _simModel.OutputPathLength.ToString("F4"),
                inputLength.ToString("F4"),
                outputLength.ToString("F4"),
                inputInflectionPointsInput.ToString("D"),
                outputInflectionPointsInput.ToString("D"),
                inputInflectionPointsCurrent.ToString("D"),
                outputInflectionPointsCurrent.ToString("D"),
                _simModel.Model.GetEncoding()
            );
        }

    }
}